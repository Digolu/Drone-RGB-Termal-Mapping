from pymavlink import mavutil
import math
import sys, tty, termios, threading, time, queue

CONNECTION = 'udp:127.0.0.1:14550'
#CONNECTION = '/dev/ttyACM0'
BAUD = 115200
TAKEOFF_ALT = 5

drone = mavutil.mavlink_connection(CONNECTION, baud=BAUD)
print("À espera de heartbeat...")
drone.wait_heartbeat()
print(f"Ligado ao sistema {drone.target_system}, componente {drone.target_component}")

ack_queue = queue.Queue()
mode_lock = threading.Lock()
current_mode = {"value": None}

position_lock = threading.Lock()
current_position = {"lat": None, "lon": None, "alt": None}


def reader_loop():
    """ÚNICA thread que lê da ligação MAVLink."""
    while True:
        msg = drone.recv_match(blocking=True, timeout=1)
        if msg is None:
            continue
        mtype = msg.get_type()
        if mtype == 'HEARTBEAT':
            with mode_lock:
                current_mode["value"] = drone.flightmode
        elif mtype == 'COMMAND_ACK':
            ack_queue.put(msg)
        elif mtype == 'GLOBAL_POSITION_INT':
            with position_lock:
                current_position["lat"] = msg.lat / 1e7
                current_position["lon"] = msg.lon / 1e7
                current_position["alt"] = msg.relative_alt / 1000.0  # mm -> m


def wait_ack(command_id, timeout=5):
    """Consome da fila de ACKs (alimentada só pela reader_loop)."""
    start = time.time()
    while time.time() - start < timeout:
        try:
            msg = ack_queue.get(timeout=timeout - (time.time() - start))
        except queue.Empty:
            return None
        if msg.command == command_id:
            return msg
        # ACK de outro comando -> descarta e continua à espera
    return None


def set_mode(mode_name):
    mode_id = drone.mode_mapping()[mode_name]
    drone.mav.set_mode_send(
        drone.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )
    while True:
        with mode_lock:
            if current_mode["value"] == mode_name:
                break
        time.sleep(0.1)


def arm():
    drone.mav.command_long_send(
        drone.target_system, drone.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    ack = wait_ack(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM)
    print("Arm ACK:", ack)
    time.sleep(1)


def disarm():
    drone.mav.command_long_send(
        drone.target_system, drone.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    ack = wait_ack(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM)
    print("Disarm ACK:", ack)


def takeoff():
    print("A armar...")
    arm()
    print("A mudar para GUIDED...")
    set_mode('GUIDED')
    print(f"A descolar para {TAKEOFF_ALT}m...")
    drone.mav.command_long_send(
        drone.target_system, drone.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, TAKEOFF_ALT
    )
    ack = wait_ack(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF)
    print("Takeoff ACK:", ack)


def land():
    print("A pousar...")
    set_mode('LAND')


def monitor_mode_changes():
    """Só LÊ o estado partilhado current_mode, nunca chama recv_match diretamente."""
    last_mode = None
    while True:
        with mode_lock:
            mode_now = current_mode["value"]
        if mode_now != last_mode:
            print(f"Modo mudou: {last_mode} -> {mode_now}")
            if mode_now == 'AUTO' and last_mode != 'AUTO':
                print(">>> Modo AUTO detetado, a correr o código Python...")
                threading.Thread(target=takeoff).start()
            last_mode = mode_now
        time.sleep(0.2)


def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key


def horizontal_distance(lat1, lon1, lat2, lon2):
    R = 6371000  # raio da Terra em metros
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2)**2
    return 2 * R * math.asin(math.sqrt(a))


def wait_reached(lat, lon, alt, radius=2.0, alt_tol=1.0, timeout=60, hold_time=2.0):
    """
    Espera até o drone estar a menos de 'radius' metros (horizontal)
    e 'alt_tol' metros (vertical) do alvo, de forma estável durante 'hold_time' segundos.
    """
    start = time.time()
    stable_since = None

    while time.time() - start < timeout:
        with position_lock:
            lat_now = current_position["lat"]
            lon_now = current_position["lon"]
            alt_now = current_position["alt"]

        if lat_now is not None:
            dist = horizontal_distance(lat_now, lon_now, lat, lon)
            dalt = abs(alt_now - alt)

            if dist <= radius and dalt <= alt_tol:
                if stable_since is None:
                    stable_since = time.time()
                elif time.time() - stable_since >= hold_time:
                    print(f"Chegou ao waypoint (dist={dist:.1f}m, dalt={dalt:.1f}m)")
                    return True
            else:
                stable_since = None

        time.sleep(0.5)

    print("Timeout: não chegou ao waypoint a tempo.")
    return False


def route(lat, lon, alt):
    print("A mudar para GUIDED...")
    set_mode('GUIDED')
    print(f"A enviar posição alvo: lat={lat}, lon={lon}, alt={alt}m...")

    # type_mask: usar apenas posição (ignora vel, accel, yaw, yaw_rate)
    type_mask = 0b0000_1111_1111_1000  # = 0x0DF8 = 3576 (Use Position)

    drone.mav.set_position_target_global_int_send(
        0,                                              # time_boot_ms (pode ser 0)
        drone.target_system,
        drone.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # alt relativa ao home
        type_mask,
        int(lat * 1e7),
        int(lon * 1e7),
        alt,        # metros acima do home (por causa do frame escolhido)
        0, 0, 0,    # vx, vy, vz (ignorados pelo type_mask)
        0, 0, 0,    # afx, afy, afz (ignorados)
        0, 0        # yaw, yaw_rate (ignorados)
    )

    wait_reached(lat, lon, alt)


def viagem():
    waypoints = [
        (-35.362532, 149.163264, 30),  # Ponto 1
        (-35.361718, 149.161394, 30),  # Ponto 2
        (-35.363000, 149.167521, 30)   # Ponto 3
    ]

    def executar():
        for lat, lon, alt in waypoints:
            print(f"Indo para: lat={lat}, lon={lon}, alt={alt}m")
            route(lat, lon, alt)  # chamada direta -> só avança quando wait_reached terminar
        print("Rota completa!")

    threading.Thread(target=executar).start()


def keyboard_loop():
    print("\nControlo por teclado ativo:")
    print("  [t] Take off")
    print("  [l] Land")
    print("  [r] Route")
    print("  [q] Sair\n")

    while True:
        key = get_key()
        if key == 't':
            threading.Thread(target=takeoff).start()
        elif key == 'l':
            threading.Thread(target=land).start()
        elif key == 'q':
            print("A sair...")
            break
        elif key == 'r':
            print("A executar rota...")
            viagem()


if __name__ == '__main__':
    threading.Thread(target=reader_loop, daemon=True).start()
    threading.Thread(target=monitor_mode_changes, daemon=True).start()
    keyboard_loop()