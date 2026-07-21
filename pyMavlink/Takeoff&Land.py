from pymavlink import mavutil
import sys, tty, termios, threading, time, queue

#CONNECTION = 'udp:127.0.0.1:14551'
CONNECTION = '/dev/ttyACM0'
BAUD = 115200
TAKEOFF_ALT = 5

drone = mavutil.mavlink_connection(CONNECTION, baud=BAUD)
print("À espera de heartbeat...")
drone.wait_heartbeat()
print(f"Ligado ao sistema {drone.target_system}, componente {drone.target_component}")

ack_queue = queue.Queue()
mode_lock = threading.Lock()
current_mode = {"value": None}


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


def keyboard_loop():
    print("\nControlo por teclado ativo:")
    print("  [t] Take off")
    print("  [l] Land")
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


if __name__ == '__main__':
    threading.Thread(target=reader_loop, daemon=True).start()
    threading.Thread(target=monitor_mode_changes, daemon=True).start()
    keyboard_loop()