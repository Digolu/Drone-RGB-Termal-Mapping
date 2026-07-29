from pymavlink import mavutil
import math
import sys, tty, termios, threading, time, queue

CONNECTION = 'udp:127.0.0.1:14550'
#CONNECTION = '/dev/ttyACM1'
BAUD = 115200
TAKEOFF_ALT = 50
waypoints = [
    (40.184603, -8.414432, 30),  # Ponto 1
    (40.184709, -8.414261, 30),  # Ponto 2
    (40.184620, -8.414051, 30),   # Ponto 3
    (40.184714, -8.413766, 30),   # Ponto 4
    (40.184618, -8.413564, 30)    # Ponto 5
]
routeType = ''
key1 = None

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

def set_mode(mode_name, timeout=5):
    mode_id = drone.mode_mapping()[mode_name]
    drone.mav.set_mode_send(
        drone.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )

    start = time.time()
    while time.time() - start < timeout:
        with mode_lock:
            if current_mode["value"] == mode_name:
                print(f"Modo alterado para {mode_name}")
                break
        time.sleep(0.1)
    
    print(f"Fim funcao set_mode, modo atual: {current_mode['value']}")

def monitor_mode_changes():
    global routeType
    print("A monitorizar mudanças de modo...")
    """Só LÊ o estado partilhado current_mode, nunca chama recv_match diretamente."""
    last_mode = None
    while True:
        #perdemos 1 hora aqui
        #print("Tipo de rota atual:", routeType)
        #print("Modo Last:", last_mode)
        #print("Modo atual:", current_mode["value"])
        with mode_lock:
            mode_now = current_mode["value"]
        if mode_now != last_mode:
            print(f"Modo mudou: {last_mode} -> {mode_now}")
            if mode_now == 'GUIDED' and last_mode != 'GUIDED' and routeType == 'linear':
                print(">>> Modo GUIDED detetado, a correr o código Python...")
                print("A executar rota linear...")
                threading.Thread(target=viagem()).start()
            if mode_now == 'GUIDED' and last_mode != 'GUIDED' and routeType == 'spline':
                print(">>> Modo GUIDED detetado, a correr o código Python...")
                print("A executar rota polinomial...")
                threading.Thread(target=spline_route_local, args=(waypoints,)).start()         
        last_mode = current_mode["value"]

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

def wait_reached(lat, lon, alt, radius=3.0, alt_tol=1.0, timeout=180, hold_time=2.0):
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
    #print("A mudar para GUIDED...")
    #set_mode('GUIDED')
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
#### ==========================       slipne route

def catmull_rom_point(p0, p1, p2, p3, t):
    """Interpola um ponto entre p1 e p2 usando Catmull-Rom, t em [0,1]."""
    t2 = t * t
    t3 = t2 * t
    def interp(c0, c1, c2, c3):
        return 0.5 * (
            2*c1 +
            (-c0 + c2) * t +
            (2*c0 - 5*c1 + 4*c2 - c3) * t2 +
            (-c0 + 3*c1 - 3*c2 + c3) * t3
        )
    lat = interp(p0[0], p1[0], p2[0], p3[0])
    lon = interp(p0[1], p1[1], p2[1], p3[1])
    alt = interp(p0[2], p1[2], p2[2], p3[2])
    return (lat, lon, alt)

def generate_spline_path(waypoints, points_per_segment=10):
    """Gera pontos intermédios suavizados entre os waypoints."""
    pts = [waypoints[0]] + waypoints + [waypoints[-1]]  # duplica extremos
    path = []
    for i in range(1, len(pts) - 2):
        p0, p1, p2, p3 = pts[i-1], pts[i], pts[i+1], pts[i+2]
        for step in range(points_per_segment):
            t = step / points_per_segment
            path.append(catmull_rom_point(p0, p1, p2, p3, t))
    path.append(waypoints[-1])
    return path

def wait_close_enough(lat, lon, alt, radius=4.0, alt_tol=1.5, timeout=15):
    """Avança assim que estiver dentro do raio, SEM exigir estabilização (hold_time=0)."""
    start = time.time()
    while time.time() - start < timeout:
        with position_lock:
            lat_now = current_position["lat"]
            lon_now = current_position["lon"]
            alt_now = current_position["alt"]
        if lat_now is not None:
            dist = horizontal_distance(lat_now, lon_now, lat, lon)
            dalt = abs(alt_now - alt)
            if dist <= radius and dalt <= alt_tol:
                return True
        time.sleep(0.1)
    return False

def spline_route_local(waypoints):
    #print("A mudar para GUIDED...")
    #set_mode('GUIDED')
    path = generate_spline_path(waypoints, points_per_segment=8)
    for lat, lon, alt in path:
        drone.mav.set_position_target_global_int_send(
            0, drone.target_system, drone.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b0000_1111_1111_1000,
            int(lat * 1e7), int(lon * 1e7), alt,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        # raio maior => avança antes de desacelerar totalmente
        wait_close_enough(lat, lon, alt, radius=6.0, alt_tol=1.5, timeout=15)
    print("Rota spline (local) completa!")


def viagem():

    def executar():
        for lat, lon, alt in waypoints:
            print(f"Indo para: lat={lat}, lon={lon}, alt={alt}m")
            route(lat, lon, alt)  # chamada direta -> só avança quando wait_reached terminar
        print("Rota completa!")

    threading.Thread(target=executar).start()

def RTL():
    print("A mudar para RTL...")
    set_mode('RTL')


def keyboard_loop():
    global routeType


    print("\nControlo de variaveis ativo:")
    print("  [s] Route Spline")
    print("  [l] Linear Route")
    print("  [h]  Helipad Landing")
    print("  [f] Follow Mode")
    print("  [q] Sair")

    while True:
        key = get_key()
        if key == 's':
            global routeType
            routeType = 'spline'  ## vida real
            print("Rota polinomial selecionada. Tipo de rota:", routeType)
        elif key == 'l':
            routeType = 'linear'
            print("Rota linear selecionada. Tipo de rota:", routeType)
        elif key == 'f':
            routeType = 'follow'
            print("Modo Follow selecionado. Tipo de rota:", routeType)
        elif key == 'h':
            routeType = 'helipad'
            print("Modo Helipad selecionado. Tipo de rota:", routeType)    
        elif key == 'q':
            print("A sair...")
            break       
            

if __name__ == '__main__':
    threading.Thread(target=reader_loop, daemon=True).start()
    threading.Thread(target=monitor_mode_changes, daemon=True).start()
    keyboard_loop()

