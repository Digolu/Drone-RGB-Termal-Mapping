from pymavlink import mavutil
import math
import sys, tty, termios, threading, time, queue
import sim_linear as jl
import sim_spline as js
import sim_helipad as jh

CONNECTION = 'udp:127.0.0.1:14550'
#CONNECTION = '/dev/ttyACM0'
BAUD = 115200
TAKEOFF_ALT = 50
waypoints = [
    (40.184603, -8.414432, 30),  # Ponto 1
    (40.184709, -8.414261, 20),  # Ponto 2
    (40.184620, -8.414051, 30),   # Ponto 3
    (40.184714, -8.413766, 20),   # Ponto 4
    (40.184618, -8.413564, 30)    # Ponto 5
]
routeType = ''
key1 = None

drone = mavutil.mavlink_connection(CONNECTION, baud=BAUD)
print("À espera de heartbeat...")
drone.wait_heartbeat()
print(f"Ligado ao sistema {drone.target_system}, componente {drone.target_component}")

# Ligação UDP de saída: "espelha" toda a telemetria para a rede WiFi.
# Qualquer GCS (Mission Planner, QGroundControl, MAVProxy...) na mesma rede
# pode ligar-se a esta porta, tal como faria a um rádio de telemetria do ArduPilot.

TELEMETRY_UDP_PORT = 14550
telem_out = None
try:
    telem_out = mavutil.mavlink_connection(
        f'udpbcast:255.255.255.255:{TELEMETRY_UDP_PORT}',
        input=False
    )
    print(f"[telemetria] a difundir em udp broadcast :{TELEMETRY_UDP_PORT}")
except Exception as e:
    print(f"[telemetria] não foi possível abrir socket UDP: {e}")



ack_queue = queue.Queue()
mode_lock = threading.Lock()
current_mode = {"value": None}

position_lock = threading.Lock()
current_position = {"lat": None, "lon": None, "alt": None, "heading": None}


def reader_loop():
    """ÚNICA thread que lê da ligação MAVLink."""
    while True:
        #print ("Dentro do reader LOOp")
        msg = drone.recv_match(blocking=True, timeout=1)
        if msg is None:
            continue

        # --- reenvia SEMPRE a mensagem para a rede WiFi, seja ela qual for ---
        if telem_out is not None:
            try:
                telem_out.mav.send(msg)
            except Exception as e:
                print(f"[telemetria] erro ao enviar: {e}")

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
                current_position["hdg"] = msg.hdg
                #print(f"Alt no linear: {current_position['alt']}")


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
            changemode= mode_now == 'GUIDED' and last_mode != 'GUIDED'    
            if changemode == True and routeType == 'linear':
                print(">>> Modo GUIDED detetado, a correr o código Python")
                print("A executar rota linear")
                threading.Thread(target=jl.viagem()).start()
            elif changemode == True and routeType == 'spline':
                print(">>> Modo GUIDED detetado, a correr o código Python")
                print("A executar rota polinomial")
                threading.Thread(target=js.spline_route_local, args=(waypoints,)).start()       
            elif changemode == True and routeType == 'follow':
                print(">>> Modo GUIDED detetado, a correr o código Python")
                print("A executar follow mode")
                #threading.Thread(target=js.spline_route_local, args=(waypoints,)).start()
            elif changemode == True and routeType == 'helipad':
                print(">>> Modo GUIDED detetado, a correr o código Python")
                print("A executar landing on helipad")
                threading.Thread(target=jh.guidedLanding, args=(waypoints,)).start()                      
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

def RTL():
    print("A mudar para RTL...")
    set_mode('RTL')

def keyboard_loop():
    global routeType

    print("\nControlo de variaveis ativo:")
    print("  [s] Route Spline")
    print("  [l] Linear Route")
    print("  [h] Helipad Landing")
    print("  [f] Follow Mode")
    print("  [q] Sair")

    while True:
        key = get_key()
        if key == 's':
            global routeType
            routeType = 'spline'  ## vida real
            print("Rota polinomial selecionada. Tipo de rota:", routeType)
            break
        elif key == 'l':
            routeType = 'linear'
            print("Rota linear selecionada. Tipo de rota:", routeType)
            break
        elif key == 'f':
            routeType = 'follow'
            print("Modo Follow selecionado. Tipo de rota:", routeType)
            break   
        elif key == 'h':
            routeType = 'helipad'
            print("Modo Helipad selecionado. Tipo de rota:", routeType)   
            break 
        elif key == 'q':
            print("A sair...")
            break       
            

if __name__ == '__main__':
    threading.Thread(target=reader_loop, daemon=True).start()
    threading.Thread(target=monitor_mode_changes, daemon=True).start()
    keyboard_loop()
