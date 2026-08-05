from pymavlink import mavutil
import math
import sys, tty, termios, threading, time, queue
import sim_globalFunc as gf
#import sim_cameraHeliDetect as hd
import requests

CAMERA_HOST = "http://192.168.0.142:5000"  # or the Jetson's IP if scripts run on different hosts
HOTSPOT_HOST = "http://0.0.0.0:5000"
LOCAL_HOST =  "http://127.0.0.1:5050"

lasttime = 0
lasttime1 = 0
yaw=0
vx = 2
vy = 2
vz = 0

def get_desvio_from_simulation():
    try:
        print("Estou no get_desvio_from_simulation follow")
        r = requests.get(f"{LOCAL_HOST}/desvio_vermelho", timeout=1)
        r.raise_for_status()
        return r.json()  # dict or None
    except requests.RequestException as e:
        #print(f"[Aviso] não consegui contactar o servidor de câmara: {e}")
        return None

def followDrone():

    print("A iniciar follow drone, correr updateDronePosition()")
    updateDronePosition()

def updateDronePositions():
    global lasttime,lasttime1,yaw
    moveD,moveE,moveN=0,0,0
    type_mask = 0b100111000000

    while True:
            
        current_time1=time.time()
        if (current_time1-lasttime1) > 0.033:     
            #desvios = hd.get_desvio_vermelho()   ##drone
            desvios = get_desvio_from_simulation()     # simulacao com pagina local host http "sim_camera.py"

            if desvios['x'] != None and desvios['y'] != None:  

                dist = desvios['tamanho']*pxToMeters()  # tamanho do alvo em metros
                
                yawrad = math.radians(gf.current_position["hdg"]/100)

                #print (f"Desvio x: {desvios['x']}px, Desvio y {desvios['y']}px ")
                
                current_time=time.time()
                if (current_time - lasttime) > 1:
                    print (f"Desvio esquerda: {desvios['esquerda']}px, Desvio direita {desvios['direita']}px ")
                    print (f"Desvio cima: {desvios['cima']}px, Desvio baixo {desvios['baixo']}px ")
                    print (f"Move norte {moveN}, move este {moveE}, move down {moveD}")
                    print (f"Distancia ao drone: {dist:.2f}m")
                    print (f"Yaw: {gf.current_position['hdg']/100:.2f} graus", f"Yaw rad: {yawrad:.2f} rad\n")
                    lasttime=current_time

                CorrectionFactor = 5

                NormalizedX = desvios['x'] /200  ## depois é por hd.FRAME_WIDTH
                NormalizedY = desvios['y'] /200

                #print (f"NormalizedX: {NormalizedX:.2f}, NormalizedY: {NormalizedY:.2f}")

                if (desvios['x']) != None:
                    moveE = math.cos(yawrad) + NormalizedX * CorrectionFactor
                else:
                    moveE = 0
                    
                if (desvios['y']) != None:
                    moveN = 2*math.sin(yawrad) + NormalizedY * CorrectionFactor
                else:
                    moveN = 0
                      
                moveD = 0


                move(moveN,moveE,moveD)
                



            lasttime1=current_time1   
            
def updateDronePosition():
    global lasttime, lasttime1
    
    KP_X = 0.05 
    KP_Y = 0.03  
    FORWARD_SPEED = 1.0

    while True:
        current_time1 = time.time()
        if (current_time1 - lasttime1) <= 0.033:
            continue
        lasttime1 = current_time1

        desvios = get_desvio_from_simulation()
        if desvios is None:
            continue

        # Erro em pixels (positivo = alvo à direita/baixo)
        error_x = desvios['x']  
        error_y = desvios['y'] 
        
        vRight = error_x * KP_X
        vDown = error_y * KP_Y

        tamanho_normalizado = desvios['tamanho'] / 100  # 0-1
        vForward = FORWARD_SPEED * (1 - tamanho_normalizado * 0.7) 
        vForward = max(0.2, vForward)
        
        yawrad = math.radians(gf.current_position['hdg'] / 100)

        moveN = vForward * math.cos(yawrad) - vRight * math.sin(yawrad)
        moveE = vForward * math.sin(yawrad) + vRight * math.cos(yawrad)
        moveD = vDown

        move(moveN, moveE, moveD)

        if (time.time() - lasttime) > 1:
            print(f"Erro: x={error_x:.1f}px, y={error_y:.1f}px, tamanho={desvios['tamanho']:.0f}px")
            print(f"Vel: Fwd={vForward:.2f}, Right={vRight:.2f}, Down={vDown:.2f}")
            print(f"Move: N={moveN:.2f}, E={moveE:.2f}, D={moveD:.2f}\n")
            lasttime = time.time()

def move(moveN, moveE, moveD):
    
    #print (f"Fiz move N={moveN},Fiz move E={moveE},Fiz moveD={moveD}")

    gf.drone.mav.set_position_target_local_ned_send(
        0,                                              # time_boot_ms (pode ser 0)
        gf.drone.target_system,
        gf.drone.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED, 
        0b100111000000,
        moveN,  # north
        moveE,  # east
        moveD,  # down (moveD > 0 significa descer)      
        vx, vy, vz,    # vx, vy, vz 
        0, 0, 0,    # afx, afy, afz (ignorados pelo type_mask)
        0, 0        # yaw (0 = norte), yaw_rate (yaw_rate ignorado pelo type_mask)
    ) 

def pxToMeters():

    alt = gf.current_position['alt']
    Vfov=24.4
    Hfov=31.1

    Vx = (alt * math.tan(math.radians(Vfov)))/640             #/hd.FRAME_HEIGHT
    Hx = (alt * math.tan(math.radians(Hfov)))/480              #/hd.FRAME_WIDTH

    #print(f"Altitude = {alt}, Vx={Vx}m, Hx={Hx}m")

    return Vx
