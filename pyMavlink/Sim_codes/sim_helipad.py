from pymavlink import mavutil
import math
import sys, tty, termios, threading, time, queue
import sim_globalFunc as gf
import sim_camera as hd
import requests

CAMERA_HOST = "http://192.168.0.142:5000"  # or the Jetson's IP if scripts run on different hosts
LOCAL_HOST = "http://127.0.0.1:5050"
yaw=0
lasttime,lasttime1=0,0

def get_desvio_from_simulation():
    try:
        r = requests.get(f"{LOCAL_HOST}/desvio_vermelho", timeout=1)
        r.raise_for_status()
        return r.json()  # dict or None
    except requests.RequestException as e:
        print(f"[Aviso] não consegui contactar o servidor de câmara: {e}")
        return None

def guidedLanding(waypoints):

    print("A iniciar aterragem no helipad...")
    updateDronePosition()
    # type_mask: usar apenas posição (ignora vel, accel, yaw, yaw_rate)


def orientaNS():
    global yaw
    print(f"current angle {yaw}")

    if yaw > 0 and yaw < 180.0:
        direction = -1
    else:
        direction = 1

    gf.drone.mav.command_long_send(
        gf.drone.target_system,
        gf.drone.target_component,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0, 
        0,  
        20,  # Yaw speed (ignored)
        direction,  # Direction: 1 for clockwise, -1 for counter-clockwise
        0, 0, 0, 0  
    )

def updateDronePosition():
    global lasttime,lasttime1,yaw
    moveD,moveE,moveN=0,0,0
    type_mask = 0b101111111000  
    
    yaw = (gf.current_position["hdg"])/100
    alt = gf.current_position['alt']

    
  
    while gf.current_position['alt'] > 0.1:   
    #while True:
        yaw = (gf.current_position["hdg"])/100

        if yaw > 1 and yaw < 359:
            orientaNS()
            print("Orientei dentro do updatePos")
        
        else:    
            current_time1=time.time()
            if (current_time1-lasttime1) > 0.033:     
                #desvios = hd.get_desvio_vermelho()   ##drone
                desvios = get_desvio_from_simulation()     # simulacao com pagina local host http "sim_camera.py"

                if desvios != None:                     
                    current_time=time.time()
                    if (current_time - lasttime) > 1:
                        print(f"Desvio norte: {desvios['esquerda']}px, Desvio sul {desvios['direita']}px ")
                        print(f"Desvio este: {desvios['cima']}px, Desvio oeste {desvios['baixo']}px ")
                        print (f"Move norte {moveN}, move este {moveE}, move down {moveD}\n")
                        print (f"Yaw: {yaw}\n")
                        lasttime=current_time

                    CorrectionMeters = 0.1

                    if (desvios['esquerda']*pxToMeters()) > (CorrectionMeters*alt):
                        moveN = .5
                    elif (desvios['direita'] *pxToMeters()) > (CorrectionMeters*alt):
                        moveN = -.5
                    else:
                        moveN = 0
                        
                    if (desvios['cima']*pxToMeters()) > (CorrectionMeters*alt):
                        moveE = .5
                    elif (desvios['baixo'] *pxToMeters()) > (CorrectionMeters*alt):
                        moveE = -.5 
                    else:
                        moveE = 0    

                    moveD = 0  ## apenas provisorio. so alinha, nao desce
                    
                    if (moveN != 0) or (moveE != 0):
                        #print (f"Fiz move N={moveN},Fiz move E={moveE},Fiz moveD={moveD}")
                        gf.drone.mav.set_position_target_local_ned_send(
                            0,                                              # time_boot_ms (pode ser 0)
                            gf.drone.target_system,
                            gf.drone.target_component,
                            mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED, 
                            type_mask,
                            moveN,  # north
                            moveE,  # east
                            moveD,  # down (moveD > 0 significa descer)      
                            0, 0, 0,    # vx, vy, vz (ignorados pelo type_mask)
                            0, 0, 0,    # afx, afy, afz (ignorados)
                            0, 0        # yaw, yaw_rate 
                        )
                lasttime1=current_time1    



def pxToMeters():

    alt = gf.current_position['alt']
    Vfov=24.4
    Hfov=31.1

    Vx = (alt * math.tan(math.radians(Vfov)))/hd.FRAME_HEIGHT
    Hx = (alt * math.tan(math.radians(Hfov)))/hd.FRAME_WIDTH

    #print(f"Altitude = {alt}, Vx={Vx}m, Hx={Hx}m")

    return Vx


    