from pymavlink import mavutil
import math
import sys, tty, termios, threading, time, queue
import sim_globalFunc as gf
import sim_camera as hd
import requests

CAMERA_HOST = "http://192.168.0.142:5000"  # or the Jetson's IP if scripts run on different hosts
LOCAL_HOST = "http://127.0.0.1:5000"
yaw=0
lasttime,lasttime1=0,0
DBM=0
DCM=0
DDM=0
DEM=0
moveD=0
moveN=0
moveE=0
vx=0
Vx=0
vy=0
Vy=0
vz=0
pitch=0
yaw=0
type_mask = 0b110111000111


http_session = requests.Session()

def get_desvio_from_simulation():
    try:
        r = http_session.get(f"{LOCAL_HOST}/desvio_vermelho", timeout=0.1)
        r.raise_for_status()
        return r.json()
    except requests.RequestException:
        return None


GRAPH_HOST = "http://127.0.0.1:5001"

def enviar_dados_grafico():
    try:
        requests.post(f"{GRAPH_HOST}/update", json={
            "DBM": DBM, "DCM": DCM, "DDM": DDM, "DEM": DEM,
            "moveN": moveN, "moveE": moveE, "moveD": moveD
        }, timeout=0.5)
    except requests.RequestException as e:
        print(f"[Aviso] não consegui contactar o servidor de gráfico: {e}")

####################################
#               gimbal              #
####################################


gimbal_stabilize_running = False

mavlink_lock = threading.Lock()

def wrapAngle180(angle):
    return (angle + 180) % 360 - 180
    #normaliza para [-180,180]

def setGimbalNadir(pitch=0, yaw=0, roll=0):
    """Envia um único comando para apontar o gimbal para baixo (nadir)."""

    pitch_delta = pitch - gf.current_position['pitch']
    yaw_delta = wrapAngle180(yaw - gf.current_position['hdg'] / 100)

    pitch1=pitch+pitch_delta
    yaw1=yaw+yaw_delta

    with mavlink_lock:
        gf.drone.mav.command_long_send(
            gf.drone.target_system,
            gf.drone.target_component,
            mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
            0,
            pitch, 
            yaw,
            float('nan'),
            float('nan'),
            0,
            0,
            0
        )

    print (f"Comando gimbal enviado: pitch={pitch-gf.current_position['pitch']}, yaw={yaw-gf.current_position['hdg']/100}")    


def gimbalStabilizeLoop(pitch=-90, yaw=0, roll=0, interval=1.0):
    """Corre em loop, reenviando o comando periodicamente para manter o
    gimbal estabilizado, mesmo que o mount 'perca' o alvo entretanto."""
    global gimbal_stabilize_running
    gimbal_stabilize_running = True
    while gimbal_stabilize_running:
        setGimbalNadir(pitch, yaw, roll)
        time.sleep(interval)


def startGimbalStabilize(pitch=-90, yaw=0, roll=0, interval=1.0):
    """Arranca a estabilização do gimbal numa thread separada, para não
    bloquear o resto do código (ex.: updateDronePosition)."""
    t = threading.Thread(
        target=gimbalStabilizeLoop,
        args=(pitch, yaw, roll, interval),
        daemon=True
    )
    t.start()
    print("Gimbal stabilize thread iniciada (nadir).")
    return t


def stopGimbalStabilize():
    """Para o loop de estabilização."""
    global gimbal_stabilize_running
    gimbal_stabilize_running = False

###############################################

def guidedLanding():

    print("A iniciar aterragem no helipad...")
    updateDronePosition()


    # type_mask: usar apenas posição (ignora vel, accel, yaw, yaw_rate)


def orientaNS():
    global yaw,moveD,moveN,moveE,vx,vy,vz
    print(f"current angle {yaw}")

    moveD=0
    moveN=0
    moveE=0
    vx=0
    vy=0
    vz=0
    move()

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
        90,  # Yaw speed (ignored)
        direction,  # Direction: 1 for clockwise, -1 for counter-clockwise
        0, 0, 0, 0  
    )

    
def updateDronePosition():
    global lasttime,lasttime1,yaw,DCM,DBM,DDM,DEM,moveN,moveE,moveD,vx,vy,vz,Vx,Vy
    
    setGimbalNadir(0, 0, 0) 

    Kp=0.007  ## james, james bond
    yaw = (gf.current_position["hdg"])/100
    alt = gf.current_position['alt']

    while gf.current_position['alt'] > 0.1:   
    #while True:

        yaw = (gf.current_position["hdg"])/100
        alt = gf.current_position['alt']

        # if yaw > 2 and yaw < 358:
        #     move
        #     orientaNS()
        #     print("Orientei dentro do updatePos")
        #     time.sleep(0.1)
        
        # else:    
        current_time1=time.time()
        if (current_time1-lasttime1) > 0.033:     
            #desvios = desvio do drone do centro da camera   ##drone
            desvios = get_desvio_from_simulation()     # simulacao com pagina local host http "sim_camera.py"

            if desvios != None:                     
                current_time=time.time()
                if (current_time - lasttime) > 1:
                    print(f"Desvio cima: {desvios['cima']}px, {DCM:0.3f}m; Desvio baixo {desvios['baixo']}px, {DBM:0.3f}m;")
                    print(f"Desvio direita: {desvios['direita']}px, {DDM:0.3f}m; Desvio esquerda {desvios['esquerda']}px, {DEM:0.3f}m;")
                    print (f"Move norte {moveN}, move este {moveE}, move down {moveD}")
                    print (f"Yaw: {yaw}, Alt: {alt}")
                    print(f"vx={vx}, vy={vy}\n")

                    lasttime=current_time

                CMeters = 60

                DBM=desvios['baixo']*pxToMeters()
                DCM=desvios['cima']*pxToMeters()
                DDM=desvios['direita']*pxToMeters()
                DEM=desvios['esquerda']*pxToMeters()

                ##  x -> positivo para a dir
                ## y -> positivo para baixo

                # if DDM > 0: x= DDM
                # elif DEM > 0: x= -DEM
                # else: x=0

                # if DBM > 0: y= -DBM
                # elif DCM > 0: y= DCM
                # else: y=0

                CMeters = 0.7

                if desvios['baixo'] >(CMeters*alt): 
                    #moveN = -0.1
                    vx=-Kp*desvios['baixo']
                elif desvios['cima'] >(CMeters*alt): 
                    #moveN = 0.1
                    vx=Kp*desvios['cima']
                else:
                    moveN = 0
                    
                if desvios['esquerda'] >(CMeters*alt): 
                    #moveE = -0.1
                    vy=-Kp*desvios['esquerda']

                elif desvios['direita'] >(CMeters*alt):
                    #moveE = 0.1
                    vy=Kp*desvios['direita']
                else:
                    moveE = 0    

                moveD = 0.5
                vz=1

                yawrad = math.radians(gf.current_position['hdg'] / 100)

                Vx = vx * math.cos(yawrad) - vy * math.sin(yawrad)
                Vy = vx * math.sin(yawrad) + vy * math.cos(yawrad)

                # if DBM >0: #(CorrectionMeters*alt):
                #     moveN = -DBM
                #     vx=Kp*moveN*DBM
                # elif DCM >0: #(CorrectionMeters*alt):
                #     moveN = DCM
                #     vx=Kp*moveN*DCM
                # else:
                #     moveN = 0
                    
                # if DEM >0: #(CorrectionMeters*alt):
                #     moveE = -DEM
                #     vy=Kp*moveE*DEM

                # elif DDM >0:# (CorrectionMeters*alt):
                #     moveE = DDM
                #     vy=Kp*moveE*DDM
                # else:
                #     moveE = 0    

                # moveD = 0.1  ## apenas provisorio. so alinha, nao desce

                #enviar_dados_grafico()
                
                if (abs(Vx) < 0.3) and (abs(Vy) < 0.3):
                    moveD = 0.5 
                else:
                    moveD = 0  # Mantém a altitude enquanto alinha

                move()

            lasttime1=current_time1    


def move():
    global moveN, moveE, moveD, vx, vy,vz
    with mavlink_lock:
        gf.drone.mav.set_position_target_local_ned_send(
            0,                                              # time_boot_ms (pode ser 0)
            gf.drone.target_system,
            gf.drone.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,#MAV_FRAME_LOCAL_OFFSET_NED, 
            type_mask,
            moveN,  # north
            moveE,  # east
            moveD,  # down (moveD > 0 significa descer)      
            Vx, Vy, vz,    # vx, vy, vz 
            0, 0, 0,    # afx, afy, afz (ignorados pelo type_mask)
            0, 0        # yaw (0 = norte), yaw_rate (yaw_rate ignorado pelo type_mask)
        )
    
def pxToMeters():

    alt = gf.current_position['alt']
    Vfov=24.4
    Hfov=31.1

    Vx = (alt * math.tan(math.radians(Vfov)))/204              #<- tamanho da cam do simuldor#/hd.FRAME_HEIGHT
    Hx = (alt * math.tan(math.radians(Hfov)))/153                    #####hd.FRAME_WIDTH

    #print(f"Altitude = {alt}, Vx={Vx}m, Hx={Hx}m")

    return Vx


    