from pymavlink import mavutil
import math
import sys, tty, termios, threading, time, queue
import Jet_globalFunc as gf
import Jet_cameraHeliDetect as hd


def guidedLanding(waypoints):

    print("A iniciar aterragem no helipad...")
    updateDronePosition()

    # type_mask: usar apenas posição (ignora vel, accel, yaw, yaw_rate)


def updateDronePosition(moveN, moveE, moveD):
    type_mask = 0b110111111000  # = 0x0DF8 = 3576 (Use Position)

    while moveD <= 0:    

        desvios = hd.get_desvio_vermelho

        print(f"Desvio norte: {desvios['cima']}px ou {desvios['baixo']}px ")
        print(f"Desvio este: {desvios['esquerda']}px ou {desvios['direita']}px ")

        print(f"Desvio vermelho: {hd.desvio_vermelho}")

        # gf.drone.mav.set_position_target_local_ned(
        #     0,                                              # time_boot_ms (pode ser 0)
        #     gf.drone.target_system,
        #     gf.drone.target_component,
        #     mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED, 
        #     type_mask,
        #     moveN,  # north
        #     moveE,  # east
        #     moveD,  # down (moveD > 0 significa descer)      
        #     0, 0, 0,    # vx, vy, vz (ignorados pelo type_mask)
        #     0, 0, 0,    # afx, afy, afz (ignorados)
        #     0, 0        # yaw, yaw_rate (ignorados)
        # )

    #gf.wait_reached(lat, lon, alt)



