from pymavlink import mavutil
import math
import sys, tty, termios, threading, time, queue
import sim_globalFunc as gf



def route(lat, lon, alt):
    #print("A mudar para GUIDED...")
    #set_mode('GUIDED')
    print(f"A enviar posição alvo: lat={lat}, lon={lon}, alt={alt}m...")

    # type_mask: usar apenas posição (ignora vel, accel, yaw, yaw_rate)
    type_mask = 0b0000_1111_1111_1000  # = 0x0DF8 = 3576 (Use Position)

    gf.drone.mav.set_position_target_global_int_send(
        0,                                              # time_boot_ms (pode ser 0)
        gf.drone.target_system,
        gf.drone.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # alt relativa ao home
        type_mask,
        int(lat * 1e7),
        int(lon * 1e7),
        alt,        # metros acima do home (por causa do frame escolhido)
        0, 0, 0,    # vx, vy, vz (ignorados pelo type_mask)
        0, 0, 0,    # afx, afy, afz (ignorados)
        0, 0        # yaw, yaw_rate (ignorados)
    )
    gf.wait_reached(lat, lon, alt)

def viagem():

    def executar():
        for lat, lon, alt in gf.waypoints:
            print(f"Indo para: lat={lat}, lon={lon}, alt={alt}m")
            route(lat, lon, alt)  # chamada direta -> só avança quando wait_reached terminar
        print("Rota completa!")

    threading.Thread(target=executar).start()
