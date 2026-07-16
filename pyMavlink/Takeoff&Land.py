from pymavlink import mavutil
import sys, tty, termios, threading, time

#CONNECTION = '/dev/ttyUSB0'
CONNECTION = '/dev/ttyACM0'
BAUD = 57600
TAKEOFF_ALT = 5  # metros

drone = mavutil.mavlink_connection(CONNECTION, baud=BAUD)
print("À espera de heartbeat...")
drone.wait_heartbeat()
print(f"Ligado ao sistema {drone.target_system}, componente {drone.target_component}")


def set_mode(mode_name):
    mode_id = drone.mode_mapping()[mode_name]
    drone.mav.set_mode_send(
        drone.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )
    time.sleep(1)


def arm():
    drone.mav.command_long_send(
        drone.target_system, drone.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0
    )
    ack = drone.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
    print("Arm ACK:", ack)


def disarm():
    drone.mav.command_long_send(
        drone.target_system, drone.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 0, 0, 0, 0, 0, 0
    )
    ack = drone.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
    print("Disarm ACK:", ack)


def takeoff():
    print("A entrar em modo NOT LOITER...")
    set_mode('STABILIZE')
    print("A armar...")
    arm()
    print(f"A descolar para {TAKEOFF_ALT}m...")
    drone.mav.command_long_send(
        drone.target_system, drone.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0, 0, 0, TAKEOFF_ALT
    )
    ack = drone.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
    print("Takeoff ACK:", ack)


def land():
    print("A pousar...")
    set_mode('LAND')


def get_key():
    """Lê uma tecla sem precisar de Enter."""
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
    keyboard_loop()