from pymavlink import mavutil
import time

drone = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)

print("À espera de heartbeat...")
drone.wait_heartbeat()
print(f"Conectado ao Sistema {drone.target_system}!")

# 1. Mudar para STABILIZE (Modo manual simples - não precisa de GPS)
print("\n1. A mudar para modo STABILIZE...")
mode_id = drone.mode_mapping()['STABILIZE']
drone.mav.set_mode_send(
    drone.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id
)
time.sleep(1)

# 2. Armar motores
print("2. A armar motores...")
drone.mav.command_long_send(
    drone.target_system, drone.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0, 1, 0, 0, 0, 0, 0, 0
)

drone.motors_armed_wait()
print("✓ DRONE ARMADO!")

# 3. Injetar Aceleração (Override do Canal 3 - Throttle)
print("\n3. A AUMENTAR VELOCIDADE DOS MOTORES (4 Segundos)...")
start_time = time.time()

while time.time() - start_time < 4:
    # Canal 1: Roll (1500 = Centro)
    # Canal 2: Pitch (1500 = Centro)
    # Canal 3: Throttle (1350 = ~35% de acelerador - Valores entre 1000 e 2000)
    # Canal 4: Yaw (1500 = Centro)
    drone.mav.rc_channels_override_send(
        drone.target_system,
        drone.target_component,
        1500, 1500, 1750 , 1500, 0, 0, 0, 0
    )
    time.sleep(0.05)

# 4. Baixar acelerador ao mínimo e Desarmar
print("\n4. A reduzir acelerador e a desarmar...")
drone.mav.rc_channels_override_send(
    drone.target_system, drone.target_component,
    1500, 1500, 1000, 1500, 0, 0, 0, 0
)
time.sleep(0.5)

drone.mav.command_long_send(
    drone.target_system, drone.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0, 0, 0, 0, 0, 0, 0, 0
)
print("✓ Drone Desarmado com sucesso!")