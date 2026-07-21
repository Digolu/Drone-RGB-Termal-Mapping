from pymavlink import mavutil
import time

TAKEOFF_ALT = 5.0

drone = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)

print("À espera de heartbeat...")
drone.wait_heartbeat()
print(f"Heartbeat recebido! (System {drone.target_system} Component {drone.target_component})")

# 1. Mudar para GUIDED
print("A mudar para modo GUIDED...")
mode_id = drone.mode_mapping()['GUIDED']
drone.mav.set_mode_send(
    drone.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id
)

while True:
    drone.wait_heartbeat()
    if drone.flightmode == 'GUIDED':
        print("✓ Modo GUIDED ativo!")
        break

# 2. Armar
print("A armar motores...")
drone.mav.command_long_send(
    drone.target_system, drone.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0, 1, 0, 0, 0, 0, 0, 0
)

drone.motors_armed_wait()
print("✓ Drone ARMADO!")

# 3. Pedir Descolagem
print(f"A descolar para {TAKEOFF_ALT}m...")
drone.mav.command_long_send(
    drone.target_system, drone.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0, 0, 0, 0, 0, 0, 0, TAKEOFF_ALT
)

# 4. MONITORIZAR A SUBIDA (Impede o script de fechar)
print("A monitorizar altitude...")
while True:
    # Lê mensagens de altitude relativa ao solo
    msg = drone.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
    if msg:
        # relative_alt vem em milímetros
        alt_atual = msg.relative_alt / 1000.0
        print(f"Altitude atual: {alt_atual:.2f} m", end='\r')
        
        # Atingiu 95% da altitude desejada
        if alt_atual >= TAKEOFF_ALT * 0.95:
            print(f"\n✓ Altitude de {TAKEOFF_ALT}m atingida!")
            break
            
    time.sleep(0.2)

# Mantém o script a correr para o drone não entrar em Failsafe
print("\nDrone em pairar (hover). Pressiona Ctrl+C para sair.")
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("\nA terminar...")