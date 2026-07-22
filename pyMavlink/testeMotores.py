from pymavlink import mavutil
import time

# Conectar ao drone via USB
drone = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)

print("À espera de heartbeat...")
drone.wait_heartbeat()
print(f"Ligado ao sistema {drone.target_system}!")

def testar_motor(motor_seq, percentagem=15, duracao_seg=3):
    """
    motor_seq: Número do motor (1, 2, 3, 4...)
    percentagem: Velocidade do motor (0 a 100%)
    duracao_seg: Tempo de rotação em segundos
    """
    print(f"A testar Motor {motor_seq} a {percentagem}% de força por {duracao_seg}s...")
    
    drone.mav.command_long_send(
        drone.target_system,
        drone.target_component,
        mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,
        0,
        motor_seq,      # Param 1: Número do motor (1 a 4/6/8)
        0,              # Param 2: Tipo de aceleração (0 = Aceleração em %)
        percentagem,    # Param 3: Valor da aceleração (ex: 15 = 15%)
        duracao_seg,    # Param 4: Duração da rotação
        0,              # Param 5: Contagem de motores adicionais (0 = só este)
        0,              # Param 6: Reservado (0 para ArduPilot)
        0
    )
    
    # Aguarda o ACK da placa
    ack = drone.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    if ack and ack.command == mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST:
        if ack.result == 0:
            print(f"Motor {motor_seq} a rodar!")
        else:
            print(f"Rejeitado pelo drone (Código: {ack.result}). Verifica se o Safety Switch está premido.")

# Executa o teste nos 4 motores em sequência
try:
    testar_motor(motor_seq=1, percentagem=20, duracao_seg=2)

    testar_motor(motor_seq=2, percentagem=20, duracao_seg=2)

    testar_motor(motor_seq=3, percentagem=20, duracao_seg=2)

    testar_motor(motor_seq=4, percentagem=20, duracao_seg=2)

finally:
    print("\nTeste concluído.")
