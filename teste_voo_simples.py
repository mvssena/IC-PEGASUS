import time
from pymavlink import mavutil

def enviar_velocidade_corpo(master, vx, vy, vz):
    """
    Envia velocidade em MAV_FRAME_BODY_NED:
    vx: Frente (+) / Trás (-) em m/s
    vy: Direita (+) / Esquerda (-) em m/s
    vz: Baixo (+) / Cima (-) em m/s
    """
    master.mav.set_position_target_local_ned_send(
        0,                                      # time_boot_ms
        master.target_system,                   # target_system
        master.target_component,                # target_component
        mavutil.mavlink.MAV_FRAME_BODY_NED,     # Referencial do corpo do drone
        0b0000111111000111,                     # Máscara: ignora posição, usa apenas velocidades (vx, vy, vz)
        0, 0, 0,                                # x, y, z (ignorados)
        vx, vy, vz,                             # vx, vy, vz (m/s)
        0, 0, 0,                                # afx, afy, afz (ignorados)
        0, 0                                    # yaw, yaw_rate (ignorados)
    )

def obter_altitude(master):
    """Lê a altitude relativa atual (em metros) a partir da telemetria."""
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1.0)
    if msg:
        return msg.relative_alt / 1000.0
    return 0.0

def main():
    print("[1/6] Conectando ao ArduPilot SITL...")
    # Tenta conectar primeiro na porta UDP 14552; se não responder em 4s, tenta TCP 5762
    master = mavutil.mavlink_connection("udpin:0.0.0.0:14552")
    hb = master.wait_heartbeat(timeout=4)
    if not hb:
        print("Sem resposta na UDP 14552. Conectando na porta TCP 5762...")
        master.close()
        master = mavutil.mavlink_connection("tcp:127.0.0.1:5762")
        master.wait_heartbeat()

    print(f"[OK] Conectado! Sistema: {master.target_system}")

    # Solicita stream de telemetria a 10 Hz para ler a altitude em tempo real
    master.mav.request_data_stream_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1
    )

    # Desativa checagens extras de pré-armamento no simulador para garantir o voo imediato
    master.mav.param_set_send(
        master.target_system, master.target_component,
        b'ARMING_CHECK', 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32
    )
    time.sleep(1.0)

    # ---------------------------------------------------------
    # PASSO A: Mudar para modo GUIDED (4) e Armar
    # ---------------------------------------------------------
    print("[2/6] Mudando para modo GUIDED e armando motores...")
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        4  # 4 = GUIDED no ArduCopter
    )
    time.sleep(1.0)

    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )

    # Aguarda confirmar que armou
    print("Aguardando confirmação de armamento...")
    master.motors_armed_wait()
    print("[OK] Drone ARMADO! Girando hélices...")
    time.sleep(2.0)

    # ---------------------------------------------------------
    # PASSO B: Subir 1 metro (MAV_CMD_NAV_TAKEOFF)
    # ---------------------------------------------------------
    altura_alvo = 1.0
    print(f"[3/6] Subindo para {altura_alvo} metro(s)...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, altura_alvo
    )

    # Monitora a subida até chegar perto de 1.0m
    while True:
        alt = obter_altitude(master)
        print(f" -> Altitude atual: {alt:.2f} m / Meta: {altura_alvo:.2f} m")
        if alt >= (altura_alvo * 0.85):
            print("[OK] Altitude de 1 metro atingida!")
            break
        time.sleep(0.5)

    # Estabiliza no ar por 2 segundos antes de avançar
    time.sleep(2.0)

    # ---------------------------------------------------------
    # PASSO C: Andar 5 metros para a frente (vx = 1.0 m/s por 5s)
    # ---------------------------------------------------------
    velocidade_frente = 1.0  # 1 metro por segundo
    tempo_avanco = 5.0       # 5 segundos (1 m/s * 5s = 5 metros)
    print(f"[4/6] Avançando 5 metros para a frente (vx = {velocidade_frente} m/s por {tempo_avanco}s)...")

    inicio_avanco = time.time()
    while (time.time() - inicio_avanco) < tempo_avanco:
        enviar_velocidade_corpo(master, vx=velocidade_frente, vy=0.0, vz=0.0)
        alt = obter_altitude(master)
        tempo_decorrido = time.time() - inicio_avanco
        print(f" -> Avançando... Tempo: {tempo_decorrido:.1f}s | Distância aprox.: {tempo_decorrido * velocidade_frente:.1f}m | Alt: {alt:.2f}m")
        time.sleep(0.2)  # Reenvia a 5 Hz para manter o movimento contínuo

    # Freia o drone (manda velocidade zero por 2 segundos para parar no lugar)
    print("[5/6] Parando (Hover) após avançar 5 metros...")
    for _ in range(10):
        enviar_velocidade_corpo(master, vx=0.0, vy=0.0, vz=0.0)
        time.sleep(0.2)

    # ---------------------------------------------------------
    # PASSO D: Descer 1 metro até o chão (Modo LAND = 9)
    # ---------------------------------------------------------
    print("[6/6] Descendo 1 metro e pousando (Modo LAND)...")
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        9  # 9 = LAND no ArduCopter
    )

    while True:
        alt = obter_altitude(master)
        print(f" -> Descendo... Altitude atual: {alt:.2f} m")
        if alt <= 0.15:
            print("[OK] Drone tocou o solo!")
            break
        time.sleep(0.5)

    print("Aguardando desarmamento dos motores...")
    master.motors_disarmed_wait()
    print("[SUCESSO] Missão concluída: Subiu 1m -> Andou 5m -> Desceu 1m!")

if __name__ == "__main__":
    main()