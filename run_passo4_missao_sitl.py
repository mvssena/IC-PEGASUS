import time
import threading
import logging
import sys
from mavlink_interface import MAVLinkInterface
from perception_consumer import PerceptionConsumer
from mission_fsm import TakeoffState, TransitState, ApproachTargetState, LandState

# Configuração para imprimir na tela E gravar num ficheiro ao mesmo tempo
logger = logging.getLogger("MissaoPrincipal")
logger.setLevel(logging.INFO)
formatter = logging.Formatter("%(asctime)s [%(levelname)s] %(message)s")
console_handler = logging.StreamHandler(sys.stdout)
console_handler.setFormatter(formatter)
file_handler = logging.FileHandler("missao_sitl_execucao.log", mode="w")
file_handler.setFormatter(formatter)
logger.addHandler(console_handler)
logger.addHandler(file_handler)

class Context:
    def __init__(self, mav, perception):
        self.mav = mav
        self.perception = perception

def replay_percepcao_colega(consumer: PerceptionConsumer):
    """Simula o envio de dados da câmera IMX500"""
    logger.info("[REPLAY] Aguardando 12 segundos para a decolagem...")
    time.sleep(12.0)
    
    logger.info("[REPLAY] Drone voando. Injetando primeira detecção instável...")
    consumer.feed_contract_payload({
        "timestamp_ms": int(time.time() * 1000), "target_id": "ALVO_01",
        "bbox": [100, 100, 200, 200], "err_x": 0.35, "err_y": -0.25,
        "confidence": 0.88, "estavel": False
    })
    time.sleep(1.5)

    logger.info("[REPLAY] Alvo estabilizado! Iniciando aproximação...")
    for i in range(15):
        fator = 1.0 - (i / 15.0)
        consumer.feed_contract_payload({
            "timestamp_ms": int(time.time() * 1000), "target_id": "ALVO_01",
            "bbox": [120, 110, 220, 210], "err_x": 0.35 * fator, "err_y": -0.25 * fator,
            "confidence": 0.95, "estavel": True
        })
        time.sleep(0.15)

def main():
    logger.info("Iniciando Missão Passo 4...")
    mav = MAVLinkInterface("udpin:0.0.0.0:14552")
    mav.connect()
    perception = PerceptionConsumer(timeout_ms=500, min_confidence=0.65)

    threading.Thread(target=replay_percepcao_colega, args=(perception,), daemon=True).start()

    # --- LAÇO DE ESPERA ATIVA DE TELEMETRIA ---
    logger.warning("Aguardando sincronização da telemetria MAVLink e satélites...")
    while not mav.verify_gps_readiness():
        logger.info("GPS ainda não validado pela telemetria. A aguardar 2 segundos...")
        time.sleep(2.0)
    logger.info("Sinal GPS validado com sucesso! A autorizar início da máquina de estados.")
    # ------------------------------------------

    ctx = Context(mav, perception)
    # Tempos limite alargados para evitar falhas na simulação
    states = {
        "DECOLAR": TakeoffState(target_alt=2.5, timeout_sec=40.0),
        "DESLOCAR": TransitState(timeout_sec=45.0),
        "APROXIMAR_ALVO": ApproachTargetState(timeout_sec=40.0),
        "POUSAR": LandState(timeout_sec=30.0)
    }

    current = "DECOLAR"
    state_obj = states[current]
    state_obj.on_enter(ctx)

    try:
        while current != "CONCLUIDO":
            next_state = state_obj.execute(ctx)
            if next_state:
                state_obj.on_exit(ctx)
                if next_state == "CONCLUIDO":
                    break
                current = next_state
                state_obj = states[current]
                state_obj.on_enter(ctx)
            time.sleep(0.05)
        logger.info("[SUCESSO] Missão concluída na íntegra em ambiente simulado!")
    finally:
        mav.close()

if __name__ == "__main__":
    main()