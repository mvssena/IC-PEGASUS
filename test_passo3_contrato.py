# test_passo3_contrato.py
import time
import logging
from perception_consumer import PerceptionConsumer

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")

if __name__ == "__main__":
    consumer = PerceptionConsumer(timeout_ms=500, min_confidence=0.65)

    print("\n--- TESTANDO REGRA 2: CONFIANÇA BAIXA (< 0.65) ---")
    consumer.feed_contract_payload({"confidence": 0.50, "estavel": True, "target_id": "ALVO_RUIDO"})
    assert consumer.get_latest_detection() is None, "Falha: Pacote com confiança baixa não deveria ter sido aceite!"
    print("REGRA 2 OK: Pacote descartado.")

    print("\n--- TESTANDO REGRA 3: ESTÁVEL = FALSE ---")
    consumer.feed_contract_payload({"confidence": 0.85, "estavel": False, "target_id": "ALVO_INSTAVEL"})
    det = consumer.get_latest_detection()
    assert det is not None and det["estavel"] is False, "Falha: Deveria reter pacote com estavel=False"
    print("REGRA 3 OK: Pacote aceite sinalizando instabilidade.")

    print("\n--- TESTANDO REGRA 1: TIMEOUT APÓS N MS (500 ms) ---")
    consumer.feed_contract_payload({"confidence": 0.90, "estavel": True, "target_id": "ALVO_VALIDO"})
    time.sleep(0.6)  # Espera 600 ms para forçar a expiração
    assert consumer.get_latest_detection() is None, "Falha: O dado deveria ter expirado após 500 ms!"
    print("REGRA 1 OK: Dado expirou e devolveu None com sucesso.\n")