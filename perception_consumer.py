import time
import threading
import logging

logger = logging.getLogger("PerceptionConsumer")

class PerceptionConsumer:
    def __init__(self, timeout_ms: int = 500, min_confidence: float = 0.65):
        self.timeout_ms = timeout_ms
        self.min_confidence = min_confidence
        self._lock = threading.Lock()
        self._latest_payload = None
        self._last_arrival_time = 0.0

    def feed_contract_payload(self, payload: dict):
        arrival_time = time.time()
        conf = payload.get("confidence", 0.0)
        is_stable = payload.get("estavel", False)

        # Situação 2: Confiança baixa -> ignora o dado
        if conf < self.min_confidence:
            logger.warning(f"[CONTRATO] Confiança ({conf:.2f} < {self.min_confidence}). Dado descartado!")
            return

        with self._lock:
            # Situação 3: Não estável -> regista alerta mas não usa para aproximação agressiva
            if not is_stable:
                logger.info("[CONTRATO] Deteção com estavel: false. Aguardando votação temporal.")
            self._latest_payload = payload
            self._last_arrival_time = arrival_time

    def get_latest_detection(self) -> dict:
        with self._lock:
            if self._latest_payload is None:
                return None
            elapsed_ms = (time.time() - self._last_arrival_time) * 1000.0
            
            # Situação 1: Nenhuma deteção válida há mais de N ms
            if elapsed_ms > self.timeout_ms:
                logger.warning(f"[CONTRATO] Deteção expirada ({elapsed_ms:.1f}ms > {self.timeout_ms}ms). Retornando vazio.")
                self._latest_payload = None
                return None
            return self._latest_payload