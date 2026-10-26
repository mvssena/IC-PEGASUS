# test_passo2_isolado.py
import logging
from mission_fsm import ApproachTargetState

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")

class MockMAV:
    def __init__(self):
        self.current_altitude = 0.75  # Simula drone já baixo
        self.comandos_enviados = []

    def send_body_velocity(self, vx, vy, vz):
        self.comandos_enviados.append((vx, vy, vz))

class MockPerception:
    def get_latest_detection(self):
        # Simula alvo perfeitamente centralizado
        return {"err_x": 0.02, "err_y": -0.03, "confidence": 0.95, "estavel": True}

class MockContext:
    def __init__(self):
        self.mav = MockMAV()
        self.perception = MockPerception()

if __name__ == "__main__":
    print("\n--- INICIANDO TESTE ISOLADO DO ESTADO 'APROXIMAR_ALVO' ---")
    ctx = MockContext()
    estado = ApproachTargetState(timeout_sec=5.0)
    estado.on_enter(ctx)

    proximo_estado = estado.execute(ctx)
    estado.on_exit(ctx)

    print(f"Resultado da transição isolada: {proximo_estado}")
    assert proximo_estado == "POUSAR", f"Erro: Esperava POUSAR, recebeu {proximo_estado}"
    print("PROVA CONCLUÍDA COM SUCESSO: O estado é 100% testável isoladamente!\n")