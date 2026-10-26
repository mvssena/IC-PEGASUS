import time
import logging

logger = logging.getLogger("MissionFSM")

class State:
    """Classe base para todos os estados da missão."""
    def __init__(self, name: str, timeout_sec: float):
        self.name = name
        self.timeout_sec = timeout_sec
        self.start_time = None

    def on_enter(self, context):
        self.start_time = time.time()
        logger.info(f"=== [FSM] ENTRANDO NO ESTADO: {self.name} (Timeout: {self.timeout_sec}s) ===")

    def execute(self, context) -> str:
        # Timeout por estado: impede o VANT de ficar retido indefinidamente no ar
        tempo_decorrido = time.time() - self.start_time
        if tempo_decorrido > self.timeout_sec:
            logger.error(f"[TIMEOUT EXCEDIDO] Estado {self.name} atingiu {self.timeout_sec}s! Forçando POUSAR.")
            return "POUSAR"
        return None

    def on_exit(self, context):
        duracao = time.time() - self.start_time
        logger.info(f"=== [FSM] SAINDO DO ESTADO: {self.name} (Ativo por {duracao:.2f}s) ===")


class TakeoffState(State):
    """Estado 1: DECOLAR
    - Condição de saída: Altitude atual >= 90% da altitude alvo.
    - Timeout: 20 segundos.
    """
    def __init__(self, target_alt=2.5, timeout_sec=20.0):
        super().__init__("DECOLAR", timeout_sec)
        self.target_alt = target_alt

    def on_enter(self, context):
        super().on_enter(context)
        context.mav.set_mode("GUIDED")
        if not context.mav.arm():
            raise RuntimeError("Recusa de pré-armamento de GPS! Missão cancelada.")
        
        # Comando MAV_CMD_NAV_TAKEOFF (156)
        context.mav.master.mav.command_long_send(
            context.mav.master.target_system,
            context.mav.master.target_component,
            156, 0, 0, 0, 0, 0, 0, 0, self.target_alt
        )
        logger.info(f"[DECOLAR] Comando de subida enviado para {self.target_alt}m.")

    def execute(self, context) -> str:
        timeout_res = super().execute(context)
        if timeout_res:
            return timeout_res

        alt = context.mav.current_altitude
        logger.info(f"[DECOLAR] Altitude: {alt:.2f}m / Alvo: {self.target_alt:.2f}m")
        if alt >= self.target_alt * 0.90:
            logger.info("[DECOLAR] Condição de saída atingida: Altitude de cruzeiro alcançada.")
            return "DESLOCAR"
            
        time.sleep(0.5)
        return None


class TransitState(State):
    """Estado 2: DESLOCAR (Varredura e busca de alvos)
    - Condição de saída: Deteção estável com confiança >= 0.70.
    - Timeout: 30 segundos.
    """
    def __init__(self, timeout_sec=30.0):
        super().__init__("DESLOCAR", timeout_sec)

    def execute(self, context) -> str:
        timeout_res = super().execute(context)
        if timeout_res:
            return timeout_res

        det = context.perception.get_latest_detection()
        if det and det.get("estavel", False) and det.get("confidence", 0.0) >= 0.70:
            logger.info(f"[DESLOCAR] Alvo '{det.get('target_id')}' confirmado estável. Parando avanço.")
            context.mav.send_body_velocity(0.0, 0.0, 0.0)
            return "APROXIMAR_ALVO"

        # Avanço contínuo para a frente no referencial do corpo (vx = 0.5 m/s)
        context.mav.send_body_velocity(vx=0.5, vy=0.0, vz=0.0)
        time.sleep(0.2)
        return None


class ApproachTargetState(State):
    """Estado 3: APROXIMAR DE ALVO
    - Condição de saída: Alvo centrado (|err_x| e |err_y| < 0.10) e altitude <= 0.8m.
    - Timeout: 25 segundos.
    """
    def __init__(self, timeout_sec=25.0):
        super().__init__("APROXIMAR_ALVO", timeout_sec)
        self.last_seen = time.time()

    def on_enter(self, context):
        super().on_enter(context)
        self.last_seen = time.time()

    def execute(self, context) -> str:
        timeout_res = super().execute(context)
        if timeout_res:
            return timeout_res

        det = context.perception.get_latest_detection()
        agora = time.time()

        if not det or not det.get("estavel", False):
            if agora - self.last_seen > 5.0:
                logger.warning("[APROXIMAR_ALVO] Alvo perdido por mais de 5s! Regredindo para DESLOCAR.")
                return "DESLOCAR"
            context.mav.send_body_velocity(0.0, 0.0, 0.0)
            time.sleep(0.1)
            return None

        self.last_seen = agora
        err_x = det.get("err_x", 0.0)
        err_y = det.get("err_y", 0.0)

        # Ajuste de velocidade no referencial MAV_FRAME_BODY_NED
        vx = max(min(err_x * 0.7, 0.35), -0.35)
        vy = max(min(err_y * 0.7, 0.35), -0.35)
        vz = 0.15  # Descida suave

        context.mav.send_body_velocity(vx, vy, vz)
        logger.info(f"[APROXIMAR] BODY_NED: vx={vx:.2f} m/s, vy={vy:.2f} m/s | Alt: {context.mav.current_altitude:.2f}m")

        # Condição de saída documentada
        if abs(err_x) < 0.10 and abs(err_y) < 0.10 and context.mav.current_altitude <= 0.8:
            logger.info("[APROXIMAR_ALVO] Condição de saída atingida: Alvo centrado a baixa altitude.")
            return "POUSAR"

        time.sleep(0.1)
        return None


class LandState(State):
    """Estado 4: POUSAR
    - Condição de saída: Motores desarmados no solo (is_armed == False).
    - Timeout: 20 segundos.
    """
    def __init__(self, timeout_sec=20.0):
        super().__init__("POUSAR", timeout_sec)

    def on_enter(self, context):
        super().on_enter(context)
        context.mav.set_mode("LAND")
        logger.info("[POUSAR] Modo LAND acionado.")

    def execute(self, context) -> str:
        if not context.mav.is_armed:
            logger.info("[POUSAR] Motores desarmados no solo. Missão concluída!")
            return "CONCLUIDO"

        if time.time() - self.start_time > self.timeout_sec:
            logger.error("[POUSAR] Timeout sem confirmação de desarmamento")
            return "FALHA_POUSO"

        time.sleep(0.5)
        return None
