import time
import threading
import logging
from pymavlink import mavutil

logger = logging.getLogger("MAVLinkInterface")

class MAVLinkInterface:
    def __init__(self, connection_string: str = "udpin:0.0.0.0:14552"):
        self.connection_string = connection_string
        self.master = None
        self.running = False

        # Telemetria básica
        self.gps_fix = 0
        self.satellites = 0
        self.hdop = 99.9
        self.gps_received_at = None
        self.current_altitude = 0.0
        self.is_armed = False
        self.current_mode = "UNKNOWN"

        self._hb_thread = None
        self._rx_thread = None

    def connect(self):
        logger.info(f"Tentando conectar via MAVLink em {self.connection_string}...")
        self.master = mavutil.mavlink_connection(self.connection_string)
        logger.info("Aguardando Heartbeat na porta 14552 (limite de 4s)...")
        
        # Tenta receber o Heartbeat por 4 segundos na porta 14552
        hb = self.master.wait_heartbeat(timeout=4)
        
        # Se você esqueceu de ligar o Ctrl+F no Mission Planner, conecta direto no SITL (porta TCP 5762)
        if not hb:
            logger.warning("Nenhum sinal na 14552! Conectando direto na porta nativa do SITL (tcp:127.0.0.1:5762)...")
            self.master.close()
            self.connection_string = "tcp:127.0.0.1:5762"
            self.master = mavutil.mavlink_connection(self.connection_string)
            self.master.wait_heartbeat()

        logger.info(f"Heartbeat recebido! Sistema: {self.master.target_system}, Componente: {self.master.target_component}")

        # Pede ao controlador de voo todos os dados de telemetria a 5 Hz
        self.master.mav.request_data_stream_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            5, 1
        )

        self.running = True
        self._hb_thread = threading.Thread(target=self._heartbeat_loop, daemon=True)
        self._rx_thread = threading.Thread(target=self._telemetry_loop, daemon=True)
        self._hb_thread.start()
        self._rx_thread.start()
        time.sleep(1.5)

    def _heartbeat_loop(self):
        while self.running:
            try:
                self.master.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0, 0, 0
                )
                logger.info("[HEARTBEAT_TX] enviado monotonic=%.3f", time.monotonic())
            except Exception as e:
                logger.error(f"Erro ao enviar Heartbeat: {e}")
            time.sleep(1.0)

    def _telemetry_loop(self):
        while self.running:
            try:
                msg = self.master.recv_match(blocking=True, timeout=0.5)
                if not msg:
                    continue
                msg_type = msg.get_type()

                if msg_type == "GPS_RAW_INT":
                    self.gps_fix = msg.fix_type
                    self.satellites = msg.satellites_visible
                    # eph = HDOP * 100; 65535 indica valor desconhecido.
                    raw_eph = msg.eph
                    self.hdop = raw_eph / 100.0 if raw_eph != 65535 else float('inf')
                    self.gps_received_at = time.monotonic()

                elif msg_type == "STATUSTEXT":
                    logger.warning("[AUTOPILOT] %s", msg.text)

                elif msg_type == "GLOBAL_POSITION_INT":
                    self.current_altitude = msg.relative_alt / 1000.0

                elif msg_type == "VFR_HUD":
                    if self.current_altitude == 0.0:
                        self.current_altitude = float(msg.alt)

                elif msg_type == "HEARTBEAT":
                    if msg.type != mavutil.mavlink.MAV_TYPE_GCS:
                        self.is_armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                        modes = self.master.mode_mapping()
                        if modes:
                            reverse_modes = {v: k for k, v in modes.items()}
                            self.current_mode = reverse_modes.get(msg.custom_mode, "UNKNOWN")
            except Exception:
                logger.exception("Erro recebendo telemetria")
                time.sleep(0.1)

    def verify_gps_readiness(self) -> bool:
        """Exige telemetria recente, fix 3D, >=10 satélites e HDOP <=1,40."""
        fresh = self.gps_received_at is not None and time.monotonic() - self.gps_received_at <= 2.0
        if fresh and self.gps_fix >= 3 and 10 <= self.satellites < 255 and self.hdop <= 1.40:
            logger.info(f"[GPS OK] Fix={self.gps_fix}, Satélites={self.satellites}, HDOP={self.hdop:.2f}")
            return True
        
        logger.warning("[GPS RECUSADO] recente=%s fix=%s satélites=%s HDOP=%.2f", fresh, self.gps_fix, self.satellites, self.hdop)
        return False

    def set_mode(self, mode_name: str):
        modes = self.master.mode_mapping()
        if modes and mode_name in modes:
            mode_id = modes[mode_name]
            self.master.set_mode(mode_id)
            logger.info(f"Comando enviado para alterar modo para: {mode_name}")
            time.sleep(1.0)

    def arm(self) -> bool:
        if not self.verify_gps_readiness():
            return False

        logger.info("Enviando comando normal de armamento...")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,      # 1 = Armar
            0,      # Não força armamento
            0, 0, 0, 0, 0
        )

        # Aguarda confirmação de armamento por até 5 segundos
        for _ in range(10):
            if self.is_armed:
                logger.info("Motores ARMADOS confirmados pela telemetria!")
                return True
            time.sleep(0.5)

        logger.error("Armamento não confirmado pela telemetria")
        return False

    def send_body_velocity(self, vx: float, vy: float, vz: float):
        """Envia velocidades no referencial do corpo (MAV_FRAME_BODY_NED)."""
        type_mask = 0b0000111111000111  # Usa apenas vx, vy, vz
        self.master.mav.set_position_target_local_ned_send(
            0,
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            type_mask,
            0, 0, 0,
            vx, vy, vz,
            0, 0, 0,
            0, 0
        )

    def close(self):
        self.running = False
        if self.master:
            self.master.close()
        logger.info("Conexão MAVLink encerrada.")
