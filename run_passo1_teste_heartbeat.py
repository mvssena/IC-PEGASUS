import time
import logging
from mavlink_interface import MAVLinkInterface

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s.%(msecs)03d [%(levelname)s] [%(threadName)s] %(message)s"
)
logger = logging.getLogger("TestePasso1")

def main():
    mav = MAVLinkInterface("udpin:127.0.0.1:14550")
    mav.connect()
    
    logger.info("Aguardando 3 segundos para estabilização da conexão...")
    time.sleep(3.0)

    logger.warning("=================================================================")
    logger.warning("INICIANDO BLOQUEIO ARTIFICIAL DO LAÇO PRINCIPAL POR 10 SEGUNDOS")
    logger.warning("(Simulação da carga de firmware / modelo neural da IMX500)")
    logger.warning("=================================================================")
    
    # Bloqueio artificial da MainThread
    time.sleep(10.0)

    logger.info("=================================================================")
    logger.info("LAÇO PRINCIPAL RETOMOU A EXECUÇÃO!")
    logger.info(f"Status do Piloto: Conectado. Modo: {mav.current_mode}. Armado: {mav.is_armed}")
    logger.info("=================================================================")
    mav.close()

if __name__ == "__main__":
    main()