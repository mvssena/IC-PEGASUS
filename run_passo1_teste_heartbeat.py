import argparse
import logging
import time
from mavlink_interface import MAVLinkInterface


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--conexao', default='udpin:0.0.0.0:14552')
    parser.add_argument('--log', default='heartbeat_pt18796.log')
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO,
                        format='%(asctime)s [%(threadName)s] %(levelname)s %(name)s: %(message)s',
                        handlers=[logging.StreamHandler(), logging.FileHandler(args.log, encoding='utf-8', mode='w')])
    log = logging.getLogger('TESTE_HEARTBEAT')
    mav = MAVLinkInterface(args.conexao)
    try:
        mav.connect()
        log.info('GPS pronto: %s', mav.verify_gps_readiness())
        log.info('Modo inicial: %s; armado: %s', mav.current_mode, mav.is_armed)
        log.info('INICIO BLOQUEIO DO LACO PRINCIPAL POR 10 SEGUNDOS')
        inicio = time.monotonic()
        time.sleep(10)
        log.info('FIM BLOQUEIO, duração %.2f s', time.monotonic() - inicio)
        log.info('Modo final: %s; armado: %s', mav.current_mode, mav.is_armed)
        log.info('Verifique HEARTBEAT_TX no intervalo e compare com log do autopiloto; envio não comprova recepção nem ausência de failsafe.')
    finally:
        mav.close()


if __name__ == '__main__':
    main()
