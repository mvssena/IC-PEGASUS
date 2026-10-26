# Missão autônoma em ArduPilot SITL e Gazebo

Códigos de simulação de um VANT Iris com comunicação MAVLink, máquina de estados e percepção simulada. Organizado a partir dos arquivos do projeto e da [conversa de referência no Gemini](https://share.gemini.google/qXV2ugbtyjdM).

## Conteúdo

| Arquivo | Uso |
| --- | --- |
| `teste_voo_simples.py` | Decolagem de 1 m, avanço comandado por 5 s a 1 m/s e pouso. A distância de 5 m é estimada pelo tempo, não medida. |
| `run_passo4_missao_sitl.py` | Missão com decolagem, deslocamento, aproximação e pouso. |
| `mavlink_interface.py` | Conexão e telemetria MAVLink, modos e comandos de voo. |
| `mission_fsm.py` | Estados e transições da missão. |
| `perception_consumer.py` | Recebe detecções e descarta dados expirados ou de baixa confiança. |
| `run_passo1_teste_heartbeat.py` | Experimento de heartbeat com bloqueio da thread principal. |
| `test_passo2_isolado.py` | Teste isolado da aproximação. |
| `test_passo3_contrato.py` | Teste da confiança, estabilidade e expiração de detecções. |

A detecção da missão principal é gerada por `replay_percepcao_colega`; este conjunto ainda não integra a câmera IMX500 em tempo real.

## Ambiente e execução

O roteiro da conversa utiliza Ubuntu 20.04 no WSL2, Gazebo 11 e ArduPilot SITL com Iris. Instale `pymavlink` com `python3 -m pip install -r requirements.txt` em um ambiente virtual e configure ArduPilot e o plugin do Gazebo separadamente.

Em terminais Ubuntu distintos:

```bash
# Terminal 1: cenário do Gazebo
gazebo --verbose ~/ardupilot_gazebo/worlds/iris_arducopter_runway.world
```

```bash
# Terminal 2: SITL e saída MAVLink para a porta utilizada pela missão
cd ~/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter -f gazebo-iris --out=udp:127.0.0.1:14552
```

Em um terceiro terminal, na pasta deste repositório:

```bash
python3 run_passo4_missao_sitl.py
# ou, para o experimento independente:
python3 teste_voo_simples.py
```

O experimento de heartbeat usa `udpin:127.0.0.1:14550`, diferente da missão principal. Confira as portas, especialmente se Python for executado no Windows e SITL no WSL2.

## Verificação local

```bash
python3 -m py_compile *.py
python3 test_passo2_isolado.py
python3 test_passo3_contrato.py
```

Os dois testes isolados passaram na preparação deste repositório. A simulação completa requer Gazebo e SITL; não foi executada aqui. O log fornecido indicava conclusão da missão, mas não demonstra sozinho a trajetória física nem valida a detecção por câmera.

## Limites de uso

Os códigos são para SITL. `mavlink_interface.py` desativa `ARMING_CHECK`, força armamento e pode presumir GPS pronto quando a telemetria não informa os dados. Não conecte este conjunto a um veículo físico sem revisar armamento, telemetria, comandos e tratamento de falhas. Os scripts foram preservados como enviados, sem correções funcionais implícitas.
