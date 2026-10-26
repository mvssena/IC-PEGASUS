# PT18796 — preparação para Gazebo (em andamento)

## Alterações

A interface MAVLink recusa armamento quando o GPS é ausente, antigo (mais de 2 s), com fix menor que 3, menos de 10 satélites ou HDOP maior que 1,40. A verificação usa `GPS_RAW_INT.eph / 100`; o valor 65535 é inválido. Não altera `ARMING_CHECK`, não força armamento e exige a confirmação na telemetria. Cada envio de heartbeat agora é registrado com relógio monotônico. O timeout do pouso passa a reportar falha quando não há desarmamento.

## Validação local

```bash
python3 -m unittest -v test_pt18796
```

Testes usam objetos simulados e cobrem critérios de GPS, recusa de armar, transições e timeouts dos quatro estados e os três casos de percepção. Eles não demonstram voo, telemetria real, recepção pelo piloto automático ou ausência de failsafe.

## Trabalho pendente antes do teste integrado

1. Receber o contrato oficial e o publicador de replay do colega (campos, unidades, transporte, porta e exemplos). O replay embutido em `run_passo4_missao_sitl.py` é sintético e não é prova de integração real. Não usar este runner como entrega final.
2. Adaptar o consumidor ao contrato recebido e validar perda, baixa confiança e instabilidade na FSM. A versão atual não está validada contra o contrato oficial.
3. Testar com ArduPilot SITL e Gazebo instalados no computador de execução. Registrar vídeo, comandos, parâmetros e logs reais de telemetria e autopiloto.
4. O script de heartbeat registra envio durante o bloqueio do laço por 10 s; isso sozinho não prova recepção nem ausência de failsafe. Anotar configuração de failsafe e fontes de heartbeat, capturar os eventos e dados do piloto automático, garantir que outra estação não mascare a perda e realizar controle negativo. Verificar comportamento e IDs conforme a versão do ArduPilot. Não afirmar êxito com base somente em mensagens de envio.

O código não foi testado no Gazebo neste ambiente e não deve ser usado em uma aeronave real.
