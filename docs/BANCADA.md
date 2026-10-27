# Preparação da bancada
## Montagem
- Use duas placas ESP32, E220 compatíveis com o conjunto, GPS, antenas adequadas, fonte estável, multímetro e captura serial de cada placa.
- Mantenha motores, bomba, contatores e tensões de potência desconectados. Observe saídas com LEDs/cargas de teste isoladas.
- Conecte antenas antes de transmitir. Para interrupção do enlace, desligue a ponta ou iniba transmissão no firmware de bancada; não retire antena com rádio transmitindo.
- Preserve a pinagem do arquivo ensaiado. Referência da central: LoRa RX GPIO33, TX GPIO25, AUX GPIO35; pressostato GPIO34; R6 GPIO23; segurança GPIO36/39. Confira esquema e arquivo antes de ligar.
- Não suponha pinagem da ponta: transcreva do sketch e confira a placa. M0/M1 dependem do modo; registre modo normal e procedimento de configuração usado.
- Nunca aplique 5 V em GPIO ESP32. GPIO34/35/36/39 não têm pull-up/pull-down interno; confira polarização externa e interface existente.
- Simule segurança e pressão apenas na bancada isolada, por interface compatível. Não elimine proteções do firmware.

## Configuração obrigatória por rodada
Registre IDs autorizados, canal salvo e canal efetivamente lido em cada rádio, modelo E220, parâmetros UART/rádio, intervalo de envio P, timeout de enlace T_LINK, timeout de GPS T_GPS, ciclo de avaliação C, limite de movimento T_MOV, limite offline T_OFF, histerese H e política de recuperação.
Use tempos monotônicos em ms para critérios; horário humano em America/Cuiaba (UTC−04:00) para correlacionar logs. Registre precisão do instrumento U (máximo 1 s para estes testes).
Se um parâmetro ou comportamento esperado não puder ser identificado, marque o caso dependente BLOQUEADO; não invente valor.

Referências a conferir: T_MOV=600 s; T_OFF=3600 s; H=2 m. Não reduza esses tempos no ensaio final. Versões aceleradas são auxiliares e devem ser identificadas no relatório.

## Observabilidade mínima
Capture em central e ponta: tempo, versão, motivo de reset, estado, saídas, segurança/pressão, ID/canal, sequência, nova amostra, validade/idade GPS, satélites, HDOP, posição, distância reconhecida, contador de movimento, conectividade e motivo da parada.
Se a página não exibir um dado necessário, use serial ou instrumentação exclusiva de bancada. Sem evidência do dado, o critério fica BLOQUEADO.

Antes de cada cenário restaure o estado inicial, esvazie somente buffers de captura (preserve histórico do equipamento), sincronize relógios e anote contadores iniciais. Repita cada cenário três vezes. Não avance após saída inesperada até registrar e entender o desvio.
