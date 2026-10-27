# LORA — comunicação E220
## LORA-01 — enlace nominal
Pré-condição: IDs autorizados, canais efetivos e parâmetros compatíveis; GPS com amostras válidas. Fixe P e registre.
1. Capture no transmissor e receptor 300 quadros únicos consecutivos, por pelo menos 10 min.
2. Conte enviados N, recebidos únicos aceitos A, lacunas, duplicados e erros JSON.
3. Repita três vezes.
Aprovar: A/N >= 99%; zero erros JSON para quadros válidos; nenhum reset ou acionamento inesperado; intervalos entre aceites <= 3P+C+U.
Conte por sequência única e identifique reinício de sequência; duplicados não aumentam A. Meta de 99% é proposta inicial para bancada.

## LORA-02 — isolamento
Use emissor de teste ou terceira ponta; central autorizada para ID A no canal X.
1. Envie 100 quadros válidos de ID B no canal X; intercale com 100 de A/X.
2. Envie 100 quadros com ID A, canal declarado Y, mantendo transporte físico X.
3. Configure realmente a ponta em Y e confirme por leitura; central permanece X. Tente 100 envios.
4. Retorne A/X e confirme recuperação. Faça três rodadas.
Aprovar: zero quadros não autorizados alteram posição, último GPS válido, temporizador de movimento ou saídas; aceites A/X >= 99%; rejeições observáveis quando bytes chegam. Divergência do canal declarado é rejeitada. Canal declarado no JSON não prova canal físico.
No teste de canal físico diferente, se houver bytes recebidos, registre a ocorrência e exija rejeição; investigue parâmetros efetivos. Não conte ausência de RX como prova isolada do filtro de software.

## LORA-03 — perda e retorno
1. Com enlace estável, interrompa transmissão por T_LINK+30 s.
2. Observe idade e estado do enlace. Restaure transmissão com o mesmo ID/canal.
Aprovar: enlace deixa de ser válido entre T_LINK−U e T_LINK+C+U desde último quadro aceito; dados antigos não viram amostra nova; retorno reconhecido em até 3P+C+U; sem reset e sem partida automática por retorno do rádio.
Registre separadamente a política de parada por perda de enlace do firmware; sem política identificada, o critério de atuação fica BLOQUEADO.

## LORA-04 — entradas inválidas
Injete 30 linhas de cada classe: JSON truncado, ID ausente, coordenada fora de faixa, sequência/amostra repetida. Registre payload e via de injeção.
Aprovar: zero crash/reset, zero atualização de posição por entrada inválida; repetição não renova idade da amostra GPS nem confirma movimento; quadro válido seguinte processado até 3P+C+U.
