# GPS — perda de posição e recuperação
## GPS-01 — coordenadas com pivô parado
1. Com saída de movimento desligada, adquira fix e capture 20 novas amostras na ponta e central.
2. Repita antes de qualquer comando e com geometria angular não configurada.
Aprovar: coordenadas válidas chegam à central em até 3P+C+U após aquisição; zero necessidade de partir; falta de geometria impede ângulo válido, mas não coordenadas válidas.

## GPS-02 — perda de fix sem perda de LoRa
1. Mantenha rádio ativo. Em bancada, injete fluxo GPS inválido/sem fix pela UART do GPS em versão instrumentada, ou bloqueie recepção GNSS de forma passiva. Registre método.
2. Mantenha condição por T_GPS+30 s; capture mensagens rádio e idade da última amostra.
3. Restaure novas amostras válidas.
Aprovar: zero coordenadas inventadas/zeradas apresentadas como válidas; última posição, se exibida, marcada antiga; GPS inválido até T_GPS+C+U desde última amostra válida (invalidação explícita pode ser imediata); LoRa continua ativo; recuperação em até 3P+C+U após primeira amostra válida restaurada.
Tempo para obter fix real deve ser medido, mas não é confundido com latência de transporte. Satélites/HDOP são evidências, não prova única de validade.

## GPS-03 — amostra congelada
1. Reenvie a mesma amostra GPS válida, com novos quadros LoRa, por T_GPS+30 s.
2. Injete também indicação explícita de fix perdido.
Aprovar: idade GPS cresce sem renovação pela retransmissão; validade expira até T_GPS+C+U; perda explícita não mantém fix válido; nenhum movimento confirmado por repetição.

## GPS-04 — caminhos de aquisição
Execute GPS-01/02/03 no GPS recebido por LoRa e no GPS local da central se este for suportado. Documente caminho ativo e prioridade.
Se GPS local não existir na revisão, registre NÃO APLICÁVEL com evidência do arquivo; os casos LoRa permanecem obrigatórios.
Política de atuação por perda GPS deve ser registrada antes do ensaio; ausência de definição bloqueia aprovação da atuação. Não aceitar evento “pivô parado” como prova de ausência real de movimento durante perda GPS.
