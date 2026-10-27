# MOV — falta de movimento e falsa parada
Use trajetória GPS simulada reproduzível por UART da ponta ou entrada de bancada da central; registre caminho e payloads. Uma simulação que pula o filtro de produção não valida esse filtro.
Configure estado operacional monitorado, geometria válida, direção definida, pressão e segurança normais. T_MOV de referência=600 s; H=2 m. Intervalo de amostra=1 s nos cenários simulados, se suportado. Documente instante de início da vigilância e regra real de confirmação de movimento.

## MOV-01 — deslocamento lento e contínuo
1. Injete trajetória no arco do pivô com passos de 0,10 m a cada segundo, por 2*T_MOV.
2. Capture distância entre amostras e desde a referência acumulada, confirmações e temporizador.
Aprovar: zero evento de falta de movimento e zero parada; deslocamento acumulado > H deve ser reconhecido antes de T_MOV; passos individuais < H não podem impedir toda confirmação. Registre velocidade e precisão da simulação.

## MOV-02 — parada real
1. Confirme movimento e anote t0 (última confirmação).
2. Envie novas amostras válidas da mesma posição por T_MOV+30 s, mantendo enlace.
Aprovar: evento “pivô parado” e desligamento entre t0+T_MOV−U e t0+T_MOV+C+U; nenhum pacote novo sem deslocamento renova movimento; sem religamento automático.

## MOV-03 — oscilação sem deslocamento real
1. Com referência estacionária, alterne posições entre −0,9 m e +0,9 m em um eixo, sem sair do raio de H=2 m, durante T_MOV+30 s.
2. Cada amostra é nova; satélites/HDOP devem permanecer dentro da validade configurada.
Aprovar: ruído não acumula distância artificial indefinidamente; parada no prazo de MOV-02; zero confirmação de movimento somente pela oscilação.
Esta é verificação do tratamento de ruído; a regra exata deve ser conferida no firmware.

## MOV-04 — interrupção GPS durante movimento
1. Repita MOV-01 e retire fix por T_GPS+30 s, mantendo LoRa.
2. Restaure amostras compatíveis com a trajetória.
Aprovar: diagnóstico distingue GPS indisponível de parada física confirmada; ausência de amostras não gera falsa confirmação de movimento; atuação segue política GPS documentada. Sem política definida, BLOQUEADO.
Pode existir parada preventiva por GPS, mas ela deve ter motivo correspondente e não ser declarada medição de imobilidade.

## MOV-05 — parado por comando
Mantenha estado idle/boot_hold com GPS válido e posição fixa por T_MOV+30 s em cada estado.
Aprovar: zero alarme de falta de movimento operacional e zero partida inesperada. Todos os cenários: três repetições.
