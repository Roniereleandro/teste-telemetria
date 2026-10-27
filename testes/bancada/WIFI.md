# WIFI — rede, portal e autonomia
Pré-condição: operação simulada estável, modo backend remoto ativo, GPS/movimento simulados válidos e pressão/segurança estáveis. Evite que outro temporizador encerre a operação. Referência T_OFF=3600 s; confirme no firmware. Registre qual evento renova a comunicação bem-sucedida com o portal.

## WIFI-01 — perda curta
1. Registre último sucesso com portal; desligue AP/roteador por 5 min.
2. Observe relés simulados, LoRa, GPS, watchdog e AP de configuração local.
3. Restaure rede e acesso ao portal.
Aprovar: operação simulada mantida durante a janela; zero reset; LoRa/GPS continuam sem ultrapassar seus limites; AP local acessível em até 60 s; novo sucesso com portal em até 120 s do retorno; zero comando antigo executado novamente. Limites 60/120 s são metas iniciais de bancada.

## WIFI-02 — limite offline
1. A partir do último sucesso com portal, bloqueie internet mantendo AP ligado por T_OFF+60 s.
2. Capture heartbeat/respostas e saídas durante toda a janela.
3. Repita com AP desligado. Execute três vezes cada condição.
Aprovar: não parar por “sem internet” antes de T_OFF−U; parar entre T_OFF−U e T_OFF+C+U; registrar motivo “sem internet” e tempo; saídas de movimento/bomba desligadas conforme mapa documentado; nenhuma partida automática ao restaurar rede.
Estar associado ao Wi-Fi ou obter HTTP de erro não renova indevidamente o temporizador. Exija evidência do sucesso de aplicação esperado.

## WIFI-03 — segurança durante offline
1. Durante perda curta de rede, abra entrada de segurança simulada em bancada isolada.
2. Meça tempo até desligamento das saídas. Repita com a outra cadeia.
Aprovar: parada dentro do limite de segurança documentado no firmware (registrar antes; se desconhecido, BLOQUEADO); motivo correto; ausência de internet não bloqueia proteção; retorno da entrada não provoca partida.
Não atribua à segurança o timeout de 60 min. Restaure a bancada antes do próximo cenário.

## WIFI-04 — interrupções e modo local
1. Faça três ciclos: 2 min offline, sucesso real com portal, nova interrupção.
2. Verifique que a contagem reinicia somente no sucesso esperado.
3. Se houver modo backend remoto=NÃO, teste operação local por T_OFF+60 s.
Aprovar: contagem offline coerente com o último sucesso; nenhuma parada por backend no modo local; seguranças locais mantidas. Documente explicitamente se esse modo não existe.
