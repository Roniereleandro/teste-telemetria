# Plano de testes de bancada
Todos os casos abaixo são obrigatórios; situação inicial NÃO EXECUTADO.
Execute com pressão e segurança estáveis, salvo a subetapa explícita de segurança.

| Grupo | Procedimento | Foco |
|---|---|---|
| LORA | [Comunicação](LORA.md) | entrega, isolamento por ID/canal, interrupção e recuperação |
| GPS | [Perda e retorno](GPS.md) | validade, amostra nova, coordenadas parado |
| WIFI | [Perda de Wi-Fi e portal](WIFI.md) | autonomia, limite offline, seguranças |
| MOV | [Falsa parada](MOVIMENTO.md) | movimento lento, ruído e parada real |

## Convenções comuns
P = intervalo de envio; C = maior ciclo de avaliação configurado/medido antes do cenário; U = incerteza do instrumento (até 1 s). Tempos em segundos nas fórmulas. As tolerâncias não autorizam ampliar o limite offline: a saída deve cair até T_OFF+C+U.
Para cada execução registre valores numéricos, instante inicial, estímulo, reação, métricas e arquivos de evidência. Não aprove pela impressão de que “funcionou”.

APROVADO: todos os critérios atendidos nas três repetições. REPROVADO: pelo menos um desvio comprovado. BLOQUEADO: faltam configuração, observabilidade ou montagem. NÃO EXECUTADO: não iniciado.
Se falhar uma repetição, mantenha a falha no histórico; correção exige nova rodada e identificação do novo firmware.
