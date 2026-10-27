# PROSERVICE — validação de telemetria
Estrutura inicial de testes de bancada para central ESP32 e ponta GPS, com comunicação LoRa E220.

**Situação: testes NÃO EXECUTADOS. Nenhum firmware foi incluído ou aprovado para campo.** Esta entrega organiza procedimentos e evidências; não comprova o funcionamento das revisões existentes.

## Começar
1. Leia [preparação da bancada](docs/BANCADA.md).
2. Copie [o modelo de execução](resultados/MODELO_EXECUCAO.md) para uma pasta de rodada.
3. Registre os arquivos exatos da central e da ponta, seus SHA-256, configuração e ferramentas.
4. Execute os [casos de teste](testes/bancada/README.md) e anexe logs com tempos.
5. Registre APROVADO, REPROVADO, BLOQUEADO ou NÃO EXECUTADO por caso. Ausência de evidência impede aprovação.

## Organização
| Caminho | Uso |
|---|---|
| firmware/bancada/central/ | Cópias instrumentadas da central; nunca instalar em campo |
| firmware/bancada/gps/ | Cópias instrumentadas da ponta; nunca instalar em campo |
| firmware/producao/ | Somente versões liberadas, com rastreabilidade |
| testes/bancada/ | Procedimentos, entradas e critérios objetivos |
| resultados/ | Rodadas, medições e evidências sem credenciais |
| docs/ | Montagem, parâmetros e fluxo de liberação |

Os diretórios de firmware contêm apenas instruções. Não há sketch fictício, binário ou versão escolhida como base.

## Escopo e limites
Requisitos de referência do projeto: até 60 min sem portal, falta de movimento em 10 min, histerese de 2 m e coordenadas válidas mesmo parado. Precisam ser conferidos na revisão realmente ensaiada. Limites adicionais de comunicação neste plano são metas propostas de bancada, não medições já realizadas.

Aprovação em bancada não equivale a liberação para produção. Consulte [fluxo de firmware](docs/FIRMWARE.md). Não há execução automática de hardware neste repositório.
