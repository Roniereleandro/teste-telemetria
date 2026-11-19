# ICON Monitor e leitura de posição Valley

O ICON Monitor contém uma consulta de posição que pode servir de referência para estudar a redundância do GPS da ponta. A compatibilidade com o painel instalado ainda depende da identificação do painel e de uma resposta serial real em bancada.

## Evidência do executável

A análise foi feita com extração do pacote PyInstaller e desmontagem estática do módulo Python 3.8 `icon_campo.py` usando xdis (`35d8df26622903e57c97a797cb7fefd2f0338267`). O executável permaneceu sem execução. Nenhum teste foi feito com um painel.

- SHA-256 de `ICON Monitor.exe` — `17e3fc62d8aa01eb715675ecba1a2e5a8b55776faf3641cb53d6b334e0b03325`.
- SHA-256 do bytecode extraído — `de664eadaad1a433bfb189b5258c8c7d3e4dec29b4156718ba5acdb4f2c2f165`.

| Item | Comportamento do ICON Monitor |
| --- | --- |
| Busca serial | Começa em 9600 baud, sem paridade e com um bit de parada. |
| Quadro enviado | `55 AA`, comprimento total em um byte, byte fixo `80`, sequência, tipo, dados e CRC de dois bytes. A função de consulta não fornece um RTU ID configurável ao montador do quadro. |
| CRC | CRC-16 com polinômio `0x1021` e inicialização `0xFFFF`, sem reflexão ou XOR final. Cobre do cabeçalho até o fim dos dados. É anexado com o byte mais significativo primeiro. |
| Consulta de estado | Tipo `0x01` com código `0x0005` em dois bytes, primeiro o mais significativo. |
| Aceitação da resposta | O aplicativo espera CRC válido, a mesma sequência e tipo `0x41` para uma leitura. |
| Ângulo atual | O aplicativo exige ao menos 35 bytes de dados na resposta de estado, pula os dois primeiros e interpreta os dois seguintes como inteiro sem sinal, byte mais significativo primeiro, dividido por 16 para obter graus. |

O vetor sintético `55 AA 0A 80 01 01 00 05 31 56` representa a consulta de estado com sequência `01`. O CRC `3156` foi conferido de forma independente com `binascii.crc_hqx`. Esse vetor foi calculado offline.

No bytecode, os pontos de referência são `crc16` (linha 33), `build_frame` (42), `FrameReader.feed`, `decode_status` (105), `Link.ler` (340) e a rotina de consulta periódica (625).

## Aplicação ao ProService

A Valley documenta telemetria para Select2 e Pro2 por meio do CommanderVP, com requisito de firmware 8.02 ou superior para Pro2. Isso demonstra suporte ao acessório oficial. O formato de quadros e o mapa dos dados do ICON Monitor ainda precisam ser comparados com os do painel instalado. [Valley CommanderVP](https://www.valleyirrigation.com/commandervp)

Os manuais devem ser escolhidos pela versão de software do painel. O manual Select2 2.00 documenta opções de baud e protocolo para os conectores de 9 e 25 pinos, RTU ID, posição atual e configuração do resolver. [Manual Select2 2.00](https://webassetsprod.valmont.com/valmontproduction/docs/librariesprovider129/owners-manuals/r-to-z-/select2-advanced-features.pdf?sfvrsn=9f0e139_4) · [Manual Pro2 9.02](https://webassetsprod.valmont.com/valmontproduction/docs/librariesprovider129/owners-manuals/i-to-q-/pro2-advanced-features.pdf?sfvrsn=c6f2e139_4)

Para a central independente, a primeira validação é ler somente a posição em bancada com as cargas desconectadas. Depois de identificar a versão, a porta, a configuração serial e a fonte de posição do painel, o teste precisa comparar o valor recebido com o visor em pelo menos duas posições conhecidas. Para servir de redundância, a posição precisa continuar atualizando quando o GPS da ponta estiver indisponível, vir de uma fonte independente dele e ter validade e idade verificáveis pela central. A comparação com o GPS em operação normal deve estabelecer a referência e o sentido angular antes de usar essa leitura na parada programada.

## Sonda de bancada para o Select2

As fotos confirmam o modelo Select2 e mostram os conectores de 9 e 25 pinos. Para a arquitetura em que o painel executa o controle, o portal ProService usa a conexão de rede existente do HF-006eNET, e o ESP32 conversa com o Select2 pela porta de 9 pinos por meio de um transceptor UART 3,3 V ↔ RS-232. Confirme a pinagem, o cabo e a disponibilidade da interface elétrica. Registre a versão do software, `Baud 9-Pin`, `Protocol 9-Pin` e `RTU ID` no painel.

A [sonda PowerShell](../tools/Testar-Select2.ps1) permite validar o protocolo no computador se houver um adaptador USB–RS-232 disponível. Use somente em bancada com as cargas desconectadas, após confirmar cabo e parâmetros. A sonda usa 8N1 como hipótese oriunda do ICON Monitor, envia uma única consulta experimental de estado e exibe o ângulo se a resposta atender aos critérios de validação, incluindo o identificador esperado `0005`. Em caso de incompatibilidade, preserva os bytes recebidos na mensagem de diagnóstico. Substitua `COM5` pela porta identificada do adaptador e informe o baud lido no painel.

```powershell
./tools/Testar-Select2.ps1 -Autoteste
./tools/Testar-Select2.ps1 -Porta COM5 -Baud 9600
```

O autoteste confere o vetor TX, a leitura de 120,5° em uma resposta sintética e a rejeição de sequência ou CRC incorretos. A compatibilidade do Select2 e o driver no HF-006eNET ficam pendentes de uma captura real.
