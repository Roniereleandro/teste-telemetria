# Organização e liberação de firmware
## Entrada de arquivos
Nenhuma revisão é presumida como mais recente. Para cada arquivo recebido registre origem, nome original, revisão visível, SHA-256, placa, core ESP32, bibliotecas/versões, partição, parâmetros de compilação, tamanho de sketch e memória.
Mantenha pasta própria para cada sketch Arduino com nome compatível com seu .ino.

## Bancada
Use firmware/bancada/central/<revisao>/ e firmware/bancada/gps/<revisao>/.
Identifique na serial e interface: BANCADA — NÃO CAMPO. Documente toda instrumentação e injeção de dados. Não publique senhas, tokens, SSIDs privados ou coordenadas reais.
Mudanças em temporizadores, GPS simulado e bypass de entradas não podem migrar silenciosamente para produção.

## Produção
firmware/producao/ fica sem sketches até existir liberação explícita do responsável.
Para propor uma versão: compilar o par exato; executar todos os casos obrigatórios; resolver falhas; comparar pinagem/proteções com a base; registrar hashes, evidências e limitações; fazer validação de campo supervisionada e registrar decisão do responsável.
Teste novamente o binário candidato final se houver qualquer mudança após ensaio. Binário de bancada não é candidato final. Não liberar com caso obrigatório REPROVADO, BLOQUEADO ou NÃO EXECUTADO.
Uma cópia em main não significa firmware aprovado. Este repositório não realiza gravação, OTA nem acionamento de equipamento.

## Checklist da proposta de liberação
- [ ] Origem, hashes, dependências e compilação reproduzível registrados.
- [ ] Casos de bancada aprovados com evidência do par candidato.
- [ ] Sem injeção, bypass, credenciais ou temporizadores acelerados.
- [ ] Pinagem, segurança, pressão e watchdog revisados.
- [ ] Plano de retorno e validação de campo documentados.
- [ ] Responsável, data e decisão de liberação registrados.
