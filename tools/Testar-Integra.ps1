[CmdletBinding()]
param(
    [ValidateSet('Escutar', 'Enviar', 'Retorno')]
    [string]$Modo = 'Escutar',
    [string]$Endereco = '169.254.1.1',
    [ValidateRange(1, 65535)]
    [int]$Porta = 2101,
    [ValidateRange(1, 300)]
    [int]$Segundos = 10,
    [ValidatePattern('^[ -~]{1,64}$')]
    [string]$Mensagem = 'INTEGRA-COM-TESTE',
    [switch]$AutorizarRF,
    [switch]$PonteIsolada
)

$ErrorActionPreference = 'Stop'
if ($Modo -eq 'Enviar' -and -not $AutorizarRF) {
    throw 'O modo Enviar pode transmitir por RF. Use -AutorizarRF apenas com antena adequada conectada e sem equipamento de controle na ponta remota.'
}
if ($Modo -eq 'Retorno' -and -not $PonteIsolada) {
    throw 'Desconecte o radio e feche apenas os pinos 2 e 3 do DB9 do HF5111B. Depois use -PonteIsolada.'
}

$cliente = [Net.Sockets.TcpClient]::new()
try {
    if (-not $cliente.ConnectAsync($Endereco, $Porta).Wait(3000)) {
        throw 'Tempo esgotado ao conectar ao HF5111B.'
    }
    "TCP conectado a ${Endereco}:$Porta. Modo: $Modo."
    $fluxo = $cliente.GetStream()
    $fluxo.ReadTimeout = 200
    $fluxo.WriteTimeout = 1000
    $tx = [Text.Encoding]::ASCII.GetBytes("$Mensagem`r`n")

    if ($Modo -ne 'Escutar') {
        $fluxo.Write($tx, 0, $tx.Length)
        "TX $($tx.Length) bytes: $([BitConverter]::ToString($tx))"
    }

    $buffer = [byte[]]::new(1024)
    $total = 0
    $retorno = [IO.MemoryStream]::new()
    $relogio = [Diagnostics.Stopwatch]::StartNew()
    while ($relogio.Elapsed.TotalSeconds -lt $Segundos) {
        try {
            $lidos = $fluxo.Read($buffer, 0, $buffer.Length)
        } catch [IO.IOException] {
            $causa = $_.Exception.InnerException
            if ($causa -is [Net.Sockets.SocketException] -and $causa.SocketErrorCode -eq 'TimedOut') { continue }
            throw
        }
        if ($lidos -eq 0) {
            'O HF5111B encerrou a conexao.'
            break
        }
        $total += $lidos
        $ascii = [Text.Encoding]::ASCII.GetString($buffer, 0, $lidos) -replace '[^ -~]', '.'
        "RX +$($relogio.ElapsedMilliseconds) ms | $([BitConverter]::ToString($buffer, 0, $lidos)) | $ascii"
        if ($Modo -eq 'Retorno') {
            $retorno.Write($buffer, 0, $lidos)
            if ($total -ge $tx.Length) { break }
        }
    }
    "RX total: $total bytes."
    if ($Modo -eq 'Retorno') {
        if ([BitConverter]::ToString($retorno.ToArray()) -cne [BitConverter]::ToString($tx)) {
            throw 'Falha no retorno: os bytes recebidos diferem dos enviados.'
        }
        'Retorno aprovado. Este resultado valida a ponte, com o radio desconectado.'
    } elseif ($Modo -eq 'Enviar') {
        'O envio TCP nao comprova transmissao por RF. Verifique a indicacao TX do radio. Para confirmar o enlace, capture os bytes em um receptor compativel.'
    }
} finally {
    $cliente.Dispose()
}
