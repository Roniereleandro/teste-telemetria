[CmdletBinding(DefaultParameterSetName = 'Serial')]
param(
    [Parameter(Mandatory, ParameterSetName = 'Serial')]
    [ValidatePattern('^COM[0-9]+$')]
    [string]$Porta,

    [Parameter(ParameterSetName = 'Serial')]
    [ValidateRange(1200, 115200)]
    [int]$Baud = 9600,

    [Parameter(Mandatory, ParameterSetName = 'Offline')]
    [switch]$Autoteste
)

$ErrorActionPreference = 'Stop'

function Get-Crc16([byte[]]$Dados, [int]$Tamanho) {
    $crc = 0xFFFF
    for ($i = 0; $i -lt $Tamanho; $i++) {
        $crc = $crc -bxor ([int]$Dados[$i] -shl 8)
        for ($bit = 0; $bit -lt 8; $bit++) {
            $alto = $crc -band 0x8000
            $crc = ($crc -shl 1) -band 0xFFFF
            if ($alto) { $crc = $crc -bxor 0x1021 }
        }
    }
    $crc
}

function Read-Estado([byte[]]$Dados, [byte]$Sequencia) {
    for ($i = 0; $i + 3 -le $Dados.Length; $i++) {
        if ($Dados[$i] -ne 0x55 -or $Dados[$i + 1] -ne 0xAA) { continue }
        $tamanho = [int]$Dados[$i + 2]
        if ($tamanho -lt 43 -or $i + $tamanho -gt $Dados.Length) { continue }
        [byte[]]$quadro = $Dados[$i..($i + $tamanho - 1)]
        if ($quadro[4] -ne $Sequencia -or $quadro[5] -ne 0x41) { continue }
        if ($quadro[6] -ne 0 -or $quadro[7] -ne 5) { continue }
        $crc = ([int]$quadro[$tamanho - 2] -shl 8) -bor $quadro[$tamanho - 1]
        if ((Get-Crc16 $quadro ($tamanho - 2)) -ne $crc) { continue }
        $angulo = (([int]$quadro[8] -shl 8) -bor $quadro[9]) / 16
        if ($angulo -ge 360) { continue }
        return [pscustomobject]@{
            AnguloGraus = $angulo
            Quadro = [BitConverter]::ToString($quadro)
        }
    }
}

[byte[]]$consulta = 0x55, 0xAA, 0x0A, 0x80, 0x01, 0x01, 0x00, 0x05, 0x00, 0x00
$crc = Get-Crc16 $consulta 8
$consulta[8] = $crc -shr 8
$consulta[9] = $crc -band 0xFF

if ($Autoteste) {
    if ([BitConverter]::ToString($consulta) -ne '55-AA-0A-80-01-01-00-05-31-56') {
        throw 'A consulta difere do vetor de referência.'
    }
    $resposta = [byte[]]::new(43)
    [byte[]]$prefixo = 0x55, 0xAA, 43, 0x80, 0x01, 0x41, 0x00, 0x05, 0x07, 0x88
    $prefixo.CopyTo($resposta, 0)
    $crc = Get-Crc16 $resposta 41
    $resposta[41] = $crc -shr 8
    $resposta[42] = $crc -band 0xFF
    if ((Read-Estado $resposta 1).AnguloGraus -ne 120.5) { throw 'Ângulo sintético incorreto.' }
    if (Read-Estado $resposta 2) { throw 'Sequência incorreta aceita.' }
    $resposta[9] = $resposta[9] -bxor 1
    if (Read-Estado $resposta 1) { throw 'CRC incorreto aceito.' }
    'Autoteste aprovado.'
    return
}

$serial = [System.IO.Ports.SerialPort]::new($Porta, $Baud, 'None', 8, 'One')
$serial.ReadTimeout = 100
$serial.WriteTimeout = 500
$recebido = [System.Collections.Generic.List[byte]]::new()
try {
    $serial.Open()
    $serial.DiscardInBuffer()
    $serial.Write($consulta, 0, $consulta.Length)
    $tempo = [System.Diagnostics.Stopwatch]::StartNew()
    $buffer = [byte[]]::new(256)
    while ($tempo.ElapsedMilliseconds -lt 2000) {
        try {
            $lidos = $serial.Read($buffer, 0, $buffer.Length)
        } catch [System.TimeoutException] {
            continue
        }
        $recebido.AddRange([byte[]]$buffer[0..($lidos - 1)])
        $estado = Read-Estado ($recebido.ToArray()) $consulta[4]
        if ($estado) { return $estado }
    }
    throw "Sem resposta compatível em $Porta. RX ($($recebido.Count) bytes): $([BitConverter]::ToString($recebido.ToArray()))"
} finally {
    $serial.Dispose()
}
