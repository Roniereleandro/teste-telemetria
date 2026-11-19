Add-Type -AssemblyName System.Windows.Forms
Add-Type -AssemblyName System.Drawing
[Windows.Forms.Application]::EnableVisualStyles()

$form = [Windows.Forms.Form]@{
    Text = 'ProService | Bancada CalAmp Integra'
    StartPosition = 'CenterScreen'
    ClientSize = [Drawing.Size]::new(700, 490)
    MinimumSize = [Drawing.Size]::new(716, 529)
    Font = [Drawing.Font]::new('Segoe UI', 10)
}
$painel = [Windows.Forms.FlowLayoutPanel]@{ Dock = 'Top'; Height = 108; Padding = [Windows.Forms.Padding]::new(8) }
$form.Controls.Add($painel)
function Add-Label($texto) {
    $painel.Controls.Add([Windows.Forms.Label]@{ Text = $texto; AutoSize = $true; Margin = [Windows.Forms.Padding]::new(4, 7, 0, 3) })
}
function Add-Button($texto) {
    $botao = [Windows.Forms.Button]@{ Text = $texto; AutoSize = $true; Height = 32 }
    $painel.Controls.Add($botao)
    $botao
}
Add-Label 'Endereço'
$endereco = [Windows.Forms.TextBox]@{ Text = '169.254.1.1'; Width = 145 }
$painel.Controls.Add($endereco)
Add-Label 'Porta TCP'
$porta = [Windows.Forms.NumericUpDown]@{ Minimum = 1; Maximum = 65535; Value = 2101; Width = 72 }
$painel.Controls.Add($porta)
Add-Label 'Duração (s)'
$duracao = [Windows.Forms.NumericUpDown]@{ Minimum = 1; Maximum = 300; Value = 15; Width = 65 }
$painel.Controls.Add($duracao)
$painel.SetFlowBreak($duracao, $true)
$conectar = Add-Button 'Conectar'
$escutar = Add-Button 'Escutar'
$pulsos = Add-Button 'Enviar pulsos'
$retorno = Add-Button 'Retorno da ponte'
$parar = Add-Button 'Parar'
$salvar = Add-Button 'Salvar registro'
$painel.SetFlowBreak($salvar, $true)
$estado = [Windows.Forms.Label]@{ Text = 'Aguardando teste.'; AutoSize = $true; Margin = [Windows.Forms.Padding]::new(4) }
$painel.Controls.Add($estado)

$observacao = [Windows.Forms.FlowLayoutPanel]@{ Dock = 'Bottom'; Height = 44; Padding = [Windows.Forms.Padding]::new(8) }
$pulsoVisto = [Windows.Forms.CheckBox]@{ Text = 'Pulsos observados na COM'; AutoSize = $true }
$rxConfirmado = [Windows.Forms.CheckBox]@{ Text = 'Recepção conferida no segundo rádio'; AutoSize = $true }
$observacao.Controls.AddRange(@($pulsoVisto, $rxConfirmado))
$form.Controls.Add($observacao)
$log = [Windows.Forms.TextBox]@{ Multiline = $true; ReadOnly = $true; ScrollBars = 'Vertical'; Dock = 'Fill'; Font = [Drawing.Font]::new('Consolas', 10) }
$form.Controls.Add($log)
$log.BringToFront()

$sessao = @{ Job = $null; Modo = ''; Parando = $false }
$parar.Enabled = $false
$backend = Join-Path $PSScriptRoot 'Testar-Integra.ps1'
function Write-Log($texto) {
    $log.AppendText(('[{0:HH:mm:ss}] {1}' -f (Get-Date), $texto) + [Environment]::NewLine)
}
function Set-Busy([bool]$ocupado) {
    foreach ($controle in @($conectar, $escutar, $pulsos, $retorno, $endereco, $porta, $duracao)) {
        $controle.Enabled = -not $ocupado
    }
    $parar.Enabled = $ocupado
}
function Start-Test($modo) {
    $hostAlvo = $endereco.Text.Trim()
    if ([Uri]::CheckHostName($hostAlvo) -eq [UriHostNameType]::Unknown) {
        Write-Log 'Informe um endereço IP ou nome de rede válido.'
        return
    }
    if ($modo -eq 'Pulsos') {
        $msg = 'A antena adequada está conectada e a ponta remota está sem equipamento de controle? O CalAmp pode transmitir por rádio.'
        if ([Windows.Forms.MessageBox]::Show($form, $msg, 'Confirmar envio', 'YesNo', 'Question') -ne 'Yes') { return }
    }
    if ($modo -eq 'Retorno') {
        $msg = 'O rádio está desconectado e somente os pinos 2 e 3 do DB9 do HF5111B estão unidos?'
        if ([Windows.Forms.MessageBox]::Show($form, $msg, 'Confirmar montagem de teste', 'YesNo', 'Question') -ne 'Yes') { return }
    }
    $sessao.Modo = $modo
    $sessao.Parando = $false
    $estado.Text = "Em execução: $modo"
    Write-Log "Início: $modo | ${hostAlvo}:$($porta.Value)"
    try {
        $sessao.Job = Start-Job -ArgumentList $backend, $modo, $hostAlvo, ([int]$porta.Value), ([int]$duracao.Value) -ScriptBlock {
            param($arquivo, $modo, $hostAlvo, $porta, $segundos)
            $ErrorActionPreference = 'Stop'
            if ($modo -eq 'Conectar') {
                $tcp = [Net.Sockets.TcpClient]::new()
                try {
                    if (-not $tcp.ConnectAsync($hostAlvo, $porta).Wait(3000)) { throw 'Tempo esgotado na conexão TCP.' }
                    'TCP conectado. Nenhum byte enviado neste teste.'
                } finally { $tcp.Dispose() }
                return
            }
            $opcoes = @{ Modo = $modo; Endereco = $hostAlvo; Porta = $porta; Segundos = $segundos }
            if ($modo -eq 'Pulsos') { $opcoes.AutorizarRF = $true }
            if ($modo -eq 'Retorno') { $opcoes.PonteIsolada = $true }
            & $arquivo @opcoes
        }
        Set-Busy $true
    } catch {
        $estado.Text = 'Falha ao iniciar.'
        Write-Log $_.Exception.Message
    }
}

$timer = [Windows.Forms.Timer]@{ Interval = 200 }
$timer.Add_Tick({
    $job = $sessao.Job
    if ($null -eq $job) { return }
    Receive-Job $job -ErrorAction Continue 2>&1 | ForEach-Object { Write-Log $_ }
    if ($job.State -notin 'Completed', 'Failed', 'Stopped') { return }
    Receive-Job $job -ErrorAction Continue 2>&1 | ForEach-Object { Write-Log $_ }
    $estado.Text = if ($sessao.Parando) { 'Teste interrompido.' } elseif ($job.State -eq 'Completed') { "Concluído: $($sessao.Modo)." } else { "Falha: $($sessao.Modo)." }
    Write-Log $estado.Text
    Remove-Job $job
    $sessao.Job = $null
    Set-Busy $false
})
$conectar.Add_Click({ Start-Test Conectar })
$escutar.Add_Click({ Start-Test Escutar })
$pulsos.Add_Click({ Start-Test Pulsos })
$retorno.Add_Click({ Start-Test Retorno })
$parar.Add_Click({
    if ($sessao.Job) {
        $sessao.Parando = $true
        Stop-Job $sessao.Job
    }
})
$pulsoVisto.Add_CheckedChanged({ Write-Log "Observação manual | Pulsos na COM: $($pulsoVisto.Checked)." })
$rxConfirmado.Add_CheckedChanged({ Write-Log "Observação manual | Recepção no segundo rádio: $($rxConfirmado.Checked)." })
$salvar.Add_Click({
    $dialogo = [Windows.Forms.SaveFileDialog]@{ Filter = 'Registro de bancada (*.txt)|*.txt'; FileName = "Integra-$(Get-Date -Format yyyyMMdd-HHmmss).txt" }
    try {
        if ($dialogo.ShowDialog($form) -eq 'OK') {
            [IO.File]::WriteAllText($dialogo.FileName, $log.Text, [Text.Encoding]::UTF8)
        }
    } catch { Write-Log $_.Exception.Message }
    finally { $dialogo.Dispose() }
})
$form.Add_FormClosed({
    $timer.Stop()
    if ($sessao.Job) { Remove-Job $sessao.Job -Force }
})
$timer.Start()
try { [void]$form.ShowDialog() }
finally { $timer.Dispose(); $form.Dispose() }
