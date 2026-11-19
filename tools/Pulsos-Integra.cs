using System;
using System.Drawing;
using System.Net.Sockets;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;

internal sealed class Bancada : Form
{
    private readonly TextBox endereco = new TextBox { Text = "169.254.1.1", Width = 145 };
    private readonly NumericUpDown porta = new NumericUpDown { Minimum = 1, Maximum = 65535, Value = 2101, Width = 78 };
    private readonly NumericUpDown duracao = new NumericUpDown { Minimum = 1, Maximum = 300, Value = 15, Width = 64 };
    private readonly Button iniciar = new Button { Text = "Enviar pulsos", AutoSize = true };
    private readonly Button parar = new Button { Text = "Parar", AutoSize = true, Enabled = false };
    private readonly TextBox registro = new TextBox { Multiline = true, ReadOnly = true, ScrollBars = ScrollBars.Vertical, Dock = DockStyle.Fill };
    private CancellationTokenSource teste;

    [STAThread]
    private static void Main()
    {
        Application.EnableVisualStyles();
        Application.SetCompatibleTextRenderingDefault(false);
        Application.Run(new Bancada());
    }

    private Bancada()
    {
        Text = "ProService | Pulsos CalAmp Integra";
        StartPosition = FormStartPosition.CenterScreen;
        ClientSize = new Size(610, 305);
        MinimumSize = new Size(626, 344);
        Font = new Font("Segoe UI", 10);

        var controles = new FlowLayoutPanel { Dock = DockStyle.Top, Height = 83, Padding = new Padding(8) };
        controles.Controls.AddRange(new Control[] {
            Rotulo("Endereço"), endereco, Rotulo("Porta TCP"), porta, Rotulo("Duração (s)"), duracao, iniciar, parar
        });
        controles.SetFlowBreak(duracao, true);
        var instrucoes = Rotulo("Padrão 0x55 a cada 500 ms. Meça a entrada COM do rádio entre os pinos 3 e 5.");
        instrucoes.Dock = DockStyle.Bottom;
        instrucoes.Padding = new Padding(8);
        Controls.Add(registro);
        Controls.Add(instrucoes);
        Controls.Add(controles);

        iniciar.Click += Iniciar;
        parar.Click += (sender, args) => teste.Cancel();
        FormClosed += (sender, args) => { if (teste != null) teste.Cancel(); };
    }

    private static Label Rotulo(string texto)
    {
        return new Label { Text = texto, AutoSize = true, Margin = new Padding(4, 6, 0, 0) };
    }

    private void Registrar(string mensagem)
    {
        if (!IsDisposed)
            registro.AppendText(DateTime.Now.ToString("HH:mm:ss") + "  " + mensagem + Environment.NewLine);
    }

    private async void Iniciar(object sender, EventArgs args)
    {
        string host = endereco.Text.Trim();
        if (Uri.CheckHostName(host) == UriHostNameType.Unknown)
        {
            Registrar("Informe um endereço IP ou nome de rede válido.");
            return;
        }

        if (MessageBox.Show(this, "O rádio pode transmitir por RF. Confirma que a antena adequada está conectada e que não há equipamento de controle na ponta remota?",
            "Confirmar envio", MessageBoxButtons.YesNo, MessageBoxIcon.Question, MessageBoxDefaultButton.Button2) != DialogResult.Yes)
            return;

        var cancelamento = new CancellationTokenSource();
        teste = cancelamento;
        iniciar.Enabled = endereco.Enabled = porta.Enabled = duracao.Enabled = false;
        parar.Enabled = true;
        try
        {
            int TCP = (int)porta.Value;
            int segundos = (int)duracao.Value;
            Registrar("Conectando a " + host + ":" + TCP + "...");
            int bytes = await Enviar(host, TCP, segundos, cancelamento.Token, Registrar);
            Registrar("Envio TCP concluído. " + bytes + " bytes. Confira os pulsos no osciloscópio.");
        }
        catch (OperationCanceledException) { Registrar("Teste interrompido."); }
        catch (Exception erro) { Registrar("Falha: " + erro.Message); }
        finally
        {
            teste = null;
            cancelamento.Dispose();
            if (!IsDisposed)
            {
                iniciar.Enabled = endereco.Enabled = porta.Enabled = duracao.Enabled = true;
                parar.Enabled = false;
            }
        }
    }

    internal static async Task<int> Enviar(string host, int porta, int segundos, CancellationToken token, Action<string> registrar)
    {
        var bloco = new byte[64];
        for (int i = 0; i < bloco.Length; i++) bloco[i] = 0x55;

        using (var cliente = new TcpClient { NoDelay = true })
        using (token.Register(() => cliente.Close()))
        {
            try
            {
                Task conexao = cliente.ConnectAsync(host, porta);
                using (var limite = CancellationTokenSource.CreateLinkedTokenSource(token))
                {
                    if (await Task.WhenAny(conexao, Task.Delay(3000, limite.Token)) != conexao)
                    {
                        token.ThrowIfCancellationRequested();
                        throw new TimeoutException("A conexão TCP não respondeu em três segundos.");
                    }
                    limite.Cancel();
                    await conexao;
                }
                registrar("TCP conectado. Enviando pulsos por " + segundos + " s.");
                using (var fluxo = cliente.GetStream())
                {
                    for (int i = 0; i < segundos * 2; i++)
                    {
                        token.ThrowIfCancellationRequested();
                        await fluxo.WriteAsync(bloco, 0, bloco.Length, token);
                        await Task.Delay(500, token);
                    }
                }
                return segundos * 2 * bloco.Length;
            }
            catch (Exception)
            {
                token.ThrowIfCancellationRequested();
                throw;
            }
        }
    }
}
