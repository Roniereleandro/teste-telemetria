using System;
using System.Drawing;
using System.IO.Ports;
using System.Text;
using System.Windows.Forms;

internal sealed class SerialIntegra : Form
{
    private readonly ComboBox porta = Lista(95);
    private readonly ComboBox baud = Lista(85, "1200", "2400", "4800", "9600", "19200", "38400", "57600", "115200");
    private readonly ComboBox paridade = Lista(80, "None", "Even", "Odd");
    private readonly ComboBox bits = Lista(55, "7", "8");
    private readonly ComboBox fim = Lista(55, "1", "2");
    private readonly NumericUpDown duracao = new NumericUpDown { Minimum = 1, Maximum = 300, Value = 15, Width = 60 };
    private readonly TextBox mensagem = new TextBox { Width = 210, MaxLength = 64, Text = "INTEGRA-COM-TESTE" };
    private readonly Button atualizar = Botao("Atualizar portas");
    private readonly Button conectar = Botao("Conectar");
    private readonly Button enviar = Botao("Enviar texto");
    private readonly Button pulsos = Botao("Enviar pulsos");
    private readonly Button parar = Botao("Parar pulsos");
    private readonly TextBox log = new TextBox { Dock = DockStyle.Fill, Multiline = true, ReadOnly = true, ScrollBars = ScrollBars.Vertical, Font = new Font("Consolas", 10) };
    private readonly Timer escuta = new Timer { Interval = 75 };
    private readonly Timer emissao = new Timer { Interval = 500 };
    private readonly byte[] bloco = new byte[64];
    private readonly byte[] entrada = new byte[1024];
    private SerialPort serial;
    private int blocosRestantes;
    private int blocosEnviados;

    [STAThread]
    private static void Main()
    {
        Application.EnableVisualStyles();
        Application.SetCompatibleTextRenderingDefault(false);
        Application.Run(new SerialIntegra());
    }

    private SerialIntegra()
    {
        Text = "ProService | CalAmp Integra USB–RS-232";
        StartPosition = FormStartPosition.CenterScreen;
        ClientSize = new Size(790, 460);
        MinimumSize = new Size(806, 499);
        Font = new Font("Segoe UI", 10);
        baud.SelectedItem = "9600";
        paridade.SelectedItem = "None";
        bits.SelectedItem = "8";
        fim.SelectedItem = "1";
        for (int i = 0; i < bloco.Length; i++) bloco[i] = 0x55;

        var controles = new FlowLayoutPanel { Dock = DockStyle.Top, Height = 122, Padding = new Padding(8) };
        controles.Controls.AddRange(new Control[] {
            Rotulo("COM"), porta, atualizar, Rotulo("Baud"), baud, Rotulo("Bits"), bits,
            Rotulo("Paridade"), paridade, Rotulo("Stop"), fim, conectar,
            Rotulo("Texto"), mensagem, enviar, Rotulo("Duração (s)"), duracao, pulsos, parar,
            Rotulo("A escuta começa ao conectar. Para os pulsos, meça o pino 3 em relação ao pino 5.")
        });
        controles.SetFlowBreak(conectar, true);
        controles.SetFlowBreak(parar, true);
        Controls.Add(log);
        Controls.Add(controles);

        atualizar.Click += (sender, args) => BuscarPortas();
        conectar.Click += (sender, args) => AlternarConexao();
        enviar.Click += (sender, args) => EnviarTexto();
        pulsos.Click += (sender, args) => IniciarPulsos();
        parar.Click += (sender, args) => EncerrarPulsos("Interrompido");
        escuta.Tick += (sender, args) => Ler();
        emissao.Tick += (sender, args) => EnviarBloco();
        FormClosed += (sender, args) =>
        {
            Desconectar();
            escuta.Dispose();
            emissao.Dispose();
        };
        BuscarPortas();
        AtualizarBotoes();
    }

    private static ComboBox Lista(int largura, params string[] itens)
    {
        var lista = new ComboBox { Width = largura, DropDownStyle = ComboBoxStyle.DropDownList };
        lista.Items.AddRange(itens);
        return lista;
    }

    private static Button Botao(string texto)
    {
        return new Button { Text = texto, AutoSize = true };
    }

    private static Label Rotulo(string texto)
    {
        return new Label { Text = texto, AutoSize = true, Margin = new Padding(4, 7, 0, 0) };
    }

    private void Registrar(string texto)
    {
        log.AppendText(DateTime.Now.ToString("HH:mm:ss") + "  " + texto + Environment.NewLine);
    }

    private void BuscarPortas()
    {
        string anterior = porta.Text;
        string[] nomes = SerialPort.GetPortNames();
        Array.Sort(nomes, StringComparer.OrdinalIgnoreCase);
        porta.Items.Clear();
        porta.Items.AddRange(nomes);
        if (porta.Items.Contains(anterior)) porta.SelectedItem = anterior;
        else if (nomes.Length == 1) porta.SelectedIndex = 0;
        Registrar(nomes.Length == 0 ? "Nenhuma porta COM encontrada." : "Portas encontradas: " + String.Join(", ", nomes));
    }

    private void AtualizarBotoes()
    {
        bool conectado = serial != null;
        bool transmitindo = emissao.Enabled;
        conectar.Text = conectado ? "Desconectar" : "Conectar";
        foreach (Control campo in new Control[] { porta, atualizar, baud, bits, paridade, fim })
            campo.Enabled = !conectado;
        enviar.Enabled = pulsos.Enabled = conectado && !transmitindo;
        parar.Enabled = transmitindo;
        duracao.Enabled = !transmitindo;
    }

    private void AlternarConexao()
    {
        if (serial != null) { Desconectar(); return; }
        if (porta.SelectedItem == null) { Registrar("Selecione a porta COM do adaptador."); return; }
        var candidata = new SerialPort(porta.Text, Int32.Parse(baud.Text),
            (Parity)Enum.Parse(typeof(Parity), paridade.Text), Int32.Parse(bits.Text),
            fim.Text == "1" ? StopBits.One : StopBits.Two)
        {
            Handshake = Handshake.None, DtrEnable = false, RtsEnable = false,
            ReadTimeout = 100, WriteTimeout = 250
        };
        try { candidata.Open(); }
        catch (Exception erro)
        {
            candidata.Dispose();
            Registrar("Falha ao abrir a porta: " + erro.Message);
            return;
        }
        serial = candidata;
        escuta.Start();
        AtualizarBotoes();
        Registrar(String.Format("{0} aberta a {1} baud, {2} bits, paridade {3}, {4} stop. Escutando.",
            porta.Text, baud.Text, bits.Text, paridade.Text, fim.Text));
    }

    private void Desconectar()
    {
        escuta.Stop();
        EncerrarPulsos("Interrompido");
        if (serial == null) return;
        try { serial.Dispose(); }
        catch (Exception erro) { Registrar("Falha ao fechar a porta: " + erro.Message); }
        serial = null;
        AtualizarBotoes();
        Registrar("Porta desconectada.");
    }

    private void Ler()
    {
        try
        {
            int disponiveis = Math.Min(serial.BytesToRead, entrada.Length);
            if (disponiveis == 0) return;
            int lidos = serial.Read(entrada, 0, disponiveis);
            if (lidos > 0) Registrar("RX " + lidos + " bytes | " + BitConverter.ToString(entrada, 0, lidos) + " | " + TextoVisivel(entrada, lidos));
        }
        catch (TimeoutException) { }
        catch (Exception erro) { Registrar("Falha na leitura: " + erro.Message); Desconectar(); }
    }

    private static string TextoVisivel(byte[] dados, int quantidade)
    {
        var texto = new StringBuilder(quantidade);
        for (int i = 0; i < quantidade; i++)
            texto.Append(dados[i] >= 32 && dados[i] <= 126 ? (char)dados[i] : '.');
        return texto.ToString();
    }

    private bool ConfirmarEnvio()
    {
        return MessageBox.Show(this, "O CalAmp pode transmitir por RF. Confirma que a antena adequada está conectada e que a ponta remota está livre de equipamentos de controle?",
            "Confirmar envio", MessageBoxButtons.YesNo, MessageBoxIcon.Question, MessageBoxDefaultButton.Button2) == DialogResult.Yes;
    }

    private void EnviarTexto()
    {
        if (mensagem.Text.Length == 0) { Registrar("Digite uma mensagem de teste."); return; }
        foreach (char caractere in mensagem.Text)
            if (caractere < 32 || caractere > 126) { Registrar("Use texto ASCII sem acentos."); return; }
        if (!ConfirmarEnvio()) return;
        try
        {
            byte[] dados = Encoding.ASCII.GetBytes(mensagem.Text + "\r\n");
            serial.Write(dados, 0, dados.Length);
            Registrar("TX " + dados.Length + " bytes | " + BitConverter.ToString(dados));
        }
        catch (Exception erro) { Registrar("Falha no envio: " + erro.Message); Desconectar(); }
    }

    private void IniciarPulsos()
    {
        if (!ConfirmarEnvio()) return;
        blocosRestantes = (int)duracao.Value * 2;
        blocosEnviados = 0;
        Registrar("Pulsos iniciados: blocos de 64 bytes (0x55), a cada 500 ms.");
        emissao.Start();
        AtualizarBotoes();
        EnviarBloco();
    }

    private void EnviarBloco()
    {
        try
        {
            serial.Write(bloco, 0, bloco.Length);
            blocosEnviados++;
            if (--blocosRestantes == 0) EncerrarPulsos("Concluído");
        }
        catch (Exception erro) { Registrar("Falha no envio: " + erro.Message); Desconectar(); }
    }

    private void EncerrarPulsos(string resultado)
    {
        if (!emissao.Enabled) return;
        emissao.Stop();
        Registrar(String.Format("{0}: {1} blocos, {2} bytes escritos na COM.", resultado, blocosEnviados, blocosEnviados * bloco.Length));
        AtualizarBotoes();
    }
}
