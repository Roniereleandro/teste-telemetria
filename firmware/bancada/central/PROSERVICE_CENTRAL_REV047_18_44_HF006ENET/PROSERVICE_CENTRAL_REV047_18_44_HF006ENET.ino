/* REVISAO 47.18.34 */
#ifndef PROSERVICE_VERBOSE_SERIAL
#define PROSERVICE_VERBOSE_SERIAL 0
#endif

#include <WiFi.h>
#include <ETH.h>
#include "UplinkPolicy.h"
#include "CommandPolicy.h"
#include "AngularTargetPolicy.h"
#include "UplinkHealthPolicy.h"
#include "PortalResponsePolicy.h"
#include "PortalBodyPolicy.h"
#include "MaintenancePolicy.h"
#include "RadioProtocolPolicy.h"
#include <utility>
#ifndef PROSERVICE_BENCH_BUILD
#define PROSERVICE_BENCH_BUILD 0
#endif
#include <esp_wifi.h>
#include "PortalLatencyPolicy.h"
#include "PortalRecoveryPolicy.h"
#include <atomic>
#include "FieldControl.h"
#include <WebServer.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <TinyGPSPlus.h>
#include <math.h>
#include <esp_task_wdt.h>
#include <esp_idf_version.h>
#include <esp_system.h>
#include <Wire.h>
#include <RTClib.h>
#include <algorithm>
#include <time.h>
#include <sys/time.h>
#include <soc/gpio_struct.h>

// ======================================================
// AJUSTES GERAIS
// ======================================================

// Entradas:
// chegou tensão = entrada ativa/liberada
const uint8_t EMERGENCY_ACTIVE_LEVEL = LOW;
const uint8_t PRESSURE_ACTIVE_LEVEL  = HIGH;

// Saídas:
// HF-006 active HIGH — relé fecha com GPIO HIGH
const uint8_t OUTPUT_ACTIVE_LEVEL    = HIGH;

// Pinos ESP32 - entradas confirmadas
const int PIN_EMERGENCY_1 = 36; // SVP -> entrada 1
const int PIN_EMERGENCY_2 = 39; // SVN -> entrada 2
const int PIN_PRESSURE    = 34; // pressostato / segurança de pressão (pinagem REV.37)

// Saídas reais mapeadas por teste
const int PIN_OUT_RIGHT      = 4;   // relé 1 - avanço/direita
const int PIN_OUT_LEFT       = 16;  // relé 2 - reversão/esquerda
const int PIN_OUT_PUMP       = 17;  // relé 3 - bomba
const int PIN_OUT_LAST_TOWER = 18;  // relé 4 - última torre
const int PIN_OUT_CANNON     = 19;  // relé 5 - canhão
const int PIN_OUT_AUX6       = 23;  // relé 6 - liberação/placa pronta durante operação
const uint32_t OUTPUT_GPIO_MASK =
    (1UL << PIN_OUT_RIGHT) | (1UL << PIN_OUT_LEFT) |
    (1UL << PIN_OUT_PUMP) | (1UL << PIN_OUT_LAST_TOWER) |
    (1UL << PIN_OUT_CANNON) | (1UL << PIN_OUT_AUX6);

// Barramento I2C físico da ESP32
// DS3231 RTC + INA226 Detecta no mesmo barramento (SDA=21, SCL=22)
const int PIN_I2C_SDA = 21;
const int PIN_I2C_SCL = 22;

// DETECTA — INA226 via I2C (SDA=21, SCL=22) + MOSFET XY-MOS no GPIO 2
#define INA226_ADDR 0x40
const int PIN_DETECTA_MOSFET = 2;

const int PIN_CONFIG_BUTTON = 0;
const int PIN_GPS_RX = -1;
const int PIN_GPS_TX = -1;
const int PIN_LORA_RX = 25;
const int PIN_LORA_TX = 32;
const int PIN_LORA_M0 = 33;
const int PIN_LORA_M1 = 26;
const int PIN_LORA_AUX = 35;
const uint32_t GPS_BAUD_RATE = 9600;
const uint8_t E220_DEFAULT_CHANNEL = 18;
const uint8_t E220_MAX_CHANNEL = 80;

const char* DEFAULT_AP_PASS = "proservice123";
const char* PREF_NAMESPACE  = "pivotcfg";

const uint32_t WIFI_CONNECT_TIMEOUT_MS        = 20000;
const uint32_t WIFI_RETRY_INTERVAL_MS         = 15000;
const uint32_t COMMAND_FETCH_INTERVAL_MS      = PortalLatencyPolicy::POLL_MS;
const uint32_t STATUS_POST_INTERVAL_MS        = 5000;
const uint32_t SAFETY_DELAY_MS                = 50;   // debounce rápido das emergências NF
const uint32_t MOVE_AFTER_PRESSURE_MS         = 10000;
const uint32_t CANNON_AFTER_PRESSURE_MS       = 30000;
const uint32_t CONFIG_BUTTON_HOLD_MS          = 6000;
const uint32_t BOOT_GUARD_MS                  = 60000;
const uint32_t AUTO_RESTART_MIN_OFF_MS        = 15UL * 60UL * 1000UL;
const uint32_t AUTO_RESTART_AUTH_FRESH_MS     = 15000UL;
const uint32_t SAFETY_STABLE_BEFORE_RUN_MS    = 1500;
const uint32_t SAFETY_PRESTART_TIMEOUT_MS     = 10000;
const uint32_t AP_RETRY_INTERVAL_MS           = 10000;
const uint32_t WIFI_ATTEMPT_TIMEOUT_MS       = 30000;
const uint32_t WIFI_STATUS_PRINT_INTERVAL_MS = 2000;

// Lógica da última torre:
// a ESP executa o percentual que vier do backend.
// Ciclo padrão corrigido para 60 segundos.
const uint32_t DEFAULT_LAST_TOWER_CYCLE_MS    = 60000;

// Endpoints
const char* DEFAULT_COMMAND_ROUTE   = "/api/pivot-command";
const char* DEFAULT_HEARTBEAT_ROUTE = "/api/telemetry/heartbeat";
const char* DEFAULT_SERVER_URL = "https://proservicemt.com.br";
const char* PRODUCT_BRAND = "PROSERVICE";
const char* PRODUCT_NAME = "Central Inteligente de Pivô";
#if PROSERVICE_BENCH_BUILD
const char* FIRMWARE_VERSION = "PROSERVICE-CENTRAL-REV.047.18.44-BANCADA-NAO-INSTALAR-EM-CAMPO";
#else
const char* FIRMWARE_VERSION = "PROSERVICE-CENTRAL-REV.047.18.44-HF006ENET-ETH-WIFI-AUTONOMIA-3H";
#endif
bool benchCloudPaused=PROSERVICE_BENCH_BUILD!=0;

// Raiz publica de confianca do portal. HTTPS sem validacao nao e permitido.
static const char PROSERVICE_ROOT_CA[] PROGMEM =
"-----BEGIN CERTIFICATE-----\n"
"MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw\n"
"TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh\n"
"cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4\n"
"WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu\n"
"ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY\n"
"MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc\n"
"h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+\n"
"0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U\n"
"A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW\n"
"T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH\n"
"B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC\n"
"B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv\n"
"KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn\n"
"OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn\n"
"jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw\n"
"qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI\n"
"rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV\n"
"HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq\n"
"hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL\n"
"ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ\n"
"3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK\n"
"NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5\n"
"ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur\n"
"TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC\n"
"jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc\n"
"oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq\n"
"4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA\n"
"mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d\n"
"emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=\n"
"-----END CERTIFICATE-----\n";


// ======================================================
// ESTRUTURAS
// ======================================================

struct DeviceConfig {
  String wifiSsid;
  String wifiPass;
  String serverUrl;
  bool backendEnabled = true;
  String commandRoute = DEFAULT_COMMAND_ROUTE;
  String heartbeatRoute = DEFAULT_HEARTBEAT_ROUTE;
  String pivotId;
  String deviceToken;
  String hardwareName;
  String gpsNodeId;

  // Perfil físico/local do pivô.
  // Estes valores ficam gravados na NVS do ESP32 e são diferentes em cada instalação.
  bool localProfileEnabled = true;
  double localCenterLat = 0.0;
  double localCenterLon = 0.0;
  float localRadiusMeters = 0.0f;
  float localAreaHectares = 0.0f;
  float localGpsCorrectionDeg = 0.0f;
  float localMinDepthMm = 0.0f;
  float localSafetyFactorPercent = 3.0f;
  uint32_t localLastTowerCycleMs = DEFAULT_LAST_TOWER_CYCLE_MS;
  int localPressureWaitSec = 240;
};

struct PivotCommand {
  String commandId = "";
  String mode = "sem_agua";
  String direction = "parado";
  String depth = "8";
  String stopDegree = "360";
  bool canhaoFinal = false;
  int pressureWaitSec = 240;
  unsigned long updatedAt = 0;

  // Campos vindos do backend
  float requestedDepthMm = 8.0f;
  float minDepthMm = 0.0f;
  float safetyFactorPercent = 3.0f;
  float lastTowerPercent = -1.0f; // -1 = backend não enviou
  uint32_t lastTowerCycleMs = DEFAULT_LAST_TOWER_CYCLE_MS;
  bool engineeringReady = false;
  bool depthClamped = false;

  // Geometria do pivô vinda do backend/portal
  double centerLat = 0.0;
  double centerLon = 0.0;
  float radiusMeters = 0.0f;
  float areaHectares = 0.0f;
  bool geometryReady = false;
  float gpsCorrectionDeg = 0.0f;
  bool peakEnabled = false;
  String peakStart = "";
  String peakEnd = "";
  uint8_t peakDaysMask = 0;
  bool restartAfterPower = false;
  uint32_t serverTimeEpoch = 0;
  bool requiresGps = false;
  bool noGpsMode = false;
};

struct CommandExecutionInfo {
  String commandId = "";
  String requestedDirection = "parado";
  String requestedMode = "sem_agua";
  String result = "none";       // received, accepted, executing, succeeded, rejected, failed, ignored
  String reason = "";
  uint32_t receivedAtMs = 0;
  uint32_t changedAtMs = 0;
  uint32_t completedAtMs = 0;
  uint32_t statusBootCounter = 0;
  uint32_t payloadHash = 0;
  bool outputsConfirmed = false;
};

// Registro compacto gravado somente nas transicoes de comando. Ele permite
// detectar uma operacao interrompida mesmo quando a energia cai com os reles
// ativos, sem gravar continuamente durante o controle.
struct OperationJournalInfo {
  bool active = false;
  String commandId = "";
  String direction = "parado";
  String mode = "sem_agua";
  uint32_t payloadHash = 0;
  uint32_t updatedAt = 0;
  bool restartAfterPower = false;
  String stage = "none";
};

struct AutoRestartInfo {
  bool required = false;
  bool pending = false;
  bool portalAuthorized = false;
  String commandId = "";
  uint32_t payloadHash = 0;
  uint32_t notBeforeMs = 0;
  uint32_t lastAuthorizationMs = 0;
  String status = "inativo";
};

// Formato binario compacto da fila persistente. Cada registro possui checksum
// proprio e e confirmado por leitura apos a gravacao. Isso reduz o uso da NVS
// e permite preservar uma hora de incidentes mesmo sem internet.
struct __attribute__((packed)) PersistentEventRecord {
  uint32_t magic = 0;
  uint32_t sequence = 0;
  uint32_t boot = 0;
  uint32_t timestamp = 0;
  uint32_t lastTimestamp = 0;
  uint32_t uptimeMs = 0;
  uint32_t gpsAgeMs = 0;
  uint32_t checksum = 0;
  uint16_t occurrenceCount = 0;
  uint8_t schemaVersion = 0;
  uint8_t priority = 0;
  uint8_t typeCode = 0;
  uint8_t commandResultCode = 0;
  uint8_t controllerStateCode = 0;
  uint8_t outputMask = 0;
  uint8_t flags = 0; // bit0 seguranca, bit1 pressao, bit2 GPS valido
  int8_t loraChannel = -1;
  char pivotId[16] = {0};
  char code[48] = {0};
  char message[96] = {0};
  char commandId[48] = {0};
};

static_assert(sizeof(PersistentEventRecord) <= 256,
              "Registro persistente de evento excedeu 256 bytes");

enum ControllerState {
  STATE_IDLE,
  STATE_WAIT_SAFETY,
  STATE_WAIT_PRESSURE,
  STATE_WAIT_START,
  STATE_RUNNING,
  STATE_WAIT_PEAK_END,
  STATE_FAULT_SAFETY,
  STATE_FAULT_LOW_PRESSURE,
  STATE_FAULT_GPS_TIMEOUT,
  STATE_FAULT_PORTAL_TIMEOUT,
  STATE_FAULT_NO_MOVEMENT,
  STATE_BOOT_HOLD,
  STATE_FAULT_PERSISTENCE
};

// ======================================================
// GLOBAIS
// ======================================================

Preferences prefs;
WebServer server(80);
HardwareSerial GPSSerial(2);
HardwareSerial LoRaSerial(1);   // RX=25, TX=32

// LoRa GPS (fonte principal quando disponivel)
double  loraGpsLat       = 0.0;
double  loraGpsLon       = 0.0;
bool    loraGpsValid     = false;
uint32_t loraLastRxMs    = 0;
uint32_t loraLastTxMs    = 0;
bool     loraConfigTxPending = false;
uint32_t loraConfigTxDueMs = 0;
String  loraLastSeenId   = "";
bool    loraLastPacketAccepted = false;
String  loraRejectReason = "";
uint32_t loraIgnoredPackets = 0;
uint32_t loraRxByteCount = 0;
uint32_t loraRxLineCount = 0;
uint32_t loraJsonErrorCount = 0;
uint32_t loraLastAnyRxMs = 0;
uint32_t loraLastAnyTxMs = 0;
String  loraLastRawPacket = "";
String  loraLastJsonError = "";
String  loraLastTxPacket = "";

// Recepcao LoRa em task dedicada: captura UART mesmo durante HTTP/Wi-Fi
struct LoRaRxFrame {
  uint32_t receivedMs;
  char data[512];
};
QueueHandle_t loraRxQueue = nullptr;
TaskHandle_t loraRxTaskHandle = nullptr;
volatile uint32_t loraRxQueueDrops = 0;
volatile uint32_t loraRxTaskCycles = 0;
volatile bool loraRxTaskRunning = false;
std::atomic<bool> loraConfigActive{false};
std::atomic<bool> loraRxPaused{false};
uint8_t e220Channel = E220_DEFAULT_CHANNEL;
int e220ReadChannel = -1;
uint32_t e220ReadMs = 0;
uint32_t loraAcceptedMs = 0;
uint32_t loraAcceptedSinceCheck = 0;
int loraReportedChannel = -1;
String e220ConfigStatus = "Ainda nao verificado";

// HF-006Enet REV0: pinagem confirmada no exemplo oficial Hoffer TESTE_HF006enet_2026.
// W5500 nativo ESP-IDF/lwIP: HTTPS e WebServer compartilham a pilha com Wi-Fi.
const int ETH_SCK_PIN=14, ETH_MISO_PIN=12, ETH_MOSI_PIN=13;
const int ETH_CS_PIN=15, ETH_RESET_PIN=27;
bool ethernetInitialized=false;
UplinkHealthPolicy::State uplinkHealth;
void serviceControlBeforeNetwork();
UplinkPolicy::Transport activeUplink=UplinkPolicy::NONE;
uint32_t lastNtpAttemptMs=0;
void resetPortalConnection();
bool tlsClockValid(); // declaracao antecipada para o servico NTP de ambas as redes

bool ethernetReady() { return ethernetInitialized && ETH.connected() && ETH.hasIP(); }
bool networkReady() {
  return (activeUplink==UplinkPolicy::CABLE && ethernetReady()) ||
         (activeUplink==UplinkPolicy::WIFI && WiFi.status()==WL_CONNECTED && WiFi.STA.hasIP());
}
const char* uplinkName() {
  return activeUplink==UplinkPolicy::CABLE ? "Ethernet" :
         activeUplink==UplinkPolicy::WIFI ? "Wi-Fi" : "Sem conexao";
}
void setupEthernet() {
  ETH.setPollPeriod(10); // placa nao fornece IRQ nos cinco sinais documentados
  ethernetInitialized=ETH.begin(ETH_PHY_W5500,0,ETH_CS_PIN,-1,ETH_RESET_PIN,
                                SPI2_HOST,ETH_SCK_PIN,ETH_MISO_PIN,ETH_MOSI_PIN,20);
  if (ethernetInitialized) ETH.setHostname("proservice-central");
  Serial.println(ethernetInitialized ? "[ETH] W5500 iniciado; aguardando cabo/DHCP" :
                                      "[ETH] Falha ao iniciar W5500; Wi-Fi disponivel como alternativa");
}
void processNetworkTransport() {
  UplinkPolicy::Transport next=uplinkHealth.choose(millis(),ethernetReady(),WiFi.status()==WL_CONNECTED && WiFi.STA.hasIP());
  bool routeOk=true;
  if(next==UplinkPolicy::CABLE && !ETH.isDefault()) routeOk=ETH.setDefault();
  if(next==UplinkPolicy::WIFI && !WiFi.STA.isDefault()) routeOk=WiFi.STA.setDefault();
  if(!routeOk) next=UplinkPolicy::NONE;
  if(next!=activeUplink) {
    resetPortalConnection();
    activeUplink=next;
    uplinkHealth.commit(next,millis());
    Serial.printf("[REDE] Saida para o portal: %s\n",uplinkName());
    lastNtpAttemptMs=0;
    // Nao renova lastServerOkMs: somente resposta HTTP valida comprova o portal.
  }
  if(networkReady() && !tlsClockValid() &&
     (lastNtpAttemptMs==0 || millis()-lastNtpAttemptMs>=30000UL)) {
    configTime(0,0,"pool.ntp.org","time.google.com","time.cloudflare.com");
    lastNtpAttemptMs=millis();
  }
}
void addNetworkStatus(JsonDocument& doc) {
  doc["networkConnected"]=networkReady();
  doc["networkTransport"]=uplinkName();
  doc["ethernetInitialized"]=ethernetInitialized;
  doc["ethernetLink"]=ethernetInitialized && ETH.connected();
  doc["ethernetConnected"]=ethernetReady();
  doc["ethernetIp"]=ethernetReady()?ETH.localIP().toString():String("");
}

// Portal offline latch (Missao 11)
uint32_t lastServerOkMs       = 0;
bool     portalOfflineLatch   = false;

// Diagnóstico e conexão Wi-Fi não bloqueante
bool     wifiAttemptActive       = false;
uint32_t wifiAttemptStartedMs     = 0;
uint32_t wifiLastStatusPrintMs    = 0;
wl_status_t wifiPreviousStatus    = WL_NO_SHIELD;
String   wifiDiagnosticMessage   = "Aguardando configuração";
String   wifiLastFailure         = "";
int      wifiSavedNetworkRssi    = -127;
int      wifiSavedNetworkChannel = 0;
String   wifiSavedNetworkSecurity = "desconhecida";
String cachedWifiOptions;bool wifiScanRequested=false,wifiScanActive=false,wifiScanAttempted=false;
uint32_t wifiScanStartedMs=0,wifiLastScanMs=0;

// GPS failsafe (Missao 9)
uint32_t gpsFailsafeAlertedMs = 0;

// ======================================================
// DETECTA — identificação de torre em falha por impedância
// ======================================================
#if 0 // Detecta/INA226 não pertence a esta central; removido da compilação.
INA226_WE ina226(INA226_ADDR);
bool     detectaInaOk          = false;
bool     detectaStartReq       = false;
bool     detectaCalibrateReq   = false;  // leitura manual de impedância
bool     detectaRunning        = false;
float    detectaResultMetros   = 0.0f;
int      detectaResultTorre    = 0;
float    detectaRawOhms        = 0.0f;   // último valor bruto em ohms (calibração)
float    detectaRawMetros      = 0.0f;   // último valor bruto em metros (calibração)
String   detectaStatus         = "idle";
String   detectaMensagem       = "";
uint32_t detectaStartedMs      = 0;

// Config vinda do CMD (portal)
float    detectaResistPorMetro = 0.035f; // Ω/m padrão
int      detectaNumTorres      = 0;
float    detectaTorreInicio[25];
float    detectaTorreFim[25];
#endif

#define LORA_GPS_FAILSAFE_MS  1500000UL  // 25 min
#define PORTAL_OFFLINE_MS     10800000UL // 3 h de autonomia sem internet
#define LORA_CONFIG_MIN_GAP_MS 7000UL    // intervalo minimo entre respostas
#define LORA_CONFIG_REPLY_DELAY_MS 350UL // responde fora da janela TX do GPS
#define GPS_RETENTION_MS      1500000UL  // 25 min
#define GPS_OPERATION_TIMEOUT_MS 300000UL // 5 min: limite operacional sem amostra/qualidade
#define GPS_PACKET_MAX_AGE_MS 30000UL     // pacote ok=1 mais antigo nao renova posicao
#define MOVEMENT_SAMPLE_FRESH_MS 60000UL  // evidencia recente para declarar falta de movimento
#define MOVEMENT_MIN_SATELLITES 5U
#define MOVEMENT_MAX_HDOP 4.0
#define NO_MOVEMENT_TIMEOUT_MS 600000UL  // 10 min sem movimento
#define NO_MOVEMENT_GRACE_MS  120000UL   // 2 min
#define COMMAND_GUARD_MS      20000UL    // 20 s
#define HTTP_CONNECT_TIMEOUT_MS 3000
#define HTTP_TIMEOUT_MS       5000
#define HEARTBEAT_FORCE_MIN_INTERVAL_MS 250UL
#define PORTAL_FAILURE_CONFIRM_MS 60000UL // não declarar queda por oscilação curta
#define PORTAL_FAILURE_MIN_COUNT 3U       // exige falhas consecutivas
#define PORTAL_RECOVERY_MIN_COUNT 3U      // exige respostas consecutivas
#define PORTAL_FAILURE_SAMPLE_MIN_MS 1000UL
#define PORTAL_STALE_RECONNECT_MS 300000UL // 5 min sem portal OK antes de reiniciar Wi-Fi
#define WIFI_FORCE_RECONNECT_COOLDOWN_MS 300000UL

TinyGPSPlus gps;

DeviceConfig cfg;
PivotCommand currentCmd;
volatile ControllerState controllerState = STATE_BOOT_HOLD;
TaskHandle_t safetyTaskHandle = nullptr;
volatile bool fastOutputInhibit = false;
volatile bool fastSafetyTripPending = false;
volatile bool fastPressureTripPending = false;
volatile bool fastPressureProtectionArmed = false;
volatile bool urgentSafetyHeartbeatPending = false;
volatile bool safetyInterruptsConfigured = false;
volatile uint32_t fastSafetyLowSinceMs = 0;
volatile uint32_t fastPressureLowSinceMs = 0;
volatile uint32_t fastSafetyCutCount = 0;
volatile uint32_t fastPressureCutCount = 0;
volatile uint32_t fastEmergencyIsrCutCount = 0;
volatile uint32_t fastPressureIsrCutCount = 0;
uint32_t highestPortalCommandUpdatedAt = 0;
String highestPortalCommandId = "";
uint32_t stalePortalCommandRejectCount = 0;
bool ntpSyncRequested = false;

bool configPortalActive = false;
bool webServerStarted = false;

// Controle local pela pagina web
bool localTestMode = false;
bool localDirectMode = false;
bool localRelayRight = false;
bool localRelayLeft = false;
bool localRelayPump = false;
bool localRelayLastTower = false;
bool localRelayCannon = false;
unsigned long localLastActivityMs = 0;
const uint32_t LOCAL_TEST_TIMEOUT_MS = 15UL * 60UL * 1000UL;

unsigned long lastWifiAttemptMs = 0;
unsigned long lastFetchMs = 0;
unsigned long lastStatusMs = 0;
unsigned long lastApRetryMs = 0;
unsigned long safetyLostSinceMs = 0;
unsigned long pressureWaitStartedMs = 0;
unsigned long pressureDetectedMs = 0;
unsigned long moveStartedMs = 0;
unsigned long configButtonPressedMs = 0;
unsigned long bootStartedMs = 0;
unsigned long safetyStableSinceMs = 0;
unsigned long safetyPrestartStartedMs = 0;

String faultMessage = "";
String alertMessage = "";

// O histórico detalhado duplica informações que também estão na fila
// estruturada. Quatro posições evitam esgotar a partição NVS de 20 KB; os
// últimos motivos de parada/falha e seus contadores continuam separados.
const uint8_t DIAG_HISTORY_SLOTS = 4;
const uint8_t LEGACY_DIAG_HISTORY_SLOTS = 12;
String diagHistory[DIAG_HISTORY_SLOTS];
uint32_t diagEventCounter = 0;
uint16_t diagDirtySlotsMask = 0;
bool diagTotalDirty = false;
String lastStopReason = "NENHUMA PARADA REGISTRADA";
String lastStopDetail = "";
String lastFaultReason = "NENHUMA FALHA REGISTRADA";
String lastFaultDetail = "";
String lastResetReason = "DESCONHECIDO";
uint32_t stopEventCounter = 0;
uint32_t faultEventCounter = 0;
uint32_t bootCounter = 0;
uint32_t invalidPortalCommandCount = 0;
String lastInvalidPortalDirection = "";
uint32_t lastInvalidPortalEventMs = 0;
uint32_t lastCommandAppliedEventUpdatedAt = 0;
String lastCommandAppliedEventDirection = "";
String lastCommandAppliedEventId = "";
bool portalRequestFailureActive = false;
uint32_t lastPortalFailureEventMs = 0;
uint32_t portalFailureWindowStartedMs = 0;
uint32_t lastPortalFailureCountedMs = 0;
uint16_t portalConsecutiveFailures = 0;
uint8_t portalConsecutiveSuccesses = 0;
String portalLastFailureReason = "";
bool loraLinkFailureActive = false;
bool gpsSignalFailureActive = false;
bool loraChannelMismatchActive = false;
uint32_t lastLoggedLoraJsonErrors = 0;
uint32_t lastLoggedLoraIgnoredPackets = 0;
uint32_t lastLoraProtocolEventMs = 0;
const uint32_t EVENT_FAILURE_REPEAT_MS = 15UL * 60UL * 1000UL;

// Fila persistente de eventos estruturados. Nunca remove um evento apenas por
// ter sido enviado; a remoção exige ACK explícito do servidor pelo eventId.
// A NVS padrão possui 20 KB. Vinte registros deixam margem real para o journal,
// configurações, comandos e coleta de lixo da NVS. Dezesseis posições ficam
// reservadas a falhas/paradas/reset; eventos repetidos são consolidados.
const uint8_t EVENT_OUTBOX_SLOTS = 20;
const uint8_t LEGACY_EVENT_OUTBOX_SLOTS = 40;
const uint8_t EVENT_NORMAL_LIMIT = 4;
const uint8_t EVENT_CRITICAL_RESERVE = 16;
const uint8_t EVENT_BATCH_MAX = 4;
const uint8_t EVENT_SCHEMA_VERSION = 3;
const uint8_t EVENT_STORAGE_FORMAT = 3;
const uint32_t EVENT_RECORD_MAGIC = 0x50534556UL; // "PSEV"
const uint8_t COMMAND_ID_MAX_LENGTH = 96;
const uint8_t OPERATION_JOURNAL_WRITE_ATTEMPTS = 3;
PersistentEventRecord eventOutbox[EVENT_OUTBOX_SLOTS];
uint8_t eventOutboxCount = 0;
uint32_t eventSequence = 0;
uint32_t eventOutboxOverflowCount = 0;
uint32_t eventOutboxDroppedNormalCount = 0;
uint32_t eventOutboxCriticalOverflowCount = 0;
bool eventOutboxDirty = false;
bool commandExecutionDirty = false;
uint32_t commandExecutionRevision = 0;
uint32_t lastPostedCommandExecutionRevision = 0;
bool operationJournalDirty = false;
bool operationJournalStartInterlock = false;
uint32_t operationJournalGeneration = 0;
uint32_t operationJournalRepairCount = 0;
String operationJournalInterlockReason = "";
bool persistenceHealthy = true;
bool persistenceFaultLatched = false;
bool persistenceRecoveryEventPending = false;
bool persistenceRecoveryEventInProgress = false;
uint32_t persistenceFailureCount = 0;
uint32_t persistenceLastFailureMs = 0;
uint32_t persistenceLastSuccessMs = 0;
uint32_t persistenceLastRetryMs = 0;
String persistenceLastError = "";
CommandExecutionInfo commandExecution;
OperationJournalInfo operationJournal;
AutoRestartInfo autoRestart;
PivotCommand autoRestartCommand;

// Confirmação ponta a ponta do comando recebido do portal. commandId + hash
// impedem repetição ou alteração silenciosa da intenção original.
bool portalCommandReceived = false;
uint32_t lastPortalCommandReceivedMs = 0;
uint32_t lastPortalCommandUpdatedAt = 0;
uint32_t lastPortalCommandPayloadHash = 0;
String lastPortalCommandDirection = "";
String lastPortalCommandMode = "";
String lastPortalCommandId = "";
bool commandIdConflictActive = false;
uint32_t commandIdConflictCount = 0;
String commandIdConflictMessage = "";

void recordStopEvent(const String& reason, const String& detail = "");
void recordFaultEvent(const String& reason, const String& detail = "");
void appendDiagnosticHistory(const String& type, const String& reason, const String& detail = "");
String buildSystemEventLog();
void loadStructuredEventOutbox();
void flushStructuredEventOutboxIfSafe();
void loadCommandExecution();
void loadOperationJournal();
bool writeOperationJournalRecord();
bool flushOperationJournalIfNeeded();
bool armOperationJournal(const PivotCommand& cmd, const String& stage);
bool clearOperationJournal(const String& stage);
void enterPersistenceFault(const String& detail);
void recoverInterruptedOperation();
void cancelPendingCommand(const String& reason, bool logEvent = true);
bool pendingCommandStillAuthorized();
void cancelAutoRestart(const String& reason, bool logEvent = true);
bool commandMatchesAutoRestart(const PivotCommand& cmd);
void authorizeAutoRestart(const PivotCommand& cmd);
void processAutoRestart();
void enqueueStructuredEvent(const String& type, const String& code, const String& message);
uint8_t addStructuredEventsToHeartbeat(DynamicJsonDocument& doc);
void processStructuredEventAcks(const String& response);
uint8_t structuredEventPriority(const String& type, const String& code);
void markPersistenceFailure(const String& component, const String& detail);
void markPersistenceSuccess();
void setCommandExecution(const String& commandId, const String& direction,
                         const String& mode, const String& result,
                         const String& reason, bool completed = false);
void persistCommandExecutionIfSafe();
void flushDiagnosticHistoryIfSafe();
void servicePersistenceRetries();
void notePortalCommunicationFailure(const String& reason);
void notePortalCommunicationRecovered();
void trackCommandAppliedEvent();
void trackLoraCommunicationEvent();
void processBoardReadyRelay();
String normalizeModeStr(const String& value);
String normalizeDirectionStr(const String& value);
uint32_t calculateCommandPayloadHash(const PivotCommand& cmd);
bool commandExecutionIsTerminal();
bool allControlledOutputsAreOff();
void setupFastSafetyTask();
void processFastSafetyTrips();
void clearFastSafetyLatch();
void enterSafetyFault();
void enterLowPressureFault();
void processUrgentSafetyHeartbeat();
void loadCommandOrderGuard();
bool validateAndPersistCommandOrder(const PivotCommand& cmd);
void syncSystemClock(uint32_t epoch);
bool tlsClockValid();

String lastPostedControllerState = "";
String lastPostedFaultMessage = "";
String lastPostedAlertMessage = "";
bool lastPostedPumpOn = false;
bool lastPostedRightOn = false;
bool lastPostedLeftOn = false;
bool lastPostedCannonOn = false;
bool lastPostedSafetyOk = false;
bool lastPostedPressureOk = false;
bool lastPostedCommandApplied = false;
uint32_t lastPostedCommandUpdatedAt = 0;
String lastPostedCommandAckStatus = "";
uint32_t lastPostedDiagEventCounter = 0;
float lastPostedLastTowerPercent = -9999.0f;
float lastPostedRequestedDepthMm = -9999.0f;
float lastPostedMinDepthMm = -9999.0f;

// GPS / posição
double gpsLat = 0.0;
double gpsLon = 0.0;
double gpsAltMeters = 0.0;
double gpsCourseDeg = 0.0;
double gpsSpeedKmph = 0.0;
double gpsHdop = 0.0;
uint32_t gpsSatellites = 0;
bool gpsValid = false;
unsigned long gpsLastFixMs = 0;
double pivotAngleDeg = 0.0;
double distanceFromCenterM = 0.0;
bool pivotAngleValid = false;


// GPIO23 não pulsa: abre parado/falha e fecha continuamente somente rodando.
bool boardReadyRelayState = false;

// Último GPS bom para retenção no portal
double lastGoodGpsLat = 0.0;
double lastGoodGpsLon = 0.0;
double lastGoodPivotAngleDeg = 0.0;
double lastGoodDistanceFromCenterM = 0.0;
bool lastGoodGpsAvailable = false;
bool lastGoodPivotAngleValid = false;
bool loraSampleSeen = false;
uint32_t lastLoraSample = 0;
uint32_t lastLoraSampleSeq = 0;
unsigned long lastGoodGpsMs = 0;
// Estado da verificacao angular: somente amostras novas e recentes.
uint32_t angularCheckedGeneration = 0;
bool angularPreviousValid = false;
double angularPreviousDeg = 0.0;
uint32_t angularPreviousMs = 0;
uint8_t angularNearCount = 0;
void resetAngularArrivalTracking() {
  angularCheckedGeneration = 0;
  angularPreviousValid = false;
  angularPreviousMs = 0;
  angularNearCount = 0;
}


// Monitor de movimento (operações com GPS)
double movementRefLat = 0.0;
double movementRefLon = 0.0;
double movementRefAngleDeg = 0.0;
bool movementRefValid = false;
unsigned long movementMonitorStartedMs = 0;
unsigned long lastMovementMs = 0;

// REV047.6 - monitor conservador; GPS comum nao comprova movimento mecanico.
struct MovementGuard {
  static constexpr uint32_t GRACE_MS = 120000UL;
  static constexpr uint32_t TIMEOUT_MS = 600000UL;
  static constexpr uint32_t CONFIRM_MS = 10000UL;
  static constexpr double MIN_PROGRESS_M = 2.0; // progresso sustentado; ainda exige 5 amostras e 10 s
  bool active=false, rawKnown=false, referenceKnown=false, candidate=false, confirmed=false;
  uint32_t started=0, lastConfirmed=0, lastInput=0, candidateSince=0, confirmations=0;
  unsigned count=0, index=0, candidateCount=0;
  double window[9]={}, rawAngle=0, unwrapped=0, radius=0, reference=0, progress=0;
  bool recovering=false;
  unsigned recoveryCount=0;
  uint32_t recoveryStarted=0, recoveryLast=0, rejectedSamples=0, recoveries=0;
  double recoveryAngle=0, recoveryRadius=0;
  const char* reason="Parado";
  void begin(uint32_t now) { *this=MovementGuard(); active=true; started=now; reason="Tolerancia inicial"; }
  uint32_t remaining(uint32_t now) const {
    if (!active) return 0;
    uint32_t elapsed=now-started;
    if (elapsed<GRACE_MS) return GRACE_MS-elapsed+TIMEOUT_MS;
    uint32_t quiet=confirmed ? now-lastConfirmed : elapsed-GRACE_MS;
    return quiet>=TIMEOUT_MS ? 0 : TIMEOUT_MS-quiet;
  }
  bool expired(uint32_t now) const { return active && remaining(now)==0; }
  void reject(const char* why) { candidate=false; candidateCount=0; count=0; index=0; reason=why; ++rejectedSamples; }
  // Reancorar exige uma sequencia coerente; nunca renova lastConfirmed.
  void recover(uint32_t now,double angle,double radial) {
    if (!recovering || now-recoveryLast>15000UL) {
      recovering=true; recoveryCount=1; recoveryStarted=now;
      recoveryLast=now; recoveryAngle=angle; recoveryRadius=radial;
      reason="Recuperando referencia; prazo continua"; return;
    }
    if (now-recoveryLast<1000UL) return;
    double arc=fabs(delta(angle,recoveryAngle))*0.017453292519943295*recoveryRadius;
    if (arc>2.0 || fabs(radial-recoveryRadius)>2.0) {
      recoveryCount=1; recoveryStarted=now;
    } else ++recoveryCount;
    recoveryLast=now; recoveryAngle=angle; recoveryRadius=radial;
    reason="Recuperando referencia; prazo continua";
    if (recoveryCount>=9 && now-recoveryStarted>=10000UL) {
      rawKnown=true; rawAngle=angle; radius=radial; lastInput=now;
      unwrapped=0; reference=0; progress=0; referenceKnown=false;
      count=0; index=0; candidate=false; candidateCount=0;
      recovering=false; ++recoveries;
      // Recuperar a referencia nao reinicia o prazo de movimento.
      reason="Referencia recuperada sem confirmar movimento";
    }
  }
  static double delta(double a,double b) {
    double d=fmod(a-b+540.0,360.0)-180.0;
    return d;
  }
  // Retorna true somente quando existe uma janela operacional completa de
  // 9 amostras coerentes. Recuperacao parcial e formacao inicial do filtro
  // nunca podem renovar o prazo de qualidade operacional.
  bool sample(uint32_t now,double angle,double radial,int direction,bool quality) {
    if (!quality || !isfinite(angle) || !isfinite(radial) || radial<10.0) {
      recovering=false;
      reject("Amostra sem qualidade/geometria; prazo continua"); return false;
    }
    if (recovering) {
      recover(now,angle,radial);
      return false;
    }
    if (rawKnown && now-lastInput<1000UL) return false;
    if (rawKnown && now-lastInput>15000UL) {
      reject("Intervalo entre amostras; prazo continua");
      recover(now,angle,radial);
      return false;
    }
    if (!rawKnown) { rawKnown=true; rawAngle=angle; radius=radial; lastInput=now; }
    double step=delta(angle,rawAngle)*0.017453292519943295*radius;
    if (fabs(step)>10.0 || fabs(radial-radius)>10.0) {
      reject("Salto GPS rejeitado; prazo continua");
      recover(now,angle,radial); return false;
    }
    unwrapped+=step*direction; rawAngle=angle; lastInput=now;
    radius=radial; // salto radial comparado com a ultima amostra aceita, nao com a partida

    window[index]=unwrapped; index=(index+1)%9;
    if (count<9) ++count;
    if (count<9) {reason="Formando filtro de 9 amostras; prazo continua"; return false;}
    double sorted[9]; for(unsigned i=0;i<9;++i) sorted[i]=window[i];
    for(unsigned i=1;i<9;++i) {double v=sorted[i];unsigned j=i;while(j && sorted[j-1]>v){sorted[j]=sorted[j-1];--j;} sorted[j]=v;}
    double filtered=sorted[4];
    if (!referenceKnown) {reference=filtered; referenceKnown=true;}
    progress=filtered-reference;
    if (now-started<GRACE_MS) {candidate=false; candidateCount=0; reason="Tolerancia inicial"; return true;}
    if (progress<MIN_PROGRESS_M) {candidate=false;candidateCount=0;reason="Sem avanco confirmado";return true;}
    if (!candidate) {candidate=true;candidateSince=now;candidateCount=1;reason="Confirmando avanco por 10 s";return true;}
    ++candidateCount;
    if (candidateCount>=5 && now-candidateSince>=CONFIRM_MS) {
      reference=filtered; progress=0; confirmed=true;lastConfirmed=now;++confirmations;
      candidate=false;candidateCount=0;reason="Avanco confirmado";
    }
    return true;
  }
};
MovementGuard movementGuard;
uint32_t gpsSampleGeneration=0, movementLastGeneration=0;
uint32_t loraQueueAgeMs=0, loraDuplicateSamples=0, loraOldSamples=0;
uint32_t peerBoot=0;
bool peerBootKnown=false;
uint32_t movementQualityRejects=0, movementAgeRejects=0, movementGeometryRejects=0;
uint32_t movementLastUsableSampleMs=0;


// Guarda de comandos
PivotCommand pendingCmd;
bool pendingCmdValid = false;
unsigned long lastCommandAcceptedMs = 0;
unsigned long lastAcceptedUpdatedAt = 0;

// Tempo de servidor para horário de ponta
uint32_t serverSyncEpoch = 0;
unsigned long serverSyncMillis = 0;

// RTC físico DS3231
RTC_DS3231 rtc;
bool rtcPresent = false;
bool rtcTimeValid = false;
bool rtcLostPowerFlag = false;
const uint32_t RTC_MIN_VALID_EPOCH = 1704067200UL; // 2024-01-01 00:00:00

// HTTP
bool httpInFlight = false;
// Uma unica sessao TLS, usada sequencialmente no loop (sem nova task/estado compartilhado).
WiFiClientSecure portalSecureClient;
// Parser HTTP persistente; cabecalhos sao limpos entre transacoes.
HTTPClient portalHttpClient;
String portalTlsOrigin;
int portalTlsNetwork=-1;
uint32_t portalTlsIp=0, portalTlsOpenedMs=0, portalTlsUsedMs=0;
uint32_t portalTlsReconnectAttempts=0;
uint32_t portalTlsReuseAttempts=0,portalTlsReuseFailures=0,portalKeepAliveFallbackStartedMs=0;
bool portalRequestReused=false,portalKeepAliveFallbackActive=false;
uint32_t portalGetAttempts=0, portalPostAttempts=0;
bool portalReceiptRecovered=false;
String portalGetOutcome="ainda_sem_consulta";
String portalGetRootKeys;
String portalPostReplyKeys;
bool portalPostReplySuccessKnown=false,portalPostReplySuccess=false;
void recordPortalPostReply(const String& response) {
  portalPostReplyKeys="";portalPostReplySuccessKnown=false;portalPostReplySuccess=false;
  DynamicJsonDocument reply(4096);
  if(deserializeJson(reply,response)) return;
  for(JsonPair kv:reply.as<JsonObject>()) {
    if(portalPostReplyKeys.length()>160) break;
    if(portalPostReplyKeys.length()) portalPostReplyKeys+=",";
    portalPostReplyKeys+=String(kv.key().c_str()).substring(0,32);
  }
  portalPostReplySuccessKnown=reply["success"].is<bool>();
  portalPostReplySuccess=reply["success"]|false;
}
uint32_t portalGetEmptyCount=0, portalGetJsonErrorCount=0, portalGetParsedCommandCount=0;
uint32_t portalGetItemCount=0;
bool canReportSavedCommandCancellation() {
  const bool outputsOff=!outputIsOnLogicLevel(PIN_OUT_RIGHT) && !outputIsOnLogicLevel(PIN_OUT_LEFT) &&
      !outputIsOnLogicLevel(PIN_OUT_PUMP) && !outputIsOnLogicLevel(PIN_OUT_LAST_TOWER) &&
      !outputIsOnLogicLevel(PIN_OUT_CANNON);
  PortalRecoveryPolicy::State state{
    !bootGuardActive(), controllerState==STATE_IDLE, outputsOff,
    !portalCommandReceived && !commandExecution.commandId.isEmpty() &&
      commandExecution.payloadHash!=0 && commandExecution.statusBootCounter!=bootCounter,
    highestPortalCommandId==commandExecution.commandId && highestPortalCommandUpdatedAt>0,
    !pendingCmdValid, !autoRestart.required && !autoRestart.pending,
    !localTestMode, persistenceHealthy && !persistenceFaultLatched && !operationJournalStartInterlock,
    portalGetOutcome=="items_vazio" && portalGetItemCount==0
  };
  return PortalRecoveryPolicy::canReportCancellation(state);
}

extern bool requireStopCommandAfterFault;
void handleReportSavedCommandCancellation() {
  if(!requireServiceAccess(false))return;
  if (!canReportSavedCommandCancellation() ||
      server.arg("commandId")!=commandExecution.commandId) {
    server.send(409,"text/plain","Recuperacao bloqueada: exige central parada, saidas desligadas, memoria valida e recibo anterior correspondente.");
    return;
  }
  // Recibo historico validado na NVS, nunca uma nova partida ou falso sucesso.
  portalCommandReceived=true; portalReceiptRecovered=true;
  lastPortalCommandId=commandExecution.commandId;
  lastPortalCommandPayloadHash=commandExecution.payloadHash;
  lastPortalCommandUpdatedAt=highestPortalCommandUpdatedAt;
  const bool journalMatches=operationJournal.commandId==commandExecution.commandId &&
      operationJournal.payloadHash==commandExecution.payloadHash;
  lastPortalCommandDirection=journalMatches?normalizeDirectionStr(operationJournal.direction):
      commandExecution.requestedDirection;
  lastPortalCommandMode=journalMatches?normalizeModeStr(operationJournal.mode):commandExecution.requestedMode;
  lastPortalCommandReceivedMs=0; // idade do recibo anterior desconhecida; nao autoriza partida pendente
  requireStopCommandAfterFault=true; // exigir novo PARAR remoto antes de outra partida
  faultMessage="Cancelamento local: envie PARAR para liberar nova partida";
  setCommandExecution(lastPortalCommandId,lastPortalCommandDirection,lastPortalCommandMode,
      "failed","Cancelamento local confirmado pelo operador; saidas desligadas; envie novo PARAR",true);
  appendDiagnosticHistory("COMANDO","CANCELAMENTO LOCAL DO RECIBO ANTERIOR",
      "commandId="+lastPortalCommandId+"; nenhuma saida acionada; exige novo PARAR");
  if(commandExecutionDirty || !persistenceHealthy) {
    server.send(503,"text/plain","Cancelamento registrado em RAM, mas persistencia nao confirmada. Mantenha a central parada.");
    return;
  }
  server.sendHeader("Location","/"); server.send(303);
}

void addCommandFetchDiagnostic(JsonDocument& doc) {
  doc["portalReceiptRecovered"]=portalReceiptRecovered;
  doc["portalLegacyReceiptAccepted"]=portalLegacyReceiptAccepted();
  const bool journalMatches=operationJournal.commandId==commandExecution.commandId &&
      operationJournal.payloadHash==commandExecution.payloadHash;
  doc["portalSavedReceiptFromJournal"]=journalMatches;
  doc["portalSavedReceiptDirection"]=journalMatches?operationJournal.direction:commandExecution.requestedDirection;
  doc["portalSavedReceiptMode"]=journalMatches?operationJournal.mode:commandExecution.requestedMode;
  doc["portalSavedReceiptCancellationAvailable"]=canReportSavedCommandCancellation();
  doc["portalCommandGetOutcome"]=portalGetOutcome;
  doc["portalCommandRootKeys"]=portalGetRootKeys;
  doc["portalPostReplyKeys"]=portalPostReplyKeys;
  doc["portalPostReplySuccessKnown"]=portalPostReplySuccessKnown;
  doc["portalPostReplySuccess"]=portalPostReplySuccess;
  doc["portalCommandItemCount"]=portalGetItemCount;
  doc["portalCommandEmptyCount"]=portalGetEmptyCount;
  doc["portalCommandJsonErrorCount"]=portalGetJsonErrorCount;
  doc["portalCommandParsedCount"]=portalGetParsedCommandCount;
  doc["portalCompatibilityMode"]="REV40_SAFE_KEEPALIVE_LEGACY_HEARTBEAT";
}
uint32_t portalLastGetDurationMs=0, portalMaxGetDurationMs=0;
uint32_t portalLastPostDurationMs=0, portalMaxPostDurationMs=0;
uint32_t portalLastGetFinishedMs=0, portalLastPostFinishedMs=0;
int portalLastGetCode=0, portalLastPostCode=0;
void resetPortalConnection() {
  portalSecureClient.stop();
  portalHttpClient.setAuthorization("");
  portalTlsOrigin="";
  portalTlsNetwork=-1;
}
struct PortalRequestScope {
  bool get;
  uint32_t started;
  int responseCode=0;
  int transportResponseCode=0;
  UplinkPolicy::Transport transport=activeUplink;
  explicit PortalRequestScope(bool isGet):get(isGet),started(millis()) {
    if(get) ++portalGetAttempts; else ++portalPostAttempts;
  }
  ~PortalRequestScope() {
    uplinkHealth.result(transport,transportResponseCode?transportResponseCode:responseCode);
    uint32_t now=millis(), elapsed=now-started;
    const int transportCode=transportResponseCode?transportResponseCode:responseCode;
    if(portalRequestReused && transportCode<0){
      ++portalTlsReuseFailures;portalKeepAliveFallbackActive=true;portalKeepAliveFallbackStartedMs=now;
      resetPortalConnection();
    }
    if(portalSecureClient.connected())portalTlsUsedMs=now;
    if(get) {
      portalLastGetDurationMs=elapsed; portalLastGetFinishedMs=now; portalLastGetCode=responseCode;
      if(elapsed>portalMaxGetDurationMs) portalMaxGetDurationMs=elapsed;
    } else {
      portalLastPostDurationMs=elapsed; portalLastPostFinishedMs=now; portalLastPostCode=responseCode;
      if(elapsed>portalMaxPostDurationMs) portalMaxPostDurationMs=elapsed;
    }
  }
};
void addPortalTimingStatus(JsonDocument& doc) {
  doc["commandPollIntervalMs"]=COMMAND_FETCH_INTERVAL_MS;
  doc["portalGetAttempts"]=portalGetAttempts;
  doc["portalPostAttempts"]=portalPostAttempts;
  doc["portalLastGetDurationMs"]=portalLastGetDurationMs;
  doc["portalLastPostDurationMs"]=portalLastPostDurationMs;
  doc["portalMaxGetDurationMs"]=portalMaxGetDurationMs;
  doc["portalMaxPostDurationMs"]=portalMaxPostDurationMs;
  doc["portalLastGetCode"]=portalLastGetCode;
  doc["portalLastPostCode"]=portalLastPostCode;
  doc["portalTlsReconnectAttempts"]=portalTlsReconnectAttempts;
  doc["portalTlsReuseAttempts"]=portalTlsReuseAttempts;
  doc["portalTlsReuseFailures"]=portalTlsReuseFailures;
  uint32_t fallbackElapsed=uint32_t(millis()-portalKeepAliveFallbackStartedMs);
  doc["portalKeepAliveFallbackRemainingMs"]=portalKeepAliveFallbackActive&&fallbackElapsed<60000UL?60000UL-fallbackElapsed:0;
  uint32_t elapsed=millis()-lastCommandAcceptedMs;
  doc["commandGuardRemainingMs"]=(pendingCmdValid && elapsed<COMMAND_GUARD_MS)?COMMAND_GUARD_MS-elapsed:0;
}
uint32_t lastWifiForceReconnectMs = 0;
bool requireStopCommandAfterFault = false;

// REV47.18.34: dados numericos publicados sob lock; a task nao le Strings do comando.
portMUX_TYPE towerPulseMux = portMUX_INITIALIZER_UNLOCKED;
bool towerPulseEnabled = false;
bool towerPulseDirect = false;
uint32_t towerPulseCycleMs = 60000;
uint32_t towerPulseOnMs = 0;
FieldControl::DirectionGuard directionGuard;
// Sessoes anteriores desta execucao nao podem voltar a renovar a posicao.
uint32_t retiredPeerBoots[16] = {};
uint8_t retiredPeerCount = 0;
uint8_t retiredPeerNext = 0;

void publishTowerPulse(bool enabled, bool direct, uint32_t cycleMs, uint32_t onMs) {
  portENTER_CRITICAL(&towerPulseMux);
  towerPulseEnabled = enabled;
  towerPulseDirect = direct;
  towerPulseCycleMs = cycleMs;
  towerPulseOnMs = onMs;
  if (!enabled) digitalWrite(PIN_OUT_LAST_TOWER, outputOffLevel());
  portEXIT_CRITICAL(&towerPulseMux);
}

void serviceTowerPulse() {
  portENTER_CRITICAL(&towerPulseMux);
  bool allowed = towerPulseEnabled && !fastOutputInhibit && !bootGuardActive() && safetyOk();
  allowed = allowed && (towerPulseDirect ?
      (localTestMode && localDirectMode) : (controllerState == STATE_RUNNING));
  if (!towerPulseDirect && fastPressureProtectionArmed && !pressureOk()) allowed = false;
  bool on = allowed && FieldControl::pulseOn(millis(), towerPulseCycleMs, towerPulseOnMs);
  digitalWrite(PIN_OUT_LAST_TOWER, on ? OUTPUT_ACTIVE_LEVEL : outputOffLevel());
  // Uma interrupcao pode cortar durante digitalWrite: respeitar o latch antes de sair.
  if (fastOutputInhibit) digitalWrite(PIN_OUT_LAST_TOWER, outputOffLevel());
  portEXIT_CRITICAL(&towerPulseMux);
}

void applyDirectionOutputs(bool rightOn, bool leftOn) {
  uint32_t now = millis();
  if (directionGuard.current != 0 &&
      !outputIsOnLogicLevel(PIN_OUT_RIGHT) && !outputIsOnLogicLevel(PIN_OUT_LEFT)) {
    directionGuard.request(now, 0); // houve corte fisico pela protecao rapida
  }
  int desired = rightOn == leftOn ? 0 : (rightOn ? 1 : -1);
  if (desired && (fastOutputInhibit || bootGuardActive() || !safetyOk())) desired = 0;
  int applied = directionGuard.request(now, desired);
  // Sempre abre o sentido oposto antes de fechar o escolhido.
  if (applied != 1) writeOutputIfChanged(PIN_OUT_RIGHT, false);
  if (applied != -1) writeOutputIfChanged(PIN_OUT_LEFT, false);
  if (applied == 1 && !fastOutputInhibit) writeOutputIfChanged(PIN_OUT_RIGHT, true);
  if (applied == -1 && !fastOutputInhibit) writeOutputIfChanged(PIN_OUT_LEFT, true);
  if (fastOutputInhibit) {
    writeOutputIfChanged(PIN_OUT_RIGHT, false);
    writeOutputIfChanged(PIN_OUT_LEFT, false);
  }
}

// Boot/restart
bool bootInitialCommandEvaluated = false;
String cancelledRestartCommandId = "";

// ======================================================
// HELPERS
// ======================================================

int outputOffLevel() {
  return (OUTPUT_ACTIVE_LEVEL == HIGH) ? LOW : HIGH;
}

bool readInputActiveLevel(int pin, uint8_t activeLevel) {
  return digitalRead(pin) == activeLevel;
}

bool readEmergency1() {
  return readInputActiveLevel(PIN_EMERGENCY_1, EMERGENCY_ACTIVE_LEVEL);
}

bool readEmergency2() {
  return readInputActiveLevel(PIN_EMERGENCY_2, EMERGENCY_ACTIVE_LEVEL);
}

bool safetyOk() {
  // readEmergencyX() é TRUE quando a entrada de emergência está ATIVA.
  // Logo, segurança está OK somente quando nenhuma emergência está ativa.
  return !readEmergency1() && !readEmergency2();
}

bool pressureOk() {
  return readInputActiveLevel(PIN_PRESSURE, PRESSURE_ACTIVE_LEVEL);
}

bool bootGuardActive() {
  return (millis() - bootStartedMs) < BOOT_GUARD_MS;
}

uint32_t autoRestartRemainingMs() {
  if (!autoRestart.required && !autoRestart.pending) return 0;
  return millis() < autoRestart.notBeforeMs
      ? autoRestart.notBeforeMs - millis() : 0;
}

bool safetyStableReady() {
  if (!safetyOk()) {
    safetyStableSinceMs = 0;
    return false;
  }

  if (safetyStableSinceMs == 0) {
    safetyStableSinceMs = millis();
    return false;
  }

  return (millis() - safetyStableSinceMs) >= SAFETY_STABLE_BEFORE_RUN_MS;
}

void writeOutputState(int pin, bool on) {
  digitalWrite(pin, on ? OUTPUT_ACTIVE_LEVEL : outputOffLevel());
}

bool outputEnergizationAllowed(int pin) {
  if(!safetyTaskHandle||fastOutputInhibit||bootGuardActive()||operationJournalStartInterlock||requireStopCommandAfterFault)return false;
  if(pin!=PIN_OUT_AUX6 && !safetyOk())return false;
  return true;
}
void writeOutputIfChanged(int pin, bool on) {
  if(on&&!outputEnergizationAllowed(pin))on=false;
  int desired=on?OUTPUT_ACTIVE_LEVEL:outputOffLevel();
  if(digitalRead(pin)!=desired)digitalWrite(pin,desired);
  if(on&&!outputEnergizationAllowed(pin))digitalWrite(pin,outputOffLevel());
}

bool outputIsOnLogicLevel(int pin) {
  return digitalRead(pin) == OUTPUT_ACTIVE_LEVEL;
}

bool notePortalCommandReceived(const PivotCommand& cmd) {
  String direction = normalizeDirectionStr(cmd.direction);
  String mode = normalizeModeStr(cmd.mode);
  uint32_t payloadHash = calculateCommandPayloadHash(cmd);
  bool stopAcceptedDespiteConflict = false;

  // O mesmo commandId nunca pode representar duas intencoes diferentes.
  // Mantemos o hash original para que a retransmissao correta continue sendo
  // reconhecida mesmo depois de uma tentativa conflitante.
  if (commandExecution.commandId == cmd.commandId &&
      commandExecution.payloadHash != 0 &&
      commandExecution.payloadHash != payloadHash) {
    invalidPortalCommandCount++;
    commandIdConflictActive = true;
    commandIdConflictCount++;
    commandIdConflictMessage = "commandId reutilizado com conteudo diferente";
    appendDiagnosticHistory("COMANDO", "CONFLITO DE COMMANDID",
        "commandId=" + cmd.commandId +
        " hashOriginal=" + String(commandExecution.payloadHash, HEX) +
        " hashRecebido=" + String(payloadHash, HEX));
    Serial.println("[CMD][SEGURANCA] commandId reutilizado com conteudo diferente");
    if (direction != "parado") return false;
    cancelPendingCommand("PARAR fail-safe com commandId reutilizado");
    cancelAutoRestart("PARAR fail-safe com commandId reutilizado");
    allOutputsOff();
    commandExecution.payloadHash = payloadHash;
    stopAcceptedDespiteConflict = true;
    appendDiagnosticHistory("COMANDO", "PARAR ACEITO COM CONFLITO DE COMMANDID",
        "commandId=" + cmd.commandId + "; seguranca priorizou desligamento");
  }

  if (!stopAcceptedDespiteConflict) {
    commandIdConflictActive = false;
    commandIdConflictMessage = "";
  }

  bool persistedRetransmission = !portalCommandReceived &&
      commandExecution.commandId == cmd.commandId &&
      commandExecution.payloadHash == payloadHash;
  bool isNewCommand = !persistedRetransmission &&
      (!portalCommandReceived || cmd.commandId != lastPortalCommandId);
  portalCommandReceived = true;
  lastPortalCommandId = cmd.commandId;
  lastPortalCommandUpdatedAt = cmd.updatedAt;
  lastPortalCommandPayloadHash = payloadHash;
  lastPortalCommandDirection = direction;
  lastPortalCommandMode = mode;
  lastPortalCommandReceivedMs = millis();
  portalReceiptRecovered=false;
  if (isNewCommand) {
    commandExecution.payloadHash = payloadHash;
    setCommandExecution(cmd.commandId, direction, mode, "received",
                        "Comando recebido e validado no protocolo", false);
    appendDiagnosticHistory("COMANDO", "RECEBIDO DO PORTAL",
        "commandId=" + cmd.commandId + " updatedAt=" + String(cmd.updatedAt) +
        " direcao=" + direction +
        " modo=" + mode +
        " stopDeg=" + cmd.stopDegree);
  } else if (persistedRetransmission) {
    // A idade ja foi atualizada acima.
  }
  return true;
}

uint32_t commandOrderChecksum(uint32_t updatedAt, const String& commandId) {
  uint32_t hash=2166136261UL;
  for (uint8_t shift=0;shift<32;shift+=8) {hash^=(updatedAt>>shift)&0xFFU; hash*=16777619UL;}
  for (size_t i=0;i<commandId.length();++i) {hash^=(uint8_t)commandId[i]; hash*=16777619UL;}
  return hash;
}

bool persistCommandOrderGuard(uint32_t updatedAt,const String& commandId) {
  DynamicJsonDocument doc(320); doc["updated"]=updatedAt; doc["id"]=commandId;
  doc["crc"]=commandOrderChecksum(updatedAt,commandId); String encoded; serializeJson(doc,encoded);
  if (!prefs.begin("cmdorder",false)) return false;
  size_t written=prefs.putString("guard",encoded); prefs.end();
  if (written!=encoded.length() || !prefs.begin("cmdorder",true)) return false;
  String verified=prefs.getString("guard",""); prefs.end();
  if (verified!=encoded) return false;
  DynamicJsonDocument check(320); if (deserializeJson(check,verified)) return false;
  return check["updated"].as<uint32_t>()==updatedAt &&
         String((const char*)(check["id"]|""))==commandId &&
         check["crc"].as<uint32_t>()==commandOrderChecksum(updatedAt,commandId);
}

void loadCommandOrderGuard() {
  highestPortalCommandUpdatedAt=0; highestPortalCommandId="";
  if (!prefs.begin("cmdorder",true)) {
    markPersistenceFailure("cmdorder","namespace indisponivel no boot");
    operationJournalStartInterlock=true; operationJournalInterlockReason="ordem persistente indisponivel"; return;
  }
  String encoded=prefs.getString("guard",""); prefs.end(); if (encoded.isEmpty()) return;
  DynamicJsonDocument doc(320); DeserializationError err=deserializeJson(doc,encoded);
  uint32_t updated=doc["updated"]|0UL; String id=String((const char*)(doc["id"]|""));
  if (err || updated==0 || id.isEmpty() || (doc["crc"]|0UL)!=commandOrderChecksum(updated,id)) {
    markPersistenceFailure("cmdorder","registro corrompido no boot");
    operationJournalStartInterlock=true; operationJournalInterlockReason="ordem persistente corrompida"; return;
  }
  highestPortalCommandUpdatedAt=updated; highestPortalCommandId=id;
}

bool validateAndPersistCommandOrder(const PivotCommand& cmd) {
  const bool stop=normalizeDirectionStr(cmd.direction)=="parado";
  if (stop) {
    uint32_t updated=max(highestPortalCommandUpdatedAt,cmd.updatedAt); if (updated==0) updated=1;
    String id=cmd.commandId.isEmpty()?String("STOP-SEM-ID"):cmd.commandId;
    if (!persistCommandOrderGuard(updated,id)) {markPersistenceFailure("cmdorder","falha na barreira PARAR"); return true;}
    highestPortalCommandUpdatedAt=updated; highestPortalCommandId=id; return true;
  }
  if (cmd.updatedAt==0 || cmd.commandId.isEmpty()) {++stalePortalCommandRejectCount; return false;}
  uint32_t hash=calculateCommandPayloadHash(cmd);
  if (commandExecution.commandId==cmd.commandId && commandExecution.payloadHash!=0 &&
      commandExecution.payloadHash!=hash) {++stalePortalCommandRejectCount; return false;}
  if (highestPortalCommandUpdatedAt>0 &&
      (cmd.updatedAt<highestPortalCommandUpdatedAt ||
       (cmd.updatedAt==highestPortalCommandUpdatedAt && cmd.commandId!=highestPortalCommandId))) {
    ++stalePortalCommandRejectCount;
    appendDiagnosticHistory("COMANDO","COMANDO ANTIGO REJEITADO","id="+cmd.commandId+" updatedAt="+String(cmd.updatedAt));
    return false;
  }
  if (cmd.updatedAt==highestPortalCommandUpdatedAt && cmd.commandId==highestPortalCommandId) return true;
  if (!persistCommandOrderGuard(cmd.updatedAt,cmd.commandId)) {
    markPersistenceFailure("cmdorder","falha ao gravar ordem");
    enterPersistenceFault("falha ao confirmar ordem persistente do comando"); return false;
  }
  highestPortalCommandUpdatedAt=cmd.updatedAt; highestPortalCommandId=cmd.commandId; return true;
}

CommandPolicy::Result executionResultCode() {
  if(commandExecution.result=="received") return CommandPolicy::RECEIVED;
  if(commandExecution.result=="accepted") return CommandPolicy::ACCEPTED;
  if(commandExecution.result=="executing") return CommandPolicy::EXECUTING;
  if(commandExecution.result=="succeeded") return CommandPolicy::SUCCEEDED;
  if(commandExecution.result=="rejected") return CommandPolicy::REJECTED;
  if(commandExecution.result=="failed") return CommandPolicy::FAILED;
  if(commandExecution.result=="ignored") return CommandPolicy::IGNORED;
  return CommandPolicy::NONE;
}
bool executionMatchesPortalCommand() {
  return portalCommandReceived && !localTestMode && commandExecution.commandId==lastPortalCommandId &&
         commandExecution.payloadHash!=0 && commandExecution.payloadHash==lastPortalCommandPayloadHash;
}
bool portalCommandWasRejected() {
  return executionMatchesPortalCommand() && CommandPolicy::rejected(executionResultCode());
}
bool portalCommandWasAccepted() {
  return CommandPolicy::accepted(executionMatchesPortalCommand(),executionResultCode());
}
bool portalLegacyReceiptAccepted() {
  // Aceite do recibo/cancelamento para processamento, nao autorizacao de movimento.
  const bool cancelledReceipt=executionMatchesPortalCommand() && portalReceiptRecovered &&
      commandExecution.result=="failed";
  return PortalRecoveryPolicy::legacyReceiptAccepted(executionMatchesPortalCommand(),cancelledReceipt);
}
String portalRejectionCode() {
  if(!portalCommandWasRejected()) return "";
  if(commandExecution.reason.startsWith("GPS indisponivel") ||
     commandExecution.reason.startsWith("Sem GPS operacional")) return "GPS_INDISPONIVEL";
  return commandExecution.result=="failed" ? "FALHA_OPERACIONAL" :
         commandExecution.result=="ignored" ? "COMANDO_IGNORADO" : "COMANDO_REJEITADO";
}

bool currentCommandMatchesLastPortalCommand() {
  if (!portalCommandReceived) return false;
  return currentCmd.commandId == lastPortalCommandId &&
         calculateCommandPayloadHash(currentCmd) == lastPortalCommandPayloadHash &&
         currentCmd.updatedAt == lastPortalCommandUpdatedAt &&
         normalizeDirectionStr(currentCmd.direction) == lastPortalCommandDirection &&
         normalizeModeStr(currentCmd.mode) == lastPortalCommandMode;
}

bool commandedOutputsAreApplied() {
  if (!portalCommandWasAccepted() || !currentCommandMatchesLastPortalCommand()) return false;

  const bool rightOn = outputIsOnLogicLevel(PIN_OUT_RIGHT);
  const bool leftOn = outputIsOnLogicLevel(PIN_OUT_LEFT);
  const bool pumpOn = outputIsOnLogicLevel(PIN_OUT_PUMP);
  const bool cannonOn = outputIsOnLogicLevel(PIN_OUT_CANNON);
  const bool lastTowerOn = outputIsOnLogicLevel(PIN_OUT_LAST_TOWER);

  // PARAR só é confirmado quando todas as saídas de potência estão desligadas.
  if (lastPortalCommandDirection == "parado") {
    return !rightOn && !leftOn && !pumpOn && !cannonOn && !lastTowerOn;
  }

  // Um comando de movimento só é confirmado depois da pré-partida e quando
  // os níveis lógicos enviados aos relés coincidem com o comando atual.
  if (controllerState != STATE_RUNNING || !safetyOk()) return false;
  const bool expectRight = lastPortalCommandDirection == "direita";
  const bool expectLeft = lastPortalCommandDirection == "esquerda";
  const bool expectPump = lastPortalCommandMode == "com_agua";
  const bool expectCannon = currentCmd.canhaoFinal && FieldControl::cannonReady(
      currentCmd.mode=="com_agua",pressureOk(),pressureDetectedMs,millis(),CANNON_AFTER_PRESSURE_MS);
  // Percentuais intermediarios pulsam: nao comparar sua fase instantanea no loop HTTP.
  if(currentCmd.lastTowerPercent<=0 && lastTowerOn) return false;
  if(currentCmd.lastTowerPercent>=100 && !lastTowerOn) return false;
  return rightOn == expectRight && leftOn == expectLeft &&
         pumpOn == expectPump && cannonOn == expectCannon;
}

String commandAcknowledgementStatus() {
  if (!portalCommandReceived) return "sem_comando_recebido";
  if (portalCommandWasRejected()) {
    if(portalRejectionCode()=="GPS_INDISPONIVEL") return "rejeitado_gps_indisponivel";
    return commandExecution.result=="failed" ? "comando_falhou" :
           commandExecution.result=="ignored" ? "comando_ignorado" : "comando_rejeitado";
  }
  if (autoRestart.pending || autoRestart.required) {
    return autoRestartRemainingMs() > 0
        ? "religamento_aguardando_15_minutos"
        : "religamento_aguardando_reautorizacao";
  }
  if (!currentCommandMatchesLastPortalCommand()) {
    if (pendingCmdValid && pendingCmd.updatedAt == lastPortalCommandUpdatedAt) {
      return "aguardando_protecao_20s";
    }
    return "recebido_nao_aplicado";
  }
  if (commandedOutputsAreApplied()) {
    return lastPortalCommandDirection == "parado" ? "parada_confirmada" : "saidas_confirmadas";
  }
  if (controllerState == STATE_WAIT_SAFETY) return "aguardando_seguranca";
  if (controllerState == STATE_WAIT_PRESSURE) return "aguardando_pressao";
  if (controllerState == STATE_WAIT_START) return "aguardando_partida";
  if (controllerState == STATE_WAIT_PEAK_END) return "aguardando_fim_horario_ponta";
  if (controllerState == STATE_FAULT_SAFETY || controllerState == STATE_FAULT_LOW_PRESSURE ||
      controllerState == STATE_FAULT_GPS_TIMEOUT || controllerState == STATE_FAULT_PORTAL_TIMEOUT ||
      controllerState == STATE_FAULT_NO_MOVEMENT ||
      controllerState == STATE_FAULT_PERSISTENCE) return "bloqueado_por_falha";
  if (controllerState == STATE_IDLE && lastPortalCommandDirection != "parado" &&
      faultMessage == "Aguardando GPS válido") return "aguardando_gps";
  if (bootGuardActive()) return "aguardando_boot";
  return "aceito_aguardando_aplicacao";
}

void allOutputsOff() {
  publishTowerPulse(false, false, 60000, 0);
  applyDirectionOutputs(false, false);
  writeOutputIfChanged(PIN_OUT_PUMP, false);
  writeOutputIfChanged(PIN_OUT_CANNON, false);
  boardReadyRelayState = false;
  writeOutputIfChanged(PIN_OUT_AUX6, false);
}

void fastHardwareOutputsOff() {
  publishTowerPulse(false, false, 60000, 0);
  const int off=outputOffLevel();
  digitalWrite(PIN_OUT_RIGHT,off); digitalWrite(PIN_OUT_LEFT,off);
  digitalWrite(PIN_OUT_PUMP,off); digitalWrite(PIN_OUT_LAST_TOWER,off);
  digitalWrite(PIN_OUT_CANNON,off); digitalWrite(PIN_OUT_AUX6,off);
}

// Corte direto no registrador GPIO. Esta rotina nao usa Serial, String, NVS,
// Wi-Fi ou alocacao de memoria e pode ser executada dentro da interrupcao.
void ARDUINO_ISR_ATTR fastHardwareOutputsOffFromIsr() {
  if (OUTPUT_ACTIVE_LEVEL == HIGH) GPIO.out_w1tc = OUTPUT_GPIO_MASK;
  else GPIO.out_w1ts = OUTPUT_GPIO_MASK;
}

void ARDUINO_ISR_ATTR emergencyCutIsr() {
  fastOutputInhibit = true;
  fastHardwareOutputsOffFromIsr();
  fastEmergencyIsrCutCount=fastEmergencyIsrCutCount+1UL;
}

void ARDUINO_ISR_ATTR pressureCutIsr() {
  // A falta de pressao somente derruba a operacao quando a protecao de agua
  // esta armada. Emergencias, ao contrario, sempre cortam as saidas.
  if (!fastPressureProtectionArmed) return;
  fastOutputInhibit = true;
  fastHardwareOutputsOffFromIsr();
  fastPressureIsrCutCount=fastPressureIsrCutCount+1UL;
}

void fastSafetyTask(void*) {
  for (;;) {
    ControllerState state=controllerState;
    bool outputsEnergized = digitalRead(PIN_OUT_RIGHT)==OUTPUT_ACTIVE_LEVEL ||
                            digitalRead(PIN_OUT_LEFT)==OUTPUT_ACTIVE_LEVEL ||
                            digitalRead(PIN_OUT_PUMP)==OUTPUT_ACTIVE_LEVEL ||
                            digitalRead(PIN_OUT_LAST_TOWER)==OUTPUT_ACTIVE_LEVEL ||
                            digitalRead(PIN_OUT_CANNON)==OUTPUT_ACTIVE_LEVEL ||
                            digitalRead(PIN_OUT_AUX6)==OUTPUT_ACTIVE_LEVEL;
    bool operating=state==STATE_WAIT_PRESSURE || state==STATE_WAIT_START ||
                   state==STATE_RUNNING || localTestMode || outputsEnergized;
    bool emergency=digitalRead(PIN_EMERGENCY_1)==EMERGENCY_ACTIVE_LEVEL ||
                   digitalRead(PIN_EMERGENCY_2)==EMERGENCY_ACTIVE_LEVEL;
    uint32_t now=millis();
    if (operating && emergency) {
      bool first=!fastOutputInhibit; fastOutputInhibit=true; fastHardwareOutputsOff();
      if (first) fastSafetyCutCount=fastSafetyCutCount+1UL;
      if (fastSafetyLowSinceMs==0) fastSafetyLowSinceMs=now==0?1:now;
      if ((uint32_t)(now-fastSafetyLowSinceMs)>=SAFETY_DELAY_MS) fastSafetyTripPending=true;
    } else {
      fastSafetyLowSinceMs=0;
      if (!fastSafetyTripPending && !fastPressureTripPending) fastOutputInhibit=false;
    }
    // A supervisao precisa continuar armada mesmo depois que a propria tarefa
    // cortar a saida da bomba. Usar o nivel do rele aqui cancelaria a contagem
    // no ciclo seguinte e permitiria religar quando a pressao voltasse.
    bool pressureTrip=state==STATE_RUNNING && fastPressureProtectionArmed &&
                      digitalRead(PIN_PRESSURE)!=PRESSURE_ACTIVE_LEVEL;
    if (pressureTrip) {
      fastOutputInhibit=true; fastHardwareOutputsOff();
      if (fastPressureLowSinceMs==0) fastPressureLowSinceMs=now==0?1:now;
      if ((uint32_t)(now-fastPressureLowSinceMs)>=SAFETY_DELAY_MS && !fastPressureTripPending) {
        fastPressureTripPending=true;
        fastPressureCutCount=fastPressureCutCount+1UL;
      }
    } else {
      fastPressureLowSinceMs=0;
      if (!fastSafetyTripPending && !fastPressureTripPending && !emergency) fastOutputInhibit=false;
    }
    serviceTowerPulse();
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void setupFastSafetyTask() {
  if (safetyTaskHandle) return;
  if (xTaskCreatePinnedToCore(fastSafetyTask,"SafetyCut",2048,nullptr,6,&safetyTaskHandle,1)!=pdPASS) {
    safetyTaskHandle=nullptr; fastOutputInhibit=true; fastSafetyTripPending=true; fastHardwareOutputsOff();
  }
}

void processFastSafetyTrips() {
  if (fastSafetyTripPending && controllerState!=STATE_FAULT_SAFETY) {enterSafetyFault(); return;}
  if (fastPressureTripPending && controllerState!=STATE_FAULT_LOW_PRESSURE) enterLowPressureFault();
}

void clearFastSafetyLatch() {
  fastSafetyTripPending=false; fastPressureTripPending=false;
  fastPressureProtectionArmed=false;
  fastSafetyLowSinceMs=0; fastPressureLowSinceMs=0; fastOutputInhibit=false;
}

bool allControlledOutputsAreOff() {
  return !outputIsOnLogicLevel(PIN_OUT_RIGHT) &&
         !outputIsOnLogicLevel(PIN_OUT_LEFT) &&
         !outputIsOnLogicLevel(PIN_OUT_PUMP) &&
         !outputIsOnLogicLevel(PIN_OUT_LAST_TOWER) &&
         !outputIsOnLogicLevel(PIN_OUT_CANNON) &&
         !outputIsOnLogicLevel(PIN_OUT_AUX6);
}

void clearLocalRelaySelection() {
  localRelayRight = false;
  localRelayLeft = false;
  localRelayPump = false;
  localRelayLastTower = false;
  localRelayCannon = false;
}

void applyLocalDirectOutputs() {
  if (bootGuardActive() || !safetyTaskHandle || fastOutputInhibit || requireStopCommandAfterFault ||
      operationJournalStartInterlock || controllerState == STATE_FAULT_PERSISTENCE) {
    clearLocalRelaySelection();
    allOutputsOff();
    return;
  }
  // Nunca permite direita e esquerda simultaneamente.
  if (localRelayRight && localRelayLeft) {
    localRelayRight = false;
    localRelayLeft = false;
  }

  // As entradas de emergência continuam tendo prioridade no teste local.
  if (!safetyOk()) {
    clearLocalRelaySelection();
    allOutputsOff();
    return;
  }

  applyDirectionOutputs(localRelayRight, localRelayLeft);
  writeOutputIfChanged(PIN_OUT_PUMP, localRelayPump);
  publishTowerPulse(localRelayLastTower, true, 60000, localRelayLastTower ? 60000 : 0);
  writeOutputIfChanged(PIN_OUT_CANNON, localRelayCannon);
}

void stopLocalTest() {
  cancelPendingCommand("teste local encerrado");
  if (controllerState == STATE_RUNNING || controllerState == STATE_WAIT_PRESSURE || controllerState == STATE_WAIT_START) {
    allOutputsOff();
    recordStopEvent("TESTE LOCAL ENCERRADO", "Encerramento manual ou tempo limite do teste local");
  }
  clearOperationJournal("local_test_stopped");
  localTestMode = false;
  localDirectMode = false;
  clearLocalRelaySelection();
  currentCmd.direction = "parado";
  currentCmd.mode = "sem_agua";
  controllerState = STATE_IDLE;
  faultMessage = "";
  alertMessage = "Teste local finalizado";
  clearControllerTimers();
  allOutputsOff();
  requireStopCommandAfterFault=true;
  faultMessage="Rearme necessario apos manutencao local";
  alertMessage="Envie PARAR pelo portal antes de uma nova partida";
}

void forceHardwareOutputsOff() {
  digitalWrite(PIN_OUT_RIGHT, outputOffLevel());
  digitalWrite(PIN_OUT_LEFT, outputOffLevel());
  digitalWrite(PIN_OUT_PUMP, outputOffLevel());
  digitalWrite(PIN_OUT_LAST_TOWER, outputOffLevel());
  digitalWrite(PIN_OUT_CANNON, outputOffLevel());
  digitalWrite(PIN_OUT_AUX6, outputOffLevel());

  pinMode(PIN_OUT_RIGHT, OUTPUT);
  pinMode(PIN_OUT_LEFT, OUTPUT);
  pinMode(PIN_OUT_PUMP, OUTPUT);
  pinMode(PIN_OUT_LAST_TOWER, OUTPUT);
  pinMode(PIN_OUT_CANNON, OUTPUT);
  pinMode(PIN_OUT_AUX6, OUTPUT);

  digitalWrite(PIN_OUT_RIGHT, outputOffLevel());
  digitalWrite(PIN_OUT_LEFT, outputOffLevel());
  digitalWrite(PIN_OUT_PUMP, outputOffLevel());
  digitalWrite(PIN_OUT_LAST_TOWER, outputOffLevel());
  digitalWrite(PIN_OUT_CANNON, outputOffLevel());
  digitalWrite(PIN_OUT_AUX6, outputOffLevel());

  delay(30);
  allOutputsOff();
}

String trimTrailingSlash(String s) {
  s.trim();
  while (s.endsWith("/")) {
    s.remove(s.length() - 1);
  }
  return s;
}

String normalizeServerUrl(String s) {
  s = trimTrailingSlash(s);
  if (s.isEmpty()) return "";

  String lower = s;
  lower.toLowerCase();

  if (!lower.startsWith("http://") && !lower.startsWith("https://")) {
    s = "https://" + s;
  } else if (lower.startsWith("http://")) {
    s = "https://" + s.substring(7);
  }

  return trimTrailingSlash(s);
}

bool doubleChanged(double a, double b, double tol = 0.000001) {
  return fabs(a - b) > tol;
}

bool isFiniteNumber(double v) {
  return !isnan(v) && !isinf(v);
}

double jsonToDouble(JsonVariantConst v, double fallback) {
  if (v.isNull()) return fallback;

  if (v.is<float>() || v.is<double>() || v.is<int>() || v.is<long>() || v.is<unsigned int>() || v.is<unsigned long>()) {
    return v.as<double>();
  }

  if (v.is<const char*>()) {
    String s = String(v.as<const char*>());
    s.trim();
    s.replace(",", ".");
    if (s.length() == 0) return fallback;
    return s.toDouble();
  }

  if (v.is<String>()) {
    String s = v.as<String>();
    s.trim();
    s.replace(",", ".");
    if (s.length() == 0) return fallback;
    return s.toDouble();
  }

  return fallback;
}

double normalizeAngle360(double angle) {
  if (!isfinite(angle)) return NAN;
  double normalized = fmod(angle, 360.0);
  if (normalized < 0.0) normalized += 360.0;
  return normalized;
}

double calculateBearingFromCenter(double centerLatDeg, double centerLonDeg, double targetLatDeg, double targetLonDeg) {
  double lat1 = radians(centerLatDeg);
  double lon1 = radians(centerLonDeg);
  double lat2 = radians(targetLatDeg);
  double lon2 = radians(targetLonDeg);
  double dLon = lon2 - lon1;

  double y = sin(dLon) * cos(lat2);
  double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
  double bearing = degrees(atan2(y, x));
  return normalizeAngle360(bearing);
}

double applyGpsCorrection(double rawAngleDeg) {
  return normalizeAngle360(rawAngleDeg + currentCmd.gpsCorrectionDeg);
}

// REV047.5: consumir coordenadas apenas quando TinyGPSPlus publica nova amostra.
void updateGpsComputedGeometry() {
  bool newLocalSample = gps.location.isUpdated() && gps.location.isValid();
  if (newLocalSample) {
    gpsLat = gps.location.lat();
    gpsLon = gps.location.lng();
    newLocalSample = isFiniteNumber(gpsLat) && isFiniteNumber(gpsLon) &&
                     fabs(gpsLat) <= 90.0 && fabs(gpsLon) <= 180.0;
    if (newLocalSample) gpsLastFixMs = millis();
  }
  gpsValid = gps.location.isValid() && gps.location.age() <= 60000UL &&
             isFiniteNumber(gpsLat) && isFiniteNumber(gpsLon);
  if (gps.altitude.isValid()) gpsAltMeters = gps.altitude.meters();
  if (gps.course.isValid()) gpsCourseDeg = gps.course.deg();
  if (gps.speed.isValid()) gpsSpeedKmph = gps.speed.kmph();
  if (!loraGpsValid) {
    if (gps.hdop.isValid()) gpsHdop = gps.hdop.hdop();
    if (gps.satellites.isValid()) gpsSatellites = gps.satellites.value();
  }
  if (loraGpsValid && millis() - loraLastRxMs > 300000UL) {
    loraGpsValid = false;
    if (controllerState != STATE_RUNNING && controllerState != STATE_WAIT_PRESSURE) {
      pivotAngleValid = false;
      distanceFromCenterM = 0.0;
    }
  }
  if (!loraGpsValid && newLocalSample) {
    pivotAngleValid = false;
    distanceFromCenterM = 0.0;
    if (currentCmd.geometryReady) {
      distanceFromCenterM = TinyGPSPlus::distanceBetween(
        currentCmd.centerLat, currentCmd.centerLon, gpsLat, gpsLon);
      pivotAngleDeg = applyGpsCorrection(calculateBearingFromCenter(
        currentCmd.centerLat, currentCmd.centerLon, gpsLat, gpsLon));
      pivotAngleValid = isFiniteNumber(distanceFromCenterM) && isFiniteNumber(pivotAngleDeg);
    }
    updateLastGoodGps(gpsLat, gpsLon, pivotAngleValid, pivotAngleDeg, distanceFromCenterM, millis());
  }
}

void processGps() {
  while (GPSSerial.available() > 0) {
    gps.encode((char)GPSSerial.read());
  }
  updateGpsComputedGeometry();
}

// ======================================================
// LORA
// ======================================================

void loraRxTask(void *parameter) {
  (void)parameter;
  loraRxTaskRunning = true;

  char line[512];
  size_t len = 0;
  bool discardLine = false;

  for (;;) {
    loraRxTaskCycles = loraRxTaskCycles + 1U;

    if (loraConfigActive) {
      len = 0; discardLine = false;
      loraRxPaused = true;
      vTaskDelay(pdMS_TO_TICKS(2));
      continue;
    }

    loraRxPaused = false;
    while (!loraConfigActive && LoRaSerial.available() > 0) {
      char c = (char)LoRaSerial.read();
      loraRxByteCount++;
      loraLastAnyRxMs = millis();

      if (discardLine) { if (c == '\n') discardLine = false; continue; }
      if (c == '\n') {
        // Remove espacos/CR do final sem usar String dentro da task.
        while (len > 0 && (line[len - 1] == '\r' || line[len - 1] == ' ' || line[len - 1] == '\t')) len--;

        if (len > 0) {
          line[len] = '\0';
          LoRaRxFrame frame{};
          frame.receivedMs = millis();
          memcpy(frame.data, line, len + 1); // inclui terminador; len <= 511

          if (loraRxQueue == nullptr || xQueueSend(loraRxQueue, &frame, 0) != pdTRUE) {
            loraRxQueueDrops = loraRxQueueDrops + 1U;
          }
        }
        len = 0;
      } else if (c != '\r') {
        if (len < sizeof(line) - 1) {
          line[len++] = c;
        } else {
          // Linha corrompida/grande demais: descarta ate o proximo newline.
          discardLine = true; len = 0;
          loraRxQueueDrops = loraRxQueueDrops + 1U;
        }
      }
    }

    // 1 ms mantem a UART drenada sem ocupar a CPU.
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void setupLora() {
  pinMode(PIN_LORA_M0, OUTPUT);
  pinMode(PIN_LORA_M1, OUTPUT);
  digitalWrite(PIN_LORA_M0, LOW);
  digitalWrite(PIN_LORA_M1, LOW);
  pinMode(PIN_LORA_AUX, INPUT);
  delay(50);
  // Buffer maior protege contra rajadas antes da task drenar a UART.
  LoRaSerial.setRxBufferSize(2048);
  LoRaSerial.begin(9600, SERIAL_8N1, PIN_LORA_RX, PIN_LORA_TX);

  if (loraRxQueue == nullptr) {
    loraRxQueue = xQueueCreate(16, sizeof(LoRaRxFrame));
  }

  BaseType_t taskOk = pdFAIL;
  if (loraRxQueue != nullptr && loraRxTaskHandle == nullptr) {
    taskOk = xTaskCreatePinnedToCore(
      loraRxTask,
      "LoRaRxTask",
      4096,
      nullptr,
      2,
      &loraRxTaskHandle,
      0
    );
  }

  
  pinMode(PIN_LORA_AUX, INPUT);
  
  Serial.println(String("[LORA] Task RX dedicada: ") +
                 ((taskOk == pdPASS || loraRxTaskHandle != nullptr) ? "ATIVA" : "FALHA"));
}

void processLoRaPacket(const String &packet, uint32_t receivedMs) {
  uint32_t queueAgeMs = millis() - receivedMs;
  loraQueueAgeMs = queueAgeMs;
  String rx = packet;
  rx.trim();
  if (rx.length() == 0) return;

  loraRxLineCount++;
  loraLastRawPacket = rx;
  loraLastPacketAccepted = false;
#if PROSERVICE_VERBOSE_SERIAL
  Serial.println("[LORA RX RAW] " + rx);
#endif

  StaticJsonDocument<768> doc;
  DeserializationError err = deserializeJson(doc, rx.c_str());

  if (err) {
    loraJsonErrorCount++;
    loraLastJsonError = String(err.c_str());
    loraRejectReason = "JSON inválido: " + loraLastJsonError;
    loraIgnoredPackets++;
    Serial.println("[LORA] JSON ERRO: " + loraLastJsonError);
    return;
  }

  loraLastJsonError = "";
  String pktType = String(doc["t"] | "");
  String nodeId  = String(doc["id"] | "");
  nodeId.trim();
  if (doc["ok"].isNull()) {
    loraRejectReason = "Campo ok ausente";
    ++loraIgnoredPackets;
    return;
  }
  int gpsOk = (int)(doc["ok"] | -1);

  loraLastSeenId = nodeId;
#if PROSERVICE_VERBOSE_SERIAL
  Serial.println("[LORA] JSON OK tipo=" + pktType + " id=" + nodeId + " ok=" + String(gpsOk));
#endif

  if (pktType != "g") {
    loraRejectReason = "Tipo diferente de g: " + pktType;
    loraIgnoredPackets++;
    Serial.println("[LORA] REJEITADO: " + loraRejectReason);
    return;
  }

  // Pareamento pelo ID configurado, como na REV045. Sem ID configurado,
  // nenhum GPS é aceito; IDs diferentes nunca renovam estado nem temporizador.
  if (cfg.gpsNodeId.length() == 0 || nodeId.length() == 0 || nodeId != cfg.gpsNodeId) {
    loraRejectReason = "ID diferente do autorizado";
    loraIgnoredPackets++;
    Serial.println("[LORA] REJEITADO: recebido=" + nodeId + " esperado=" + cfg.gpsNodeId);
    return;
  }

  if (gpsOk < 0 || gpsOk > 1) {
    loraRejectReason = "Valor ok invalido"; ++loraIgnoredPackets; return;
  }
  bool complete=doc["sample"].is<uint32_t>() && doc["seq"].is<uint32_t>() &&
      doc["boot"].is<uint32_t>() && doc["age"].is<uint32_t>() &&
      doc["sa"].is<uint32_t>() && (doc["hd"].is<float>() || doc["hd"].is<double>() ||
      doc["hd"].is<int>() || doc["hd"].is<uint32_t>()) && doc["ch"].is<int>();
  if (!complete) {loraRejectReason="Protocolo GPS incompleto; exigida REV25.8"; ++loraIgnoredPackets; return;}
  const uint32_t packetSample=doc["sample"].as<uint32_t>();
  const uint32_t packetSeq=doc["seq"].as<uint32_t>();
  const uint32_t packetBoot=doc["boot"].as<uint32_t>();
  const uint32_t packetAgeMs=doc["age"].as<uint32_t>();
  const uint32_t packetSatellites=doc["sa"].as<uint32_t>();
  const double packetHdop=doc["hd"].as<double>();
  loraReportedChannel=doc["ch"].as<int>();
  if (packetSeq==0 || packetBoot==0 || packetSatellites>80 || !isFiniteNumber(packetHdop) ||
      packetHdop<0 || packetHdop>100.0 || (gpsOk==1 && packetSample==0)) { // tolera arredondamento do sentinela float 99.9
    loraRejectReason="Campos de controle GPS invalidos"; ++loraIgnoredPackets; return;
  }
  if (loraReportedChannel < 0 || loraReportedChannel > 80) loraReportedChannel = -1;

  // Isolamento obrigatório entre conjuntos. O ID sozinho não basta em locais
  // com vários pivôs: ambos os E220 precisam informar o mesmo canal realmente
  // lido dos módulos. Pacote sem canal confirmado ou com canal divergente não
  // renova link, GPS, movimento nem agenda resposta da central.
  if (e220ReadChannel < 0) {
    loraRejectReason = "Canal real da central nao confirmado no E220";
    ++loraIgnoredPackets;
    return;
  }
  if (loraReportedChannel < 0) {
    loraRejectReason = "GPS nao informou canal real confirmado";
    ++loraIgnoredPackets;
    return;
  }
  if (loraReportedChannel != e220ReadChannel) {
    loraRejectReason = "Canal divergente: central=" + String(e220ReadChannel) +
                       " GPS=" + String(loraReportedChannel);
    ++loraIgnoredPackets;
    return;
  }

  double receivedLat = 0.0;
  double receivedLon = 0.0;
  if (gpsOk == 1) {
    if (packetAgeMs > GPS_PACKET_MAX_AGE_MS) {
      loraRejectReason = "Amostra GPS antiga: age=" + String(packetAgeMs) + "ms";
      ++loraOldSamples;
      ++loraIgnoredPackets;
      return;
    }
    if (!doc["la"].is<double>() || !doc["lo"].is<double>()) {
      loraRejectReason = "Coordenadas GPS ausentes ou nao numericas";
      ++loraIgnoredPackets;
      return;
    }
    receivedLat = doc["la"].as<double>();
    receivedLon = doc["lo"].as<double>();
    if (!isFiniteNumber(receivedLat) || !isFiniteNumber(receivedLon) ||
        fabs(receivedLat) > 90.0 || fabs(receivedLon) > 180.0 ||
        (receivedLat == 0.0 && receivedLon == 0.0)) {
      loraRejectReason = "Coordenadas GPS invalidas";
      ++loraIgnoredPackets;
      return;
    }
  }

  bool newPeerSession = !peerBootKnown || packetBoot != peerBoot;
  if (newPeerSession && peerBootKnown) {
    for (uint8_t i=0; i<retiredPeerCount; ++i) {
      if (retiredPeerBoots[i] == packetBoot) {
        loraRejectReason="Pacote de sessao GPS encerrada"; ++loraIgnoredPackets; return;
      }
    }
  }
  if (!newPeerSession && !FieldControl::counterAfter(packetSeq, lastLoraSampleSeq)) {
    loraRejectReason="Sequencia LoRa repetida ou fora de ordem"; ++loraIgnoredPackets; return;
  }
  if (!newPeerSession && gpsOk == 1 && loraSampleSeen &&
      packetSample != lastLoraSample && !FieldControl::counterAfter(packetSample, lastLoraSample)) {
    loraRejectReason="Amostra GPS anterior a ultima aceita"; ++loraOldSamples; ++loraIgnoredPackets; return;
  }
  if (newPeerSession) {
    if (peerBootKnown) {
      retiredPeerBoots[retiredPeerNext] = peerBoot;
      retiredPeerNext = (retiredPeerNext + 1) % 16;
      if (retiredPeerCount < 16) ++retiredPeerCount;
    }
    peerBoot=packetBoot; peerBootKnown=true; loraSampleSeen=false;
  }
  lastLoraSampleSeq=packetSeq;
  loraAcceptedMs = receivedMs;
  ++loraAcceptedSinceCheck;
  // Agenda a resposta depois de receber um pacote valido do GPS.
  // O atraso impede a central de transmitir enquanto o E220 remoto ainda envia.
  if (!loraConfigTxPending) {
    loraConfigTxPending = true;
    loraConfigTxDueMs = millis() + LORA_CONFIG_REPLY_DELAY_MS;
  }

  gpsSatellites=packetSatellites; gpsHdop=packetHdop;

  if (gpsOk == 1) {
    const bool newGpsSample=!loraSampleSeen || FieldControl::counterAfter(packetSample,lastLoraSample);
    if (!newGpsSample) {
      ++loraDuplicateSamples;
      loraLastPacketAccepted = true;
      loraRejectReason = "Retransmissao: enlace atualizado sem alterar posicao";
      return;
    }
    loraSampleSeen=true; lastLoraSample=packetSample;
    loraGpsLat = receivedLat;
    loraGpsLon = receivedLon;
    loraGpsValid = true;
    loraLastPacketAccepted = true;
    loraRejectReason = "";


    Serial.println("[LORA] GPS " + String(newGpsSample ? "AMOSTRA NOVA" : "RETRANSMISSAO") +
                   " id=" + nodeId + " lat=" + String(loraGpsLat,6) +
                   " lon=" + String(loraGpsLon,6));

    pivotAngleValid = false;
    distanceFromCenterM = 0.0;
    if (currentCmd.geometryReady) {
      pivotAngleDeg = applyGpsCorrection(calculateBearingFromCenter(currentCmd.centerLat, currentCmd.centerLon, loraGpsLat, loraGpsLon));
      distanceFromCenterM = TinyGPSPlus::distanceBetween(currentCmd.centerLat, currentCmd.centerLon, loraGpsLat, loraGpsLon);
      pivotAngleValid = isFiniteNumber(distanceFromCenterM) && isFiniteNumber(pivotAngleDeg);
#if PROSERVICE_VERBOSE_SERIAL
      Serial.println("[LORA] Angulo=" + String(pivotAngleDeg,2) + " deg dist=" + String(distanceFromCenterM,1) + "m valid=" + String(pivotAngleValid));
#endif
    }
    // Somente uma amostra GNSS realmente nova renova a validade da posicao.
    // Retransmissoes mantem o link LoRa visivel, mas nao simulam GPS ativo nem
    // movimento mecanico.
    if (newGpsSample) {
      // Usa o instante da recepcao menos a idade informada pelo GPS. Uma fila
      // atrasada nunca pode transformar coordenada antiga em posicao nova.
      const uint32_t sampleMs = receivedMs - packetAgeMs; // subtracao modular preserva idade no rollover
      loraLastRxMs = sampleMs;
      updateLastGoodGps(loraGpsLat, loraGpsLon, pivotAngleValid, pivotAngleDeg,
                        distanceFromCenterM, sampleMs);
    }
    if (newGpsSample && controllerState == STATE_RUNNING &&
        commandRequiresGps(currentCmd) && movementGuard.active) {
      uint32_t sampleMs = lastGoodGpsMs;
      bool afterStart = (int32_t)(sampleMs-movementGuard.started) >= 0;
      if (afterStart) {
        if (!currentCmd.geometryReady || !lastGoodPivotAngleValid || lastGoodDistanceFromCenterM < 10.0) {
          ++movementGeometryRejects;
          ++movementGuard.rejectedSamples;
          movementGuard.reason = "Centro/raio/geometria insuficiente; prazo continua";
        } else if ((uint32_t)(millis() - sampleMs) > MOVEMENT_SAMPLE_FRESH_MS) {
          ++movementAgeRejects;
          ++movementGuard.rejectedSamples;
          movementGuard.reason = "Amostra atrasada; aguardando GPS operacional";
        } else if (gpsSatellites < MOVEMENT_MIN_SATELLITES || !isFiniteNumber(gpsHdop) ||
                   gpsHdop <= 0 || gpsHdop > MOVEMENT_MAX_HDOP) {
          ++movementQualityRejects;
          ++movementGuard.rejectedSamples;
          movementGuard.reason = "Qualidade insuficiente; prazo continua";
        } else if (!movementGuard.rawKnown || (int32_t)(sampleMs-movementGuard.lastInput) > 0) {
          if (movementGuard.sample(sampleMs,lastGoodPivotAngleDeg,lastGoodDistanceFromCenterM,
              currentCmd.direction == "direita" ? 1 : -1,true)) {
            movementLastUsableSampleMs = sampleMs;
          }
        }
      }
      movementLastGeneration = gpsSampleGeneration;
    }

  } else {
    // O link foi recebido e validado, embora o NEO-6M ainda nao tenha FIX.
    loraGpsValid = false;
    loraRejectReason = "GPS conectado via LoRa, aguardando FIX";
    Serial.println("[LORA] LINK OK; GPS sem FIX. sats=" + String(gpsSatellites) + " hdop=" + String(gpsHdop,2));
  }
}

// Esperas limitadas; chamadas somente com o controlador parado.
bool waitE220Ready(uint32_t timeoutMs) {
  uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    if (digitalRead(PIN_LORA_AUX) == HIGH) {
      delay(3);
      if (digitalRead(PIN_LORA_AUX) == HIGH) return true;
    }
    delay(2);
  }
  return false;
}

bool readE220Config(uint8_t config[6]) {
  const uint8_t request[3] = {0xC1, 0x00, 0x06};
  for (int attempt = 1; attempt <= 3; ++attempt) {
    if (!waitE220Ready(1500)) {
      Serial.println("[E220] AUX permaneceu LOW antes da leitura");
      return false;
    }
    for (int i = 0; i < 2048 && LoRaSerial.available(); ++i) LoRaSerial.read();
    LoRaSerial.write(request, sizeof(request));
    LoRaSerial.flush();
    uint8_t response[9] = {};
    size_t count = 0, total = 0;
    uint32_t start = millis();
    while (millis() - start < 1000UL) {
      if (!LoRaSerial.available()) { delay(2); continue; }
      uint8_t value = (uint8_t)LoRaSerial.read();

      ++total;
      if (count == sizeof(response)) {
        memmove(response, response + 1, sizeof(response) - 1);
        --count;
      }
      response[count++] = value;
      if (count == sizeof(response) && response[0] == 0xC1 &&
          response[1] == 0x00 && response[2] == 0x06) {
        memcpy(config, response + 3, 6);
        Serial.println(" [E220] Leitura OK");
        return true;
      }
    }
    Serial.printf("\n[E220] Tentativa %d: %u bytes, resposta ausente/invalida; AUX=%d\n",
                  attempt, (unsigned)total, digitalRead(PIN_LORA_AUX));
    delay(100);
  }
  return false;
}

bool writeE220Channel(int channel, String &message) {
  message = "";
  // -1 consulta somente. Nunca usa a preferencia salva como prova de leitura.
  if (channel < -1 || channel > E220_MAX_CHANNEL) { message = "Canal invalido"; return false; }
  e220ReadChannel = -1;
  loraConfigActive = true;
  uint32_t start = millis();
  while (loraRxTaskHandle != nullptr && !loraRxPaused && millis()-start < 500UL) delay(2);
  if (loraRxTaskHandle != nullptr && !loraRxPaused) {
    loraConfigActive = false;
    message = "Task RX nao liberou a UART; consulta nao realizada";
    return false;
  }
  // Descarta quadros anteriores ao modo de configuracao (operacao em idle).
  if (loraRxQueue) xQueueReset(loraRxQueue);
  loraConfigTxPending = false;
  loraLastPacketAccepted = false;
  loraAcceptedMs = 0; loraAcceptedSinceCheck = 0; loraReportedChannel = -1;
  loraLastRawPacket = ""; loraLastTxPacket = "";
  loraGpsValid = false; loraLastRxMs = 0;
  lastGoodGpsAvailable = false; lastGoodGpsMs = 0;
  pivotAngleValid = false;
  bool confirmed = false;
  if (!waitE220Ready(2000)) message = "AUX LOW antes da consulta; nenhuma gravacao realizada";
  else {
    digitalWrite(PIN_LORA_M0,HIGH); digitalWrite(PIN_LORA_M1,HIGH); delay(120);
    uint8_t regs[6];
    if (!waitE220Ready(2000)) message = "AUX LOW no modo configuracao; nenhuma gravacao realizada";
    else if (!readE220Config(regs)) message = "Leitura inicial falhou; nenhuma gravacao realizada";
    else {
      if (regs[4] <= 80) { e220ReadChannel=regs[4]; e220ReadMs=millis(); }
      if (channel < 0 || e220ReadChannel == channel) confirmed = e220ReadChannel >= 0;
      else if (!waitE220Ready(1500)) message = "AUX LOW antes da gravacao; canal anterior lido preservado";
      else {
        uint8_t command[4]={0xC0,0x04,0x01,(uint8_t)channel};
        e220ReadChannel=-1;
        LoRaSerial.write(command,sizeof(command)); LoRaSerial.flush(); delay(200);
        if (readE220Config(regs) && regs[4] <= 80) {
          e220ReadChannel=regs[4]; e220ReadMs=millis();
          confirmed = e220ReadChannel == channel;
          if (!confirmed) message="Divergencia: solicitado " + String(channel) + ", lido " + String(e220ReadChannel);
        } else message="Gravacao enviada, mas releitura falhou; canal real NAO CONFIRMADO";
      }
      if (!confirmed && message.isEmpty()) message="Registro de canal fora do intervalo esperado";
    }
  }
  if (confirmed) message="Canal " + String(e220ReadChannel) + " lido diretamente do E220";
  digitalWrite(PIN_LORA_M0,LOW); digitalWrite(PIN_LORA_M1,LOW); delay(120);
  bool normalReady=waitE220Ready(2000);
  for (int i=0;i<2048 && LoRaSerial.available();++i) LoRaSerial.read();
  loraConfigActive=false;
  if (!normalReady) message += "; AUX LOW no retorno: modo normal nao confirmado";
  Serial.println("[E220] " + message);
  return confirmed && normalReady;
}

void handleReadLoraChannel() {
  if(!requireServiceAccess(false))return;
  if (controllerState != STATE_IDLE || localTestMode) {
    server.send(409,"text/plain; charset=utf-8","Consulta interrompe o radio: aguarde idle e encerre testes locais."); return;
  }
  writeE220Channel(-1,e220ConfigStatus);
  server.sendHeader("Location","/"); server.send(303);
}

void handleSaveLoraChannel() {
  if(!requireServiceAccess(false))return;
  if (controllerState != STATE_IDLE || localTestMode) {
    server.send(409, "text/html; charset=utf-8", "<html><body style='font-family:Arial;padding:20px'><h2>Alteracao bloqueada</h2><p>Aguarde estado idle e encerre o teste local antes de mudar o canal LoRa.</p><a href='/'>Voltar</a></body></html>");
    return;
  }
  String rawChannel=server.arg("channel");
  bool validChannel=rawChannel.length()>0 && rawChannel.length()<=2;
  for (size_t i=0;i<rawChannel.length();++i) if(rawChannel[i]<'0'||rawChannel[i]>'9') validChannel=false;
  int requested = rawChannel.toInt();
  if (!validChannel || requested < 0 || requested > E220_MAX_CHANNEL) {
    server.send(400, "text/plain; charset=utf-8", "Canal invalido. Use de 0 a 80.");
    return;
  }
  String message;
  if (!writeE220Channel((uint8_t)requested, message)) {
    e220ConfigStatus = message;
    server.send(500, "text/html; charset=utf-8", "<html><body style='font-family:Arial;padding:20px'><h2>Falha</h2><p>" + htmlEscape(message) + "</p><a href='/'>Voltar</a></body></html>");
    return;
  }
  e220Channel = (uint8_t)requested;
  bool saved = false;
  if (prefs.begin(PREF_NAMESPACE, false)) {
    saved = prefs.putUChar("lora_ch", e220Channel) == sizeof(uint8_t) &&
        prefs.getUChar("lora_ch", 255) == e220Channel;
    prefs.end();
  }
  if (!saved) {
    markPersistenceFailure("config", "canal E220 gravado no radio, mas nao confirmado na NVS");
    e220ConfigStatus = message + "; FALHA ao salvar canal na NVS";
    server.send(500, "text/html; charset=utf-8", "<html><body style='font-family:Arial;padding:20px'><h2>Canal alterado com ressalva</h2><p>O E220 confirmou o canal, mas a NVS da central nao confirmou a gravacao. Nao desligue a placa e repita a configuracao.</p><a href='/'>Voltar</a></body></html>");
    return;
  }
  markPersistenceSuccess();
  e220ConfigStatus = message;
  server.sendHeader("Location", "/"); server.send(303);
}

void processLoRaRx() {
  if (loraRxQueue == nullptr) return;

  LoRaRxFrame frame;
  // Limita o trabalho por passagem do loop, mas a task continua capturando tudo.
  uint8_t processed = 0;
  while (processed < 16 && xQueueReceive(loraRxQueue, &frame, 0) == pdTRUE) {
    processLoRaPacket(String(frame.data), frame.receivedMs);
    processed++;
  }
}

void sendLoRaConfig() {
  if (!loraConfigTxPending) return;
  uint32_t now = millis();
  if ((int32_t)(now - loraConfigTxDueMs) < 0) return;
  if (loraLastTxMs != 0 && now - loraLastTxMs < LORA_CONFIG_MIN_GAP_MS) return;
  if (loraConfigActive || e220ReadChannel < 0 || digitalRead(PIN_LORA_AUX) != HIGH) return;

  StaticJsonDocument<512> doc;
  doc["t"]   = "c";
  // Destinatario obrigatorio: o GPS REV.23 rejeita respostas de outro pivo.
  doc["id"]  = cfg.gpsNodeId;
  doc["cla"] = currentCmd.centerLat;
  doc["clo"] = currentCmd.centerLon;
  doc["st"]  = currentCmd.stopDegree.toFloat();
  doc["di"]  = currentCmd.direction == "direita" ? "d" :
               currentCmd.direction == "esquerda" ? "e" : "p";
  doc["ack"] = lastLoraSampleSeq;
  doc["boot"] = peerBoot;
  doc["v"] = 2;
  if (e220ReadChannel >= 0) doc["ch"] = e220ReadChannel;

  String out;
  serializeJson(doc, out);
  if (doc.overflowed()) return;
  loraConfigTxPending = false;
  loraLastTxMs = now;
  loraLastTxPacket = out;

  LoRaSerial.print(out);
  LoRaSerial.print('\n');
  LoRaSerial.flush();
  loraLastAnyTxMs = millis();
#if PROSERVICE_VERBOSE_SERIAL
  Serial.println("[LORA TX] " + out);
#endif
}

String formatDoubleValue(double value, uint8_t decimals = 6) {
  if (!isFiniteNumber(value)) return "";
  return String(value, (unsigned int)decimals);
}

String htmlEscape(String s) {
  s.replace("&", "&amp;");
  s.replace("<", "&lt;");
  s.replace(">", "&gt;");
  s.replace("\"", "&quot;");
  s.replace("'", "&#39;");
  return s;
}

String urlEncode(String s) {
  String encoded = "";
  char c;
  char buf[4];
  for (size_t i = 0; i < s.length(); i++) {
    c = s.charAt(i);
    if (
      (c >= 'a' && c <= 'z') ||
      (c >= 'A' && c <= 'Z') ||
      (c >= '0' && c <= '9') ||
      c == '-' || c == '_' || c == '.' || c == '~'
    ) {
      encoded += c;
    } else {
      snprintf(buf, sizeof(buf), "%%%02X", (unsigned char)c);
      encoded += buf;
    }
  }
  return encoded;
}

String buildApName() {
  String idPart = cfg.pivotId;
  idPart.trim();
  if (idPart.isEmpty()) {
    uint64_t chipid = ESP.getEfuseMac();
    char temp[10];
    snprintf(temp, sizeof(temp), "%04X", (uint16_t)(chipid & 0xFFFF));
    idPart = temp;
  }
  return "PIVOT-" + idPart + "-SETUP";
}

String stateToString(ControllerState st) {
  switch (st) {
    case STATE_IDLE:                return "idle";
    case STATE_WAIT_SAFETY:         return "aguardando_seguranca_prepartida";
    case STATE_WAIT_PRESSURE:       return "aguardando_pressao";
    case STATE_WAIT_START:          return "aguardando_partida";
    case STATE_RUNNING:             return "rodando";
    case STATE_WAIT_PEAK_END:       return "bloqueado_horario_ponta";
    case STATE_FAULT_SAFETY:        return "falha_seguranca";
    case STATE_FAULT_LOW_PRESSURE:  return "falha_baixa_pressao";
    case STATE_FAULT_GPS_TIMEOUT:   return "falha_gps_timeout";
    case STATE_FAULT_PORTAL_TIMEOUT:return "falha_portal_timeout";
    case STATE_FAULT_NO_MOVEMENT:   return "falha_sem_movimento";
    case STATE_BOOT_HOLD:           return "boot_hold";
    case STATE_FAULT_PERSISTENCE:   return "falha_persistencia_nvs";
    default:                        return "idle";
  }
}

String normalizeModeStr(const String& value) {
  String v = value;
  v.trim();
  v.toLowerCase();
  return (v == "com_agua") ? "com_agua" : "sem_agua";
}

String normalizeDirectionStr(const String& value) {
  String v = value;
  v.trim();
  v.toLowerCase();
  if (v == "direita") return "direita";
  if (v == "esquerda") return "esquerda";
  return "parado";
}

// Exclusiva para comandos remotos: dado ausente ou desconhecido nunca pode
// virar um comando PARAR por conversão automática.
bool tryNormalizePortalDirection(const String& value, String& normalized) {
  String v = value;
  v.trim();
  v.toLowerCase();
  v.replace("-", "_");
  v.replace(" ", "_");

  if (v == "direita" || v == "dir" || v == "d" || v == "right" ||
      v == "forward" || v == "frente" || v == "avanco" || v == "avanço" ||
      v == "horario" || v == "clockwise" || v == "cw" || v == "1") {
    normalized = "direita";
    return true;
  }
  if (v == "esquerda" || v == "esq" || v == "e" || v == "left" ||
      v == "reverse" || v == "reverso" || v == "reversao" || v == "reversão" ||
      v == "anti_horario" || v == "antihorario" || v == "counterclockwise" ||
      v == "ccw" || v == "2") {
    normalized = "esquerda";
    return true;
  }
  if (v == "parado" || v == "parar" || v == "pare" || v == "stop" ||
      v == "stopped" || v == "off" || v == "desligado" ||
      v == "desligar" || v == "0") {
    normalized = "parado";
    return true;
  }

  normalized = "";
  return false;
}

float clampFloat(float value, float minVal, float maxVal) {
  if (value < minVal) return minVal;
  if (value > maxVal) return maxVal;
  return value;
}

float jsonToFloat(JsonVariantConst v, float fallback) {
  if (v.isNull()) return fallback;

  if (v.is<float>() || v.is<double>() || v.is<int>() || v.is<long>() || v.is<unsigned int>() || v.is<unsigned long>()) {
    return v.as<float>();
  }

  if (v.is<const char*>()) {
    String s = String(v.as<const char*>());
    s.trim();
    s.replace(",", ".");
    if (s.length() == 0) return fallback;
    return s.toFloat();
  }

  if (v.is<String>()) {
    String s = v.as<String>();
    s.trim();
    s.replace(",", ".");
    if (s.length() == 0) return fallback;
    return s.toFloat();
  }

  return fallback;
}

uint32_t jsonToUInt32(JsonVariantConst v, uint32_t fallback) {
  if (v.isNull()) return fallback;

  if (v.is<unsigned long>()) return v.as<unsigned long>();
  if (v.is<long>()) {
    long val = v.as<long>();
    if (val < 0) return fallback;
    return (uint32_t)val;
  }
  if (v.is<unsigned int>()) return (uint32_t)v.as<unsigned int>();
  if (v.is<int>()) {
    int val = v.as<int>();
    if (val < 0) return fallback;
    return (uint32_t)val;
  }
  if (v.is<const char*>()) {
    String s = String(v.as<const char*>());
    s.trim();
    if (s.length() == 0) return fallback;
    long val = s.toInt();
    if (val < 0) return fallback;
    return (uint32_t)val;
  }

  return fallback;
}

bool floatChanged(float a, float b, float tol = 0.01f) {
  return fabs(a - b) > tol;
}

uint32_t fnv1aAppend(uint32_t hash, const String& value) {
  for (size_t i = 0; i < value.length(); i++) {
    hash ^= (uint8_t)value.charAt(i);
    hash *= 16777619UL;
  }
  hash ^= 0xFFU; // separador de campos
  hash *= 16777619UL;
  return hash;
}

uint32_t calculateCommandPayloadHash(const PivotCommand& cmd) {
  uint32_t hash = 2166136261UL;
  hash = fnv1aAppend(hash, normalizeDirectionStr(cmd.direction));
  hash = fnv1aAppend(hash, normalizeModeStr(cmd.mode));
  hash = fnv1aAppend(hash, cmd.depth);
  hash = fnv1aAppend(hash, cmd.stopDegree);
  hash = fnv1aAppend(hash, String(cmd.canhaoFinal));
  hash = fnv1aAppend(hash, String(cmd.pressureWaitSec));
  hash = fnv1aAppend(hash, String(cmd.updatedAt));
  hash = fnv1aAppend(hash, String(cmd.requestedDepthMm, 4));
  hash = fnv1aAppend(hash, String(cmd.minDepthMm, 4));
  hash = fnv1aAppend(hash, String(cmd.safetyFactorPercent, 4));
  hash = fnv1aAppend(hash, String(cmd.lastTowerPercent, 4));
  hash = fnv1aAppend(hash, String(cmd.lastTowerCycleMs));
  hash = fnv1aAppend(hash, String(cmd.engineeringReady));
  hash = fnv1aAppend(hash, String(cmd.centerLat, 8));
  hash = fnv1aAppend(hash, String(cmd.centerLon, 8));
  hash = fnv1aAppend(hash, String(cmd.radiusMeters, 3));
  hash = fnv1aAppend(hash, String(cmd.areaHectares, 4));
  hash = fnv1aAppend(hash, String(cmd.gpsCorrectionDeg, 4));
  hash = fnv1aAppend(hash, String(cmd.peakEnabled));
  hash = fnv1aAppend(hash, cmd.peakStart);
  hash = fnv1aAppend(hash, cmd.peakEnd);
  hash = fnv1aAppend(hash, String(cmd.peakDaysMask));
  hash = fnv1aAppend(hash, String(cmd.restartAfterPower));
  hash = fnv1aAppend(hash, String(cmd.requiresGps));
  hash = fnv1aAppend(hash, String(cmd.noGpsMode));
  return hash == 0 ? 1 : hash;
}

bool commandExecutionIsTerminal() {
  return commandExecution.result == "succeeded" ||
         commandExecution.result == "rejected" ||
         commandExecution.result == "failed" ||
         commandExecution.result == "ignored";
}

bool commandIsNewOrChanged(const PivotCommand& a, const PivotCommand& b) {
  // No protocolo REV47.16, commandId identifica de forma exclusiva e imutável
  // uma execução. Repetir o mesmo ID é consulta/retransmissão, não novo comando.
  if (a.commandId.length() && b.commandId.length()) return a.commandId != b.commandId;
  if (a.commandId != b.commandId) return true;
  if (a.updatedAt != b.updatedAt) return true;
  if (a.mode != b.mode) return true;
  if (a.direction != b.direction) return true;
  if (a.depth != b.depth) return true;
  if (a.stopDegree != b.stopDegree) return true;
  if (a.canhaoFinal != b.canhaoFinal) return true;
  if (a.pressureWaitSec != b.pressureWaitSec) return true;

  if (floatChanged(a.requestedDepthMm, b.requestedDepthMm)) return true;
  if (floatChanged(a.minDepthMm, b.minDepthMm)) return true;
  if (floatChanged(a.safetyFactorPercent, b.safetyFactorPercent)) return true;
  if (floatChanged(a.lastTowerPercent, b.lastTowerPercent)) return true;
  if (a.lastTowerCycleMs != b.lastTowerCycleMs) return true;
  if (a.engineeringReady != b.engineeringReady) return true;
  if (a.depthClamped != b.depthClamped) return true;
  if (doubleChanged(a.centerLat, b.centerLat)) return true;
  if (doubleChanged(a.centerLon, b.centerLon)) return true;
  if (floatChanged(a.radiusMeters, b.radiusMeters)) return true;
  if (floatChanged(a.areaHectares, b.areaHectares)) return true;
  if (a.geometryReady != b.geometryReady) return true;
  if (a.peakEnabled != b.peakEnabled) return true;
  if (a.peakStart != b.peakStart) return true;
  if (a.peakEnd != b.peakEnd) return true;
  if (a.peakDaysMask != b.peakDaysMask) return true;
  if (a.restartAfterPower != b.restartAfterPower) return true;
  if (a.requiresGps != b.requiresGps) return true;
  if (a.noGpsMode != b.noGpsMode) return true;

  return false;
}

String wifiSecurityName(wifi_auth_mode_t auth) {
  switch (auth) {
    case WIFI_AUTH_OPEN: return "aberta";
    case WIFI_AUTH_WEP: return "WEP (não recomendado)";
    case WIFI_AUTH_WPA_PSK: return "WPA";
    case WIFI_AUTH_WPA2_PSK: return "WPA2";
    case WIFI_AUTH_WPA_WPA2_PSK: return "WPA/WPA2";
    case WIFI_AUTH_WPA2_ENTERPRISE: return "WPA2 Enterprise";
    case WIFI_AUTH_WPA3_PSK: return "WPA3";
    case WIFI_AUTH_WPA2_WPA3_PSK: return "WPA2/WPA3";
    default: return "segurança desconhecida";
  }
}

String wifiStatusName(wl_status_t status) {
  switch (status) {
    case WL_IDLE_STATUS: return "iniciando";
    case WL_NO_SSID_AVAIL: return "rede não encontrada";
    case WL_SCAN_COMPLETED: return "busca concluída";
    case WL_CONNECTED: return "conectado";
    case WL_CONNECT_FAILED: return "falha de autenticação/senha";
    case WL_CONNECTION_LOST: return "conexão perdida";
    case WL_DISCONNECTED: return "desconectado";
    default: return "estado " + String((int)status);
  }
}

void inspectSavedNetwork() {wifiScanRequested=true;}
void processWifiScan() {
  bool safelyIdle=(controllerState==STATE_IDLE||controllerState==STATE_BOOT_HOLD)&&!localTestMode;
  if(wifiScanActive){
    if(!safelyIdle){esp_wifi_scan_stop();WiFi.scanDelete();wifiScanActive=false;return;}
    int count=WiFi.scanComplete();
    if(count==WIFI_SCAN_RUNNING){if(uint32_t(millis()-wifiScanStartedMs)>15000){esp_wifi_scan_stop();WiFi.scanDelete();wifiScanActive=false;}return;}
    wifiScanActive=false;cachedWifiOptions="";String seen="\n";
    wifiSavedNetworkRssi=-127;wifiSavedNetworkChannel=0;wifiSavedNetworkSecurity="nao encontrada";
    for(int i=0;i<count;++i){
      String ssid=WiFi.SSID(i);if(ssid==cfg.wifiSsid){wifiSavedNetworkRssi=WiFi.RSSI(i);wifiSavedNetworkChannel=WiFi.channel(i);wifiSavedNetworkSecurity=wifiSecurityName(WiFi.encryptionType(i));}
      if(i>=16||ssid.isEmpty()||seen.indexOf("\n"+ssid+"\n")>=0)continue;
      seen+=ssid+"\n";String selected=ssid==cfg.wifiSsid?" selected":"";
      String entry="<option value='"+htmlEscape(ssid)+"'"+selected+">"+htmlEscape(ssid)+" | "+String(WiFi.RSSI(i))+" dBm</option>";
      if(cachedWifiOptions.length()+entry.length()<=4096)cachedWifiOptions+=entry;
    }
    WiFi.scanDelete();return;
  }
  if(!wifiScanRequested||!safelyIdle||wifiAttemptActive||httpInFlight)return;
  if(wifiScanAttempted&&uint32_t(millis()-wifiLastScanMs)<60000)return;
  wifiScanRequested=false;wifiScanAttempted=true;wifiLastScanMs=millis();
  WiFi.scanDelete();int result=WiFi.scanNetworks(true,true,false,120);
  if(result==WIFI_SCAN_RUNNING||result>=0){wifiScanActive=true;wifiScanStartedMs=millis();}
}

String buildWifiOptions() {
  wifiScanRequested=true;
  return cachedWifiOptions.length()?cachedWifiOptions:String("<option value=''>Lista em atualizacao; use SSID manual ou atualize a pagina</option>");
}

String normalizeBackendRoute(String route, const char* fallback) {
  route.trim();
  if (route.isEmpty()) route = String(fallback);

  // Pode ser uma URL completa do novo painel.
  String lower = route;
  lower.toLowerCase();
  if (lower.startsWith("http://")) {
    route="https://"+route.substring(7);
    return trimTrailingSlash(route);
  }
  if (lower.startsWith("https://")) {
    return trimTrailingSlash(route);
  }

  // Ou somente uma rota relativa à base configurada.
  if (!route.startsWith("/")) route = "/" + route;
  return route;
}

String buildBackendUrl(const String &route) {
  if (!cfg.backendEnabled) return "";

  String target = route;
  target.trim();
  String lower = target;
  lower.toLowerCase();

  // Se o usuário informou URL completa, usa diretamente.
  if (lower.startsWith("http://")) {
    target="https://"+target.substring(7);
    return trimTrailingSlash(target);
  }
  if (lower.startsWith("https://")) {
    return trimTrailingSlash(target);
  }

  String base = trimTrailingSlash(cfg.serverUrl);
  if (base.isEmpty()) return "";
  if (!target.startsWith("/")) target = "/" + target;
  return base + target;
}

String buildCommandUrl() {
  return buildBackendUrl(cfg.commandRoute);
}

String buildHeartbeatUrl() {
  return buildBackendUrl(cfg.heartbeatRoute);
}

void clearControllerTimers() {
  safetyLostSinceMs = 0;
  pressureWaitStartedMs = 0;
  pressureDetectedMs = 0;
  moveStartedMs = 0;
  movementMonitorStartedMs = 0;
  lastMovementMs = 0;
  movementRefValid = false;
  movementGuard = MovementGuard();
  movementLastGeneration = gpsSampleGeneration;
  movementLastUsableSampleMs = 0;
  movementQualityRejects = movementAgeRejects = movementGeometryRejects = 0;
  safetyPrestartStartedMs = 0;
}

bool epochLooksValid(uint32_t epoch) {
  return epoch >= RTC_MIN_VALID_EPOCH;
}

void syncSystemClock(uint32_t epoch) {
  if (!epochLooksValid(epoch)) return;
  timeval tv={}; tv.tv_sec=epoch; settimeofday(&tv,nullptr);
}

bool tlsClockValid() {return epochLooksValid((uint32_t)time(nullptr));}

void setupRtc() {
  // Inicializa I2C uma vez para RTC DS3231 e demais módulos no barramento.
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);

  rtcPresent = rtc.begin();
  if (!rtcPresent) {
    rtcTimeValid = false;
    rtcLostPowerFlag = false;
    Serial.println("[RTC] DS3231 nao encontrado no I2C SDA=21 SCL=22");
    return;
  }

  rtcLostPowerFlag = rtc.lostPower();
  DateTime now = rtc.now();
  rtcTimeValid = epochLooksValid(now.unixtime());
  if (rtcTimeValid) syncSystemClock(now.unixtime());

  Serial.print("[RTC] DS3231 encontrado. Valido=");
  Serial.print(rtcTimeValid ? "SIM" : "NAO");
  Serial.print(" lostPower=");
  Serial.println(rtcLostPowerFlag ? "SIM" : "NAO");
}

void syncServerTimeFromCommand(const PivotCommand& cmd) {
  if (!epochLooksValid(cmd.serverTimeEpoch)) return;

  // O portal continua sendo a fonte principal quando estiver online.
  serverSyncEpoch = cmd.serverTimeEpoch;
  serverSyncMillis = millis();
  syncSystemClock(cmd.serverTimeEpoch);

  // O RTC físico fica como cópia de segurança para reinício/offline.
  if (rtcPresent) {
    DateTime rtcNow = rtc.now();
    int32_t diffSec = (int32_t)cmd.serverTimeEpoch - (int32_t)rtcNow.unixtime();
    if (!epochLooksValid(rtcNow.unixtime()) || abs(diffSec) > 2) {
      rtc.adjust(DateTime(cmd.serverTimeEpoch));
      Serial.println("[RTC] DS3231 sincronizado pelo horario do portal");
    }
    rtcTimeValid = true;
    rtcLostPowerFlag = false;
  }
}

uint32_t currentServerEpoch() {
  // 1) Enquanto a ESP nao reiniciar, usa a ultima hora recebida do portal + millis().
  if (epochLooksValid(serverSyncEpoch)) {
    return serverSyncEpoch + ((millis() - serverSyncMillis) / 1000UL);
  }

  // 2) Se reiniciou ou ficou sem portal, usa o RTC físico DS3231.
  if (rtcPresent) {
    DateTime now = rtc.now();
    uint32_t rtcEpoch = now.unixtime();
    if (epochLooksValid(rtcEpoch)) {
      rtcTimeValid = true;
      return rtcEpoch;
    }
  }

  // 3) Sem portal e sem RTC valido: nao toma decisão dependente de horario.
  rtcTimeValid = false;
  return 0;
}

String resetReasonToString(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_POWERON:   return "ENERGIZACAO / QUEDA DE ALIMENTACAO";
    case ESP_RST_EXT:       return "RESET EXTERNO";
    case ESP_RST_SW:        return "RESET POR SOFTWARE";
    case ESP_RST_PANIC:     return "ERRO/PANIC DO SISTEMA";
    case ESP_RST_INT_WDT:   return "WATCHDOG DE INTERRUPCAO";
    case ESP_RST_TASK_WDT:  return "WATCHDOG DE TAREFA";
    case ESP_RST_WDT:       return "WATCHDOG GERAL";
    case ESP_RST_DEEPSLEEP: return "RETORNO DE DEEP SLEEP";
    case ESP_RST_BROWNOUT:  return "BROWNOUT / TENSAO BAIXA";
    case ESP_RST_SDIO:      return "RESET SDIO";
    default:                return "RESET DESCONHECIDO (" + String((int)reason) + ")";
  }
}

String diagnosticTimestamp() {
  uint32_t epoch = currentServerEpoch();
  if (!epochLooksValid(epoch)) return "SEM_HORA_VALIDA";
  // Mato Grosso permanece UTC-4; o RTC guarda o epoch recebido do portal.
  DateTime localTime(epoch - 4UL * 3600UL);
  char out[32];
  snprintf(out, sizeof(out), "%04u-%02u-%02u %02u:%02u:%02u",
           localTime.year(), localTime.month(), localTime.day(),
           localTime.hour(), localTime.minute(), localTime.second());
  return String(out);
}

String diagnosticSnapshot(const String& detail) {
  bool loraFresh = loraGpsValid && loraLastRxMs > 0 && (millis() - loraLastRxMs < 10000UL);
  String s = detail;
  s += " | horaMT=" + diagnosticTimestamp();
  s += " up=" + String(millis() / 1000UL) + "s estado=" + stateToString(controllerState);
  s += " GPIO36=" + String(digitalRead(PIN_EMERGENCY_1));
  s += " GPIO39=" + String(digitalRead(PIN_EMERGENCY_2));
  s += " GPIO34=" + String(digitalRead(PIN_PRESSURE));
  s += " R=" + String(outputIsOnLogicLevel(PIN_OUT_RIGHT)) +
       String(outputIsOnLogicLevel(PIN_OUT_LEFT)) +
       String(outputIsOnLogicLevel(PIN_OUT_PUMP)) +
       String(outputIsOnLogicLevel(PIN_OUT_LAST_TOWER)) +
       String(outputIsOnLogicLevel(PIN_OUT_CANNON)) +
       String(outputIsOnLogicLevel(PIN_OUT_AUX6));
  s += " GPS=" + String(loraFresh);
  s += " lat=" + String(loraFresh ? loraGpsLat : gpsLat, 7);
  s += " lon=" + String(loraFresh ? loraGpsLon : gpsLon, 7);
  s += " ang=" + String(pivotAngleValid ? pivotAngleDeg : -1.0, 2);
  s += " wifi=" + String(WiFi.status() == WL_CONNECTED);
  s += " portalAge=" + String(lastServerOkMs ? (millis() - lastServerOkMs) / 1000UL : 0) + "s";
  s += " cmd=" + currentCmd.direction + " updatedAt=" + String(currentCmd.updatedAt);
  s += " cmdInv=" + String(invalidPortalCommandCount);
  s += " loraJsonErr=" + String(loraJsonErrorCount);
  s += " loraRej=" + String(loraIgnoredPackets);
  if (wifiLastFailure.length()) s += " wifiErro=" + wifiLastFailure;
  return s;
}

void appendDiagnosticHistory(const String& type, const String& reason, const String& detail) {
  diagEventCounter++;
  uint8_t slot = (diagEventCounter - 1) % DIAG_HISTORY_SLOTS;
  String event = "#" + String(diagEventCounter) + " [" + type + "] " + reason + " | " + diagnosticSnapshot(detail);
  diagHistory[slot] = event;
  diagDirtySlotsMask |= (uint16_t)(1U << slot);
  diagTotalDirty = true;
  // Nunca inicia escrita de flash com relés energizados. O evento já fica
  // disponível na RAM/portal e será persistido assim que todas as saídas
  // estiverem desligadas.
  flushDiagnosticHistoryIfSafe();
  Serial.println("[DIAG] " + event);
  enqueueStructuredEvent(type, reason, detail);
}

void flushDiagnosticHistoryIfSafe() {
  if (!diagTotalDirty && diagDirtySlotsMask == 0) return;
  if (outputIsOnLogicLevel(PIN_OUT_RIGHT) || outputIsOnLogicLevel(PIN_OUT_LEFT) ||
      outputIsOnLogicLevel(PIN_OUT_PUMP) || outputIsOnLogicLevel(PIN_OUT_LAST_TOWER) ||
      outputIsOnLogicLevel(PIN_OUT_CANNON) || outputIsOnLogicLevel(PIN_OUT_AUX6)) return;

  if (!prefs.begin("stopdiag", false)) {
    markPersistenceFailure("stopdiag", "nao abriu historico");
    return;
  }
  bool ok = true;
  for (uint8_t slot = 0; slot < DIAG_HISTORY_SLOTS; slot++) {
    if ((diagDirtySlotsMask & (uint16_t)(1U << slot)) == 0) continue;
    String key = "e" + String(slot);
    size_t written = prefs.putString(key.c_str(), diagHistory[slot]);
    String check = prefs.getString(key.c_str(), "");
    if (written != diagHistory[slot].length() || check != diagHistory[slot]) ok = false;
  }
  if (diagTotalDirty) {
    if (prefs.putUInt("total", diagEventCounter) != sizeof(uint32_t) ||
        prefs.getUInt("total", 0) != diagEventCounter) ok = false;
  }
  prefs.end();
  if (!ok) {
    markPersistenceFailure("stopdiag", "historico nao confirmado");
    return;
  }
  diagDirtySlotsMask = 0;
  diagTotalDirty = false;
  markPersistenceSuccess();
}

void loadDiagnostics() {
  prefs.begin("stopdiag", false);
  lastStopReason = prefs.getString("reason", "NENHUMA PARADA REGISTRADA");
  lastStopDetail = prefs.getString("detail", "");
  lastFaultReason = prefs.getString("freason", "NENHUMA FALHA REGISTRADA");
  lastFaultDetail = prefs.getString("fdetail", "");
  stopEventCounter = prefs.getUInt("stops", 0);
  faultEventCounter = prefs.getUInt("faults", 0);
  diagEventCounter = prefs.getUInt("total", 0);
  bootCounter = prefs.getUInt("boots", 0) + 1;
  for (uint8_t i = 0; i < DIAG_HISTORY_SLOTS; i++) {
    String key = "e" + String(i);
    diagHistory[i] = prefs.getString(key.c_str(), "");
  }
  // Limpeza única das posições usadas pelas revisões anteriores. Os campos
  // reason/detail/freason/fdetail e os contadores não são apagados.
  for (uint8_t i = DIAG_HISTORY_SLOTS; i < LEGACY_DIAG_HISTORY_SLOTS; i++) {
    String key = "e" + String(i);
    if (prefs.isKey(key.c_str())) prefs.remove(key.c_str());
  }
  bool bootStored = prefs.putUInt("boots", bootCounter) == sizeof(uint32_t);
  lastResetReason = resetReasonToString(esp_reset_reason());
  bool resetStored = prefs.putString("reset", lastResetReason) == lastResetReason.length();
  bool bootVerified = prefs.getUInt("boots", 0) == bootCounter;
  bool resetVerified = prefs.getString("reset", "") == lastResetReason;
  prefs.end();
  if (!bootStored || !resetStored || !bootVerified || !resetVerified) {
    markPersistenceFailure("stopdiag", "boot/reset nao confirmado");
  }

  loadStructuredEventOutbox();
  loadCommandExecution();
  loadOperationJournal();
  appendDiagnosticHistory("RESET", lastResetReason, "Inicializacao=" + String(bootCounter));
  recoverInterruptedOperation();
}

void recordStopEvent(const String& reason, const String& detail) {
  lastStopReason = reason;
  lastStopDetail = diagnosticSnapshot(detail);
  stopEventCounter++;
  bool stored = false;
  if (prefs.begin("stopdiag", false)) {
    stored = prefs.putString("reason", lastStopReason) == lastStopReason.length();
    stored = (prefs.putString("detail", lastStopDetail) == lastStopDetail.length()) && stored;
    stored = (prefs.putUInt("stops", stopEventCounter) == sizeof(uint32_t)) && stored;
    stored = stored && prefs.getString("reason", "") == lastStopReason &&
        prefs.getString("detail", "") == lastStopDetail &&
        prefs.getUInt("stops", 0) == stopEventCounter;
    prefs.end();
  }
  if (!stored) markPersistenceFailure("stopdiag", "ultima parada nao confirmada");
  else markPersistenceSuccess();
  appendDiagnosticHistory("PARADA", reason, detail);
}

void recordFaultEvent(const String& reason, const String& detail) {
  lastFaultReason = reason;
  lastFaultDetail = diagnosticSnapshot(detail);
  faultEventCounter++;
  bool stored = false;
  if (prefs.begin("stopdiag", false)) {
    stored = prefs.putString("freason", lastFaultReason) == lastFaultReason.length();
    stored = (prefs.putString("fdetail", lastFaultDetail) == lastFaultDetail.length()) && stored;
    stored = (prefs.putUInt("faults", faultEventCounter) == sizeof(uint32_t)) && stored;
    stored = stored && prefs.getString("freason", "") == lastFaultReason &&
        prefs.getString("fdetail", "") == lastFaultDetail &&
        prefs.getUInt("faults", 0) == faultEventCounter;
    prefs.end();
  }
  if (!stored) markPersistenceFailure("stopdiag", "ultima falha nao confirmada");
  else markPersistenceSuccess();
  appendDiagnosticHistory("FALHA", reason, detail);
}

String buildSystemEventLog() {
  // Uma unica variavel, ordenada do evento mais novo para o mais antigo.
  // O anel persistente evita crescimento ilimitado e permanece apos reset.
  String log;
  const size_t maxLogBytes = 6500;
  log.reserve(maxLogBytes + 1);
  uint32_t available = diagEventCounter < DIAG_HISTORY_SLOTS ?
      diagEventCounter : DIAG_HISTORY_SLOTS;
  for (uint32_t n = 0; n < available; n++) {
    uint8_t slot = (diagEventCounter - 1U - n) % DIAG_HISTORY_SLOTS;
    if (!diagHistory[slot].length()) continue;
    size_t separatorBytes = log.length() ? 1 : 0;
    size_t remaining = maxLogBytes > log.length() + separatorBytes ?
        maxLogBytes - log.length() - separatorBytes : 0;
    if (remaining == 0) break;
    if (separatorBytes) log += "\n";
    if (diagHistory[slot].length() <= remaining) {
      log += diagHistory[slot];
    } else {
      log += diagHistory[slot].substring(0, remaining);
      break;
    }
  }
  return log;
}

void copyPersistentText(char* destination, size_t capacity, const String& value) {
  if (capacity == 0) return;
  size_t length = value.length();
  if (length >= capacity) length = capacity - 1;
  memcpy(destination, value.c_str(), length);
  destination[length] = '\0';
}

uint32_t persistentRecordChecksum(const PersistentEventRecord& source) {
  PersistentEventRecord record = source;
  record.checksum = 0;
  const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&record);
  uint32_t hash = 2166136261UL;
  for (size_t i = 0; i < sizeof(record); i++) {
    hash ^= bytes[i];
    hash *= 16777619UL;
  }
  return hash;
}

bool persistentRecordValid(const PersistentEventRecord& record) {
  return record.magic == EVENT_RECORD_MAGIC &&
      record.schemaVersion == EVENT_SCHEMA_VERSION &&
      record.sequence > 0 && record.priority >= 1 && record.priority <= 3 &&
      record.checksum == persistentRecordChecksum(record);
}

void finalizePersistentRecord(PersistentEventRecord& record) {
  record.magic = EVENT_RECORD_MAGIC;
  record.schemaVersion = EVENT_SCHEMA_VERSION;
  if (record.occurrenceCount == 0) record.occurrenceCount = 1;
  if (record.lastTimestamp == 0) record.lastTimestamp = record.timestamp;
  record.checksum = persistentRecordChecksum(record);
}

uint8_t eventTypeToCode(const String& raw) {
  String value = raw; value.trim(); value.toUpperCase();
  if (value == "FALHA") return 1;
  if (value == "PARADA") return 2;
  if (value == "RESET") return 3;
  if (value == "COMANDO") return 4;
  if (value == "COMUNICACAO") return 5;
  if (value == "GPS") return 6;
  return 7;
}

String eventTypeFromCode(uint8_t code) {
  switch (code) {
    case 1: return "FALHA";
    case 2: return "PARADA";
    case 3: return "RESET";
    case 4: return "COMANDO";
    case 5: return "COMUNICACAO";
    case 6: return "GPS";
    default: return "SISTEMA";
  }
}

uint8_t commandResultToCode(const String& raw) {
  if (raw == "received") return 1;
  if (raw == "accepted") return 2;
  if (raw == "executing") return 3;
  if (raw == "succeeded") return 4;
  if (raw == "rejected") return 5;
  if (raw == "failed") return 6;
  if (raw == "ignored") return 7;
  return 0;
}

String commandResultFromCode(uint8_t code) {
  switch (code) {
    case 1: return "received";
    case 2: return "accepted";
    case 3: return "executing";
    case 4: return "succeeded";
    case 5: return "rejected";
    case 6: return "failed";
    case 7: return "ignored";
    default: return "none";
  }
}

uint8_t controllerStateFromText(const String& value) {
  for (uint8_t state = 0; state <= (uint8_t)STATE_FAULT_PERSISTENCE; state++) {
    if (stateToString((ControllerState)state) == value) return state;
  }
  return (uint8_t)STATE_IDLE;
}

String persistentEventId(const PersistentEventRecord& record) {
  return String(record.pivotId) + "-" + String(record.boot) + "-" + String(record.sequence);
}

String persistentEventKey(uint8_t slot) {
  char key[5];
  snprintf(key, sizeof(key), "e%02u", slot);
  return String(key);
}

void markPersistenceFailure(const String& component, const String& detail) {
  persistenceHealthy = false;
  persistenceFaultLatched = true;
  persistenceRecoveryEventPending = true;
  persistenceFailureCount++;
  persistenceLastFailureMs = millis();
  persistenceLastError = component + ": " + detail;
  Serial.println("[NVS][FALHA] " + persistenceLastError);
}

void markPersistenceSuccess() {
  persistenceHealthy = !persistenceRecoveryEventPending && !eventOutboxDirty &&
      !commandExecutionDirty && !operationJournalDirty && !operationJournalStartInterlock;
  persistenceLastSuccessMs = millis();
}

bool writePersistentEventSlot(uint8_t slot) {
  if (slot >= EVENT_OUTBOX_SLOTS || !persistentRecordValid(eventOutbox[slot])) return false;
  String key = persistentEventKey(slot);
  if (!prefs.begin("eventq", false)) {
    markPersistenceFailure("eventq", "nao abriu namespace");
    return false;
  }
  size_t written = prefs.putBytes(key.c_str(), &eventOutbox[slot], sizeof(PersistentEventRecord));
  PersistentEventRecord check = {};
  size_t read = prefs.getBytes(key.c_str(), &check, sizeof(check));
  prefs.end();
  bool ok = written == sizeof(PersistentEventRecord) &&
      read == sizeof(PersistentEventRecord) &&
      memcmp(&check, &eventOutbox[slot], sizeof(check)) == 0 &&
      persistentRecordValid(check);
  if (!ok) {
    markPersistenceFailure("eventq", "gravacao/leitura divergente em " + key);
    return false;
  }
  markPersistenceSuccess();
  return true;
}

bool erasePersistentEventSlot(uint8_t slot) {
  if (slot >= EVENT_OUTBOX_SLOTS) return false;
  bool wasValid = persistentRecordValid(eventOutbox[slot]);
  String key = persistentEventKey(slot);
  if (!prefs.begin("eventq", false)) {
    markPersistenceFailure("eventq", "nao abriu namespace para ACK");
    return false;
  }
  bool existed = prefs.isKey(key.c_str());
  bool removed = !existed || prefs.remove(key.c_str());
  bool verified = !prefs.isKey(key.c_str());
  prefs.end();
  if (!removed || !verified) {
    markPersistenceFailure("eventq", "nao removeu evento confirmado em " + key);
    return false;
  }
  memset(&eventOutbox[slot], 0, sizeof(eventOutbox[slot]));
  if (wasValid && eventOutboxCount > 0) eventOutboxCount--;
  markPersistenceSuccess();
  return true;
}

uint8_t buildOrderedEventSlots(uint8_t* ordered) {
  uint8_t count = 0;
  for (uint8_t slot = 0; slot < EVENT_OUTBOX_SLOTS; slot++) {
    if (persistentRecordValid(eventOutbox[slot])) ordered[count++] = slot;
  }
  for (uint8_t i = 1; i < count; i++) {
    uint8_t selected = ordered[i];
    int j = i - 1;
    while (j >= 0 && eventOutbox[ordered[j]].sequence > eventOutbox[selected].sequence) {
      ordered[j + 1] = ordered[j];
      j--;
    }
    ordered[j + 1] = selected;
  }
  return count;
}

int findEmptyPersistentEventSlot() {
  for (uint8_t slot = 0; slot < EVENT_OUTBOX_SLOTS; slot++) {
    if (!persistentRecordValid(eventOutbox[slot])) return slot;
  }
  return -1;
}

bool writeEventStorageMetadata() {
  if (!prefs.begin("eventq", false)) {
    markPersistenceFailure("eventq", "nao abriu metadados");
    return false;
  }
  bool ok = prefs.putUChar("fmt", EVENT_STORAGE_FORMAT) == sizeof(uint8_t);
  ok = (prefs.putUInt("seq", eventSequence) == sizeof(uint32_t)) && ok;
  ok = (prefs.putUInt("overflow", eventOutboxOverflowCount) == sizeof(uint32_t)) && ok;
  ok = (prefs.putUInt("dropnorm", eventOutboxDroppedNormalCount) == sizeof(uint32_t)) && ok;
  ok = (prefs.putUInt("critfull", eventOutboxCriticalOverflowCount) == sizeof(uint32_t)) && ok;
  ok = ok && prefs.getUChar("fmt", 0) == EVENT_STORAGE_FORMAT &&
      prefs.getUInt("seq", 0) == eventSequence;
  prefs.end();
  if (!ok) {
    markPersistenceFailure("eventq", "metadados nao confirmados");
    return false;
  }
  markPersistenceSuccess();
  return true;
}

PersistentEventRecord recordFromLegacyJson(const String& serialized) {
  PersistentEventRecord record = {};
  DynamicJsonDocument doc(1536);
  if (deserializeJson(doc, serialized)) return record;
  String eventId = String((const char*)(doc["eventId"] | ""));
  int lastDash = eventId.lastIndexOf('-');
  int previousDash = lastDash > 0 ? eventId.lastIndexOf('-', lastDash - 1) : -1;
  uint32_t sequence = lastDash >= 0 ? eventId.substring(lastDash + 1).toInt() : 0;
  uint32_t eventBoot = previousDash >= 0 ? eventId.substring(previousDash + 1, lastDash).toInt() : 0;
  String eventPivot = previousDash > 0 ? eventId.substring(0, previousDash) : cfg.pivotId;
  if (sequence == 0) sequence = ++eventSequence;
  if (eventBoot == 0) eventBoot = bootCounter;
  record.sequence = sequence;
  record.boot = eventBoot;
  record.timestamp = doc["timestamp"] | 0UL;
  record.lastTimestamp = record.timestamp;
  record.uptimeMs = doc["uptimeMs"] | 0UL;
  record.gpsAgeMs = doc["gpsAgeMs"] | 0UL;
  record.priority = doc["priority"] | structuredEventPriority(
      String((const char*)(doc["type"] | "")), String((const char*)(doc["code"] | "")));
  record.typeCode = eventTypeToCode(String((const char*)(doc["type"] | "")));
  record.commandResultCode = commandResultToCode(String((const char*)(doc["commandResult"] | "none")));
  record.controllerStateCode = controllerStateFromText(String((const char*)(doc["controllerState"] | "idle")));
  String outputs = String((const char*)(doc["outputs"] | "000000"));
  for (uint8_t bit = 0; bit < 6 && bit < outputs.length(); bit++) {
    if (outputs[bit] == '1') record.outputMask |= (1U << bit);
  }
  if (doc["safetyOk"] | false) record.flags |= 1U;
  if (doc["pressureOk"] | false) record.flags |= 2U;
  if (doc["gpsValid"] | false) record.flags |= 4U;
  record.loraChannel = doc["loraChannel"] | -1;
  record.occurrenceCount = doc["occurrenceCount"] | 1;
  copyPersistentText(record.pivotId, sizeof(record.pivotId), eventPivot);
  copyPersistentText(record.code, sizeof(record.code), String((const char*)(doc["code"] | "EVENTO LEGADO")));
  copyPersistentText(record.message, sizeof(record.message), String((const char*)(doc["message"] | "")));
  copyPersistentText(record.commandId, sizeof(record.commandId), String((const char*)(doc["commandId"] | "")));
  finalizePersistentRecord(record);
  return record;
}

void loadStructuredEventOutbox() {
  memset(eventOutbox, 0, sizeof(eventOutbox));
  eventOutboxCount = 0;
  if (!prefs.begin("eventq", false)) {
    markPersistenceFailure("eventq", "nao abriu durante inicializacao");
    return;
  }
  uint8_t format = prefs.getUChar("fmt", 0);
  eventSequence = prefs.getUInt("seq", 0);
  eventOutboxOverflowCount = prefs.getUInt("overflow", 0);
  eventOutboxDroppedNormalCount = prefs.getUInt("dropnorm", 0);
  eventOutboxCriticalOverflowCount = prefs.getUInt("critfull", 0);

  if (format == EVENT_STORAGE_FORMAT) {
    uint64_t corruptSlots = 0;
    for (uint8_t slot = 0; slot < EVENT_OUTBOX_SLOTS; slot++) {
      String key = persistentEventKey(slot);
      if (prefs.getBytesLength(key.c_str()) == 0) continue;
      PersistentEventRecord record = {};
      size_t read = prefs.getBytes(key.c_str(), &record, sizeof(record));
      if (read == sizeof(record) && persistentRecordValid(record)) {
        eventOutbox[slot] = record;
        eventOutboxCount++;
        if (record.sequence > eventSequence) eventSequence = record.sequence;
      } else {
        markPersistenceFailure("eventq", "registro binario corrompido em " + key);
        corruptSlots |= (1ULL << slot);
      }
    }
    // REV47.18.17 utilizava 40 posições e, junto com o histórico duplicado,
    // podia consumir todas as entradas da NVS. Libera as posições excedentes
    // sem tocar nos vinte registros mantidos nem nas configurações do pivô.
    for (uint8_t slot = EVENT_OUTBOX_SLOTS; slot < LEGACY_EVENT_OUTBOX_SLOTS; slot++) {
      String key = persistentEventKey(slot);
      if (prefs.isKey(key.c_str())) prefs.remove(key.c_str());
    }
    prefs.end();
    for (uint8_t slot = 0; slot < EVENT_OUTBOX_SLOTS; slot++) {
      if ((corruptSlots & (1ULL << slot)) != 0) erasePersistentEventSlot(slot);
    }
    if (persistenceHealthy) markPersistenceSuccess();
    return;
  }

  // Migração REV47.16/anteriores (12 posições) e REV47.17 (16 posições).
  // A capacidade antiga e inferida antes de percorrer o anel. Assim, uma fila
  // de 12 posições com head=10 preserva 10,11,0,1... exatamente nessa ordem.
  uint8_t legacyHead = prefs.getUChar("head", 0);
  uint8_t legacyCount = prefs.getUChar("count", 0);
  String legacy[16];
  bool upperSlotUsed = false;
  for (uint8_t slot = 0; slot < 16; slot++) {
    String key = "q" + String(slot);
    legacy[slot] = prefs.getString(key.c_str(), "");
    if (slot >= 12 && legacy[slot].length()) upperSlotUsed = true;
  }
  uint8_t legacyCapacity = upperSlotUsed ? 16 : 12;
  if (legacyHead >= legacyCapacity || legacyCount > legacyCapacity) {
    legacyHead = 0;
    legacyCount = legacyCapacity;
  }
  prefs.end();

  uint8_t migrated = 0;
  for (uint8_t offset = 0; offset < legacyCount && migrated < EVENT_OUTBOX_SLOTS; offset++) {
    uint8_t legacySlot = (legacyHead + offset) % legacyCapacity;
    if (!legacy[legacySlot].length()) continue;
    PersistentEventRecord record = recordFromLegacyJson(legacy[legacySlot]);
    if (!persistentRecordValid(record)) {
      eventOutboxOverflowCount++;
      continue;
    }
    eventOutbox[migrated] = record;
    eventOutboxCount++;
    if (record.sequence > eventSequence) eventSequence = record.sequence;
    migrated++;
  }

  // Os q0..q15 só são removidos depois que TODOS os registros binários e o
  // marcador de formato forem gravados e relidos. Uma queda no meio mantém o
  // formato antigo como fonte autoritativa no próximo boot.
  bool migrationOk = true;
  for (uint8_t slot = 0; slot < migrated; slot++) {
    if (!writePersistentEventSlot(slot)) migrationOk = false;
  }
  if (migrationOk && writeEventStorageMetadata()) {
    if (prefs.begin("eventq", false)) {
      for (uint8_t slot = 0; slot < 16; slot++) {
        String key = "q" + String(slot);
        if (prefs.isKey(key.c_str())) prefs.remove(key.c_str());
      }
      prefs.remove("head");
      prefs.remove("count");
      prefs.end();
    }
    markPersistenceSuccess();
  } else {
    eventOutboxDirty = true;
    markPersistenceFailure("eventq", "migracao 12/16 para 40 incompleta; legado preservado");
  }
}

void loadCommandExecution() {
  prefs.begin("cmdexec", true);
  String record = prefs.getString("record2", "");
  if (record.length()) {
    StaticJsonDocument<768> doc;
    if (!deserializeJson(doc, record) && (int)(doc["v"] | 0) == 2) {
      commandExecution.commandId = String((const char*)(doc["id"] | ""));
      commandExecution.requestedDirection = String((const char*)(doc["dir"] | "parado"));
      commandExecution.requestedMode = String((const char*)(doc["mode"] | "sem_agua"));
      commandExecution.result = String((const char*)(doc["result"] | "none"));
      commandExecution.reason = String((const char*)(doc["reason"] | ""));
      commandExecution.receivedAtMs = doc["recv"] | 0UL;
      commandExecution.changedAtMs = doc["changed"] | 0UL;
      commandExecution.completedAtMs = doc["done"] | 0UL;
      commandExecution.statusBootCounter = doc["boot"] | 0UL;
      commandExecution.payloadHash = doc["hash"] | 0UL;
      commandExecution.outputsConfirmed = doc["confirmed"] | false;
      prefs.end();
      return;
    }
    markPersistenceFailure("cmdexec", "record2 invalido; usando formato legado");
  }
  commandExecution.commandId = prefs.getString("id", "");
  commandExecution.requestedDirection = prefs.getString("dir", "parado");
  commandExecution.requestedMode = prefs.getString("mode", "sem_agua");
  commandExecution.result = prefs.getString("result", "none");
  commandExecution.reason = prefs.getString("reason", "");
  commandExecution.receivedAtMs = prefs.getUInt("recv", 0);
  commandExecution.changedAtMs = prefs.getUInt("changed", 0);
  commandExecution.completedAtMs = prefs.getUInt("done", 0);
  commandExecution.statusBootCounter = prefs.getUInt("boot", 0);
  commandExecution.payloadHash = prefs.getUInt("hash", 0);
  commandExecution.outputsConfirmed = prefs.getBool("confirmed", false);
  prefs.end();
}

uint32_t operationJournalChecksum(const OperationJournalInfo& journal, uint32_t generation) {
  uint32_t hash = 2166136261UL;
  auto mixText = [&hash](const String& value) {
    for (size_t i = 0; i < value.length(); i++) {
      hash ^= (uint8_t)value[i];
      hash *= 16777619UL;
    }
    hash ^= 0xFFU;
    hash *= 16777619UL;
  };
  auto mixU32 = [&hash](uint32_t value) {
    for (uint8_t shift = 0; shift < 32; shift += 8) {
      hash ^= (uint8_t)(value >> shift);
      hash *= 16777619UL;
    }
  };
  mixU32(generation);
  mixU32(journal.active ? 1UL : 0UL);
  mixText(journal.commandId);
  mixText(journal.direction);
  mixText(journal.mode);
  mixU32(journal.payloadHash);
  mixU32(journal.updatedAt);
  mixU32(journal.restartAfterPower ? 1UL : 0UL);
  mixText(journal.stage);
  return hash;
}

bool decodeOperationJournalRecord(const String& record, OperationJournalInfo& decoded,
                                  uint32_t& generation) {
  if (!record.length()) return false;
  StaticJsonDocument<640> doc;
  if (deserializeJson(doc, record) || (int)(doc["v"] | 0) != 2) return false;
  decoded = OperationJournalInfo();
  decoded.active = doc["active"] | false;
  decoded.commandId = String((const char*)(doc["id"] | ""));
  decoded.direction = String((const char*)(doc["dir"] | "parado"));
  decoded.mode = String((const char*)(doc["mode"] | "sem_agua"));
  decoded.payloadHash = doc["hash"] | 0UL;
  decoded.updatedAt = doc["updatedAt"] | 0UL;
  decoded.restartAfterPower = doc["restart"] | false;
  decoded.stage = String((const char*)(doc["stage"] | "unknown"));
  generation = doc["gen"] | 0UL;
  uint32_t storedChecksum = doc["crc"] | 0UL;
  if (generation == 0 || storedChecksum == 0 ||
      storedChecksum != operationJournalChecksum(decoded, generation)) return false;
  if (decoded.active && (decoded.commandId.isEmpty() || decoded.payloadHash == 0)) return false;
  return true;
}

void loadOperationJournal() {
  operationJournal = OperationJournalInfo();
  operationJournalGeneration = 0;
  operationJournalDirty = false;
  operationJournalStartInterlock = false;
  operationJournalInterlockReason = "";
  if (!prefs.begin("opjournal", true)) {
    operationJournalDirty = true;
    operationJournalStartInterlock = true;
    operationJournalInterlockReason = "namespace indisponivel no boot";
    markPersistenceFailure("opjournal", operationJournalInterlockReason);
    return;
  }
  String recordA = prefs.getString("recordA", "");
  String recordB = prefs.getString("recordB", "");
  String legacy = prefs.getString("record", "");
  prefs.end();

  OperationJournalInfo journalA;
  OperationJournalInfo journalB;
  uint32_t generationA = 0;
  uint32_t generationB = 0;
  bool validA = decodeOperationJournalRecord(recordA, journalA, generationA);
  bool validB = decodeOperationJournalRecord(recordB, journalB, generationB);
  bool hadV2Data = recordA.length() || recordB.length();

  if (validA || validB) {
    bool chooseB = validB && (!validA || (int32_t)(generationB - generationA) > 0);
    operationJournal = chooseB ? journalB : journalA;
    operationJournalGeneration = chooseB ? generationB : generationA;
    if (!validA || !validB) {
      operationJournalDirty = true;
      operationJournalStartInterlock = true;
      operationJournalInterlockReason = "uma copia do journal esta corrompida";
      // A cópia válida permanece como fonte da verdade. Recria imediatamente
      // a redundância antes do controlador sair do boot.
      if (writeOperationJournalRecord()) {
        ++operationJournalRepairCount;
        appendDiagnosticHistory("MEMORIA", "JOURNAL REDUNDANTE REPARADO",
            "Uma copia valida preservada; copia corrompida reconstruida");
      }
    }
    return;
  }

  // Migração do registro único da REV47.18.4. O registro antigo só é removido
  // depois que uma cópia V2 com geração e checksum for escrita e relida.
  if (legacy.length()) {
    StaticJsonDocument<512> doc;
    if (!deserializeJson(doc, legacy) && (int)(doc["v"] | 0) == 1) {
      operationJournal.active = doc["active"] | false;
      operationJournal.commandId = String((const char*)(doc["id"] | ""));
      operationJournal.direction = String((const char*)(doc["dir"] | "parado"));
      operationJournal.mode = String((const char*)(doc["mode"] | "sem_agua"));
      operationJournal.payloadHash = doc["hash"] | 0UL;
      operationJournal.updatedAt = doc["updatedAt"] | 0UL;
      operationJournal.restartAfterPower = doc["restart"] | false;
      operationJournal.stage = String((const char*)(doc["stage"] | "legacy"));
      if (!operationJournal.active ||
          (!operationJournal.commandId.isEmpty() && operationJournal.payloadHash != 0)) {
        operationJournalDirty = true;
        if (writeOperationJournalRecord()) return;
      }
    }
  }

  if (hadV2Data || legacy.length()) {
    operationJournal = OperationJournalInfo();
    operationJournalDirty = true;
    operationJournalStartInterlock = true;
    operationJournalInterlockReason = "nenhuma copia valida do journal";
    // Se o último resultado persistido já é terminal (ou nunca houve comando),
    // não existe operação que possa ser retomada. Com todas as saídas OFF é
    // seguro reconstruir um journal inativo. Estado não terminal continua
    // bloqueado até o portal enviar PARAR.
    bool terminalOrUnused = commandExecution.commandId.isEmpty() || commandExecutionIsTerminal();
    if (terminalOrUnused && allControlledOutputsAreOff()) {
      operationJournal.active = false;
      operationJournal.stage = "auto_recovered_terminal";
      if (writeOperationJournalRecord()) {
        ++operationJournalRepairCount;
        appendDiagnosticHistory("MEMORIA", "JOURNAL INATIVO RECONSTRUIDO",
            "Registros invalidos; saidas OFF e ultimo comando terminal");
        return;
      }
      return; // writeOperationJournalRecord preserva a causa exata da falha.
    }
    markPersistenceFailure("opjournal", operationJournalInterlockReason +
        "; estado anterior nao terminal exige PARAR");
  }
}

void reclaimNvsSpaceForOperationJournal(bool aggressive = false) {
  // Nunca faz manutenção de flash com uma saída energizada. Se a falha surgir
  // durante operação, o controlador primeiro executa a parada segura e a
  // rotina periódica tentará novamente com todas as saídas OFF.
  if (!allControlledOutputsAreOff()) return;

  // O histórico detalhado é duplicado pela fila estruturada. Em emergência,
  // libera somente as cópias e preserva último motivo, contadores e eventos.
  if (prefs.begin("stopdiag", false)) {
    if (aggressive) {
      // Último recurso antes de bloquear a partida: remove apenas o namespace
      // de diagnóstico duplicado. Configuração, journal, comando e fila de
      // eventos estruturados ficam intactos.
      prefs.clear();
      diagDirtySlotsMask = 0;
      diagTotalDirty = false;
    } else {
      for (uint8_t slot = 0; slot < LEGACY_DIAG_HISTORY_SLOTS; slot++) {
        String key = "e" + String(slot);
        if (prefs.isKey(key.c_str())) prefs.remove(key.c_str());
      }
    }
    prefs.end();
  }

  // Remove posições que pertenciam à fila de 40 registros da REV47.18.17.
  if (prefs.begin("eventq", false)) {
    for (uint8_t slot = EVENT_OUTBOX_SLOTS; slot < LEGACY_EVENT_OUTBOX_SLOTS; slot++) {
      String key = persistentEventKey(slot);
      if (prefs.isKey(key.c_str())) prefs.remove(key.c_str());
    }
    prefs.end();
  }

  // Chaves legadas já substituídas pelos registros verificados atuais.
  if (prefs.begin("opjournal", false)) {
    if (prefs.isKey("record")) prefs.remove("record");
    prefs.end();
  }
  if (prefs.begin("cmdexec", false)) {
    const char* legacyKeys[] = {"id", "dir", "mode", "result", "reason",
                                "recv", "changed", "done", "boot", "hash", "confirmed"};
    for (const char* key : legacyKeys) if (prefs.isKey(key)) prefs.remove(key);
    prefs.end();
  }
}

bool writeOperationJournalRecord() {
  uint32_t newGeneration = operationJournalGeneration + 1UL;
  if (newGeneration == 0) newGeneration = 1;
  const char* targetKey = (newGeneration & 1U) ? "recordA" : "recordB";
  StaticJsonDocument<640> doc;
  doc["v"] = 2;
  doc["gen"] = newGeneration;
  doc["active"] = operationJournal.active;
  doc["id"] = operationJournal.commandId;
  doc["dir"] = operationJournal.direction;
  doc["mode"] = operationJournal.mode;
  doc["hash"] = operationJournal.payloadHash;
  doc["updatedAt"] = operationJournal.updatedAt;
  doc["restart"] = operationJournal.restartAfterPower;
  doc["stage"] = operationJournal.stage;
  doc["crc"] = operationJournalChecksum(operationJournal, newGeneration);
  String record;
  serializeJson(doc, record);

  bool ok = false;
  String failureStage = "nao executado";
  for (uint8_t attempt = 1; attempt <= OPERATION_JOURNAL_WRITE_ATTEMPTS; attempt++) {
    if (!prefs.begin("opjournal", false)) {
      failureStage = "abertura do namespace falhou";
      if (attempt == 1) reclaimNvsSpaceForOperationJournal(false);
      else if (attempt == 2) reclaimNvsSpaceForOperationJournal(true);
    } else {
      size_t written = prefs.putString(targetKey, record);
      String check = prefs.getString(targetKey, "");
      prefs.end();

      OperationJournalInfo verified;
      uint32_t verifiedGeneration = 0;
      bool decoded = decodeOperationJournalRecord(check, verified, verifiedGeneration);
      if (written != record.length()) {
        failureStage = "gravacao incompleta " + String(written) + "/" + String(record.length());
      } else if (check != record) {
        failureStage = "releitura diferente da gravacao";
      } else if (!decoded) {
        failureStage = "checksum ou formato invalido na releitura";
      } else if (verifiedGeneration != newGeneration) {
        failureStage = "geracao divergente na releitura";
      } else if (verified.active != operationJournal.active ||
                 verified.commandId != operationJournal.commandId ||
                 verified.payloadHash != operationJournal.payloadHash ||
                 verified.stage != operationJournal.stage) {
        failureStage = "campos divergentes na releitura";
      } else {
        ok = true;
        break;
      }
      if (written == 0 && attempt == 1) reclaimNvsSpaceForOperationJournal(false);
      else if (written == 0 && attempt == 2) reclaimNvsSpaceForOperationJournal(true);
    }
    esp_task_wdt_reset();
    delay(15UL * attempt);
  }
  if (!ok) {
    operationJournalStartInterlock = true;
    operationJournalInterlockReason = failureStage + " apos " +
        String(OPERATION_JOURNAL_WRITE_ATTEMPTS) + " tentativas";
    markPersistenceFailure("opjournal", operationJournalInterlockReason);
    return false;
  }

  operationJournalGeneration = newGeneration;
  operationJournalDirty = false;
  operationJournalStartInterlock = false;
  operationJournalInterlockReason = "";
  // Remoção do legado ocorre somente depois do commit V2 confirmado.
  if (prefs.begin("opjournal", false)) {
    if (prefs.isKey("record")) prefs.remove("record");
    prefs.end();
  }
  markPersistenceSuccess();
  return true;
}

bool flushOperationJournalIfNeeded() {
  if (!operationJournalDirty) return !operationJournalStartInterlock;
  return writeOperationJournalRecord();
}

bool armOperationJournal(const PivotCommand& cmd, const String& stage) {
  operationJournal.active = true;
  operationJournal.commandId = cmd.commandId;
  // Comandos locais nao possuem ID do portal, mas o journal exige identidade.
  // Mantem-se o bloqueio para comandos remotos sem ID.
  if(operationJournal.commandId.isEmpty() && localTestMode)
    operationJournal.commandId = String("LOCAL-")+String(bootCounter)+"-"+String(millis());
  operationJournal.direction = normalizeDirectionStr(cmd.direction);
  operationJournal.mode = normalizeModeStr(cmd.mode);
  operationJournal.payloadHash = calculateCommandPayloadHash(cmd);
  operationJournal.updatedAt = cmd.updatedAt;
  operationJournal.restartAfterPower = !localTestMode && cmd.restartAfterPower;
  operationJournal.stage = stage;
  operationJournalDirty = true;
  return flushOperationJournalIfNeeded();
}

bool clearOperationJournal(const String& stage) {
  operationJournal.active = false;
  operationJournal.stage = stage;
  operationJournalDirty = true;
  return flushOperationJournalIfNeeded();
}

void recoverInterruptedOperation() {
  if (!operationJournal.active) return;

  String detail = "commandId=" + operationJournal.commandId +
      " etapa=" + operationJournal.stage +
      " direcao=" + operationJournal.direction +
      " restartAfterPower=" + String(operationJournal.restartAfterPower ? 1 : 0);
  commandExecution.payloadHash = operationJournal.payloadHash;
  if (operationJournal.restartAfterPower) {
    autoRestart.required = true;
    autoRestart.pending = false;
    autoRestart.portalAuthorized = false;
    autoRestart.commandId = operationJournal.commandId;
    autoRestart.payloadHash = operationJournal.payloadHash;
    autoRestart.notBeforeMs = bootStartedMs + AUTO_RESTART_MIN_OFF_MS;
    autoRestart.lastAuthorizationMs = 0;
    autoRestart.status = "aguardando_reautorizacao_portal";
    setCommandExecution(operationJournal.commandId, operationJournal.direction,
                        operationJournal.mode, "accepted",
                        "Operacao interrompida; aguardando portal e bloqueio minimo de 15 minutos", false);
  } else {
    setCommandExecution(operationJournal.commandId, operationJournal.direction,
                        operationJournal.mode, "ignored",
                        "Operacao interrompida e retomada automatica nao autorizada", true);
  }
  appendDiagnosticHistory("FALHA", "OPERACAO INTERROMPIDA POR RESET", detail);
  // Preserve a operacao pendente entre quedas sucessivas de energia.
  // Somente uma cancelamento confirmado ou nova partida altera o journal.
  if (!operationJournal.restartAfterPower) clearOperationJournal("recovered_without_restart");
  commandExecutionDirty = true;
  persistCommandExecutionIfSafe();
}

bool commandMatchesAutoRestart(const PivotCommand& cmd) {
  return autoRestart.required &&
         cmd.commandId == autoRestart.commandId &&
         calculateCommandPayloadHash(cmd) == autoRestart.payloadHash &&
         normalizeDirectionStr(cmd.direction) != "parado" &&
         cmd.restartAfterPower;
}

void cancelAutoRestart(const String& reason, bool logEvent) {
  if (!autoRestart.required && !autoRestart.pending) return;
  String oldId = autoRestart.commandId;
  cancelledRestartCommandId = oldId; // retransmissao antiga nao pode virar partida manual
  autoRestart.required = false;
  autoRestart.pending = false;
  autoRestart.portalAuthorized = false;
  autoRestart.commandId = "";
  autoRestart.payloadHash = 0;
  autoRestart.lastAuthorizationMs = 0;
  autoRestart.status = "cancelado: " + reason;
  autoRestartCommand = PivotCommand();
  // Cancelamento deve sobreviver a novo reset; falha NVS bloqueia partidas.
  if (operationJournal.active) clearOperationJournal("auto_restart_cancelled");
  if (logEvent) {
    appendDiagnosticHistory("COMANDO", "RELIGAMENTO AUTOMATICO CANCELADO",
        "commandId=" + oldId + " motivo=" + reason);
  }
}

void authorizeAutoRestart(const PivotCommand& cmd) {
  autoRestartCommand = cmd;
  autoRestart.pending = true;
  autoRestart.portalAuthorized = true;
  autoRestart.lastAuthorizationMs = millis();
  autoRestart.status = millis() < autoRestart.notBeforeMs
      ? "aguardando_prazo_15_minutos"
      : "aguardando_verificacoes_finais";
  allOutputsOff();
  setCommandExecution(cmd.commandId, cmd.direction, cmd.mode, "accepted",
      "Religamento automatico autorizado; aguardando prazo minimo de 15 minutos", false);
}

void processAutoRestart() {
  if (!autoRestart.pending) return;
  bool freshAuthorization = autoRestart.portalAuthorized &&
      autoRestart.lastAuthorizationMs > 0 &&
      millis() - autoRestart.lastAuthorizationMs <= AUTO_RESTART_AUTH_FRESH_MS;
  if (!freshAuthorization) {
    autoRestart.portalAuthorized = false;
    autoRestart.status = "aguardando_reautorizacao_portal";
    allOutputsOff();
    return;
  }
  if (millis() < autoRestart.notBeforeMs) {
    autoRestart.status = "aguardando_prazo_15_minutos";
    allOutputsOff();
    return;
  }
  if (bootGuardActive()) {
    autoRestart.status = "aguardando_boot";
    allOutputsOff();
    return;
  }
  PivotCommand commandToRestart = autoRestartCommand;
  String restartId = autoRestart.commandId;
  // Nao remover a intencao persistente antes de armar a nova partida.
  // Se houver reset durante a sequencia, a operacao permanece recuperavel.
  autoRestart.required = false;
  autoRestart.pending = false;
  autoRestart.portalAuthorized = false;
  autoRestart.status = "iniciando_sequencia_segura";
  appendDiagnosticHistory("COMANDO", "RELIGAMENTO AUTOMATICO LIBERADO",
      "commandId=" + restartId + " prazoMinimo=900s; refazendo seguranca, GPS e pressao");
  startCommandSequence(commandToRestart);
}

void persistCommandExecutionIfSafe() {
  if (!commandExecutionDirty) return;
  StaticJsonDocument<768> doc;
  doc["v"] = 2;
  doc["id"] = commandExecution.commandId;
  doc["dir"] = commandExecution.requestedDirection;
  doc["mode"] = commandExecution.requestedMode;
  doc["result"] = commandExecution.result;
  doc["reason"] = commandExecution.reason;
  doc["recv"] = commandExecution.receivedAtMs;
  doc["changed"] = commandExecution.changedAtMs;
  doc["done"] = commandExecution.completedAtMs;
  doc["boot"] = commandExecution.statusBootCounter;
  doc["hash"] = commandExecution.payloadHash;
  doc["confirmed"] = commandExecution.outputsConfirmed;
  String record;
  serializeJson(doc, record);
  if (!prefs.begin("cmdexec", false)) {
    markPersistenceFailure("cmdexec", "nao abriu namespace");
    return;
  }
  size_t written = prefs.putString("record2", record);
  String check = prefs.getString("record2", "");
  prefs.end();
  if (written != record.length() || check != record) {
    markPersistenceFailure("cmdexec", "resultado do comando nao confirmado");
    return;
  }
  commandExecutionDirty = false;
  markPersistenceSuccess();
}

void setCommandExecution(const String& commandId, const String& direction,
                         const String& mode, const String& result,
                         const String& reason, bool completed) {
  bool newCommand = commandExecution.commandId != commandId;
  commandExecution.commandId = commandId;
  commandExecution.requestedDirection = direction;
  commandExecution.requestedMode = mode;
  commandExecution.result = result;
  commandExecution.reason = reason;
  if (newCommand || commandExecution.receivedAtMs == 0) commandExecution.receivedAtMs = millis();
  commandExecution.changedAtMs = millis();
  ++commandExecutionRevision;
  commandExecution.completedAtMs = completed ? millis() : 0;
  commandExecution.statusBootCounter = bootCounter;
  // accepted/received nunca significam saidas confirmadas. executing somente
  // e atribuido por trackCommandAppliedEvent depois da leitura real das saidas.
  commandExecution.outputsConfirmed = result == "executing" || result == "succeeded";
  commandExecutionDirty = true;
  persistCommandExecutionIfSafe();
}

void flushStructuredEventOutboxIfSafe() {
  if (!eventOutboxDirty) return;
  bool ok = true;
  uint8_t counted = 0;
  for (uint8_t slot = 0; slot < EVENT_OUTBOX_SLOTS; slot++) {
    if (!persistentRecordValid(eventOutbox[slot])) continue;
    counted++;
    if (!writePersistentEventSlot(slot)) ok = false;
  }
  eventOutboxCount = counted;
  if (!writeEventStorageMetadata()) ok = false;
  if (ok) {
    eventOutboxDirty = false;
    markPersistenceSuccess();
  }
}

uint8_t structuredEventPriority(const String& type, const String& code) {
  String normalizedType = type;
  normalizedType.trim();
  normalizedType.toUpperCase();
  if (normalizedType == "FALHA" || normalizedType == "PARADA" ||
      normalizedType == "RESET") return 3;
  if (normalizedType == "COMANDO") {
    String normalizedCode = code;
    normalizedCode.toUpperCase();
    if (normalizedCode.indexOf("REJEIT") >= 0 ||
        normalizedCode.indexOf("IGNOR") >= 0 ||
        normalizedCode.indexOf("BLOQUE") >= 0 ||
        normalizedCode.indexOf("CONFLITO") >= 0) return 2;
  }
  return 1;
}

uint8_t storedEventPriority(const PersistentEventRecord& record) {
  return persistentRecordValid(record) ? constrain(record.priority, 1, 3) : 1;
}

uint8_t countNormalPendingEvents() {
  uint8_t count = 0;
  for (uint8_t slot = 0; slot < EVENT_OUTBOX_SLOTS; slot++) {
    if (persistentRecordValid(eventOutbox[slot]) && storedEventPriority(eventOutbox[slot]) < 3) count++;
  }
  return count;
}

void removePendingEventAtOffset(uint8_t offset) {
  uint8_t ordered[EVENT_OUTBOX_SLOTS];
  uint8_t count = buildOrderedEventSlots(ordered);
  if (offset >= count) return;
  erasePersistentEventSlot(ordered[offset]);
}

void enqueueStructuredEvent(const String& type, const String& code, const String& message) {
  const uint8_t incomingPriority = structuredEventPriority(type, code);

  // Falhas repetidas com o mesmo tipo/motivo sao consolidadas. O primeiro e
  // o ultimo horario, alem da quantidade, permanecem no mesmo registro. Isso
  // impede que oscilacoes de uma unica protecao ocupem toda a reserva critica.
  if (incomingPriority == 3) {
    const uint8_t typeCode = eventTypeToCode(type);
    for (uint8_t slot = 0; slot < EVENT_OUTBOX_SLOTS; slot++) {
      PersistentEventRecord& existing = eventOutbox[slot];
      if (!persistentRecordValid(existing) || existing.priority != 3 ||
          existing.typeCode != typeCode || String(existing.code) != code) continue;
      if (existing.occurrenceCount < 65535U) existing.occurrenceCount++;
      existing.lastTimestamp = currentServerEpoch();
      if (message.length() >= sizeof(existing.message)) existing.flags |= (1U << 4);
      copyPersistentText(existing.message, sizeof(existing.message), message);
      finalizePersistentRecord(existing);
      if (!writePersistentEventSlot(slot)) eventOutboxDirty = true;
      writeEventStorageMetadata();
      return;
    }
  }

  // Eventos normais nao ocupam a reserva destinada a falhas, paradas e reset.
  if (incomingPriority < 3 && countNormalPendingEvents() >= EVENT_NORMAL_LIMIT) {
    eventOutboxOverflowCount++;
    eventOutboxDroppedNormalCount++;
    eventOutboxDirty = true;
    flushStructuredEventOutboxIfSafe();
    return;
  }

  if (eventOutboxCount >= EVENT_OUTBOX_SLOTS) {
    // Uma falha/parada pode substituir o evento normal mais antigo, mas nunca
    // apaga outro evento critico que ainda aguarda ACK do servidor.
    int replaceSlot = -1;
    if (incomingPriority == 3) {
      uint8_t ordered[EVENT_OUTBOX_SLOTS];
      uint8_t orderedCount = buildOrderedEventSlots(ordered);
      for (uint8_t n = 0; n < orderedCount; n++) {
        uint8_t slot = ordered[n];
        if (storedEventPriority(eventOutbox[slot]) < 3) {
          replaceSlot = slot;
          break;
        }
      }
    }
    if (replaceSlot >= 0 && erasePersistentEventSlot((uint8_t)replaceSlot)) {
      eventOutboxOverflowCount++;
      eventOutboxDroppedNormalCount++;
    } else {
      eventOutboxOverflowCount++;
      if (incomingPriority == 3) eventOutboxCriticalOverflowCount++;
      else eventOutboxDroppedNormalCount++;
      eventOutboxDirty = true;
      flushStructuredEventOutboxIfSafe();
      return;
    }
  }

  eventSequence++;
  PersistentEventRecord record = {};
  record.sequence = eventSequence;
  record.boot = bootCounter;
  record.timestamp = currentServerEpoch();
  record.lastTimestamp = record.timestamp;
  record.uptimeMs = millis();
  record.gpsAgeMs = lastGoodGpsMs ? (uint32_t)(millis() - lastGoodGpsMs) : 0;
  record.occurrenceCount = 1;
  record.priority = incomingPriority;
  record.typeCode = eventTypeToCode(type);
  record.commandResultCode = commandResultToCode(commandExecution.result);
  record.controllerStateCode = (uint8_t)controllerState;
  if (outputIsOnLogicLevel(PIN_OUT_RIGHT)) record.outputMask |= 1U << 0;
  if (outputIsOnLogicLevel(PIN_OUT_LEFT)) record.outputMask |= 1U << 1;
  if (outputIsOnLogicLevel(PIN_OUT_PUMP)) record.outputMask |= 1U << 2;
  if (outputIsOnLogicLevel(PIN_OUT_LAST_TOWER)) record.outputMask |= 1U << 3;
  if (outputIsOnLogicLevel(PIN_OUT_CANNON)) record.outputMask |= 1U << 4;
  if (outputIsOnLogicLevel(PIN_OUT_AUX6)) record.outputMask |= 1U << 5;
  if (safetyOk()) record.flags |= 1U;
  if (pressureOk()) record.flags |= 2U;
  if (hasGoodGpsWithinTimeout()) record.flags |= 4U;
  if (code.length() >= sizeof(record.code)) record.flags |= 1U << 3;
  if (message.length() >= sizeof(record.message)) record.flags |= 1U << 4;
  String eventCommandId = commandExecution.commandId.length() ?
      commandExecution.commandId : lastPortalCommandId;
  if (eventCommandId.length() >= sizeof(record.commandId)) record.flags |= 1U << 5;
  record.loraChannel = loraReportedChannel;
  copyPersistentText(record.pivotId, sizeof(record.pivotId), cfg.pivotId);
  copyPersistentText(record.code, sizeof(record.code), code);
  copyPersistentText(record.message, sizeof(record.message), message);
  copyPersistentText(record.commandId, sizeof(record.commandId), eventCommandId);
  finalizePersistentRecord(record);

  int freeSlot = findEmptyPersistentEventSlot();
  if (freeSlot < 0) {
    eventOutboxOverflowCount++;
    if (incomingPriority == 3) eventOutboxCriticalOverflowCount++;
    else eventOutboxDroppedNormalCount++;
    eventOutboxDirty = true;
    flushStructuredEventOutboxIfSafe();
    return;
  }
  eventOutbox[freeSlot] = record;
  eventOutboxCount++;
  if (!writePersistentEventSlot((uint8_t)freeSlot)) eventOutboxDirty = true;
  if (!writeEventStorageMetadata()) eventOutboxDirty = true;
}

uint8_t addStructuredEventsToHeartbeat(DynamicJsonDocument& doc) {
  JsonArray events = doc.createNestedArray("events");
  uint8_t batchCount = 0;
  uint8_t ordered[EVENT_OUTBOX_SLOTS];
  uint8_t orderedCount = buildOrderedEventSlots(ordered);
  for (uint8_t n = 0; n < orderedCount && batchCount < EVENT_BATCH_MAX; n++) {
    const PersistentEventRecord& record = eventOutbox[ordered[n]];
    JsonObject eventDoc = events.createNestedObject();
    eventDoc["eventId"] = persistentEventId(record);
    eventDoc["schemaVersion"] = record.schemaVersion;
    eventDoc["pivotId"] = record.pivotId;
    eventDoc["firmware"] = FIRMWARE_VERSION;
    eventDoc["type"] = eventTypeFromCode(record.typeCode);
    eventDoc["code"] = record.code;
    eventDoc["message"] = record.message;
    eventDoc["priority"] = record.priority;
    eventDoc["critical"] = record.priority == 3;
    eventDoc["timestamp"] = record.timestamp;
    eventDoc["lastTimestamp"] = record.lastTimestamp;
    eventDoc["occurrenceCount"] = record.occurrenceCount;
    eventDoc["uptimeMs"] = record.uptimeMs;
    eventDoc["commandId"] = record.commandId;
    eventDoc["commandResult"] = commandResultFromCode(record.commandResultCode);
    eventDoc["controllerState"] = stateToString((ControllerState)record.controllerStateCode);
    eventDoc["safetyOk"] = (record.flags & 1U) != 0;
    eventDoc["pressureOk"] = (record.flags & 2U) != 0;
    String outputs;
    outputs.reserve(6);
    for (uint8_t bit = 0; bit < 6; bit++) outputs += (record.outputMask & (1U << bit)) ? '1' : '0';
    eventDoc["outputs"] = outputs;
    eventDoc["gpsValid"] = (record.flags & 4U) != 0;
    eventDoc["gpsAgeMs"] = record.gpsAgeMs;
    eventDoc["loraChannel"] = record.loraChannel;
    eventDoc["codeTruncated"] = (record.flags & (1U << 3)) != 0;
    eventDoc["messageTruncated"] = (record.flags & (1U << 4)) != 0;
    eventDoc["commandIdTruncated"] = (record.flags & (1U << 5)) != 0;
    batchCount++;
  }
  doc["eventsPending"] = eventOutboxCount;
  doc["eventsBatchCount"] = batchCount;
  doc["eventsMorePending"] = eventOutboxCount > batchCount;
  doc["eventsCapacity"] = EVENT_OUTBOX_SLOTS;
  doc["eventsNormalLimit"] = EVENT_NORMAL_LIMIT;
  doc["eventsCriticalReserve"] = EVENT_CRITICAL_RESERVE;
  doc["eventsOverflow"] = eventOutboxOverflowCount;
  doc["eventsDroppedNormal"] = eventOutboxDroppedNormalCount;
  doc["eventsCriticalOverflow"] = eventOutboxCriticalOverflowCount;
  doc["eventsRequireAck"] = true;
  doc["persistenceHealthy"] = persistenceHealthy;
  doc["persistenceFaultLatched"] = persistenceFaultLatched;
  doc["persistenceFailureCount"] = persistenceFailureCount;
  doc["persistenceLastError"] = persistenceLastError;
  return batchCount;
}

bool jsonAckArrayContains(JsonArray array, const String& eventId) {
  if (array.isNull()) return false;
  for (JsonVariant value : array) {
    if (value.is<const char*>() && eventId == String(value.as<const char*>())) return true;
  }
  return false;
}

void processStructuredEventAcks(const String& response) {
  if (!response.length() || eventOutboxCount == 0) return;
  DynamicJsonDocument ackDoc(4096);
  if (deserializeJson(ackDoc, response)) return;
  JsonArray ackIds = ackDoc["ackEventIds"].as<JsonArray>();
  if (ackIds.isNull()) ackIds = ackDoc["eventAckIds"].as<JsonArray>();
  if (ackIds.isNull()) ackIds = ackDoc["data"]["ackEventIds"].as<JsonArray>();
  String singleAck = String((const char*)(ackDoc["ackEventId"] | ""));
  String ackThrough = String((const char*)(ackDoc["ackThroughEventId"] | ""));
  // Um ACK cumulativo so e aceito se o ID realmente existir na fila local.
  // Isso impede que uma resposta antiga, incorreta ou de outro equipamento
  // apague eventos ainda nao confirmados.
  bool validAckThrough = false;
  uint8_t ordered[EVENT_OUTBOX_SLOTS];
  uint8_t orderedCount = buildOrderedEventSlots(ordered);
  if (!ackThrough.isEmpty()) {
    for (uint8_t n = 0; n < orderedCount; n++) {
      uint8_t candidateSlot = ordered[n];
      String candidateId = persistentEventId(eventOutbox[candidateSlot]);
      if (candidateId == ackThrough) {
        validAckThrough = true;
        break;
      }
    }
  }
  bool changed = false;

  while (eventOutboxCount > 0) {
    orderedCount = buildOrderedEventSlots(ordered);
    if (orderedCount == 0) break;
    uint8_t slot = ordered[0];
    String eventId = persistentEventId(eventOutbox[slot]);
    bool acknowledged = eventId.length() &&
        (eventId == singleAck || jsonAckArrayContains(ackIds, eventId));
    if (validAckThrough) acknowledged = true;
    if (!acknowledged) break;

    if (!erasePersistentEventSlot(slot)) break;
    changed = true;
    if (validAckThrough && eventId == ackThrough) {
      validAckThrough = false;
      ackThrough = "";
    }
  }

  if (!changed) return;
  if (!writeEventStorageMetadata()) eventOutboxDirty = true;
}

void servicePersistenceRetries() {
  uint32_t now = millis();
  if (now - persistenceLastRetryMs < 5000UL) return;
  persistenceLastRetryMs = now;
  flushOperationJournalIfNeeded();
  persistCommandExecutionIfSafe();
  flushStructuredEventOutboxIfSafe();
  flushDiagnosticHistoryIfSafe();
  bool allPendingWritesStored = !eventOutboxDirty && !commandExecutionDirty &&
      !operationJournalDirty && !operationJournalStartInterlock &&
      !diagTotalDirty && diagDirtySlotsMask == 0;
  if (persistenceRecoveryEventPending && allPendingWritesStored &&
      !persistenceRecoveryEventInProgress) {
    persistenceRecoveryEventInProgress = true;
    String failureDetail = persistenceLastError +
        "; ocorrencias=" + String(persistenceFailureCount) +
        "; armazenamento voltou a responder";
    enqueueStructuredEvent("FALHA", "FALHA DE PERSISTENCIA NVS", failureDetail);
    if (!eventOutboxDirty) {
      persistenceRecoveryEventPending = false;
      persistenceHealthy = true;
      persistenceLastSuccessMs = millis();
    }
    persistenceRecoveryEventInProgress = false;
  }
}

void notePortalCommunicationFailure(const String& reason) {
  uint32_t now = millis();
  portalConsecutiveSuccesses = 0;
  portalLastFailureReason = reason;

  bool countThisFailure = lastPortalFailureCountedMs == 0 ||
      (uint32_t)(now - lastPortalFailureCountedMs) >= PORTAL_FAILURE_SAMPLE_MIN_MS;
  if (countThisFailure) {
    lastPortalFailureCountedMs = now == 0 ? 1 : now;
    if (portalFailureWindowStartedMs == 0) {
      portalFailureWindowStartedMs = now == 0 ? 1 : now;
    }
    if (portalConsecutiveFailures < UINT16_MAX) {
      portalConsecutiveFailures++;
    }
  }

  // Uma falha isolada de DNS/TLS/HTTP não significa que o equipamento saiu
  // da rede. A queda só é confirmada depois de falhas repetidas E de 60 s sem
  // nenhuma resposta válida do servidor.
  uint32_t referenceMs = lastServerOkMs > 0 ? lastServerOkMs : bootStartedMs;
  bool enoughFailures = portalConsecutiveFailures >= PORTAL_FAILURE_MIN_COUNT;
  bool enoughTime = (uint32_t)(now - referenceMs) >= PORTAL_FAILURE_CONFIRM_MS;
  if (!enoughFailures || !enoughTime) return;

  if (!portalRequestFailureActive || lastPortalFailureEventMs == 0 ||
      (uint32_t)(now - lastPortalFailureEventMs) >= EVENT_FAILURE_REPEAT_MS) {
    appendDiagnosticHistory("COMUNICACAO", "FALHA PORTAL", portalLastFailureReason);
    lastPortalFailureEventMs = now;
  }
  portalRequestFailureActive = true;
}

void notePortalCommunicationRecovered() {
  portalConsecutiveFailures = 0;
  portalFailureWindowStartedMs = 0;
  lastPortalFailureCountedMs = 0;
  portalLastFailureReason = "";
  if (portalConsecutiveSuccesses < PORTAL_RECOVERY_MIN_COUNT) {
    portalConsecutiveSuccesses++;
  }
  if (!portalRequestFailureActive) return;
  if (portalConsecutiveSuccesses < PORTAL_RECOVERY_MIN_COUNT) return;
  portalRequestFailureActive = false;
  appendDiagnosticHistory("COMUNICACAO", "PORTAL RESTABELECIDO",
      "Tres respostas HTTP validas consecutivas recebidas");
}

void trackCommandAppliedEvent() {
  if (!commandedOutputsAreApplied()) return;
  // Reconciliar o resultado ANTES de deduplicar o historico. Uma atualizacao
  // pode voltar a accepted para o mesmo ID ja registrado como aplicado.
  const bool stopped = lastPortalCommandDirection == "parado";
  const String expectedResult = stopped ? "succeeded" : "executing";
  if (commandExecution.result != expectedResult || !commandExecution.outputsConfirmed) {
    setCommandExecution(lastPortalCommandId, lastPortalCommandDirection,
        lastPortalCommandMode, expectedResult,
        stopped ? "Parada confirmada nas saidas" : "Saidas confirmadas; comando em execucao",
        stopped);
  }
  if (lastCommandAppliedEventId == lastPortalCommandId &&
      lastCommandAppliedEventUpdatedAt == lastPortalCommandUpdatedAt &&
      lastCommandAppliedEventDirection == lastPortalCommandDirection) return;
  lastCommandAppliedEventId = lastPortalCommandId;
  lastCommandAppliedEventUpdatedAt = lastPortalCommandUpdatedAt;
  lastCommandAppliedEventDirection = lastPortalCommandDirection;
  appendDiagnosticHistory("COMANDO", "APLICADO NAS SAIDAS",
      "commandId=" + lastPortalCommandId +
      " updatedAt=" + String(lastPortalCommandUpdatedAt) +
      " direcao=" + lastPortalCommandDirection +
      " modo=" + lastPortalCommandMode +
      " ack=" + commandAcknowledgementStatus());
}

void trackLoraCommunicationEvent() {
  if (bootGuardActive() || cfg.gpsNodeId.isEmpty()) return;
  const uint32_t now = millis();
  const bool rawRadioRecent = loraLastAnyRxMs > 0 && now - loraLastAnyRxMs <= 30000UL;
  const bool authorizedLinkRecent = loraAcceptedMs > 0 && now - loraAcceptedMs <= 30000UL;
  const bool gpsSampleRecent = lastGoodGpsAvailable && lastGoodGpsMs > 0 &&
      now - lastGoodGpsMs <= 30000UL;
  const bool channelMismatch = rawRadioRecent && loraLastSeenId == cfg.gpsNodeId &&
      e220ReadChannel >= 0 && loraReportedChannel >= 0 &&
      e220ReadChannel != loraReportedChannel;
  const bool protocolCountersChanged = loraJsonErrorCount > lastLoggedLoraJsonErrors ||
      loraIgnoredPackets > lastLoggedLoraIgnoredPackets;
  if (protocolCountersChanged &&
      (lastLoraProtocolEventMs == 0 || now - lastLoraProtocolEventMs >= EVENT_FAILURE_REPEAT_MS)) {
    appendDiagnosticHistory("COMUNICACAO", "PACOTE LORA REJEITADO",
        "jsonErros=" + String(loraJsonErrorCount) +
        " ignorados=" + String(loraIgnoredPackets) +
        " ultimoID=" + loraLastSeenId +
        " motivo=" + (loraRejectReason.length() ? loraRejectReason : loraLastJsonError));
    lastLoggedLoraJsonErrors = loraJsonErrorCount;
    lastLoggedLoraIgnoredPackets = loraIgnoredPackets;
    lastLoraProtocolEventMs = now;
  }
  const bool shouldExpectLink = commandRequiresGps(currentCmd) ||
      controllerState == STATE_RUNNING || controllerState == STATE_WAIT_SAFETY ||
      controllerState == STATE_WAIT_PRESSURE || controllerState == STATE_WAIT_START;
  if (!shouldExpectLink) {
    loraLinkFailureActive = false;
    gpsSignalFailureActive = false;
    loraChannelMismatchActive = false;
    return;
  }
  if (!authorizedLinkRecent && !loraLinkFailureActive) {
    loraLinkFailureActive = true;
    appendDiagnosticHistory("COMUNICACAO", "SEM RECEPCAO LORA",
        String(rawRadioRecent ? "Radio recebe dados, mas nenhum pacote autorizado" :
                                "Nenhum byte recebido do radio") +
        " por mais de 30s; esperado=" + cfg.gpsNodeId +
        " ultimoID=" + loraLastSeenId + " rejeicao=" + loraRejectReason);
  } else if (authorizedLinkRecent && loraLinkFailureActive) {
    loraLinkFailureActive = false;
    appendDiagnosticHistory("COMUNICACAO", "LORA RESTABELECIDO",
        "ID visto=" + loraLastSeenId + " canalGPS=" + String(loraReportedChannel));
  }

  if (authorizedLinkRecent && !gpsSampleRecent && !gpsSignalFailureActive) {
    gpsSignalFailureActive = true;
    appendDiagnosticHistory("GPS", "SEM AMOSTRA VALIDA",
        "LoRa ativo, mas sem nova posicao valida por mais de 30s; sats=" +
        String(gpsSatellites) + " hdop=" + String(gpsHdop, 2));
  } else if (authorizedLinkRecent && gpsSampleRecent && gpsSignalFailureActive) {
    gpsSignalFailureActive = false;
    appendDiagnosticHistory("GPS", "POSICAO RESTABELECIDA",
        "sats=" + String(gpsSatellites) + " hdop=" + String(gpsHdop, 2));
  }
  if (channelMismatch && !loraChannelMismatchActive) {
    loraChannelMismatchActive = true;
    appendDiagnosticHistory("COMUNICACAO", "CANAIS LORA DIVERGENTES",
        "central=" + String(e220ReadChannel) + " GPS_informa=" +
        String(loraReportedChannel) + " ID=" + loraLastSeenId);
  } else if (!channelMismatch && loraChannelMismatchActive) {
    loraChannelMismatchActive = false;
    appendDiagnosticHistory("COMUNICACAO", "CANAIS LORA COMPATIVEIS",
        "central=" + String(e220ReadChannel) + " GPS=" + String(loraReportedChannel));
  }
}

uint8_t peakDayLabelToBit(const String& raw) {
  String s = raw;
  s.trim(); s.toLowerCase();
  if (s == "dom" || s == "domingo" || s == "sun" || s == "0") return 1 << 0;
  if (s == "seg" || s == "segunda" || s == "mon" || s == "1") return 1 << 1;
  if (s == "ter" || s == "terca" || s == "terça" || s == "tue" || s == "2") return 1 << 2;
  if (s == "qua" || s == "quarta" || s == "wed" || s == "3") return 1 << 3;
  if (s == "qui" || s == "quinta" || s == "thu" || s == "4") return 1 << 4;
  if (s == "sex" || s == "sexta" || s == "fri" || s == "5") return 1 << 5;
  if (s == "sab" || s == "sábado" || s == "sabado" || s == "sat" || s == "6") return 1 << 6;
  return 0;
}

int parseTimeToMinutes(const String& value) {
  int sep = value.indexOf(':');
  if (sep <= 0) return -1;
  int hh = value.substring(0, sep).toInt();
  int mm = value.substring(sep + 1).toInt();
  if (hh < 0 || hh > 23 || mm < 0 || mm > 59) return -1;
  return hh * 60 + mm;
}

bool isPeakActiveNow(const PivotCommand& cmd) {
  if (!cmd.peakEnabled) return false;
  uint32_t nowEpoch = currentServerEpoch();
  if (nowEpoch == 0) return false;
  int startMin = parseTimeToMinutes(cmd.peakStart);
  int endMin = parseTimeToMinutes(cmd.peakEnd);
  if (startMin < 0 || endMin < 0) return false;
  uint32_t days = nowEpoch / 86400UL;
  uint8_t dow = (days + 4) % 7; // Unix epoch started on Thursday
  if (cmd.peakDaysMask != 0 && ((cmd.peakDaysMask & (1 << dow)) == 0)) return false;
  uint32_t secsOfDay = nowEpoch % 86400UL;
  int nowMin = secsOfDay / 60UL;
  if (startMin == endMin) return true;
  if (startMin < endMin) return nowMin >= startMin && nowMin < endMin;
  return nowMin >= startMin || nowMin < endMin;
}

bool commandRequiresGps(const PivotCommand& cmd) {
  return CommandPolicy::needsGps(cmd.requiresGps,cmd.noGpsMode);
}

bool gpsReadyForStart() {
  return CommandPolicy::gpsStartReady(lastGoodGpsAvailable && lastGoodGpsMs!=0,
      loraGpsValid, millis()-lastGoodGpsMs,gpsSatellites,gpsHdop,
      lastGoodPivotAngleValid,lastGoodDistanceFromCenterM);
}
void addCommandOutcomeStatus(JsonDocument& doc) {
  doc["firmware"]=FIRMWARE_VERSION;
  doc["commandRejected"]=portalCommandWasRejected();
  doc["commandResult"]=commandExecution.result;
  doc["commandReason"]=commandExecution.reason;
  doc["commandAckCommandId"]=commandExecution.commandId;
  doc["commandAckReason"]=commandExecution.reason;
  doc["commandRejectionCode"]=portalRejectionCode();
  doc["gpsStartRequired"]=commandRequiresGps(currentCmd);
  doc["gpsStartReady"]=gpsReadyForStart();
  doc["gpsStartMaxAgeMs"]=15000;
  bool blocked=bootGuardActive() || !safetyTaskHandle || fastOutputInhibit || !safetyOk() ||
      requireStopCommandAfterFault || operationJournalStartInterlock ||
      (commandRequiresGps(currentCmd) && !gpsReadyForStart());
  doc["startBlocked"]=blocked;
  doc["startBlockReason"]=!blocked?String(""):portalCommandWasRejected()?commandExecution.reason:
      bootGuardActive()?String("Protecao inicial de 60 segundos"):
      !safetyOk()?String("Emergencia/seguranca nao liberada"):
      requireStopCommandAfterFault?String("Falha anterior: envie PARAR"):
      operationJournalStartInterlock?String("Bloqueio de persistencia NVS"):
      !safetyTaskHandle?String("Tarefa de seguranca indisponivel"):
      fastOutputInhibit?String("Corte rapido de seguranca ativo"):
      (commandRequiresGps(currentCmd) && !gpsReadyForStart())?
        String("GPS indisponivel para nova partida"):String("");
}

void updateLastGoodGps(double lat, double lon, bool angleValidValue, double angleDeg,
                       double distM, uint32_t sampleMs) {
  ++gpsSampleGeneration;
  lastGoodGpsLat = lat;
  lastGoodGpsLon = lon;
  lastGoodGpsMs = sampleMs; // preserva idade inclusive no instante zero/rollover; zero bloqueia conservadoramente ate nova amostra
  lastGoodGpsAvailable = true;
  lastGoodPivotAngleValid = angleValidValue;
  if (angleValidValue) {
    lastGoodPivotAngleDeg = angleDeg;
    lastGoodDistanceFromCenterM = distM;
    lastGoodPivotAngleValid = true;
  }
}

bool hasGoodGpsWithinTimeout() {
  return lastGoodGpsAvailable && lastGoodGpsMs > 0 && (millis() - lastGoodGpsMs <= GPS_RETENTION_MS);
}

bool hasOperationalGpsSample(uint32_t now) {
  return lastGoodGpsAvailable && lastGoodGpsMs > 0 &&
         (uint32_t)(now - lastGoodGpsMs) <= GPS_OPERATION_TIMEOUT_MS;
}

bool movementObservationReady(uint32_t now) {
  return movementGuard.count >= 9 && movementLastUsableSampleMs > 0 &&
         (uint32_t)(now - movementLastUsableSampleMs) <= MOVEMENT_SAMPLE_FRESH_MS;
}

void processBoardReadyRelay() {
  bool mustClose = safetyTaskHandle && !fastOutputInhibit && !bootGuardActive() && !operationJournalStartInterlock &&
                   (controllerState == STATE_WAIT_SAFETY ||
                    controllerState == STATE_WAIT_PRESSURE ||
                    controllerState == STATE_WAIT_START ||
                    controllerState == STATE_RUNNING) &&
                   currentCmd.direction != "parado";
  boardReadyRelayState = mustClose;
  writeOutputIfChanged(PIN_OUT_AUX6, boardReadyRelayState);
}

void enterPersistenceFault(const String& detail) {
  bool alreadyFaulted = controllerState == STATE_FAULT_PERSISTENCE;
  cancelPendingCommand("falha de persistencia NVS");
  cancelAutoRestart("falha de persistencia NVS");
  allOutputsOff();
  controllerState = STATE_FAULT_PERSISTENCE;
  operationJournalStartInterlock = true;
  operationJournalInterlockReason = detail;
  requireStopCommandAfterFault = true;
  setCommandExecution(currentCmd.commandId, currentCmd.direction, currentCmd.mode,
                      "failed", "Falha ao confirmar registro persistente da operacao: " + detail, true);
  if (!alreadyFaulted) {
    recordFaultEvent("FALHA DE PERSISTENCIA NVS", detail);
    recordStopEvent("FALHA DE PERSISTENCIA NVS",
                    "Saidas desligadas e nova partida bloqueada");
  }
  faultMessage = "Falha de persistencia NVS";
  alertMessage = "PROSERVICE INFORMA: memoria nao confirmou a operacao; pivo bloqueado";
  clearControllerTimers();
  allOutputsOff();
}

void enterPortalTimeoutFault() {
  cancelPendingCommand("falha de comunicacao com o portal");
  allOutputsOff(); // antes de gravar historico e imprimir
  clearOperationJournal("portal_timeout");
  controllerState = STATE_FAULT_PORTAL_TIMEOUT;
  setCommandExecution(currentCmd.commandId, currentCmd.direction, currentCmd.mode,
                      "failed", "Sem comunicacao com o portal por 3 horas", true);
  recordFaultEvent("FALHA DE COMUNICACAO COM O PORTAL", "Sem resposta valida por 3 horas");
  recordStopEvent("FALHA DE COMUNICACAO COM O PORTAL", "Parada automatica por 3 horas offline");
  faultMessage = "Falha de comunicação com o portal";
  alertMessage = "PROSERVICE INFORMA: Sem comunicação com o portal por 3h";
  requireStopCommandAfterFault = true;
  clearControllerTimers();
  allOutputsOff();
}

void enterGpsTimeoutFault(const String& detail) {
  cancelPendingCommand("falha de GPS/LoRa");
  allOutputsOff(); // antes de gravar historico e imprimir
  clearOperationJournal("gps_timeout");
  controllerState = STATE_FAULT_GPS_TIMEOUT;
  setCommandExecution(currentCmd.commandId, currentCmd.direction, currentCmd.mode,
                      "failed", detail, true);
  recordFaultEvent("FALHA DE GPS/LORA", detail);
  recordStopEvent("FALHA DE GPS/LORA", "Operacao dependente de GPS interrompida: " + detail);
  faultMessage = "GPS fora dos parâmetros de funcionamento";
  alertMessage = "PROSERVICE INFORMA: " + detail;
  requireStopCommandAfterFault = true;
  clearControllerTimers();
  allOutputsOff();
}

void enterNoMovementFault() {
  String detail = "Sem avanco filtrado: " + String(movementGuard.reason) +
      " progressoM=" + String(movementGuard.progress,2) +
      " confirmacoes=" + String(movementGuard.confirmations) +
      " recuperacoes=" + String(movementGuard.recoveries) +
      " rejeicoes=" + String(movementGuard.rejectedSamples) +
      " hdop=" + String(gpsHdop,2) + " sats=" + String(gpsSatellites) +
      " raioMedido=" + String(lastGoodDistanceFromCenterM,2) +
      " torrePct=" + String(currentCmd.lastTowerPercent,2) +
      " rejeitaQualidade=" + String(movementQualityRejects) +
      " rejeitaIdade=" + String(movementAgeRejects) +
      " rejeitaGeometria=" + String(movementGeometryRejects) +
      " filaMs=" + String(loraQueueAgeMs);
  cancelPendingCommand("falha de movimento");
  allOutputsOff();
  clearOperationJournal("no_movement");
  controllerState = STATE_FAULT_NO_MOVEMENT;
  setCommandExecution(currentCmd.commandId, currentCmd.direction, currentCmd.mode,
                      "failed", "Sem movimento confirmado no prazo", true);
  recordFaultEvent("SEM MOVIMENTO CONFIRMADO", detail);
  recordStopEvent("SEM MOVIMENTO CONFIRMADO", "Parada automatica do monitor de movimento");
  faultMessage = "Sem movimentação do pivô";
  alertMessage = "PROSERVICE INFORMA: Comando de movimento sem alteração de posição por 10min";
  requireStopCommandAfterFault = true;
  clearControllerTimers();
  allOutputsOff();
}

void cancelPendingCommand(const String& reason, bool logEvent) {
  if (!pendingCmdValid) return;
  String cancelledId = pendingCmd.commandId;
  String cancelledDirection = pendingCmd.direction;
  pendingCmdValid = false;
  pendingCmd = PivotCommand();
  if (logEvent) {
    appendDiagnosticHistory("COMANDO", "PARTIDA PENDENTE CANCELADA",
        "commandId=" + cancelledId + " direcao=" + cancelledDirection +
        " motivo=" + reason);
  }
}

bool pendingCommandStillAuthorized() {
  if (!pendingCmdValid || !portalCommandReceived) return false;
  if (localTestMode || requireStopCommandAfterFault) return false;
  if (autoRestart.required || autoRestart.pending) return false;
  if (lastPortalCommandReceivedMs == 0 ||
      millis() - lastPortalCommandReceivedMs > AUTO_RESTART_AUTH_FRESH_MS) return false;
  if (normalizeDirectionStr(pendingCmd.direction) == "parado") return false;
  if (lastPortalCommandDirection == "parado") return false;
  if (pendingCmd.commandId != lastPortalCommandId) return false;
  if (pendingCmd.updatedAt != lastPortalCommandUpdatedAt) return false;
  if (normalizeDirectionStr(pendingCmd.direction) != lastPortalCommandDirection) return false;
  if (normalizeModeStr(pendingCmd.mode) != lastPortalCommandMode) return false;
  if (calculateCommandPayloadHash(pendingCmd) != lastPortalCommandPayloadHash) return false;
  if (controllerState == STATE_FAULT_SAFETY ||
      controllerState == STATE_FAULT_LOW_PRESSURE ||
      controllerState == STATE_FAULT_GPS_TIMEOUT ||
      controllerState == STATE_FAULT_PORTAL_TIMEOUT ||
      controllerState == STATE_FAULT_NO_MOVEMENT ||
      controllerState == STATE_FAULT_PERSISTENCE) return false;
  return true;
}

void maybeAcceptPendingCommand() {
  if (!pendingCmdValid) return;
  if (millis() - lastCommandAcceptedMs < COMMAND_GUARD_MS) return;
  if (!pendingCommandStillAuthorized()) {
    cancelPendingCommand("nao corresponde mais ao ultimo comando autorizado pelo portal");
    return;
  }
  PivotCommand cmd = pendingCmd;
  cancelPendingCommand("liberado apos protecao de 20s", false);
  startCommandSequence(cmd);
}

bool shouldForceHeartbeatNow() {
  if (commandExecutionRevision != lastPostedCommandExecutionRevision) return true;
  bool pumpOn = outputIsOnLogicLevel(PIN_OUT_PUMP);
  bool rightOn = outputIsOnLogicLevel(PIN_OUT_RIGHT);
  bool leftOn = outputIsOnLogicLevel(PIN_OUT_LEFT);
  bool cannonOn = outputIsOnLogicLevel(PIN_OUT_CANNON);

  String currentState = stateToString(controllerState);
  bool currentSafety = safetyOk();
  bool currentPressure = pressureOk();
  bool currentCommandApplied = commandedOutputsAreApplied();
  String currentCommandAckStatus = commandAcknowledgementStatus();

  if (currentState != lastPostedControllerState) return true;
  if (faultMessage != lastPostedFaultMessage) return true;
  if (alertMessage != lastPostedAlertMessage) return true;
  if (pumpOn != lastPostedPumpOn) return true;
  if (rightOn != lastPostedRightOn) return true;
  if (leftOn != lastPostedLeftOn) return true;
  if (cannonOn != lastPostedCannonOn) return true;
  if (currentSafety != lastPostedSafetyOk) return true;
  if (currentPressure != lastPostedPressureOk) return true;
  if (currentCommandApplied != lastPostedCommandApplied) return true;
  if (lastPortalCommandUpdatedAt != lastPostedCommandUpdatedAt) return true;
  if (currentCommandAckStatus != lastPostedCommandAckStatus) return true;
  if (diagEventCounter != lastPostedDiagEventCounter) return true;
  if (floatChanged(currentCmd.lastTowerPercent, lastPostedLastTowerPercent)) return true;
  if (floatChanged(currentCmd.requestedDepthMm, lastPostedRequestedDepthMm)) return true;
  if (floatChanged(currentCmd.minDepthMm, lastPostedMinDepthMm)) return true;

  return false;
}

void printControllerSnapshot() {
#if PROSERVICE_VERBOSE_SERIAL

  Serial.println("[SNAPSHOT]");
  Serial.println("  state=" + stateToString(controllerState));
  Serial.println("  mode=" + currentCmd.mode);
  Serial.println("  direction=" + currentCmd.direction);
  Serial.println("  pressureWaitSec=" + String(currentCmd.pressureWaitSec));
  Serial.println("  requestedDepthMm=" + String(currentCmd.requestedDepthMm, 2));
  Serial.println("  minDepthMm=" + String(currentCmd.minDepthMm, 2));
  Serial.println("  safetyFactorPercent=" + String(currentCmd.safetyFactorPercent, 2));
  Serial.println("  lastTowerPercent=" + String(currentCmd.lastTowerPercent, 2));
  Serial.println("  lastTowerCycleMs=" + String(currentCmd.lastTowerCycleMs));
  Serial.println("  engineeringReady=" + String(currentCmd.engineeringReady ? "true" : "false"));
  Serial.println("  depthClamped=" + String(currentCmd.depthClamped ? "true" : "false"));
  Serial.println("  centerLat=" + String(currentCmd.centerLat, 6));
  Serial.println("  centerLon=" + String(currentCmd.centerLon, 6));
  Serial.println("  radiusMeters=" + String(currentCmd.radiusMeters, 2));
  Serial.println("  areaHectares=" + String(currentCmd.areaHectares, 4));
  Serial.println("  geometryReady=" + String(currentCmd.geometryReady ? "true" : "false"));
  Serial.println("  gpsValid=" + String(loraGpsValid ? "true" : "false"));
  if (gps.location.isValid()) {
    Serial.println("  gpsLat=" + String(gpsLat, 6));
    Serial.println("  gpsLon=" + String(gpsLon, 6));
    Serial.println("  gpsSatellites=" + String(gpsSatellites));
    Serial.println("  gpsHdop=" + String(gpsHdop, 2));
    Serial.println("  gpsCourseDeg=" + String(gpsCourseDeg, 2));
    Serial.println("  distanceFromCenterM=" + String(distanceFromCenterM, 2));
    Serial.println("  pivotAngleDeg=" + String(pivotAngleDeg, 2));
    Serial.println("  pivotAngleValid=" + String(pivotAngleValid ? "true" : "false"));
  }
  Serial.println("  emergency1=" + String(readEmergency1() ? "ON" : "OFF"));
  Serial.println("  emergency2=" + String(readEmergency2() ? "ON" : "OFF"));
  Serial.println("  safetyOk=" + String(safetyOk() ? "true" : "false"));
  Serial.println("  pressureOk=" + String(pressureOk() ? "true" : "false"));
  Serial.println("  pumpOn=" + String(outputIsOnLogicLevel(PIN_OUT_PUMP) ? "true" : "false"));
  Serial.println("  rightOn=" + String(outputIsOnLogicLevel(PIN_OUT_RIGHT) ? "true" : "false"));
  Serial.println("  leftOn=" + String(outputIsOnLogicLevel(PIN_OUT_LEFT) ? "true" : "false"));
  Serial.println("  lastTowerOn=" + String(outputIsOnLogicLevel(PIN_OUT_LAST_TOWER) ? "true" : "false"));
  Serial.println("  cannonOn=" + String(outputIsOnLogicLevel(PIN_OUT_CANNON) ? "true" : "false"));
  Serial.println("  aux6On=" + String(outputIsOnLogicLevel(PIN_OUT_AUX6) ? "true" : "false"));
  Serial.println("  gpsNodeId=" + cfg.gpsNodeId);
  Serial.println("  fault=" + faultMessage);
  Serial.println("  alert=" + alertMessage);
#endif
}

bool shouldTurnLastTowerOn() {
  if (bootGuardActive()) return false;
  if (controllerState != STATE_RUNNING) return false;
  if (currentCmd.direction == "parado") return false;

  if (currentCmd.lastTowerPercent < 0.0f) {
    return false; // engenharia não configurada — não pulsa
  }

  float pct = clampFloat(currentCmd.lastTowerPercent, 0.0f, 100.0f);

  if (pct <= 0.0f) return false;
  if (pct >= 100.0f) return true;

  uint32_t cycleMs = currentCmd.lastTowerCycleMs;
  if (cycleMs < 1000) cycleMs = 1000;
  if (cycleMs > 60000) cycleMs = 60000;

  uint32_t onMs = (uint32_t)((pct / 100.0f) * (float)cycleMs);
  if (onMs == 0) return false;
  if (onMs >= cycleMs) return true;

  uint32_t phase = millis() % cycleMs;
  return phase < onMs;
}

// ======================================================
// PERSISTÊNCIA
// ======================================================

void loadConfig() {
  prefs.begin(PREF_NAMESPACE, true);
  cfg.wifiSsid     = prefs.getString("ssid", "");
  cfg.wifiPass     = prefs.getString("pass", "");
  cfg.serverUrl    = prefs.getString("server", DEFAULT_SERVER_URL);
  cfg.backendEnabled = prefs.getBool("backend", true);
  cfg.commandRoute = prefs.getString("cmdroute", DEFAULT_COMMAND_ROUTE);
  cfg.heartbeatRoute = prefs.getString("hbroute", DEFAULT_HEARTBEAT_ROUTE);
  cfg.pivotId      = prefs.getString("pivot", "1");
  cfg.deviceToken  = prefs.getString("token", "");
  cfg.hardwareName = prefs.getString("hwname", "");
  cfg.gpsNodeId    = prefs.getString("gpsnode", "");
  e220Channel      = prefs.getUChar("lora_ch", E220_DEFAULT_CHANNEL);

  cfg.localProfileEnabled       = prefs.getBool("locprof", true);
  cfg.localCenterLat            = prefs.getDouble("clat", 0.0);
  cfg.localCenterLon            = prefs.getDouble("clon", 0.0);
  cfg.localRadiusMeters         = prefs.getFloat("radius", 0.0f);
  cfg.localAreaHectares         = prefs.getFloat("area", 0.0f);
  cfg.localGpsCorrectionDeg     = prefs.getFloat("gpscorr", 0.0f);
  cfg.localMinDepthMm           = prefs.getFloat("mindepth", 0.0f);
  cfg.localSafetyFactorPercent  = prefs.getFloat("safetyf", 3.0f);
  cfg.localLastTowerCycleMs     = prefs.getUInt("towercyc", DEFAULT_LAST_TOWER_CYCLE_MS);
  cfg.localPressureWaitSec      = prefs.getInt("presswait", 240);
  prefs.end();
  if (e220Channel > E220_MAX_CHANNEL) e220Channel = E220_DEFAULT_CHANNEL;

  cfg.serverUrl = normalizeServerUrl(cfg.serverUrl);
  // Instalações sem endereço salvo usam o servidor padrão PROSERVICE.
  if (cfg.serverUrl.isEmpty()) {
    cfg.serverUrl = DEFAULT_SERVER_URL;
  }
  cfg.commandRoute = normalizeBackendRoute(cfg.commandRoute, DEFAULT_COMMAND_ROUTE);
  cfg.heartbeatRoute = normalizeBackendRoute(cfg.heartbeatRoute, DEFAULT_HEARTBEAT_ROUTE);
  cfg.pivotId.trim();
  cfg.deviceToken.trim();
  cfg.hardwareName.trim();
  cfg.gpsNodeId.trim();

  if (cfg.pivotId.isEmpty()) cfg.pivotId = "1";
  if (cfg.hardwareName.isEmpty()) cfg.hardwareName = "ESP32-PIVO-" + cfg.pivotId;

  if (!isFiniteNumber(cfg.localCenterLat)) cfg.localCenterLat = 0.0;
  if (!isFiniteNumber(cfg.localCenterLon)) cfg.localCenterLon = 0.0;
  if (!isFiniteNumber(cfg.localRadiusMeters) || cfg.localRadiusMeters < 0.0f) cfg.localRadiusMeters = 0.0f;
  if (!isFiniteNumber(cfg.localAreaHectares) || cfg.localAreaHectares < 0.0f) cfg.localAreaHectares = 0.0f;
  if (!isFiniteNumber(cfg.localGpsCorrectionDeg)) cfg.localGpsCorrectionDeg = 0.0f;
  if (!isFiniteNumber(cfg.localMinDepthMm) || cfg.localMinDepthMm < 0.0f) cfg.localMinDepthMm = 0.0f;
  cfg.localSafetyFactorPercent = clampFloat(cfg.localSafetyFactorPercent, 0.0f, 30.0f);
  if (cfg.localLastTowerCycleMs < 1000) cfg.localLastTowerCycleMs = 1000;
  if (cfg.localLastTowerCycleMs > 60000) cfg.localLastTowerCycleMs = 60000;
  if (cfg.localPressureWaitSec < 10) cfg.localPressureWaitSec = 10;
  if (cfg.localPressureWaitSec > 30000) cfg.localPressureWaitSec = 30000;

  // Se apenas área ou raio foi informado, calcula o outro automaticamente.
  if (cfg.localAreaHectares > 0.0f && cfg.localRadiusMeters <= 0.0f) {
    cfg.localRadiusMeters = sqrt((cfg.localAreaHectares * 10000.0f) / PI);
  } else if (cfg.localRadiusMeters > 0.0f && cfg.localAreaHectares <= 0.0f) {
    cfg.localAreaHectares = (PI * cfg.localRadiusMeters * cfg.localRadiusMeters) / 10000.0f;
  }
}

bool saveConfig() {
  if (!prefs.begin(PREF_NAMESPACE, false)) {
    markPersistenceFailure("config", "nao abriu namespace");
    return false;
  }
  prefs.putString("ssid", cfg.wifiSsid);
  prefs.putString("pass", cfg.wifiPass);
  prefs.putString("server", cfg.serverUrl);
  prefs.putBool("backend", cfg.backendEnabled);
  prefs.putString("cmdroute", cfg.commandRoute);
  prefs.putString("hbroute", cfg.heartbeatRoute);
  prefs.putString("pivot", cfg.pivotId);
  prefs.putString("token", cfg.deviceToken);
  prefs.putString("hwname", cfg.hardwareName);
  prefs.putString("gpsnode", cfg.gpsNodeId);

  prefs.putBool("locprof", cfg.localProfileEnabled);
  prefs.putDouble("clat", cfg.localCenterLat);
  prefs.putDouble("clon", cfg.localCenterLon);
  prefs.putFloat("radius", cfg.localRadiusMeters);
  prefs.putFloat("area", cfg.localAreaHectares);
  prefs.putFloat("gpscorr", cfg.localGpsCorrectionDeg);
  prefs.putFloat("mindepth", cfg.localMinDepthMm);
  prefs.putFloat("safetyf", cfg.localSafetyFactorPercent);
  prefs.putUInt("towercyc", cfg.localLastTowerCycleMs);
  prefs.putInt("presswait", cfg.localPressureWaitSec);
  bool verified = prefs.getString("ssid", "") == cfg.wifiSsid &&
      prefs.getString("pass", "") == cfg.wifiPass &&
      prefs.getString("server", "") == cfg.serverUrl &&
      prefs.getBool("backend", !cfg.backendEnabled) == cfg.backendEnabled &&
      prefs.getString("cmdroute", "") == cfg.commandRoute &&
      prefs.getString("hbroute", "") == cfg.heartbeatRoute &&
      prefs.getString("pivot", "") == cfg.pivotId &&
      prefs.getString("token", "") == cfg.deviceToken &&
      prefs.getString("hwname", "") == cfg.hardwareName &&
      prefs.getString("gpsnode", "") == cfg.gpsNodeId &&
      prefs.getBool("locprof", !cfg.localProfileEnabled) == cfg.localProfileEnabled &&
      prefs.getDouble("clat", NAN) == cfg.localCenterLat &&
      prefs.getDouble("clon", NAN) == cfg.localCenterLon &&
      prefs.getFloat("radius", NAN) == cfg.localRadiusMeters &&
      prefs.getFloat("area", NAN) == cfg.localAreaHectares &&
      prefs.getFloat("gpscorr", NAN) == cfg.localGpsCorrectionDeg &&
      prefs.getFloat("mindepth", NAN) == cfg.localMinDepthMm &&
      prefs.getFloat("safetyf", NAN) == cfg.localSafetyFactorPercent &&
      prefs.getUInt("towercyc", 0) == cfg.localLastTowerCycleMs &&
      prefs.getInt("presswait", 0) == cfg.localPressureWaitSec;
  prefs.end();
  if (!verified) markPersistenceFailure("config", "leitura apos gravacao divergente");
  else markPersistenceSuccess();
  return verified;
}

bool pivotCenterValid(double lat, double lon) {
  return isFiniteNumber(lat) && isFiniteNumber(lon) &&
         fabs(lat) <= 90.0 && fabs(lon) <= 180.0 &&
         (fabs(lat) > 0.000001 || fabs(lon) > 0.000001);
}

bool pivotGeometryValuesValid(double lat, double lon, float radiusMeters) {
  return pivotCenterValid(lat, lon) && isFiniteNumber(radiusMeters) &&
         radiusMeters >= 10.0f;
}

bool localPivotGeometryValid() {
  return pivotGeometryValuesValid(cfg.localCenterLat, cfg.localCenterLon,
                                  cfg.localRadiusMeters);
}

void applyLocalPivotProfile(PivotCommand &cmd) {
  // O perfil local somente pode substituir o servidor quando a geometria
  // gravada estiver completa. Isso impede centro/raio zero de apagar o mapa.
  if (!cfg.localProfileEnabled || !localPivotGeometryValid()) return;

  cmd.centerLat = cfg.localCenterLat;
  cmd.centerLon = cfg.localCenterLon;
  cmd.radiusMeters = cfg.localRadiusMeters;
  cmd.areaHectares = cfg.localAreaHectares;
  cmd.gpsCorrectionDeg = cfg.localGpsCorrectionDeg;
  cmd.minDepthMm = cfg.localMinDepthMm;
  cmd.safetyFactorPercent = cfg.localSafetyFactorPercent;
  cmd.lastTowerCycleMs = cfg.localLastTowerCycleMs;
  cmd.pressureWaitSec = cfg.localPressureWaitSec;

  cmd.geometryReady = pivotGeometryValuesValid(
      cmd.centerLat, cmd.centerLon, cmd.radiusMeters);

  if (cmd.minDepthMm > 0.0f && cmd.requestedDepthMm < cmd.minDepthMm) {
    cmd.requestedDepthMm = cmd.minDepthMm;
    cmd.depthClamped = true;
  }
}

bool saveCurrentGpsAsLocalCenter(String &message) {
  // Preferência pelo GPS LoRa da ponta, pois é o GPS usado pelo sistema.
  bool haveFix = loraGpsValid && (millis() - loraLastRxMs < 300000UL) &&
                 isFiniteNumber(loraGpsLat) && isFiniteNumber(loraGpsLon);

  if (false) { // GPS local removido
    cfg.localCenterLat = gps.location.lat();
    cfg.localCenterLon = gps.location.lng();
    haveFix = true;
  } else if (haveFix) {
    cfg.localCenterLat = loraGpsLat;
    cfg.localCenterLon = loraGpsLon;
  }

  if (!haveFix) {
    message = "GPS sem FIX válido. Aguarde satélites e tente novamente.";
    return false;
  }

  if (!saveConfig()) {
    message = "Falha ao confirmar o centro na memoria persistente. A configuracao anterior foi mantida.";
    return false;
  }
  applyLocalPivotProfile(currentCmd);
  message = "Posição GPS atual gravada como centro local do pivô.";
  return true;
}

bool clearWifiConfigOnly() {
  bool cleared = false;
  if (prefs.begin(PREF_NAMESPACE, false)) {
    prefs.remove("ssid");
    prefs.remove("pass");
    cleared = !prefs.isKey("ssid") && !prefs.isKey("pass");
    prefs.end();
  }
  if (!cleared) markPersistenceFailure("config", "nao confirmou limpeza do Wi-Fi");
  else markPersistenceSuccess();
  return cleared;
}

// ======================================================
// WIFI E PORTAL
// ======================================================

void setupWiFiBase() {
  WiFi.persistent(false);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
}

bool startSoftApRobust() {
  String apName = buildApName();

  WiFi.softAPdisconnect(true);
  delay(200);

  WiFi.mode(WIFI_AP_STA);
  delay(150);

  bool ok = false;
  for (int attempt = 0; attempt < 3; attempt++) {
    ok = WiFi.softAP(apName.c_str(), DEFAULT_AP_PASS);
    delay(300);
    if (ok) break;

#if PROSERVICE_VERBOSE_SERIAL
    Serial.println("[AP] Tentativa falhou, repetindo...");
#endif
    WiFi.softAPdisconnect(true);
    delay(250);
    WiFi.mode(WIFI_AP_STA);
    delay(150);
  }

#if PROSERVICE_VERBOSE_SERIAL
  Serial.println("[AP] " + String(ok ? "ATIVO" : "FALHOU") + " " + apName);
#endif

  return ok;
}

bool ensureConfigPortalRunning(bool forceRestart = false) {
  bool apSeemsDown = WiFi.softAPIP()[0] == 0;

  if (forceRestart || !configPortalActive || apSeemsDown) {
    bool ok = startSoftApRobust();
    configPortalActive = ok;
    if (ok) {
      setupWebServer();
    }
    return ok;
  }

  if (!webServerStarted) {
    setupWebServer();
  }

  return true;
}

void beginSavedWifiConnection() {
  if (cfg.wifiSsid.isEmpty()) {
    wifiAttemptActive = false;
    wifiDiagnosticMessage = "Nenhuma rede Wi-Fi configurada";
#if PROSERVICE_VERBOSE_SERIAL
    Serial.println("[WIFI] Nenhum SSID salvo.");
#endif
    return;
  }

  WiFi.mode(WIFI_AP_STA);
  WiFi.setSleep(false);
  WiFi.disconnect(false, false);
  delay(80);
  WiFi.begin(cfg.wifiSsid.c_str(), cfg.wifiPass.c_str());

  wifiAttemptActive = true;
  wifiAttemptStartedMs = millis();
  wifiLastStatusPrintMs = 0;
  wifiPreviousStatus = WiFi.status();
  wifiDiagnosticMessage = "Tentando conectar em " + cfg.wifiSsid;
  wifiLastFailure = "";
  ntpSyncRequested=false;
#if PROSERVICE_VERBOSE_SERIAL
  Serial.println("[WIFI] Conectando: " + cfg.wifiSsid);
#endif
}

void processWifiConnection() {
  wl_status_t status = WiFi.status();

  if (status != wifiPreviousStatus) {
    wifiPreviousStatus = status;
#if PROSERVICE_VERBOSE_SERIAL
    Serial.println("[WIFI] Estado: " + wifiStatusName(status));
#endif
  }

  if (status == WL_CONNECTED) {
    if (wifiAttemptActive) {
      wifiAttemptActive = false;
      wifiDiagnosticMessage = "Conectado com sucesso";
      wifiLastFailure = "";
#if PROSERVICE_VERBOSE_SERIAL
      Serial.println("[WIFI] Conectado");
#endif
    }
    return;
  }

  if (!wifiAttemptActive) return;

  if (millis() - wifiLastStatusPrintMs >= WIFI_STATUS_PRINT_INTERVAL_MS) {
    wifiLastStatusPrintMs = millis();
#if PROSERVICE_VERBOSE_SERIAL
    Serial.println("[WIFI] " + wifiStatusName(status));
#endif
  }

  bool terminalFailure = status == WL_NO_SSID_AVAIL || status == WL_CONNECT_FAILED;
  bool timedOut = millis() - wifiAttemptStartedMs >= WIFI_ATTEMPT_TIMEOUT_MS;
  if (terminalFailure || timedOut) {
    wifiAttemptActive = false;
    if (!ethernetReady()) inspectSavedNetwork();

    if (status == WL_NO_SSID_AVAIL || wifiSavedNetworkRssi == -127) {
      wifiLastFailure = "Rede não encontrada. Confirme se ela transmite em 2,4 GHz e se o SSID está correto.";
    } else if (status == WL_CONNECT_FAILED) {
      wifiLastFailure = "Falha de autenticação. Confira a senha e prefira WPA2-PSK/AES.";
    } else if (wifiSavedNetworkRssi < -80) {
      wifiLastFailure = "Tempo esgotado e sinal muito fraco (" + String(wifiSavedNetworkRssi) + " dBm). Aproxime o ESP32 do roteador.";
    } else if (wifiSavedNetworkChannel > 11) {
      wifiLastFailure = "Rede encontrada no canal " + String(wifiSavedNetworkChannel) + ". Configure o roteador no canal 1 a 11.";
    } else if (wifiSavedNetworkSecurity.indexOf("WPA3") >= 0 && wifiSavedNetworkSecurity.indexOf("WPA2/") < 0) {
      wifiLastFailure = "Rede WPA3 exclusiva. Configure WPA2-PSK/AES ou modo misto WPA2/WPA3.";
    } else {
      wifiLastFailure = "Tempo esgotado. Verifique senha, DHCP do roteador, WPA2 e isolamento de clientes.";
    }

    wifiDiagnosticMessage = "Falha: " + wifiLastFailure;
#if PROSERVICE_VERBOSE_SERIAL
    Serial.println("[WIFI] " + wifiDiagnosticMessage);
#endif
    ensureConfigPortalRunning(false);
  }
}

static const uint8_t ROOT_CSS_GZ[] PROGMEM = {
  0x1F, 0x8B, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0xFF, 0x95, 0x56, 0xDD, 0x8E, 0xB3, 0x36,
  0x10, 0x7D, 0x95, 0x48, 0xAB, 0x4A, 0xA1, 0x02, 0x84, 0x21, 0x21, 0xC4, 0xDC, 0xB4, 0x77, 0xED,
  0x6D, 0xFB, 0x04, 0x06, 0x1B, 0xE2, 0x86, 0xD8, 0xC8, 0x98, 0x25, 0x29, 0xE2, 0xDD, 0x3B, 0xE6,
  0x67, 0xD7, 0x40, 0xBE, 0x56, 0xDD, 0x95, 0x50, 0xC6, 0x78, 0xFE, 0xCE, 0x9C, 0x99, 0x01, 0x2B,
  0x29, 0x75, 0xEF, 0x79, 0x82, 0x7C, 0xBE, 0xF0, 0x07, 0x0A, 0xA3, 0x2C, 0xBE, 0xA4, 0x9E, 0x97,
  0x55, 0x2D, 0x03, 0xF1, 0x12, 0xC7, 0x24, 0x06, 0x31, 0x7F, 0x11, 0x81, 0x3F, 0xC2, 0xEC, 0x8A,
  0xF2, 0xAB, 0x79, 0x5B, 0xE2, 0x8F, 0x22, 0x2A, 0xE2, 0xC2, 0x08, 0x39, 0x51, 0x14, 0xC4, 0xA2,
  0x80, 0xDF, 0x5C, 0xDC, 0x41, 0x2B, 0x0E, 0xA3, 0x08, 0x81, 0xF4, 0x68, 0x35, 0x83, 0x57, 0x71,
  0x7C, 0x49, 0x12, 0x02, 0x72, 0xC5, 0x05, 0x18, 0xA5, 0x39, 0x3B, 0x33, 0x0A, 0xA2, 0x34, 0x77,
  0xA3, 0x84, 0x9C, 0x23, 0x63, 0x92, 0xC0, 0xCD, 0x3C, 0x88, 0xAE, 0x61, 0x06, 0x52, 0x47, 0x14,
  0xF8, 0xA3, 0xA7, 0x24, 0x0E, 0x8C, 0xD8, 0xDC, 0x08, 0x95, 0x1D, 0x0E, 0x0E, 0x49, 0xFD, 0x3C,
  0x84, 0xE6, 0xA1, 0xCA, 0x8C, 0x1C, 0x23, 0xE4, 0x9E, 0x13, 0x37, 0x89, 0x5D, 0x1F, 0x05, 0xCE,
  0xF0, 0x73, 0x9F, 0xC9, 0xA7, 0xD7, 0xF0, 0xBF, 0xB9, 0x28, 0x71, 0x26, 0x15, 0x65, 0xCA, 0x83,
  0x93, 0x21, 0x93, 0xF4, 0xD5, 0x3F, 0x88, 0x2A, 0xB9, 0xC0, 0x41, 0x9A, 0x91, 0xFC, 0x5E, 0x2A,
  0xD9, 0x0A, 0x8A, 0x3F, 0x89, 0x3A, 0x9A, 0x5C, 0x9C, 0xB4, 0x90, 0x42, 0x7B, 0x05, 0x79, 0xF0,
  0xEA, 0x85, 0x7F, 0x55, 0x9C, 0x54, 0xEE, 0x6F, 0xAC, 0xFA, 0x64, 0x9A, 0xE7, 0xC4, 0x6D, 0x88,
  0x68, 0xBC, 0x86, 0x29, 0x5E, 0xA4, 0xB9, 0xAC, 0xA4, 0x9A, 0xD5, 0x20, 0x53, 0x67, 0xF0, 0x6B,
  0x52, 0x32, 0xB0, 0xFD, 0xF4, 0x3A, 0x4E, 0xF5, 0x0D, 0x23, 0x94, 0x04, 0xF5, 0x33, 0x9D, 0x9D,
  0x91, 0x56, 0xCB, 0xB4, 0x26, 0x94, 0x9A, 0x80, 0x10, 0x44, 0x3D, 0xF8, 0x37, 0xA6, 0x64, 0x6F,
  0x85, 0x60, 0x10, 0x21, 0xCA, 0x2B, 0x15, 0xA1, 0x9C, 0x09, 0x7D, 0x44, 0xD1, 0x99, 0xB2, 0xD2,
  0x9D, 0x5C, 0x98, 0x92, 0x38, 0xF3, 0x6F, 0x53, 0x0F, 0xC7, 0x99, 0x23, 0xE8, 0x6E, 0x5C, 0xB3,
  0x74, 0x4E, 0xD1, 0xA8, 0xB6, 0x0D, 0x0E, 0x43, 0x70, 0xBC, 0x38, 0x33, 0xC2, 0x21, 0x3C, 0xC1,
  0xC9, 0x08, 0xC9, 0x84, 0xDE, 0x64, 0x68, 0x12, 0x9C, 0x39, 0x46, 0x00, 0x48, 0x6B, 0xF9, 0xC0,
  0x28, 0x5E, 0xA2, 0xFB, 0x43, 0x76, 0x3D, 0xE5, 0x4D, 0x5D, 0x91, 0x17, 0x2E, 0x2A, 0xF6, 0x4C,
  0x4B, 0x52, 0x8F, 0xAF, 0x53, 0x52, 0xF1, 0x52, 0x78, 0xE0, 0xF9, 0xD1, 0xE0, 0x1C, 0x82, 0x65,
  0x2A, 0xFD, 0xAB, 0x6D, 0x34, 0x2F, 0x5E, 0x5E, 0x0E, 0xF8, 0xC1, 0x09, 0x6E, 0x6A, 0x92, 0x33,
  0x2F, 0x63, 0xBA, 0x63, 0x4C, 0xA4, 0x46, 0xDD, 0xEB, 0x14, 0xE8, 0x9B, 0xC7, 0xE0, 0x67, 0x8A,
  0x08, 0xDA, 0x8F, 0x58, 0x43, 0x95, 0x18, 0x46, 0x11, 0x58, 0xAD, 0x98, 0x06, 0x4B, 0x9E, 0xD1,
  0x34, 0x91, 0xFB, 0x28, 0x61, 0x8F, 0xA9, 0x1E, 0x1D, 0xE3, 0xE5, 0x4D, 0xE3, 0x24, 0x08, 0x52,
  0x69, 0xDE, 0xEA, 0x17, 0xF6, 0x93, 0xF3, 0x14, 0xE5, 0xE1, 0x86, 0x2C, 0x43, 0x86, 0x11, 0x0B,
  0xE8, 0x67, 0xC8, 0x3C, 0x38, 0x9C, 0x96, 0x74, 0xFE, 0x6C, 0x33, 0xDB, 0xA3, 0x81, 0xE4, 0xDB,
  0x58, 0x3C, 0xF8, 0x45, 0x67, 0xBF, 0x0E, 0x47, 0xC4, 0x0C, 0xAC, 0x18, 0x81, 0x9D, 0x46, 0x56,
  0x9C, 0x1E, 0x0C, 0xB5, 0xE1, 0x2F, 0x4A, 0x6C, 0xEE, 0xCC, 0x87, 0xE8, 0xB4, 0x29, 0xC3, 0xF5,
  0x7A, 0xB5, 0xEA, 0x60, 0x98, 0x3A, 0x1A, 0xB5, 0x13, 0xBA, 0x04, 0xC1, 0xE0, 0x9B, 0xA6, 0xE9,
  0x77, 0x5C, 0x34, 0xA7, 0xCE, 0x3E, 0x82, 0xE9, 0xA5, 0xE1, 0x8A, 0xB3, 0x71, 0x67, 0x58, 0xB5,
  0xA2, 0xD8, 0x9B, 0xBA, 0xFE, 0x90, 0x03, 0xC3, 0x2D, 0xB4, 0x93, 0x37, 0xDA, 0x36, 0xC3, 0x47,
  0xFA, 0x2D, 0xB0, 0x86, 0x23, 0xAC, 0x06, 0xBF, 0xE1, 0x16, 0xD9, 0x5A, 0xE7, 0x77, 0x5A, 0x83,
  0x3F, 0xB6, 0xFE, 0xAE, 0xD6, 0x90, 0x81, 0x77, 0x9B, 0x50, 0x40, 0xFE, 0xF9, 0xBC, 0x52, 0x1C,
  0x35, 0x40, 0x53, 0xDE, 0x7B, 0xFB, 0x58, 0xDE, 0x9D, 0x2D, 0x1D, 0x80, 0x49, 0x84, 0xAE, 0x2E,
  0x81, 0xFC, 0xEE, 0x96, 0x7C, 0x7E, 0x51, 0xB9, 0x54, 0x9C, 0xA6, 0xE6, 0xE1, 0x01, 0x7F, 0xE1,
  0x44, 0x33, 0x20, 0x6D, 0xD5, 0x3E, 0x44, 0x83, 0x15, 0xAB, 0x19, 0xD1, 0xC7, 0xC8, 0x7D, 0x70,
  0x01, 0xAD, 0x7C, 0x0C, 0x5C, 0x54, 0x28, 0xE8, 0x35, 0x43, 0x7C, 0x53, 0x4C, 0xAB, 0x4A, 0x1A,
  0x18, 0x0C, 0x4C, 0x55, 0xC0, 0xF5, 0xA5, 0x4A, 0xC1, 0x17, 0xFC, 0x93, 0xC7, 0xDF, 0xC1, 0x7E,
  0xBF, 0x22, 0x4A, 0x52, 0x90, 0x22, 0xFF, 0x5F, 0x45, 0xB5, 0x5B, 0x19, 0x05, 0x2B, 0x0E, 0x7D,
  0xF3, 0x74, 0x8F, 0x5D, 0x0A, 0x09, 0x2C, 0xF0, 0xC6, 0xE6, 0x8A, 0xFC, 0x64, 0xAA, 0xA8, 0x64,
  0x37, 0x35, 0x21, 0x11, 0xAF, 0x0E, 0x5A, 0x82, 0x7D, 0x85, 0x79, 0xC8, 0xBE, 0xE0, 0xC9, 0x2A,
  0x99, 0xDF, 0x77, 0xF3, 0x2D, 0xDD, 0x74, 0xCE, 0xCC, 0x2D, 0x2D, 0x6B, 0xBC, 0x2F, 0x67, 0x98,
  0x0C, 0x15, 0xC9, 0x58, 0xB5, 0xB1, 0xB9, 0x89, 0x7A, 0xDB, 0xD9, 0x93, 0xCB, 0x8F, 0xD3, 0xE9,
  0x7C, 0x8D, 0xA9, 0xED, 0x00, 0x05, 0x3B, 0x32, 0x03, 0xD5, 0x06, 0x2E, 0xEA, 0x56, 0xBB, 0x0D,
  0xAB, 0x58, 0xAE, 0xFB, 0x79, 0xE8, 0x06, 0xC1, 0x4F, 0xDF, 0x70, 0xBD, 0x6F, 0xE1, 0x3C, 0xA3,
  0x09, 0x8B, 0xB6, 0x38, 0xA3, 0x75, 0x75, 0xA7, 0xB1, 0xFA, 0x5F, 0x20, 0xC8, 0x56, 0x8F, 0x7B,
  0x4C, 0x48, 0xC1, 0xA6, 0x70, 0x70, 0x21, 0xF3, 0xB6, 0x99, 0x83, 0x9A, 0x84, 0x7E, 0xF6, 0xB4,
  0x22, 0xA9, 0x19, 0xE1, 0x76, 0x33, 0x06, 0x07, 0xF3, 0x1F, 0x2D, 0xDB, 0x2C, 0x8C, 0x5C, 0x14,
  0x84, 0x2E, 0x8A, 0xCD, 0x3E, 0x0B, 0x9D, 0x21, 0x6B, 0x21, 0x6B, 0xD1, 0x7F, 0xF1, 0xEC, 0x4D,
  0xF0, 0x76, 0xD6, 0x87, 0xB1, 0xD7, 0x37, 0xC1, 0xEE, 0xE0, 0x6E, 0x55, 0x03, 0x01, 0xD5, 0x92,
  0x8F, 0xF3, 0x7B, 0x6E, 0x6E, 0x50, 0x3C, 0x4C, 0x73, 0x13, 0x38, 0x5C, 0x2B, 0x0E, 0xC7, 0xAF,
  0xFD, 0x70, 0x9A, 0xE2, 0x9F, 0x0B, 0x06, 0xB3, 0x6F, 0xF0, 0x29, 0x51, 0xF7, 0x15, 0xD7, 0xC3,
  0x38, 0xBA, 0x9C, 0xE2, 0xD5, 0x1D, 0xB3, 0xC9, 0xF7, 0xB6, 0xCC, 0xE9, 0xD6, 0x96, 0x28, 0x99,
  0x7A, 0xE3, 0xD5, 0xB4, 0xB6, 0x7D, 0xB1, 0x80, 0xEF, 0x15, 0xB8, 0xF8, 0xDF, 0xDD, 0xA0, 0xD9,
  0x53, 0x7B, 0xE3, 0xD2, 0x5A, 0xD6, 0x95, 0x3D, 0x96, 0x61, 0x98, 0x99, 0x8D, 0xFC, 0xCB, 0x83,
  0x51, 0x4E, 0x8E, 0xDF, 0x2B, 0xFC, 0x1A, 0x00, 0xF1, 0x9C, 0x7E, 0x1C, 0x1E, 0xFF, 0x3A, 0x2F,
  0xC2, 0xCD, 0xBC, 0x18, 0xF6, 0xB6, 0xE2, 0x70, 0xB2, 0x35, 0x7E, 0x25, 0xD8, 0x0D, 0xBD, 0x7C,
  0x08, 0xAC, 0x2B, 0x1A, 0xDB, 0x15, 0xBD, 0x2C, 0xB7, 0x36, 0xAB, 0x2E, 0x34, 0xE7, 0xD3, 0xF6,
  0x58, 0x6B, 0x9F, 0x6C, 0xED, 0x71, 0xFF, 0xFD, 0x38, 0x05, 0x08, 0xF8, 0x7B, 0x58, 0x59, 0x53,
  0x23, 0x58, 0x48, 0x67, 0x75, 0xD6, 0xDC, 0x83, 0x6A, 0xBE, 0x30, 0xFC, 0x03, 0xBD, 0x5F, 0xDC,
  0xE8, 0x31, 0x0A, 0x00, 0x00
};
static void handleRootCSS() {
  server.sendHeader("Content-Encoding", "gzip");
  server.sendHeader("Cache-Control", "public, max-age=3600");
  server.send_P(200, "text/css", (PGM_P)ROOT_CSS_GZ, sizeof(ROOT_CSS_GZ));
}

static const uint8_t ROOT_JS_GZ[] PROGMEM = {
  0x1F, 0x8B, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0xFF, 0x55, 0x8F, 0xC1, 0x6E, 0xC3, 0x20,
  0x10, 0x44, 0x7F, 0x25, 0x37, 0xA0, 0x8E, 0x70, 0x7A, 0xAD, 0x83, 0x2B, 0x35, 0xB5, 0xD4, 0x4A,
  0x49, 0x73, 0x68, 0x8F, 0xB9, 0x00, 0xBB, 0x56, 0x90, 0x30, 0x50, 0x43, 0x22, 0x57, 0x91, 0xFF,
  0xBD, 0x58, 0x71, 0xD4, 0xF4, 0xB6, 0xCC, 0x32, 0x33, 0x6F, 0xC1, 0xEB, 0x53, 0x87, 0x2E, 0x71,
  0x09, 0xD0, 0x9C, 0xF3, 0xB0, 0x35, 0x31, 0xA1, 0xC3, 0x9E, 0x92, 0xD7, 0xFD, 0x6E, 0xE3, 0x5D,
  0x9A, 0x34, 0x2F, 0x01, 0x81, 0x2C, 0x29, 0x13, 0xF5, 0x45, 0x7B, 0x17, 0xD3, 0x42, 0x09, 0xB8,
  0x39, 0xBF, 0x4F, 0xD8, 0xFF, 0x7C, 0xA2, 0x45, 0x9D, 0x7C, 0xB6, 0x71, 0xE5, 0x07, 0xC2, 0x2A,
  0xD3, 0x52, 0xC5, 0xE6, 0xCF, 0x52, 0x28, 0x6E, 0x5C, 0x0E, 0x7D, 0xFB, 0xDA, 0x6D, 0x79, 0x0C,
  0xD6, 0x24, 0x5A, 0xAE, 0x55, 0x7F, 0x88, 0x0F, 0x87, 0xF2, 0xB9, 0x2E, 0x0D, 0xE3, 0x9D, 0x0C,
  0x74, 0x10, 0xF5, 0xC0, 0x53, 0x6F, 0x3A, 0xCA, 0x18, 0x6F, 0x8D, 0x4D, 0x99, 0xE2, 0xC5, 0x7B,
  0x8B, 0xD2, 0xB1, 0xEA, 0x2E, 0x41, 0x10, 0x52, 0x49, 0xDE, 0xFA, 0xBE, 0x91, 0xFA, 0x38, 0xB9,
  0xE6, 0x1A, 0xF8, 0x63, 0xD2, 0x3D, 0xCA, 0x84, 0x8D, 0xC5, 0xE9, 0x45, 0x09, 0x98, 0x73, 0x46,
  0x02, 0xAE, 0xAD, 0x8C, 0xF1, 0x43, 0x76, 0x28, 0x48, 0xA6, 0x7C, 0x4F, 0xD8, 0x91, 0xEA, 0xEA,
  0x0D, 0x62, 0xC8, 0x05, 0x80, 0xC3, 0xBE, 0xA5, 0xE4, 0xE9, 0xCA, 0x1F, 0xEA, 0x15, 0xBB, 0xC0,
  0x5D, 0xEF, 0xC0, 0xA3, 0x35, 0x1A, 0xE9, 0x6A, 0x19, 0x8A, 0x47, 0x56, 0x90, 0xB5, 0xAA, 0x49,
  0x71, 0x13, 0x27, 0x69, 0xA6, 0xCF, 0x9B, 0x32, 0xAF, 0x46, 0xB4, 0x11, 0x17, 0xFF, 0x02, 0xF2,
  0x19, 0x32, 0x04, 0x74, 0xB0, 0x39, 0x1A, 0x0B, 0x14, 0xD8, 0xC8, 0xC6, 0x91, 0x55, 0xBF, 0x5F,
  0x90, 0x0D, 0xFC, 0x86, 0x01, 0x00, 0x00
};
static void handleRootJS() {
  server.sendHeader("Content-Encoding", "gzip");
  server.sendHeader("Cache-Control", "public, max-age=3600");
  server.send_P(200, "application/javascript", (PGM_P)ROOT_JS_GZ, sizeof(ROOT_JS_GZ));
}

String htmlHeader(const String& title) {
  String html;
  html.reserve(12000);
  html += "<!DOCTYPE html><html lang='pt-BR'><head><meta charset='utf-8'>"
          "<meta name='viewport' content='width=device-width,initial-scale=1,viewport-fit=cover'>"
          "<meta name='theme-color' content='#123b67'>";
  html += "<title>" + htmlEscape(title) + "</title>";
  html += "<link rel='stylesheet' href='/root.css'><script defer src='/root.js'></script></head><body><div class='page'>";
  return html;
}

void handleRoot() {
  server.sendHeader("Cache-Control","no-store");
  String html = htmlHeader("PROSERVICE | Central de Pivô");
  html += "<div class='hero'><div class='heroRow'><div>"
          "<div class='brand'>PROSERVICE</div>"
          "<h1>Central Inteligente de Pivô</h1>";
  html += "<div class='heroSub'>Pivô " + htmlEscape(cfg.pivotId) + " • " + htmlEscape(cfg.hardwareName) + "</div>";
  html += "</div><div class='fw'>Firmware " + String(FIRMWARE_VERSION) + "</div></div></div>";
  html += R"DIAG(<section class='card'><h2>Rede e autorizacao de partida</h2>
<p id='diagLive'>Consultando central...</p><p>Rede: <b id='diagNetwork'>---</b></p>
<p>Cabo Ethernet: <b id='diagEthernet'>---</b> | IP: <b id='diagIp'>---</b></p>
<p>GPS para nova partida: <b id='diagGps'>---</b></p>
<p>Ultimo comando: <b id='diagResult'>---</b></p><p id='diagReason'></p>
<p>Tempo HTTP: <b id='diagHttp'>---</b></p><p>Protecao entre comandos: <b id='diagGuard'>---</b></p>
<p>Protecao de movimento: <b id='movementLive'>Consultando...</b></p>
<p><a href='/status'>Abrir diagnostico JSON</a></p></section>
<script>
let diagBusy=false;
async/*JS*/ function commandDiagnostic(){if(diagBusy)return;diagBusy=true;let e=id=>document.getElementById(id),ctl=new AbortController(),timer=setTimeout(()=>ctl.abort(),5000);try{let r=await fetch('/status',{cache:'no-store',signal:ctl.signal});if(!r.ok)throw Error();let d=await r.json();e('diagLive').textContent='Atualizado: '+new Date().toLocaleTimeString();e('diagNetwork').textContent=d.networkTransport||'Sem conexao';e('diagEthernet').textContent=!d.ethernetInitialized?'FALHA W5500':d.ethernetConnected?'CONECTADO':d.ethernetLink?'AGUARDANDO DHCP':'SEM LINK';e('diagIp').textContent=d.ethernetIp||'Sem IP';e('diagGps').textContent=d.gpsStartReady?'PRONTO':'INDISPONIVEL';e('diagResult').textContent=d.commandRejected?'REJEITADO / FALHOU':d.commandApplied?'APLICADO NAS SAIDAS':d.commandAccepted?'ACEITO, AINDA SEM CONFIRMACAO':d.commandReceived?'RECEBIDO, NAO ACEITO':'SEM COMANDO';e('diagReason').textContent=d.commandRejected?d.commandReason:(d.alertMessage||d.startBlockReason||d.commandReason||'');e('diagHttp').textContent='GET '+(d.portalGetAttempts?d.portalLastGetDurationMs+' ms (HTTP '+d.portalLastGetCode+')':'aguardando')+' | POST '+(d.portalPostAttempts?d.portalLastPostDurationMs+' ms (HTTP '+d.portalLastPostCode+')':'aguardando');e('diagGuard').textContent=d.commandGuardRemainingMs>0?Math.ceil(d.commandGuardRemainingMs/1000)+' s restantes':'Sem espera pendente'}catch(x){e('diagLive').textContent='SEM ATUALIZACAO: estado atual nao confirmado';for(let id of ['diagNetwork','diagEthernet','diagIp','diagGps','diagResult','diagHttp','diagGuard'])e(id).textContent='NAO CONFIRMADO';e('diagReason').textContent='Reconecte e atualize o diagnostico'}finally{clearTimeout(timer);diagBusy=false}}
commandDiagnostic();setInterval(commandDiagnostic,2000);
</script>)DIAG";
  html += R"RADIO(<div class='card'><h2>Rádio CENTRAL</h2><p id='radioLive'>Atualizando...</p><h1 id='radioActual'>NÃO CONFIRMADO</h1><p id='radioLink'></p><p id='radioPeer'></p><p id='radioMessage'></p><form method='POST' action='/loraread'><button type='submit'>Ler canal real</button></form></div>
<script>
async/*JS*/ function radioRefresh(){let e=x=>document.getElementById(x);try{let d=await(await fetch('/status',{cache:'no-store'})).json();e('radioLive').textContent='Atualizado';e('radioActual').textContent=d.radioChannelKnown?'CANAL '+d.radioChannel:'NÃO CONFIRMADO';e('radioLink').textContent=(d.radioLinkRecent?'LINK RECENTE':'SEM LINK')+' | aceitos '+d.acceptedSinceRadioCheck;e('radioPeer').textContent='Canal GPS informado: '+(d.peerReportedChannel>=0?d.peerReportedChannel:'nenhum');e('movementLive').textContent=d.movementReason+(d.movementProtectionActive?' | resta '+Math.ceil(d.movementRemainingMs/1000)+' s | avanço '+Number(d.movementProgressM).toFixed(2)+' m':'');e('radioMessage').textContent=d.radioMessage}catch(x){e('radioLive').textContent='SEM ATUALIZAÇÃO';e('radioActual').textContent='NÃO CONFIRMADO';e('movementLive').textContent='SEM ATUALIZAÇÃO';}}radioRefresh();setInterval(radioRefresh,2000);
</script>)RADIO";
  // O diagnostico completo continua disponivel em /status (JSON). A versao
  // extensa abaixo fica preservada no fonte para manutencao, mas nao entra no
  // binario da particao padrao.
#if 0
  html += "<div class='card'><h2>Status operacional</h2><div class='box'>";
  html += "Estado controlador: <b>" + stateToString(controllerState) + "</b><br>";
  html += "Pivô: <b>" + cfg.pivotId + "</b><br>";
  html += "Hardware: <b>" + htmlEscape(cfg.hardwareName) + "</b><br>";
  html += "Wi-Fi salvo: <b>" + (cfg.wifiSsid.length() ? htmlEscape(cfg.wifiSsid) : String("(vazio)")) + "</b><br>";
  html += "Backend remoto: <b>" + String(cfg.backendEnabled ? "<span class='ok'>ATIVO</span>" : "DESATIVADO / somente local") + "</b><br>";
  html += "Wi-Fi conectado: <b>" + String(WiFi.status() == WL_CONNECTED ? "<span class='ok'>SIM</span>" : "<span class='bad'>NÃO</span>") + "</b><br>";
  if (WiFi.status() == WL_CONNECTED) {
    html += "IP STA: <b>" + WiFi.localIP().toString() + "</b><br>";
    html += "Sinal: <b>" + String(WiFi.RSSI()) + " dBm</b><br>";
    html += "SSID atual: <b>" + htmlEscape(WiFi.SSID()) + "</b><br>";
  }
  html += "Diagnóstico Wi-Fi: <b>" + htmlEscape(wifiDiagnosticMessage) + "</b><br>";
  if (wifiLastFailure.length()) html += "<span class='bad'>" + htmlEscape(wifiLastFailure) + "</span><br>";
  html += "Comunicação portal: <b>";
  if (portalRequestFailureActive) html += "<span class='bad'>OFF-LINE CONFIRMADO</span>";
  else if (portalConsecutiveFailures > 0) html += "OSCILAÇÃO EM OBSERVAÇÃO";
  else html += "<span class='ok'>ON-LINE CONFIRMADO</span>";
  html += "</b><br>";
  html += "Falhas HTTP consecutivas: <b>" + String(portalConsecutiveFailures) + "</b><br>";
  html += "Respostas válidas consecutivas: <b>" + String(portalConsecutiveSuccesses) + "</b><br>";
  html += "Última resposta válida: <b>" + String(lastServerOkMs ? (millis() - lastServerOkMs) / 1000UL : 0) + " s atrás</b><br>";
  html += "Emergência 1: <b>" + String(readEmergency1() ? "ATIVA" : "INATIVA") + "</b><br>";
  html += "Emergência 2: <b>" + String(readEmergency2() ? "ATIVA" : "INATIVA") + "</b><br>";
  html += "Segurança OK: <b>" + String(safetyOk() ? "SIM" : "NÃO") + "</b><br>";
  html += "Pressostato: <b>" + String(pressureOk() ? "ATIVO" : "INATIVO") + "</b><br>";
  html += "Corte ISR instalado: <b>" + String(safetyInterruptsConfigured ? "SIM" : "NÃO") + "</b><br>";
  html += "Cortes ISR emergência: <b>" + String((uint32_t)fastEmergencyIsrCutCount) + "</b><br>";
  html += "Cortes ISR pressão: <b>" + String((uint32_t)fastPressureIsrCutCount) + "</b><br>";
  html += "Cortes tarefa emergência: <b>" + String((uint32_t)fastSafetyCutCount) + "</b><br>";
  html += "Cortes tarefa pressão: <b>" + String((uint32_t)fastPressureCutCount) + "</b><br>";
  html += "Falha: <b>" + htmlEscape(faultMessage) + "</b><br>";
  html += "Alerta: <b>" + htmlEscape(alertMessage) + "</b><br>";
  html += "Comando aplicado nas saídas: <b>" + String(commandedOutputsAreApplied() ? "<span class='ok'>SIM</span>" : "<span class='bad'>NÃO</span>") + "</b><br>";
  html += "Estado da confirmação: <b>" + htmlEscape(commandAcknowledgementStatus()) + "</b><br>";
  html += "Partida pendente de 20s: <b>" + String(pendingCmdValid ? "SIM" : "NÃO") + "</b><br>";
  if (pendingCmdValid) {
    html += "commandId pendente: <b>" + htmlEscape(pendingCmd.commandId) + "</b><br>";
    html += "Pendente ainda autorizado: <b>" + String(pendingCommandStillAuthorized() ? "SIM" : "NÃO") + "</b><br>";
  }
  html += "commandId atual: <b>" + (commandExecution.commandId.length() ? htmlEscape(commandExecution.commandId) : String("(nenhum)")) + "</b><br>";
  html += "Resultado da execução: <b>" + htmlEscape(commandExecution.result) + "</b><br>";
  html += "Motivo do resultado: <b>" + (commandExecution.reason.length() ? htmlEscape(commandExecution.reason) : String("(nenhum)")) + "</b><br>";
  html += "Conflito de commandId: <b>" + String(commandIdConflictActive ? "<span class='bad'>SIM</span>" : "NÃO") + "</b><br>";
  html += "Eventos aguardando ACK: <b>" + String(eventOutboxCount) + "/" + String(EVENT_OUTBOX_SLOTS) + "</b><br>";
  html += "Saturação crítica da fila: <b>" + String(eventOutboxCriticalOverflowCount) + "</b><br>";
  html += "Persistência NVS atual: <b>" + String(persistenceHealthy ? "<span class='ok'>OK</span>" : "<span class='bad'>FALHA</span>") + "</b><br>";
  html += "Falha de persistência já detectada: <b>" + String(persistenceFaultLatched ? "SIM" : "NÃO") + "</b><br>";
  html += "Último erro de persistência: <b>" + (persistenceLastError.length() ? htmlEscape(persistenceLastError) : String("(nenhum)")) + "</b><br>";
  html += "Reparos automáticos do journal neste boot: <b>" + String(operationJournalRepairCount) + "</b><br>";
  html += "Intertravamento NVS da partida: <b>" + String(operationJournalStartInterlock ? "<span class='bad'>ATIVO</span>" : "<span class='ok'>LIBERADO</span>") + "</b><br>";
  html += "Motivo do intertravamento: <b>" + (operationJournalInterlockReason.length() ? htmlEscape(operationJournalInterlockReason) : String("(nenhum)")) + "</b><br>";
  html += "Religamento habilitado no comando: <b>" + String(currentCmd.restartAfterPower ? "SIM" : "NÃO") + "</b><br>";
  html += "Religamento pendente: <b>" + String(autoRestart.pending ? "SIM" : "NÃO") + "</b><br>";
  html += "Estado do religamento: <b>" + htmlEscape(autoRestart.status) + "</b><br>";
  html += "Tempo mínimo restante: <b>" + String((autoRestartRemainingMs() + 999UL) / 1000UL) + " s</b><br>";
  html += "Lâmina solicitada: <b>" + String(currentCmd.requestedDepthMm, 2) + " mm</b><br>";
  html += "Lâmina mínima: <b>" + String(currentCmd.minDepthMm, 2) + " mm</b><br>";
  html += "Última torre %: <b>" + String(currentCmd.lastTowerPercent, 2) + "</b><br>";
  html += "Ciclo relé 4: <b>" + String(currentCmd.lastTowerCycleMs) + " ms</b><br>";
  html += "R1 direita: <b>" + String(outputIsOnLogicLevel(PIN_OUT_RIGHT) ? "ON" : "OFF") + "</b><br>";
  html += "R2 esquerda: <b>" + String(outputIsOnLogicLevel(PIN_OUT_LEFT) ? "ON" : "OFF") + "</b><br>";
  html += "R3 bomba: <b>" + String(outputIsOnLogicLevel(PIN_OUT_PUMP) ? "ON" : "OFF") + "</b><br>";
  html += "R4 ult_torre: <b>" + String(outputIsOnLogicLevel(PIN_OUT_LAST_TOWER) ? "ON" : "OFF") + "</b><br>";
  html += "R5 canhao: <b>" + String(outputIsOnLogicLevel(PIN_OUT_CANNON) ? "ON" : "OFF") + "</b><br>";
  html += "R6 liberação/placa pronta: <b>" + String(outputIsOnLogicLevel(PIN_OUT_AUX6) ? "FECHADO" : "ABERTO") + "</b><br>";
  html += "ID GPS autorizado: <b>" + (cfg.gpsNodeId.length() ? htmlEscape(cfg.gpsNodeId) : String("(vazio)")) + "</b><br>";
  html += "Último ID LoRa visto: <b>" + (loraLastSeenId.length() ? htmlEscape(loraLastSeenId) : String("(nenhum)")) + "</b><br>";
  html += "Último pacote processado foi aceito (histórico): <b>" + String(loraLastPacketAccepted ? "<span class='ok'>SIM</span>" : "<span class='bad'>NÃO</span>") + "</b><br>";
  html += "Motivo última rejeição LoRa: <b>" + (loraRejectReason.length() ? htmlEscape(loraRejectReason) : String("(nenhum)")) + "</b><br>";
  html += "Pacotes LoRa ignorados: <b>" + String(loraIgnoredPackets) + "</b><br>";
  html += "AUX E220 (GPIO35): <b>" + String(digitalRead(PIN_LORA_AUX) == HIGH ? "HIGH / pronto" : "LOW / ocupado ou indisponivel") + "</b><br>";
  html += "Preferência de canal salva no ESP32 (não é leitura do rádio): <b>" + String(e220Channel) + " (" + String(850.125f + e220Channel, 3) + " MHz)</b><br>";
  html += "Bytes LoRa recebidos: <b>" + String(loraRxByteCount) + "</b><br>";
  html += "Linhas LoRa recebidas: <b>" + String(loraRxLineCount) + "</b><br>";
  html += "Task RX LoRa: <b>" + String(loraRxTaskRunning ? "ATIVA" : "INATIVA") + "</b><br>";
  html += "Fila RX LoRa: <b>" + String(loraRxQueue ? uxQueueMessagesWaiting(loraRxQueue) : 0) + "</b><br>";
  html += "Descartes fila RX: <b>" + String((uint32_t)loraRxQueueDrops) + "</b><br>";
  html += "Erros JSON LoRa: <b>" + String(loraJsonErrorCount) + "</b><br>";
  html += "Última atividade RX LoRa: <b>" + String(loraLastAnyRxMs ? String((millis()-loraLastAnyRxMs)/1000UL)+" s atrás" : String("(nunca)")) + "</b><br>";
  bool loraGpsFreshForPortal = loraGpsValid && loraLastRxMs > 0 && (millis() - loraLastRxMs < 10000UL);
  bool effectiveGpsForPortal = hasGoodGpsWithinTimeout();
  double effectiveLatForPortal = effectiveGpsForPortal ? lastGoodGpsLat : 0.0;
  double effectiveLonForPortal = effectiveGpsForPortal ? lastGoodGpsLon : 0.0;
  html += "Protecao de movimento: <b id='movementLive'>Consultando...</b><br>";
  html += "GPS válido: <b>" + String(effectiveGpsForPortal ? "<span class='ok'>SIM</span>" : "<span class='bad'>NÃO</span>") + "</b><br>";
  html += "Amostra GPS recente (10s): <b>" + String(loraGpsFreshForPortal ? "<span class='ok'>SIM</span>" : "NÃO") + "</b><br>";
  html += "Fonte GPS: <b>" + String(loraGpsFreshForPortal ? "LoRa / ponta do pivô" : (effectiveGpsForPortal ? "LoRa / última posição retida" : "(nenhuma)")) + "</b><br>";
  html += "Idade da última posição válida: <b>" + String(effectiveGpsForPortal ? String((millis()-lastGoodGpsMs)/1000UL)+" s" : String("(indisponível)")) + "</b><br>";
  html += "GPS satélites (última amostra): <b>" + String(gpsSatellites) + "</b><br>";
  html += "GPS HDOP (última amostra): <b>" + String(gpsHdop, 2) + "</b><br>";
  html += "GPS latitude: <b>" + (effectiveGpsForPortal ? String(effectiveLatForPortal, 8) : String("(sem fix)")) + "</b><br>";
  html += "GPS longitude: <b>" + (effectiveGpsForPortal ? String(effectiveLonForPortal, 8) : String("(sem fix)")) + "</b><br>";
  html += "GPS curso: <b>" + String(gpsCourseDeg, 2) + " °</b><br>";
  html += "Centro pivô lat: <b>" + String(currentCmd.centerLat, 6) + "</b><br>";
  html += "Centro pivô lon: <b>" + String(currentCmd.centerLon, 6) + "</b><br>";
  html += "Raio pivô: <b>" + String(currentCmd.radiusMeters, 2) + " m</b><br>";
  html += "Área pivô: <b>" + String(currentCmd.areaHectares, 4) + " ha</b><br>";
  html += "Perfil físico: <b>" + String((cfg.localProfileEnabled && localPivotGeometryValid()) ? "<span class='ok'>LOCAL / NVS VÁLIDO</span>" : (cfg.localProfileEnabled ? "<span class='bad'>LOCAL INVÁLIDO — usando servidor</span>" : "Servidor")) + "</b><br>";
  html += "Correção angular GPS: <b>" + String(currentCmd.gpsCorrectionDeg, 2) + " °</b><br>";
  html += "Distância ao centro: <b>" + (pivotAngleValid ? String(distanceFromCenterM, 2) + " m" : String("(indisponível)")) + "</b><br>";
  html += "Grau atual do pivô: <b>" + (pivotAngleValid ? String(pivotAngleDeg, 2) + " °" : String("(indisponível)")) + "</b><br>";
  html += "</div></div>";
#endif

  html += "<div class='card'><h2>Status operacional</h2><div class='box'>";
  html += "Estado: <b>" + stateToString(controllerState) + "</b><br>";
  html += "Rede para o portal: <b>" + String(uplinkName()) + "</b><br>";
  html += "Ethernet: <b>" + String(!ethernetInitialized ? "FALHA W5500" :
      ethernetReady() ? "CONECTADO" : ETH.connected() ? "AGUARDANDO IP/DHCP" : "SEM CABO/LINK") + "</b><br>";
  if(ethernetReady()) html += "IP Ethernet: <b>" + ETH.localIP().toString() + "</b><br>";
  html += "Wi-Fi: <b>" + String(WiFi.status() == WL_CONNECTED ? "<span class='ok'>CONECTADO</span>" : "<span class='bad'>DESCONECTADO</span>") + "</b><br>";
  html += "Segurança: <b>" + String(safetyOk() ? "<span class='ok'>OK</span>" : "<span class='bad'>FALHA</span>") + "</b><br>";
  html += "Pressão: <b>" + String(pressureOk() ? "OK" : "SEM PRESSÃO") + "</b><br>";
  html += "GPS: <b>" + String(hasGoodGpsWithinTimeout() ? "<span class='ok'>VÁLIDO</span>" : "<span class='bad'>SEM SINAL</span>") + "</b><br>";
  html += "Falha: <b>" + htmlEscape(faultMessage.length() ? faultMessage : String("(nenhuma)")) + "</b><br>";
  html += "Eventos sem ACK: <b>" + String(eventOutboxCount) + "/" + String(EVENT_OUTBOX_SLOTS) + "</b><br>";
  html += "Persistência: <b>" + String(persistenceHealthy ? "<span class='ok'>OK</span>" : "<span class='bad'>FALHA</span>") + "</b><br>";
  html += "<a href='/status'>Abrir diagnóstico completo em JSON</a>";
  html += "</div></div>";

  html += "<div class='card'><h2>Registro unificado persistente — systemEventLog</h2>";
  html += "<div class='muted'>A mesma variável é enviada ao portal e permanece gravada após desligamento. Reúne reset, falha, parada, comunicação e comandos. R=R1..R6, 1 significa relé ligado.</div>";
  html += "<p><b>Último reset:</b> " + htmlEscape(lastResetReason) + " | Inicializações: " + String(bootCounter) + "</p>";
  html += "<p><b>Última parada:</b> " + htmlEscape(lastStopReason) + " | Paradas: " + String(stopEventCounter) + "<br><span class='muted'>" + htmlEscape(lastStopDetail) + "</span></p>";
  html += "<p><b>Última falha:</b> " + htmlEscape(lastFaultReason) + " | Falhas: " + String(faultEventCounter) + "<br><span class='muted'>" + htmlEscape(lastFaultDetail) + "</span></p>";
  html += "<p><b>Comandos inválidos nesta inicialização:</b> " + String(invalidPortalCommandCount) + " | Último valor: " + htmlEscape(lastInvalidPortalDirection) + "</p>";
  html += "<h3>Últimos eventos</h3><div class='muted'>";
  uint32_t shownEvents = min(diagEventCounter, (uint32_t)DIAG_HISTORY_SLOTS);
  for (uint32_t i = 0; i < shownEvents; i++) {
    uint8_t slot = (diagEventCounter - 1 - i) % DIAG_HISTORY_SLOTS;
    if (diagHistory[slot].length()) html += htmlEscape(diagHistory[slot]) + "<br><br>";
  }
  html += "</div></div>";

  html += "<div class='card'><h2>Configuração da central</h2>"
          "<div class='muted'>Configure rede, integração remota e identificação desta unidade. Os dados ficam salvos localmente no ESP32.</div>"
          "<form method='POST' action='/save'>";
  html += "<label>Redes Wi-Fi 2,4 GHz encontradas (sinal, canal e segurança)</label><select name='ssid'>" + buildWifiOptions() + "</select>";
  html += "<label>SSID manual</label><input name='ssid_manual' value='' placeholder='Opcional'>";
  html += "<label>Senha Wi-Fi</label><input name='pass' value='' type='password' autocomplete='new-password' placeholder='Em branco conserva senha da mesma rede'>";
  html += "<h2>Integração com painel / servidor</h2>";
  html += "<label>Usar backend remoto</label><select name='backend'>";
  html += String(cfg.backendEnabled ? "<option value='1' selected>SIM</option><option value='0'>NÃO - operar somente local/LoRa</option>"
                                   : "<option value='1'>SIM</option><option value='0' selected>NÃO - operar somente local/LoRa</option>");
  html += "</select>";
  html += "<label>URL base do servidor</label><input name='server' value='" + htmlEscape(cfg.serverUrl) + "' placeholder='https://novo-painel.exemplo.com'>";
  html += "<label>Rota/URL de comando</label><input name='cmdroute' value='" + htmlEscape(cfg.commandRoute) + "' placeholder='/api/pivot-command ou https://...'>";
  html += "<label>Rota/URL de heartbeat</label><input name='hbroute' value='" + htmlEscape(cfg.heartbeatRoute) + "' placeholder='/api/telemetry/heartbeat ou https://...'>";
  html += "<div class='muted'>Esses três campos podem ser alterados no futuro quando o novo painel web estiver pronto.</div>";
  html += "<label>ID do pivô</label><input name='pivot' value='" + htmlEscape(cfg.pivotId) + "' placeholder='Ex: 4A'>";
  html += "<label>Token do dispositivo</label><input name='token' value='' type='password' autocomplete='new-password' placeholder='Em branco conserva o token salvo'>";
  html += "<label>Nome do hardware</label><input name='hwname' value='" + htmlEscape(cfg.hardwareName) + "' placeholder='Ex: ESP32-PIVO-PC01'>";
  html += "<label>ID do GPS LoRa autorizado</label><input name='gpsnode' value='" + htmlEscape(cfg.gpsNodeId) + "' placeholder='Ex: GPS-PIVO-01'>";

  html += "<h2>Dados locais deste pivô</h2>"
          "<div class='muted'>Estes dados ficam salvos somente neste ESP32. O mesmo firmware pode ser usado em vários pivôs sem recompilar.</div>"
          "<label>Usar perfil físico local</label><select name='localprofile'>";
  html += String(cfg.localProfileEnabled ? "<option value='1' selected>SIM - dados locais têm prioridade</option><option value='0'>NÃO - usar dados do servidor</option>"
                                         : "<option value='1'>SIM - dados locais têm prioridade</option><option value='0' selected>NÃO - usar dados do servidor</option>");
  html += "</select>";
  html += "<label>Latitude do centro do pivô</label><input name='centerlat' value='" + String(cfg.localCenterLat, 8) + "' placeholder='Ex: -14.936119'>";
  html += "<label>Longitude do centro do pivô</label><input name='centerlon' value='" + String(cfg.localCenterLon, 8) + "' placeholder='Ex: -54.037939'>";
  html += "<label>Raio do pivô (m)</label><input name='radius' type='number' step='0.01' min='0' value='" + String(cfg.localRadiusMeters, 2) + "'>";
  html += "<label>Área do pivô (ha)</label><input name='area' type='number' step='0.0001' min='0' value='" + String(cfg.localAreaHectares, 4) + "'>";
  html += "<div class='muted'>Preencha raio OU área; se um deles estiver zero, o ESP32 calcula automaticamente.</div>";
  html += "<label>Correção angular do GPS (graus)</label><input name='gpscorr' type='number' step='0.01' value='" + String(cfg.localGpsCorrectionDeg, 2) + "'>";
  html += "<label>Lâmina mínima local (mm)</label><input name='mindepth' type='number' step='0.01' min='0' value='" + String(cfg.localMinDepthMm, 2) + "'>";
  html += "<label>Fator de segurança (%)</label><input name='safetyfactor' type='number' step='0.1' min='0' max='30' value='" + String(cfg.localSafetyFactorPercent, 2) + "'>";
  html += "<label>Tempo máximo para pressão (s)</label><input name='presswait' type='number' min='10' max='30000' value='" + String(cfg.localPressureWaitSec) + "'>";
  html += "<label>Ciclo da última torre / Relé 4 (ms)</label><input name='towercycle' type='number' min='1000' max='60000' value='" + String(cfg.localLastTowerCycleMs) + "'>";

  html += "<button class='primary' type='submit'>Salvar tudo localmente e reiniciar</button>"
          "<a href='/'><button class='dark' type='button'>Atualizar lista Wi-Fi</button></a>"
          "</form></div>";

  html += "<div class='card'><h2>Canal do radio E220</h2>"
          "<div class='muted'>O GPS deste pivo deve ser configurado exatamente no mesmo canal. A troca fica bloqueada enquanto o pivo estiver rodando.</div>"
          "<form method='POST' action='/lorachannel'><label>Novo canal desejado (não é o canal lido)</label><select name='channel'>";
  for (int ch = 0; ch <= 80; ch++) html += "<option value='" + String(ch) + "'" + (e220Channel == ch ? " selected" : "") + ">Canal " + String(ch) + " - " + String(850.125f + ch, 3) + " MHz</option>";
  html += "</select><button class='primary' type='submit'>Gravar canal no E220</button></form>";
  html += "<div class='muted'>Resultado: " + htmlEscape(e220ConfigStatus) + "</div></div>";


  if(canReportSavedCommandCancellation()) {
    html += "<p>O servidor nao esta reenviando comandos. Para cancelar a confirmacao antiga com a central parada, informe a interrupcao abaixo. Nao liga reles e exige novo PARAR pelo portal.</p>";
    html += "<form method='POST' action='/maintenance/cancel-saved-command'><input type='hidden' name='commandId' value='"+htmlEscape(commandExecution.commandId)+"'><button type='submit'>Informar cancelamento local ao portal</button></form>";
  }
  html += "<div class='card'><h2>Ações e manutenção</h2>"
          "<form method='POST' action='/setcenter'><button class='primary' type='submit'>Usar posição GPS atual como CENTRO</button></form>"
          "<div class='muted'>Use este botão somente quando o GPS da ponta estiver fisicamente colocado no centro do pivô durante a instalação.</div>"
          "<p><a href='/local'><button class='primary' type='button'>Abrir painel de teste local</button></a></p>"
          "<form method='POST' action='/resetwifi' style='display:inline-block;'><button class='warn' type='submit'>Limpar apenas Wi-Fi</button></form>"
          "<form method='POST' action='/reboot' style='display:inline-block;'><button class='dark' type='submit'>Reiniciar ESP32</button></form>"
          "<div class='muted' style='margin-top:16px;'>Segure o botão BOOT por 6 segundos para abrir o modo configuração.</div>"
          "</div>";
  html += "<div class='footer'>PROSERVICE • Central Inteligente de Pivô • " + String(FIRMWARE_VERSION) + "</div>";
  html += "</div></body></html>";

  server.send(200, "text/html", html);
}

static const uint8_t LOCAL_PANEL_GZIP[] PROGMEM = {
  0x1F, 0x8B, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0xFF, 0xBD, 0x58, 0x4B, 0x93, 0xDB, 0x36, 0x12, 0xBE, 0xCF, 0xAF,
  0x40, 0xE4, 0x4D, 0x48, 0xC5, 0x14, 0xF5, 0x9A, 0xF1, 0x8C, 0xA9, 0x47, 0x6A, 0x6C, 0x8F, 0x93, 0x49, 0xD9, 0x1E, 0x95,
  0xA5, 0x24, 0xB5, 0xE5, 0x72, 0x39, 0x10, 0x09, 0x4A, 0x98, 0x21, 0x09, 0x06, 0x00, 0xF5, 0x58, 0x8D, 0x2E, 0xFB, 0x33,
  0xF6, 0x96, 0xDA, 0x43, 0x6A, 0xB7, 0x2A, 0x55, 0x39, 0xE4, 0xB4, 0xC7, 0xD5, 0x1F, 0xDB, 0x06, 0x40, 0x6A, 0xF4, 0xF4,
  0x26, 0xB5, 0xA9, 0xBD, 0x50, 0x64, 0x37, 0xFA, 0x89, 0xC6, 0xD7, 0x0D, 0x9D, 0xB4, 0x3F, 0x79, 0x71, 0xF3, 0x7C, 0xF0,
  0xE7, 0xDE, 0x15, 0x1A, 0xCB, 0x38, 0xEA, 0xB6, 0xD5, 0x13, 0x45, 0x38, 0x19, 0x75, 0x4A, 0xA9, 0xAC, 0x3C, 0x7B, 0x5B,
  0x02, 0x12, 0xC1, 0x41, 0xB7, 0x1D, 0x13, 0x89, 0x91, 0x3F, 0xC6, 0x5C, 0x10, 0xD9, 0x29, 0x65, 0x32, 0xAC, 0x5C, 0x94,
  0xBA, 0x27, 0x86, 0x9C, 0xE0, 0x98, 0x74, 0x4A, 0x13, 0x4A, 0xA6, 0x29, 0xE3, 0xB2, 0x84, 0x7C, 0x96, 0x48, 0x92, 0xC0,
  0xB2, 0x29, 0x0D, 0xE4, 0xB8, 0x13, 0x90, 0x09, 0xF5, 0x49, 0x45, 0x7F, 0x38, 0x34, 0xA1, 0x92, 0xE2, 0xA8, 0x22, 0x7C,
  0x1C, 0x91, 0x4E, 0x5D, 0xE9, 0x90, 0x54, 0x46, 0xA4, 0x3B, 0x20, 0x42, 0x12, 0x14, 0x31, 0xA0, 0xA3, 0x80, 0xA1, 0x94,
  0x4E, 0x56, 0xBF, 0xB6, 0xAB, 0x86, 0x77, 0xD2, 0x16, 0x72, 0xAE, 0x7E, 0x87, 0x2C, 0x98, 0x2F, 0x42, 0x50, 0x5F, 0x09,
  0x71, 0x4C, 0xA3, 0xB9, 0x77, 0xC9, 0x41, 0x99, 0x23, 0x70, 0x22, 0x2A, 0x82, 0x70, 0x1A, 0xB6, 0x62, 0xCC, 0x47, 0x34,
  0xF1, 0x6A, 0xAD, 0x21, 0xF6, 0xEF, 0x46, 0x9C, 0x65, 0x49, 0xE0, 0x3D, 0xAA, 0x85, 0xF5, 0xF3, 0x06, 0x6E, 0xF9, 0x2C,
  0x62, 0xDC, 0x7B, 0x44, 0x1A, 0xE4, 0x22, 0xAC, 0x2D, 0xDD, 0x29, 0xC7, 0xE9, 0x22, 0xC6, 0x33, 0xE3, 0x99, 0x57, 0xAF,
  0x9D, 0xD5, 0xD2, 0x59, 0xA1, 0x00, 0x67, 0x92, 0xB5, 0x52, 0x1C, 0x04, 0x34, 0x19, 0x79, 0xF5, 0x27, 0xE9, 0x6C, 0x79,
  0xE2, 0xFA, 0x98, 0x07, 0x8B, 0x4D, 0xBD, 0x75, 0xD2, 0x78, 0xDA, 0x1C, 0xB6, 0x86, 0x8C, 0x07, 0x84, 0x7B, 0xF5, 0x74,
  0x86, 0x04, 0x8B, 0x68, 0x80, 0x1E, 0x35, 0x9B, 0xA7, 0xF5, 0xB3, 0xB3, 0x9C, 0x51, 0xE1, 0x38, 0xA0, 0x99, 0xD0, 0x5A,
  0xB6, 0x54, 0xE6, 0xB6, 0x2A, 0x43, 0x26, 0x25, 0x8B, 0xBD, 0xFA, 0x29, 0x90, 0x86, 0x6C, 0x56, 0x11, 0x63, 0x1C, 0xB0,
  0xA9, 0x57, 0x43, 0x17, 0xA0, 0xB1, 0x01, 0x54, 0xF4, 0xA8, 0x56, 0xAB, 0x9D, 0x2E, 0x4F, 0xC6, 0x75, 0x67, 0xDC, 0x58,
  0x14, 0x21, 0xA2, 0x1A, 0xAA, 0x37, 0xC0, 0x31, 0x77, 0xC4, 0x69, 0xB0, 0x08, 0xA8, 0x48, 0x23, 0x3C, 0xF7, 0xD4, 0x47,
  0x4B, 0x3D, 0x2A, 0x92, 0xC4, 0x40, 0x91, 0xA4, 0x02, 0x71, 0x67, 0x71, 0x22, 0x3C, 0x4E, 0x52, 0x82, 0xA5, 0xAD, 0x42,
  0xAB, 0x84, 0x54, 0x3A, 0x31, 0x4D, 0x20, 0x7C, 0xBB, 0x51, 0x87, 0xB8, 0x9D, 0x7A, 0xC8, 0xCB, 0xE5, 0xD6, 0x08, 0xA7,
  0x9E, 0xFA, 0x84, 0x68, 0x85, 0x5C, 0x1C, 0xC8, 0xE1, 0x4E, 0x48, 0xB5, 0xCD, 0x90, 0x94, 0x9C, 0x3B, 0x31, 0xFB, 0x33,
  0x25, 0x74, 0x34, 0x96, 0xDE, 0x79, 0x0D, 0x12, 0xCD, 0xEE, 0x16, 0x79, 0xEA, 0x4F, 0x71, 0x40, 0x2E, 0x80, 0x32, 0xC4,
  0x41, 0x41, 0x0A, 0x2F, 0xCE, 0xEB, 0xE7, 0x75, 0xD8, 0x0D, 0xCC, 0x93, 0x35, 0x6D, 0x38, 0x0C, 0x1B, 0x10, 0xEF, 0x30,
  0x83, 0xC4, 0x24, 0x8B, 0x62, 0x7F, 0x6A, 0x9F, 0xB6, 0xC0, 0xE5, 0xCA, 0xD8, 0xA8, 0x3E, 0xBD, 0xD0, 0xE9, 0xD2, 0xA9,
  0xAF, 0x1D, 0xF2, 0x6B, 0xC7, 0x8F, 0x96, 0x9F, 0x71, 0x01, 0xDA, 0x53, 0x46, 0xA1, 0x3C, 0xF9, 0x96, 0xDB, 0x5B, 0xD5,
  0xD2, 0x38, 0x7B, 0xD2, 0x24, 0xC3, 0xBC, 0x5A, 0xA6, 0x63, 0x2A, 0x49, 0xE1, 0x09, 0xE4, 0x84, 0xA5, 0x5B, 0x59, 0x09,
  0xFC, 0xC6, 0x93, 0xC6, 0x13, 0x08, 0x31, 0x0C, 0xB7, 0xE8, 0xA7, 0xE7, 0x67, 0x67, 0x4F, 0x9E, 0x2E, 0xDD, 0x34, 0x8B,
  0xB7, 0x05, 0x6A, 0x17, 0x4F, 0xEB, 0xC3, 0xC6, 0x12, 0x6A, 0x29, 0x49, 0x20, 0xB4, 0x4D, 0xD6, 0xB9, 0xDF, 0xC4, 0x24,
  0x58, 0xBA, 0x92, 0x4D, 0x09, 0xDF, 0xE2, 0x10, 0x7C, 0x76, 0x51, 0xF3, 0x97, 0x27, 0x11, 0x1E, 0x92, 0x68, 0xBD, 0xD3,
  0x43, 0x38, 0x2C, 0x77, 0x45, 0xBD, 0xAA, 0x52, 0xA9, 0x21, 0xA8, 0x95, 0x25, 0x4D, 0xD2, 0x4C, 0x3A, 0x82, 0x44, 0xC4,
  0x97, 0x9B, 0xA9, 0xD3, 0x95, 0x45, 0xFF, 0xA2, 0x42, 0xCE, 0x93, 0x05, 0x94, 0x9D, 0x2C, 0x6C, 0x25, 0xF1, 0xE9, 0x43,
  0x7E, 0x37, 0x4A, 0xDB, 0x44, 0xF6, 0x91, 0xE3, 0x15, 0x86, 0xA1, 0xAA, 0x9E, 0x18, 0x47, 0x91, 0x29, 0x05, 0x30, 0x4A,
  0xBC, 0x7A, 0x13, 0xB4, 0xE5, 0x2B, 0x9E, 0x9E, 0xE2, 0xE6, 0xF0, 0x62, 0xE9, 0x62, 0x5F, 0xD2, 0x09, 0x59, 0xB0, 0x4C,
  0x46, 0x34, 0x21, 0x5E, 0x73, 0xC3, 0x48, 0x5E, 0x27, 0x01, 0x20, 0x91, 0xCA, 0x85, 0xF1, 0x2B, 0x17, 0xCF, 0x73, 0x7E,
  0xD2, 0xAE, 0x1A, 0x54, 0x68, 0x57, 0x0D, 0x44, 0x29, 0x70, 0xE8, 0xB6, 0x03, 0x3A, 0x41, 0x7E, 0x84, 0x85, 0x00, 0xF8,
  0x81, 0x03, 0xAE, 0x00, 0x66, 0x83, 0xA4, 0x8E, 0xB0, 0xC2, 0xB4, 0x7A, 0xB7, 0x87, 0xC1, 0x66, 0x54, 0x00, 0x0E, 0x41,
  0x52, 0xE1, 0x0F, 0xA8, 0xAA, 0x6F, 0xA9, 0xD0, 0x61, 0x94, 0xBA, 0x97, 0x3E, 0x11, 0x02, 0x40, 0x89, 0x44, 0x18, 0x71,
  0x12, 0x14, 0x38, 0xC5, 0x32, 0x45, 0x62, 0xE8, 0xB2, 0x87, 0x02, 0x8C, 0xAE, 0xFA, 0xBD, 0x66, 0xC3, 0x45, 0x37, 0x28,
  0x66, 0x00, 0x60, 0x66, 0x45, 0x8A, 0x33, 0x01, 0xB8, 0xC9, 0x62, 0x9C, 0x04, 0x4C, 0xA0, 0x09, 0xD5, 0x3F, 0xC0, 0x06,
  0xB4, 0x9A, 0xD0, 0x80, 0x71, 0x17, 0x5D, 0x0A, 0x44, 0x62, 0xC2, 0x47, 0xAB, 0x7F, 0x26, 0x3E, 0xC5, 0x42, 0xA3, 0x27,
  0x4D, 0x32, 0x1C, 0x23, 0xD8, 0xE0, 0x1F, 0x32, 0xA2, 0x24, 0x11, 0xD0, 0x05, 0x5E, 0xFD, 0x1C, 0x60, 0xE1, 0xB6, 0xAB,
  0xE0, 0x5F, 0xD7, 0x3C, 0x0F, 0xC6, 0xD6, 0xE8, 0x5E, 0x09, 0x09, 0x08, 0x02, 0x6A, 0x91, 0xC2, 0x00, 0x06, 0x2E, 0xE3,
  0x08, 0x42, 0x6B, 0x6C, 0x85, 0xA6, 0x30, 0xA2, 0x84, 0x68, 0x00, 0x41, 0x4A, 0x2C, 0x33, 0x51, 0xEA, 0xFE, 0x06, 0xCD,
  0xCF, 0x4D, 0x24, 0x2A, 0xA2, 0x34, 0x22, 0x92, 0x1D, 0xD6, 0x6A, 0xA4, 0xBB, 0x6D, 0x5D, 0xAF, 0xDD, 0x17, 0x94, 0x93,
  0xD5, 0x4F, 0xAB, 0xBF, 0xC3, 0x62, 0x43, 0x68, 0x9B, 0xE2, 0xD4, 0xB6, 0x03, 0xCA, 0x41, 0x33, 0x4B, 0x25, 0x65, 0x09,
  0x9A, 0xE0, 0x28, 0x23, 0x9A, 0x46, 0xA8, 0xC4, 0x25, 0x2D, 0x09, 0x2F, 0xA8, 0x8A, 0xF0, 0x04, 0x27, 0xAB, 0x9F, 0x40,
  0x83, 0x59, 0xB9, 0x2B, 0x41, 0x04, 0x24, 0x8A, 0x07, 0x20, 0x72, 0x95, 0xBF, 0x81, 0x0C, 0x27, 0x13, 0xC2, 0x85, 0xB6,
  0x5B, 0x48, 0x55, 0x8D, 0xE5, 0xCD, 0x18, 0x0B, 0x2F, 0x5F, 0xC3, 0xA6, 0x1D, 0x72, 0x10, 0x36, 0x93, 0xEC, 0x79, 0x28,
  0x48, 0xFC, 0x01, 0x8F, 0x32, 0xB0, 0xD7, 0x87, 0x24, 0xAF, 0x7E, 0x84, 0xD7, 0x63, 0xAE, 0x41, 0xA6, 0xF2, 0xA5, 0x90,
  0xBB, 0xDD, 0xA5, 0x1F, 0xF1, 0x67, 0xF5, 0xB7, 0x48, 0xD2, 0x18, 0x23, 0xC9, 0x38, 0x27, 0xC8, 0xFE, 0xB4, 0xBC, 0x76,
  0x4E, 0x1F, 0x70, 0xED, 0x5B, 0xEA, 0x43, 0xB7, 0x95, 0xF3, 0x14, 0xCC, 0x24, 0x59, 0x3C, 0x24, 0xBC, 0x84, 0x00, 0x20,
  0x3B, 0xA5, 0x1A, 0xFC, 0xE2, 0x59, 0xA7, 0x04, 0xE7, 0xBE, 0x84, 0xA0, 0xAC, 0x53, 0x78, 0x2D, 0x15, 0x0E, 0x29, 0xE2,
  0x21, 0x83, 0x03, 0x5D, 0x2A, 0xF1, 0xEA, 0xC7, 0x19, 0x8D, 0x99, 0x3A, 0x11, 0x29, 0x87, 0xA2, 0x87, 0xF4, 0x21, 0x5B,
  0x1C, 0x34, 0xAE, 0xD8, 0x07, 0xCD, 0x9F, 0x15, 0xE6, 0x2F, 0x94, 0xFD, 0xDC, 0x6C, 0xE3, 0xF4, 0xB0, 0xD9, 0xE7, 0x38,
  0x19, 0x2B, 0x23, 0x21, 0x4D, 0x54, 0x91, 0xEE, 0x6F, 0x80, 0x41, 0xCA, 0xBD, 0x2D, 0x00, 0x6D, 0x2F, 0x88, 0x88, 0xE8,
  0x08, 0x07, 0x47, 0xCB, 0x02, 0xC6, 0x8B, 0x57, 0x3B, 0x0B, 0x3E, 0x92, 0xF1, 0xEB, 0x51, 0xC2, 0x38, 0xE6, 0xE8, 0xCB,
  0x5E, 0x1F, 0x25, 0xAC, 0x00, 0x84, 0x7D, 0x87, 0x12, 0x36, 0x4A, 0x45, 0xE9, 0x80, 0xAD, 0x3E, 0x8D, 0x8F, 0x79, 0x02,
  0xDE, 0xBE, 0xF9, 0x68, 0x25, 0x9A, 0xA3, 0xB7, 0x77, 0x3C, 0x35, 0xC2, 0x41, 0x11, 0x9A, 0x41, 0x01, 0xBA, 0x8F, 0xA7,
  0x5A, 0x3E, 0x18, 0x37, 0x0D, 0x09, 0xB1, 0xC4, 0x8F, 0xA8, 0x7F, 0xD7, 0x29, 0xF1, 0x2C, 0x81, 0xFA, 0x52, 0x87, 0xD3,
  0x2E, 0xC3, 0x29, 0x98, 0x11, 0x3F, 0x93, 0x10, 0x8B, 0x20, 0x3F, 0x64, 0x06, 0x58, 0xDA, 0x55, 0x23, 0xB2, 0x16, 0x2D,
  0x10, 0x0E, 0x94, 0x96, 0x1E, 0xF4, 0xA8, 0xCF, 0xCB, 0x28, 0x52, 0x4A, 0x7A, 0x97, 0x6F, 0x2F, 0xDF, 0xA2, 0xC1, 0x37,
  0x2F, 0x6E, 0x1E, 0x64, 0x3F, 0x86, 0x10, 0xC8, 0xA0, 0xB5, 0x01, 0x0A, 0x33, 0xCE, 0x01, 0xDC, 0x51, 0x80, 0xB9, 0x4C,
  0xCF, 0x74, 0x02, 0x4E, 0x64, 0xB4, 0xFA, 0x87, 0xD8, 0xC3, 0x8C, 0x1C, 0x64, 0xAF, 0x94, 0x84, 0xC6, 0x4E, 0xEC, 0x43,
  0x8E, 0x30, 0x52, 0x38, 0x20, 0x61, 0xB6, 0x84, 0x76, 0x8D, 0xD6, 0xD2, 0x0E, 0xC4, 0x14, 0x23, 0xBC, 0x11, 0xD9, 0x66,
  0xAD, 0xBA, 0xA8, 0xC0, 0x0C, 0x82, 0x0A, 0x50, 0x40, 0x29, 0x13, 0x22, 0x03, 0x21, 0xDD, 0xF7, 0x25, 0x07, 0x2C, 0x51,
  0x3A, 0x99, 0xFB, 0x7B, 0x93, 0x7E, 0x52, 0xA4, 0x4E, 0x55, 0x01, 0xFF, 0xC0, 0xD5, 0x64, 0xB1, 0x91, 0x3A, 0xF0, 0x0F,
  0xCF, 0x6D, 0x4B, 0x93, 0x2D, 0xC8, 0xDF, 0xDB, 0x7A, 0xE1, 0xCC, 0x5E, 0xEE, 0x8D, 0x82, 0x88, 0x84, 0x07, 0xE4, 0x15,
  0x55, 0x8B, 0x37, 0x50, 0x01, 0x66, 0x47, 0xE4, 0xD5, 0x70, 0x51, 0x2A, 0x5C, 0x37, 0x1F, 0xBB, 0xCA, 0x14, 0x55, 0x2B,
  0x6B, 0xA2, 0x67, 0x2C, 0x1E, 0x1E, 0xD3, 0xA4, 0x47, 0x8E, 0xB5, 0xAA, 0xFC, 0x6B, 0x57, 0x97, 0x26, 0x6B, 0x65, 0xA7,
  0x68, 0x0B, 0x9D, 0x8E, 0x28, 0xCD, 0x0F, 0xEE, 0x43, 0x89, 0x98, 0xCF, 0x5D, 0xB5, 0x86, 0xAE, 0xF5, 0x9E, 0xA1, 0x1C,
  0x0D, 0x7E, 0x63, 0xB1, 0xE6, 0x1A, 0x60, 0xF8, 0x52, 0xE2, 0x39, 0x1C, 0xF0, 0x75, 0x99, 0xE5, 0x3A, 0x4E, 0xFE, 0x5B,
  0x5F, 0xDB, 0x6B, 0x5F, 0x3B, 0x56, 0x41, 0xFF, 0x86, 0x51, 0x18, 0xBF, 0xF0, 0x30, 0x22, 0xAF, 0x54, 0x7F, 0x57, 0xC7,
  0xA4, 0x8F, 0x29, 0x57, 0x1D, 0xFD, 0xA1, 0xEB, 0x1F, 0xF3, 0x7E, 0x5B, 0x8F, 0x5A, 0xAA, 0xB0, 0xC0, 0x1D, 0x73, 0x12,
  0x76, 0xAC, 0xAA, 0xA5, 0x3A, 0x44, 0x12, 0xD2, 0x51, 0xC6, 0x71, 0xDE, 0x2E, 0x0F, 0x9E, 0x3A, 0xF3, 0x21, 0x7C, 0x4E,
  0x53, 0xD9, 0x3D, 0x81, 0x69, 0x41, 0x48, 0x34, 0xEC, 0x4C, 0x3A, 0xDD, 0xC9, 0x17, 0x56, 0x5B, 0xA4, 0xF8, 0xC1, 0xDC,
  0x1D, 0x0C, 0x2E, 0x83, 0xEB, 0x6F, 0xE1, 0xF8, 0x2A, 0x72, 0xD7, 0xF2, 0xB6, 0xF9, 0x30, 0x84, 0x97, 0xBA, 0xD7, 0x6F,
  0xB6, 0x96, 0xB4, 0x4E, 0xB0, 0x98, 0x27, 0x7E, 0xF5, 0xF3, 0xAF, 0xFB, 0x9F, 0x57, 0x51, 0x98, 0x25, 0xBE, 0x46, 0x31,
  0x9C, 0x52, 0x3B, 0xE3, 0x91, 0x03, 0xE8, 0x55, 0x5E, 0x18, 0x93, 0xBC, 0x83, 0xA7, 0x98, 0x4A, 0x14, 0x12, 0xE9, 0x8F,
  0xD7, 0xCC, 0x96, 0x61, 0xCA, 0x9C, 0xC9, 0x5D, 0x49, 0x66, 0xD2, 0x2E, 0xB7, 0x60, 0x56, 0x40, 0xB7, 0x9D, 0xC5, 0xB2,
  0x25, 0xF9, 0x7C, 0x71, 0xDB, 0xF9, 0xBA, 0x7F, 0xF3, 0xC6, 0x4D, 0xD5, 0x65, 0xD2, 0x96, 0xE5, 0x25, 0xE4, 0x01, 0x54,
  0x90, 0x32, 0x30, 0x16, 0xEC, 0xCE, 0x0B, 0x71, 0x24, 0x88, 0x13, 0xC3, 0x81, 0xC6, 0x23, 0xE2, 0xC9, 0xE5, 0x92, 0x86,
  0xF6, 0x27, 0x1C, 0x6E, 0x11, 0xF7, 0xF7, 0xB7, 0xF0, 0xEC, 0x74, 0x3A, 0x7A, 0x45, 0x59, 0x8E, 0x39, 0x9B, 0xA2, 0x84,
  0x4C, 0xD1, 0x15, 0xE7, 0x8C, 0xDB, 0xB7, 0x6E, 0x2E, 0x73, 0x7F, 0x6F, 0x5B, 0x5F, 0x0D, 0x06, 0x3D, 0x64, 0x3D, 0xE6,
  0xAE, 0x19, 0x6E, 0xE0, 0x6A, 0x03, 0x58, 0x92, 0xF1, 0x04, 0xDD, 0x2E, 0x0F, 0x87, 0x08, 0x9B, 0x00, 0x18, 0x32, 0xB6,
  0xCB, 0x0B, 0xE5, 0xA2, 0x09, 0x43, 0xE4, 0x61, 0xA8, 0xF0, 0xAD, 0xAA, 0xD1, 0x64, 0x15, 0x31, 0x86, 0x94, 0x44, 0x81,
  0xE8, 0xBC, 0x7B, 0x67, 0xBD, 0x5E, 0xEF, 0xBC, 0xE5, 0x0C, 0x6D, 0xE1, 0xEA, 0x57, 0x05, 0x80, 0xC0, 0x20, 0xE5, 0xF7,
  0xCE, 0x3B, 0x0B, 0x36, 0x56, 0x72, 0x16, 0x11, 0x03, 0x69, 0x6C, 0x63, 0x99, 0x42, 0x08, 0x7F, 0x6F, 0x21, 0xB4, 0x2C,
  0x6E, 0x39, 0xC2, 0xF5, 0xCD, 0x67, 0x44, 0x78, 0x1F, 0x8C, 0x13, 0xB5, 0xA2, 0x4F, 0x54, 0x81, 0xC0, 0x30, 0x84, 0x8D,
  0x16, 0x81, 0x61, 0x07, 0xE6, 0x37, 0x77, 0x5A, 0xFC, 0xEA, 0x61, 0x90, 0x44, 0x75, 0xC3, 0xD7, 0xB3, 0x25, 0x49, 0xFC,
  0x79, 0x7D, 0x6F, 0x45, 0x63, 0x67, 0x45, 0x43, 0xAF, 0xE8, 0xE5, 0x50, 0x6A, 0x98, 0x1A, 0x58, 0x33, 0x4E, 0x72, 0x03,
  0x80, 0x69, 0xF9, 0x74, 0x66, 0xD8, 0x1A, 0xEC, 0x6E, 0x12, 0xC3, 0x6B, 0xAC, 0x21, 0x37, 0x0F, 0x10, 0x90, 0xAC, 0xE0,
  0x35, 0xD1, 0x50, 0xE1, 0x4F, 0xAE, 0x14, 0x50, 0xA9, 0x60, 0x00, 0x96, 0xFC, 0x6B, 0x03, 0x4B, 0x72, 0x49, 0x2C, 0xE4,
  0x40, 0xC1, 0x4D, 0xB1, 0xEA, 0x0C, 0xF9, 0x06, 0x19, 0x0C, 0xDF, 0x60, 0x46, 0xCE, 0x84, 0x7E, 0x6D, 0xA8, 0xD0, 0x96,
  0xBF, 0xC5, 0x70, 0x7F, 0xD0, 0xD4, 0xD5, 0x5F, 0x93, 0x51, 0x16, 0x31, 0x95, 0xC5, 0x94, 0x4E, 0x98, 0xBC, 0x4C, 0x46,
  0x11, 0xD1, 0xEC, 0x2F, 0xDE, 0xE8, 0x61, 0xC5, 0xDE, 0x64, 0xBC, 0x20, 0xA3, 0x32, 0x5C, 0xBC, 0x5E, 0xD2, 0x19, 0x09,
  0xEC, 0x46, 0xF9, 0xB1, 0xF5, 0xEF, 0x5F, 0xE0, 0xB8, 0xA8, 0xF6, 0x25, 0x52, 0x96, 0xAC, 0x7E, 0x9E, 0x90, 0xC8, 0x52,
  0x5A, 0xBF, 0xA3, 0x95, 0x97, 0x14, 0x5D, 0xF7, 0x94, 0xDA, 0x29, 0x0D, 0xE9, 0x75, 0x7A, 0x7F, 0x6F, 0x55, 0x34, 0x0B,
  0x2E, 0x01, 0x86, 0x8E, 0xD3, 0x0D, 0xEA, 0x4B, 0x1C, 0x8D, 0xB1, 0xA2, 0x86, 0x38, 0x8B, 0xE4, 0xEB, 0xA2, 0x40, 0x0B,
  0x19, 0xD8, 0x59, 0xA9, 0xD9, 0x58, 0xBD, 0x6D, 0xB1, 0xDF, 0xB7, 0x02, 0xE6, 0x67, 0xAA, 0x53, 0xB9, 0x23, 0x22, 0xAF,
  0x22, 0xA2, 0x5E, 0x9F, 0xCD, 0xAF, 0x03, 0xDB, 0x2A, 0x2A, 0xD1, 0xA5, 0x49, 0x42, 0xF8, 0x57, 0x83, 0xD7, 0xAF, 0x3A,
  0xA6, 0x1A, 0xDD, 0x18, 0xA7, 0xF6, 0xAC, 0xD3, 0xFD, 0x7E, 0xAB, 0xBD, 0xCA, 0x52, 0xF7, 0x4F, 0x8B, 0xD9, 0xBB, 0xDA,
  0xFB, 0x65, 0x7B, 0xC8, 0xBB, 0x5B, 0x18, 0x30, 0x31, 0xAC, 0x3A, 0xB0, 0x0C, 0x02, 0x18, 0x78, 0xF9, 0xBE, 0xEC, 0xDE,
  0xC2, 0x35, 0xD9, 0xB6, 0xA0, 0xDA, 0xA1, 0xC0, 0x4D, 0x4B, 0x73, 0xD6, 0xDB, 0xAD, 0x5C, 0xD7, 0x6D, 0xCA, 0x29, 0x36,
  0x59, 0x51, 0x74, 0xAF, 0x71, 0x8A, 0xDD, 0x55, 0x14, 0xD3, 0x31, 0x9C, 0xAD, 0xFD, 0x54, 0xF4, 0x1C, 0xF2, 0x9D, 0x87,
  0x8D, 0x7C, 0xFF, 0xDE, 0x0D, 0x19, 0xBF, 0xC2, 0x80, 0x03, 0xE0, 0xFE, 0xD1, 0xC0, 0xF9, 0x07, 0xEB, 0xB1, 0x0A, 0xA4,
  0xEC, 0x6A, 0xFF, 0x5F, 0x51, 0x21, 0x61, 0xD7, 0x46, 0xB0, 0x81, 0xB6, 0x65, 0x6E, 0x93, 0x96, 0xA3, 0xA2, 0xF9, 0xEC,
  0xB3, 0xFD, 0xE3, 0x55, 0x6E, 0x3D, 0x00, 0xCD, 0xEF, 0xC9, 0xEC, 0x01, 0xD0, 0x54, 0x43, 0x3F, 0xCC, 0xF5, 0x59, 0x42,
  0x7D, 0x03, 0xD5, 0x1E, 0x40, 0x0D, 0x29, 0xE0, 0xE7, 0xB1, 0x55, 0xA0, 0xE9, 0xF2, 0x18, 0xD6, 0x6C, 0x0C, 0x6B, 0x1A,
  0x6E, 0x36, 0x61, 0x46, 0xFB, 0x5D, 0xF5, 0x0D, 0xDF, 0x72, 0x16, 0x31, 0x91, 0x63, 0x16, 0x78, 0x56, 0xEF, 0xA6, 0x3F,
  0xB0, 0x1C, 0x75, 0xD5, 0x85, 0x5B, 0x8D, 0xB7, 0xD0, 0x50, 0x01, 0x7E, 0x57, 0x06, 0x30, 0x80, 0x43, 0xAD, 0xE2, 0x34,
  0x85, 0xBE, 0xA2, 0x3B, 0x4A, 0x75, 0x56, 0x99, 0x4E, 0xA7, 0x15, 0x48, 0x68, 0x5C, 0x01, 0x60, 0x86, 0xA3, 0x0D, 0xF1,
  0x07, 0xD6, 0xD2, 0x51, 0x37, 0x64, 0x4F, 0x01, 0xE6, 0x37, 0x6F, 0x5F, 0xF5, 0x09, 0xE6, 0xFE, 0xB8, 0x87, 0x39, 0x8E,
  0x85, 0xBD, 0x08, 0x74, 0x9E, 0x40, 0xD6, 0x83, 0x37, 0x57, 0x0F, 0xAD, 0x8E, 0xBA, 0xF9, 0x78, 0xEA, 0x91, 0x7F, 0xAF,
  0x37, 0xB1, 0x47, 0xB8, 0x0F, 0x96, 0x3D, 0xB8, 0x7E, 0xE4, 0xAC, 0x02, 0x25, 0xBE, 0x83, 0x30, 0xFA, 0xC4, 0xF7, 0xF4,
  0x77, 0xCE, 0x33, 0x5B, 0xEC, 0x99, 0x9F, 0x9C, 0x96, 0xB0, 0x2F, 0x53, 0xE1, 0xE9, 0x41, 0xDA, 0x50, 0x96, 0xE5, 0x65,
  0xB9, 0x95, 0xF7, 0x8C, 0x02, 0x88, 0x1F, 0xB6, 0x4B, 0x1F, 0x0E, 0x7B, 0x9D, 0xE0, 0xF2, 0xD1, 0xBC, 0xEA, 0x79, 0x40,
  0xFD, 0x1D, 0x79, 0x2C, 0xAD, 0x7A, 0xC5, 0xFF, 0x2B, 0xA9, 0xDA, 0x98, 0xA7, 0xDC, 0xF9, 0x63, 0xC2, 0x5B, 0xCF, 0xE6,
  0x47, 0x82, 0x53, 0xFC, 0xDD, 0xD8, 0xFE, 0x08, 0xBB, 0xDB, 0x13, 0xCF, 0x11, 0xE3, 0xF9, 0xA2, 0xFF, 0xD1, 0xBE, 0x20,
  0xF2, 0x5A, 0x4D, 0xE9, 0x50, 0x14, 0x76, 0x2E, 0xE1, 0xC0, 0x85, 0xB5, 0xA6, 0xDA, 0x77, 0x2E, 0xDF, 0x52, 0x7F, 0xFC,
  0x98, 0xE9, 0x07, 0x26, 0x24, 0xFD, 0x9F, 0x4F, 0x55, 0xFF, 0x79, 0xFD, 0x1F, 0x2A, 0xA1, 0x41, 0x70, 0xCD, 0x16, 0x00,
  0x00,
};
MaintenancePolicy::Window serviceWindow;
bool serviceWasOpen=false;
char serviceSerialLine[112]={};size_t serviceSerialLength=0;bool discardServiceLine=false;
bool serviceSafelyStopped() {
  return (controllerState==STATE_IDLE||controllerState==STATE_BOOT_HOLD)&&
    !localTestMode&&!pendingCmdValid&&!autoRestart.pending&&!autoRestart.required&&
    !outputIsOnLogicLevel(PIN_OUT_RIGHT)&&!outputIsOnLogicLevel(PIN_OUT_LEFT)&&
    !outputIsOnLogicLevel(PIN_OUT_PUMP)&&!outputIsOnLogicLevel(PIN_OUT_LAST_TOWER)&&
    !outputIsOnLogicLevel(PIN_OUT_CANNON)&&!outputIsOnLogicLevel(PIN_OUT_AUX6);
}
void openServiceWindow() {
  if(serviceWindow.open(millis(),serviceSafelyStopped())){benchCloudPaused=true;serviceWasOpen=true;Serial.println("[SERVICE] Manutencao liberada por 15 minutos; protecoes permanecem ativas");}
  else Serial.println("[SERVICE] Acesso negado: pare as saidas e encerre comandos pendentes");
}
bool requireServiceAccess(bool allowLocal) {
  if(!serviceWindow.active(millis())){server.send(403,"application/json","{\"ok\":false,\"message\":\"Manutencao bloqueada. Segure CONFIG por 6s ou envie BENCH ENABLE pelo USB.\"}");return false;}
  if(!serviceSafelyStopped()&&!(allowLocal&&localTestMode)){server.send(409,"application/json","{\"ok\":false,\"message\":\"Pare a central e encerre o modo local antes de configurar.\"}");return false;}
  return true;
}
void testBenchInternet() {
  if(!serviceWindow.active(millis()))return;
  JsonDocument result;result["test"]="internet";result["transport"]=uplinkName();
  int code=0;uint32_t started=millis();
  if(networkReady()&&tlsClockValid()){
    serviceControlBeforeNetwork();HTTPClient& http=portalHttpClient;
    if(httpBeginWithAutoClient(http,String(DEFAULT_SERVER_URL)+"/")){
      http.setConnectTimeout(3000);http.setTimeout(5000);http.setFollowRedirects(HTTPC_DISABLE_FOLLOW_REDIRECTS);
      code=http.GET();http.end();
    }
  }
  result["httpCode"]=code;result["elapsedMs"]=uint32_t(millis()-started);result["ok"]=code>0;
  result["credentialsSent"]=false;Serial.print("[BENCH] ");serializeJson(result,Serial);Serial.println();
}
void configureBenchWifi(const char* text) {
  if(!serviceWindow.active(millis())||!serviceSafelyStopped()){Serial.println("[SERVICE] Configuracao WiFi bloqueada");return;}
  String value(text);int split=value.indexOf('|');
  if(split<1||split>32||value.length()-split-1>63){Serial.println("[SERVICE] Parametros WiFi invalidos");return;}
  String oldSsid=cfg.wifiSsid,oldPass=cfg.wifiPass;
  cfg.wifiSsid=value.substring(0,split);cfg.wifiPass=value.substring(split+1);
  if(!saveConfig()){cfg.wifiSsid=oldSsid;cfg.wifiPass=oldPass;Serial.println("[SERVICE] Falha ao persistir WiFi");return;}
  benchCloudPaused=true;beginSavedWifiConnection();Serial.println("[SERVICE] WiFi configurado; segredo nao exibido");
}
void enableBenchOnline() {
  if(!serviceWindow.active(millis())||!serviceSafelyStopped()||cfg.pivotId!="P014"||cfg.deviceToken.isEmpty()){
    Serial.println("[SERVICE] Online bloqueado: configure P014 e sua credencial no painel");return;
  }
  stopLocalTest();benchCloudPaused=false;Serial.println("[SERVICE] Testes online liberados exclusivamente para P014");
}
void printBenchStatus() {
  JsonDocument doc;
  doc["firmware"]=FIRMWARE_VERSION;doc["mac"]=WiFi.macAddress();doc["pivotId"]=cfg.pivotId;
  doc["deviceTokenConfigured"]=!cfg.deviceToken.isEmpty();
  doc["controllerState"]=stateToString(controllerState);doc["bootGuardRemainingMs"]=bootGuardActive()?BOOT_GUARD_MS-uint32_t(millis()-bootStartedMs):0;
  doc["benchBuild"]=PROSERVICE_BENCH_BUILD!=0;doc["cloudPausedForBench"]=benchCloudPaused;
  doc["safetyOk"]=safetyOk();doc["pressureOk"]=pressureOk();doc["serviceWindowOpen"]=serviceWindow.active(millis());
  doc["localTestMode"]=localTestMode;doc["pendingCommandActive"]=pendingCmdValid;doc["requireStop"]=requireStopCommandAfterFault;
  doc["network"]=uplinkName();doc["ethernetIp"]=ethernetReady()?ETH.localIP().toString():String("");
  doc["wifiIp"]=WiFi.status()==WL_CONNECTED?WiFi.localIP().toString():String("");doc["gpsStartReady"]=gpsReadyForStart();
  doc["radioChannel"]=e220ReadChannel;doc["commandResult"]=commandExecution.result;doc["commandReason"]=commandExecution.reason;
  doc["rightOn"]=outputIsOnLogicLevel(PIN_OUT_RIGHT);doc["leftOn"]=outputIsOnLogicLevel(PIN_OUT_LEFT);
  doc["pumpOn"]=outputIsOnLogicLevel(PIN_OUT_PUMP);doc["towerOn"]=outputIsOnLogicLevel(PIN_OUT_LAST_TOWER);doc["cannonOn"]=outputIsOnLogicLevel(PIN_OUT_CANNON);doc["readyOn"]=outputIsOnLogicLevel(PIN_OUT_AUX6);
  Serial.print("[BENCH] ");serializeJson(doc,Serial);Serial.println();
}
void startBenchSequence(bool needGps,bool reverse) {
  if(!serviceWindow.active(millis())||bootGuardActive()||!safetyTaskHandle||fastOutputInhibit||requireStopCommandAfterFault||!safetyOk()){
    allOutputsOff();Serial.println("[SERVICE] Partida de bancada bloqueada");return;
  }
  cancelAutoRestart("teste USB iniciado");cancelPendingCommand("teste USB iniciado");
  PivotCommand cmd=currentCmd;cmd.commandId="";cmd.direction=reverse?"esquerda":"direita";cmd.mode="sem_agua";cmd.requiresGps=needGps;cmd.noGpsMode=!needGps;
  cmd.canhaoFinal=false;cmd.lastTowerPercent=75;cmd.lastTowerCycleMs=DEFAULT_LAST_TOWER_CYCLE_MS;cmd.peakEnabled=false;cmd.restartAfterPower=false;cmd.updatedAt=millis();
  localTestMode=true;localDirectMode=false;clearLocalRelaySelection();localLastActivityMs=millis();lastCommandAcceptedMs=0;
  startCommandSequence(cmd);
}
void processServiceAccess() {
  for(unsigned n=0;n<64&&Serial.available();++n){char c=char(Serial.read());
    if(c=='\r')continue;
    if(c=='\n'){
      serviceSerialLine[serviceSerialLength]=0;
      if(!discardServiceLine){
        if(strcmp(serviceSerialLine,"BENCH ENABLE")==0)openServiceWindow();
        else if(strcmp(serviceSerialLine,"BENCH STATUS")==0)printBenchStatus();
        else if(strcmp(serviceSerialLine,"BENCH NETTEST")==0)testBenchInternet();
        else if(strncmp(serviceSerialLine,"BENCH WIFI ",11)==0)configureBenchWifi(serviceSerialLine+11);
        else if(strcmp(serviceSerialLine,"BENCH ONLINE")==0)enableBenchOnline();
        else if(strcmp(serviceSerialLine,"BENCH OFFLINE")==0)benchCloudPaused=true;
        else if(strcmp(serviceSerialLine,"BENCH STOP")==0)stopBenchOutputs();
        else if(strcmp(serviceSerialLine,"BENCH CLOSE")==0){stopLocalTest();serviceWindow.close();serviceWasOpen=false;}
        else if(strcmp(serviceSerialLine,"BENCH START GPS")==0)startBenchSequence(true,false);
        else if(strcmp(serviceSerialLine,"BENCH START DRY RIGHT NOGPS")==0)startBenchSequence(false,false);
        else if(strcmp(serviceSerialLine,"BENCH START DRY LEFT NOGPS")==0)startBenchSequence(false,true);
        else if(strncmp(serviceSerialLine,"BENCH RELAY ",12)==0){String message;executeBenchRelay(String(serviceSerialLine+12),message);Serial.println("[SERVICE] "+message);}
      }
      memset(serviceSerialLine,0,sizeof(serviceSerialLine));serviceSerialLength=0;discardServiceLine=false;
    }else if(!discardServiceLine){if(serviceSerialLength<sizeof(serviceSerialLine)-1)serviceSerialLine[serviceSerialLength++]=c;else{discardServiceLine=true;serviceSerialLength=0;}}
  }
  if(serviceWasOpen&&!serviceWindow.active(millis())){
    serviceWasOpen=false;serviceWindow.close();if(localTestMode)stopLocalTest();
    Serial.println("[SERVICE] Janela de manutencao encerrada");
  }
}
void handleLocalPanel() {
  // Conteudo HTML identico, compactado em gzip na flash; nenhuma logica de controle alterada.
  server.sendHeader("Content-Encoding", "gzip");
  server.sendHeader("Cache-Control", "no-store");
  server.send_P(200, "text/html; charset=utf-8", reinterpret_cast<const char*>(LOCAL_PANEL_GZIP), sizeof(LOCAL_PANEL_GZIP));
}

void sendLocalJson(bool ok, const String& message) {
  DynamicJsonDocument doc(512);
  doc["ok"] = ok;
  doc["message"] = message;
  String out;
  serializeJson(doc, out);
  server.send(ok ? 200 : 400, "application/json", out);
}

void handleLocalCommand() {
  if(!requireServiceAccess(true))return;
  if (bootGuardActive()) {
    allOutputsOff();
    sendLocalJson(false, "Aguarde terminar a proteção de 60 segundos após ligar a ESP32");
    return;
  }
  if (!safetyOk()) {
    allOutputsOff();
    sendLocalJson(false, "Entradas de emergência não estão liberadas");
    return;
  }
  if (!safetyTaskHandle || fastOutputInhibit || requireStopCommandAfterFault) {
    allOutputsOff();
    sendLocalJson(false,"Falha anterior: pressione PARAR antes de nova partida");
    return;
  }

  cancelAutoRestart("modo de teste local iniciado");
  // A página já confirmou as duas entradas; prepara a estabilidade para o comando local.
  safetyStableSinceMs = millis() - SAFETY_STABLE_BEFORE_RUN_MS;
  PivotCommand cmd = currentCmd;
  cmd.commandId = "";
  cmd.direction = normalizeDirectionStr(server.arg("direction"));
  cmd.mode = normalizeModeStr(server.arg("mode"));
  cmd.canhaoFinal = server.arg("cannon") == "1";
  cmd.pressureWaitSec = constrain(server.arg("pressureWaitSec").toInt(), 5, 1800);
  cmd.lastTowerPercent = clampFloat(server.arg("lastTowerPercent").toFloat(), 0.0f, 100.0f);
  cmd.lastTowerCycleMs = DEFAULT_LAST_TOWER_CYCLE_MS;
  cmd.requiresGps = server.arg("noGps") != "1";
  cmd.noGpsMode = server.arg("noGps") == "1";
  cmd.peakEnabled = false;
  cmd.restartAfterPower = false;
  cmd.updatedAt = millis();

  localTestMode = true;
  localDirectMode = false;
  clearLocalRelaySelection();
  localLastActivityMs = millis();
  lastCommandAcceptedMs = 0;
  cancelPendingCommand("modo de teste local iniciado");
  startCommandSequence(cmd);
  sendLocalJson(true, "Comando local iniciado");
}

bool executeBenchRelay(const String& requested,String& message) {
  if(!serviceWindow.active(millis())){message="Manutencao bloqueada";return false;}
  if(bootGuardActive()||!safetyTaskHandle||fastOutputInhibit||requireStopCommandAfterFault||!safetyOk()){
    clearLocalRelaySelection();allOutputsOff();message="Saidas bloqueadas por protecao";return false;
  }
  String relay=requested;relay.trim();relay.toLowerCase();
  if(relay!="off"&&relay!="right"&&relay!="left"&&relay!="pump"&&relay!="tower"&&relay!="cannon"){
    clearLocalRelaySelection();allOutputsOff();message="Rele invalido; saidas desligadas";return false;
  }
  cancelAutoRestart("controle direto local iniciado");localTestMode=true;localDirectMode=true;localLastActivityMs=millis();
  currentCmd.direction="parado";currentCmd.mode="sem_agua";controllerState=STATE_IDLE;cancelPendingCommand("controle direto local iniciado");
  if(relay=="off")clearLocalRelaySelection();
  else if(relay=="right"){localRelayRight=!localRelayRight;if(localRelayRight)localRelayLeft=false;}
  else if(relay=="left"){localRelayLeft=!localRelayLeft;if(localRelayLeft)localRelayRight=false;}
  else if(relay=="pump")localRelayPump=!localRelayPump;
  else if(relay=="tower")localRelayLastTower=!localRelayLastTower;
  else if(relay=="cannon")localRelayCannon=!localRelayCannon;
  faultMessage="";alertMessage="Teste direto local ativo";applyLocalDirectOutputs();message="Saidas atualizadas";return true;
}
void handleLocalRelay() {
  if(!requireServiceAccess(true))return;
  String message;bool ok=executeBenchRelay(server.arg("relay"),message);sendLocalJson(ok,message);
}

void stopBenchOutputs() {
  cancelAutoRestart("parada local solicitada");
  localTestMode = true;
  localDirectMode = false;
  localLastActivityMs = millis();
  clearLocalRelaySelection();
  PivotCommand stopCmd = currentCmd;
  stopCmd.commandId = "";
  stopCmd.direction = "parado";
  stopCmd.mode = "sem_agua";
  stopCmd.updatedAt = millis();
  requireStopCommandAfterFault = false;
  lastCommandAcceptedMs = 0;
  cancelPendingCommand("parada local solicitada");
  startCommandSequence(stopCmd);
  allOutputsOff();
}
void handleLocalStop() {stopBenchOutputs();sendLocalJson(true,"Todas as saidas desligadas");}

void handleLocalDisable() {
  stopLocalTest();
  sendLocalJson(true, "Modo local desativado; servidor remoto liberado");
}

void handleSave() {
  if(!requireServiceAccess(false))return;
  String requestedPivot=server.arg("pivot");requestedPivot.trim();
  String requestedToken=server.arg("token");requestedToken.trim();
  if(requestedPivot!=cfg.pivotId && requestedToken.isEmpty()){
    server.send(400,"text/plain; charset=utf-8","Ao trocar o ID, informe a credencial correspondente. Nenhuma configuracao foi alterada.");return;
  }
  String selectedSsid = server.arg("ssid");
  String manualSsid   = server.arg("ssid_manual");

  manualSsid.trim();
  selectedSsid.trim();

  const String previousSsid=cfg.wifiSsid;
  cfg.wifiSsid = manualSsid.length() ? manualSsid : selectedSsid;
  cfg.wifiSsid.trim();
  if(server.arg("pass").length()||cfg.wifiSsid!=previousSsid)cfg.wifiPass=server.arg("pass");

  if (cfg.wifiSsid.isEmpty()) {
    server.send(400, "text/html", "<html><body style='font-family:Arial;padding:20px'><h2>SSID vazio</h2><p>Selecione uma rede ou digite o nome manualmente.</p><a href='/'>Voltar</a></body></html>");
    return;
  }
  cfg.backendEnabled = server.arg("backend") != "0";
  cfg.serverUrl = normalizeServerUrl(server.arg("server"));
  cfg.commandRoute = normalizeBackendRoute(server.arg("cmdroute"), DEFAULT_COMMAND_ROUTE);
  cfg.heartbeatRoute = normalizeBackendRoute(server.arg("hbroute"), DEFAULT_HEARTBEAT_ROUTE);
  cfg.pivotId = server.arg("pivot");
  String replacementToken=server.arg("token");replacementToken.trim();
  if(replacementToken.length())cfg.deviceToken=replacementToken;
  cfg.hardwareName = server.arg("hwname");
  cfg.gpsNodeId = server.arg("gpsnode");

  cfg.pivotId.trim();
  cfg.deviceToken.trim();
  cfg.hardwareName.trim();
  cfg.gpsNodeId.trim();
  cfg.gpsNodeId.trim();

  if (cfg.pivotId.isEmpty()) cfg.pivotId = "1";
  if (cfg.hardwareName.isEmpty()) cfg.hardwareName = "ESP32-PIVO-" + cfg.pivotId;

  cfg.localProfileEnabled = server.arg("localprofile") != "0";
  cfg.localCenterLat = server.arg("centerlat").toDouble();
  cfg.localCenterLon = server.arg("centerlon").toDouble();
  cfg.localRadiusMeters = server.arg("radius").toFloat();
  cfg.localAreaHectares = server.arg("area").toFloat();
  cfg.localGpsCorrectionDeg = server.arg("gpscorr").toFloat();
  cfg.localMinDepthMm = max(0.0f, server.arg("mindepth").toFloat());
  cfg.localSafetyFactorPercent = clampFloat(server.arg("safetyfactor").toFloat(), 0.0f, 30.0f);
  cfg.localPressureWaitSec = constrain(server.arg("presswait").toInt(), 10, 30000);
  cfg.localLastTowerCycleMs = constrain((uint32_t)server.arg("towercycle").toInt(), (uint32_t)1000, (uint32_t)60000);

  if (cfg.localAreaHectares > 0.0f && cfg.localRadiusMeters <= 0.0f) {
    cfg.localRadiusMeters = sqrt((cfg.localAreaHectares * 10000.0f) / PI);
  } else if (cfg.localRadiusMeters > 0.0f && cfg.localAreaHectares <= 0.0f) {
    cfg.localAreaHectares = (PI * cfg.localRadiusMeters * cfg.localRadiusMeters) / 10000.0f;
  }

  if (!saveConfig()) {
    server.send(500, "text/html",
                "<html><body style='font-family:Arial;padding:20px;'><h2>Falha ao salvar.</h2><p>A NVS nao confirmou os dados. A placa nao sera reiniciada.</p><a href='/'>Voltar</a></body></html>");
    return;
  }

  server.send(200, "text/html",
              "<html><body style='font-family:Arial;padding:20px;'><h2>Salvo com sucesso.</h2><p>Reiniciando a ESP32...</p></body></html>");
  delay(1200);
  ESP.restart();
}

void handleSetCenter() {
  if(!requireServiceAccess(false))return;
  String message;
  bool ok = saveCurrentGpsAsLocalCenter(message);

  String html = "<html><body style='font-family:Arial;padding:20px'>";
  html += ok ? "<h2>Centro gravado.</h2>" : "<h2>Não foi possível gravar.</h2>";
  html += "<p>" + htmlEscape(message) + "</p>";
  if (ok) {
    html += "<p>Latitude: <b>" + String(cfg.localCenterLat, 8) + "</b><br>";
    html += "Longitude: <b>" + String(cfg.localCenterLon, 8) + "</b></p>";
  }
  html += "<p><a href='/'>Voltar</a></p></body></html>";
  server.send(ok ? 200 : 409, "text/html", html);
}

void handleResetWifi() {
  if(!requireServiceAccess(false))return;
  if (!clearWifiConfigOnly()) {
    server.send(500, "text/html",
                "<html><body style='font-family:Arial;padding:20px;'><h2>Falha ao apagar Wi-Fi.</h2><p>A NVS nao confirmou a operacao. A placa nao sera reiniciada.</p><a href='/'>Voltar</a></body></html>");
    return;
  }
  server.send(200, "text/html",
              "<html><body style='font-family:Arial;padding:20px;'><h2>Wi-Fi apagado.</h2><p>Reiniciando a ESP32...</p></body></html>");
  delay(1200);
  ESP.restart();
}

void handleReboot() {
  if(!requireServiceAccess(false))return;
  server.send(200, "text/html",
              "<html><body style='font-family:Arial;padding:20px;'><h2>Reiniciando a ESP32...</h2></body></html>");
  delay(800);
  ESP.restart();
}

void handleJsonStatus() {
  server.sendHeader("Cache-Control","no-store");
  DynamicJsonDocument doc(32768);
  doc["benchBuild"]=PROSERVICE_BENCH_BUILD!=0;
  doc["cloudPausedForBench"]=benchCloudPaused;
  doc["serviceWindowOpen"]=serviceWindow.active(millis());
  doc["serviceWindowRemainingMs"]=serviceWindow.remaining(millis());
  doc["uplinkSwitchCount"]=uplinkHealth.switches;
  doc["ethernetTransportFailures"]=uplinkHealth.failures[UplinkPolicy::CABLE];
  doc["wifiTransportFailures"]=uplinkHealth.failures[UplinkPolicy::WIFI];
  doc["portalResponseMaxBytes"]=PortalBodyPolicy::Budget::MAX_BYTES;
  doc["pivotId"] = cfg.pivotId;
  doc["hardwareName"] = cfg.hardwareName;
  addNetworkStatus(doc);
  addCommandOutcomeStatus(doc);
  addPortalTimingStatus(doc);
  addCommandFetchDiagnostic(doc);
  doc["wifiConnected"] = (WiFi.status() == WL_CONNECTED);
  doc["wifiIp"] = WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : "";
  doc["wifiRssi"] = WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : 0;
  doc["wifiSsid"] = WiFi.status() == WL_CONNECTED ? WiFi.SSID() : cfg.wifiSsid;
  doc["wifiAttemptActive"] = wifiAttemptActive;
  doc["wifiDiagnostic"] = wifiDiagnosticMessage;
  doc["wifiLastFailure"] = wifiLastFailure;
  doc["wifiChannel"] = WiFi.status() == WL_CONNECTED ? WiFi.channel() : wifiSavedNetworkChannel;
  doc["wifiSecurity"] = wifiSavedNetworkSecurity;
  doc["portalCommunicationConfirmedOffline"] = portalRequestFailureActive;
  doc["portalConsecutiveFailures"] = portalConsecutiveFailures;
  doc["portalConsecutiveSuccesses"] = portalConsecutiveSuccesses;
  doc["portalLastSuccessAgeMs"] = lastServerOkMs ? (uint32_t)(millis() - lastServerOkMs) : 0;
  doc["apName"] = buildApName();
  doc["apIp"] = WiFi.softAPIP().toString();
  doc["backendEnabled"] = cfg.backendEnabled;
  doc["serverUrl"] = cfg.serverUrl;
  doc["commandRoute"] = cfg.commandRoute;
  doc["heartbeatRoute"] = cfg.heartbeatRoute;
  doc["commandUrl"] = buildCommandUrl();
  doc["heartbeatUrl"] = buildHeartbeatUrl();
  doc["emergency1"] = readEmergency1();
  doc["emergency2"] = readEmergency2();
  doc["safetyOk"] = safetyOk();
  doc["pressureOk"] = pressureOk();
  doc["controllerState"] = stateToString(controllerState);
  doc["faultMessage"] = faultMessage;
  doc["alertMessage"] = alertMessage;
  doc["requestedDepthMm"] = currentCmd.requestedDepthMm;
  doc["minDepthMm"] = currentCmd.minDepthMm;
  doc["safetyFactorPercent"] = currentCmd.safetyFactorPercent;
  doc["lastTowerPercent"] = currentCmd.lastTowerPercent;
  doc["lastTowerCycleMs"] = currentCmd.lastTowerCycleMs;
  doc["engineeringReady"] = currentCmd.engineeringReady;
  doc["depthClamped"] = currentCmd.depthClamped;

  // ACK legado do comando. commandApplied/commandConfirmed so ficam true
  // quando commandId, hash, updatedAt e GPIOs correspondem ao solicitado.
  const bool commandAccepted = portalCommandWasAccepted();
  const bool commandApplied = commandedOutputsAreApplied();
  doc["commandReceived"] = portalCommandReceived;
  doc["commandAccepted"] = commandAccepted;
  doc["commandApplied"] = commandApplied;
  doc["commandConfirmed"] = commandApplied;
  doc["commandSentSuccessfully"] = commandApplied; // alias para integração do portal
  doc["commandAckUpdatedAt"] = lastPortalCommandUpdatedAt;
  doc["commandAckDirection"] = lastPortalCommandDirection;
  doc["commandAckMode"] = lastPortalCommandMode;
  doc["commandAckStatus"] = commandAcknowledgementStatus();
  doc["commandReceivedAgeMs"] = portalReceiptRecovered ? UINT32_MAX : portalCommandReceived ?
      (uint32_t)(millis() - lastPortalCommandReceivedMs) : 0;
  doc["commandId"] = commandExecution.commandId;
  doc["commandPayloadHash"] = String(commandExecution.payloadHash, HEX);
  JsonObject execution = doc.createNestedObject("commandExecution");
  execution["commandId"] = commandExecution.commandId;
  execution["requestedDirection"] = commandExecution.requestedDirection;
  execution["requestedMode"] = commandExecution.requestedMode;
  execution["result"] = commandExecution.result;
  execution["reason"] = commandExecution.reason;
  execution["receivedUptimeMs"] = commandExecution.receivedAtMs;
  execution["changedUptimeMs"] = commandExecution.changedAtMs;
  execution["completedUptimeMs"] = commandExecution.completedAtMs;
  execution["statusBootCounter"] = commandExecution.statusBootCounter;
  execution["payloadHash"] = String(commandExecution.payloadHash, HEX);
  execution["outputsConfirmed"] = commandExecution.outputsConfirmed;
  JsonObject commandProtocol = doc.createNestedObject("commandProtocol");
  commandProtocol["conflictActive"] = commandIdConflictActive;
  commandProtocol["conflictCount"] = commandIdConflictCount;
  commandProtocol["message"] = commandIdConflictMessage;
  doc["eventsPending"] = eventOutboxCount;
  doc["eventsCapacity"] = EVENT_OUTBOX_SLOTS;
  doc["eventsNormalLimit"] = EVENT_NORMAL_LIMIT;
  doc["eventsCriticalReserve"] = EVENT_CRITICAL_RESERVE;
  doc["eventsOverflow"] = eventOutboxOverflowCount;
  doc["eventsDroppedNormal"] = eventOutboxDroppedNormalCount;
  doc["eventsCriticalOverflow"] = eventOutboxCriticalOverflowCount;
  doc["persistenceHealthy"] = persistenceHealthy;
  doc["persistenceFaultLatched"] = persistenceFaultLatched;
  doc["persistenceFailureCount"] = persistenceFailureCount;
  doc["persistenceLastError"] = persistenceLastError;
  doc["persistenceLastFailureMs"] = persistenceLastFailureMs;
  doc["persistenceLastSuccessMs"] = persistenceLastSuccessMs;
  JsonObject journalStatus = doc.createNestedObject("operationJournal");
  journalStatus["active"] = operationJournal.active;
  journalStatus["commandId"] = operationJournal.commandId;
  journalStatus["stage"] = operationJournal.stage;
  journalStatus["restartAfterPower"] = operationJournal.restartAfterPower;
  journalStatus["writePending"] = operationJournalDirty;
  journalStatus["generation"] = operationJournalGeneration;
  journalStatus["repairCount"] = operationJournalRepairCount;
  journalStatus["startInterlock"] = operationJournalStartInterlock;
  journalStatus["interlockReason"] = operationJournalInterlockReason;
  JsonObject autoRestartStatus = doc.createNestedObject("autoRestartAfterPower");
  autoRestartStatus["supported"] = true;
  autoRestartStatus["enabledForCommand"] = currentCmd.restartAfterPower;
  autoRestartStatus["recoveryRequired"] = autoRestart.required;
  autoRestartStatus["pending"] = autoRestart.pending;
  autoRestartStatus["portalAuthorized"] = autoRestart.portalAuthorized;
  autoRestartStatus["minimumDelaySec"] = AUTO_RESTART_MIN_OFF_MS / 1000UL;
  autoRestartStatus["remainingMs"] = autoRestartRemainingMs();
  autoRestartStatus["status"] = autoRestart.status;
  autoRestartStatus["commandId"] = autoRestart.commandId;
  doc["pendingCommandActive"] = pendingCmdValid;
  doc["pendingCommandId"] = pendingCmdValid ? pendingCmd.commandId : "";
  doc["pendingCommandDirection"] = pendingCmdValid ? pendingCmd.direction : "";
  doc["pendingCommandStillAuthorized"] = pendingCommandStillAuthorized();
  JsonObject deviceState = doc.createNestedObject("deviceState");
  deviceState["controllerState"] = stateToString(controllerState);
  deviceState["safetyOk"] = safetyOk();
  deviceState["pressureOk"] = pressureOk();
  deviceState["rightOn"] = outputIsOnLogicLevel(PIN_OUT_RIGHT);
  deviceState["leftOn"] = outputIsOnLogicLevel(PIN_OUT_LEFT);
  deviceState["pumpOn"] = outputIsOnLogicLevel(PIN_OUT_PUMP);
  deviceState["lastTowerOn"] = outputIsOnLogicLevel(PIN_OUT_LAST_TOWER);
  deviceState["cannonOn"] = outputIsOnLogicLevel(PIN_OUT_CANNON);
  deviceState["boardReadyRelayOn"] = outputIsOnLogicLevel(PIN_OUT_AUX6);
  doc["pumpOn"] = outputIsOnLogicLevel(PIN_OUT_PUMP);
  doc["rightOn"] = outputIsOnLogicLevel(PIN_OUT_RIGHT);
  doc["leftOn"] = outputIsOnLogicLevel(PIN_OUT_LEFT);
  doc["lastTowerOn"] = outputIsOnLogicLevel(PIN_OUT_LAST_TOWER);
  doc["cannonOn"] = outputIsOnLogicLevel(PIN_OUT_CANNON);
  doc["aux6On"] = outputIsOnLogicLevel(PIN_OUT_AUX6);
  doc["boardReadyRelayOn"] = outputIsOnLogicLevel(PIN_OUT_AUX6);
  doc["lastResetReason"] = lastResetReason;
  doc["bootCounter"] = bootCounter;
  doc["lastStopReason"] = lastStopReason;
  doc["stopEventCounter"] = stopEventCounter;
  doc["lastFaultReason"] = lastFaultReason;
  doc["faultEventCounter"] = faultEventCounter;
  doc["diagnosticEventCounter"] = diagEventCounter;
  doc["systemEventLog"] = buildSystemEventLog();
  doc["invalidPortalCommandCount"] = invalidPortalCommandCount;
  doc["lastInvalidPortalDirection"] = lastInvalidPortalDirection;
  doc["gpsNodeId"] = cfg.gpsNodeId;
  doc["loraLastSeenId"] = loraLastSeenId;
  doc["loraLastPacketAccepted"] = loraLastPacketAccepted;
  doc["radioChannel"] = e220ReadChannel;
  doc["radioChannelKnown"] = e220ReadChannel >= 0;
  doc["radioReadAgeMs"] = e220ReadChannel >= 0 ? millis()-e220ReadMs : 0;
  doc["radioPreference"] = e220Channel;
  doc["radioMessage"] = e220ConfigStatus;
  doc["acceptedSinceRadioCheck"] = loraAcceptedSinceCheck;
  doc["acceptedAgeMs"] = loraAcceptedSinceCheck ? millis()-loraAcceptedMs : 0;
  doc["radioLinkRecent"] = loraAcceptedSinceCheck && millis()-loraAcceptedMs < 10000UL;
  doc["movementProtectionRequired"] = commandRequiresGps(currentCmd);
  doc["movementProtectionActive"] = controllerState == STATE_RUNNING && movementGuard.active;
  doc["movementRemainingMs"] = movementGuard.remaining(millis());
  doc["movementProgressM"] = movementGuard.progress;
  doc["movementConfirmations"] = movementGuard.confirmations;
  doc["movementRejectedSamples"] = movementGuard.rejectedSamples;
  doc["movementRecoveries"] = movementGuard.recoveries;
  doc["movementSampleAgeMs"] = lastGoodGpsAvailable ? (uint32_t)(millis()-lastGoodGpsMs) : 0;
  doc["movementHdop"] = gpsHdop;
  doc["movementSatellites"] = gpsSatellites;
  doc["movementQualityRejects"] = movementQualityRejects;
  doc["movementAgeRejects"] = movementAgeRejects;
  doc["movementGeometryRejects"] = movementGeometryRejects;
  doc["movementObservationReady"] = movementObservationReady(millis());
  doc["movementUsableSampleAgeMs"] = movementLastUsableSampleMs ?
      (uint32_t)(millis() - movementLastUsableSampleMs) : 0;
  doc["loraQueueAgeMs"] = loraQueueAgeMs;
  doc["loraDuplicateSamples"] = loraDuplicateSamples;
  doc["loraOldSamples"] = loraOldSamples;
  doc["movementReason"] = !commandRequiresGps(currentCmd) ? "Desativada pelo modo/comando sem GPS" :
      (controllerState != STATE_RUNNING ? "Aguardando operacao" : movementGuard.reason);

  doc["peerReportedChannel"] = loraReportedChannel;
  doc["loraRejectReason"] = loraRejectReason;
  doc["loraIgnoredPackets"] = loraIgnoredPackets;
  doc["loraAuxHigh"] = false;
  doc["loraRxByteCount"] = loraRxByteCount;
  doc["loraRxLineCount"] = loraRxLineCount;
  doc["loraRxTaskRunning"] = loraRxTaskRunning;
  doc["loraRxQueueWaiting"] = loraRxQueue ? uxQueueMessagesWaiting(loraRxQueue) : 0;
  doc["loraRxQueueDrops"] = (uint32_t)loraRxQueueDrops;
  doc["loraJsonErrorCount"] = loraJsonErrorCount;
  doc["loraLastRawPacket"] = loraLastRawPacket;
  doc["loraLastTxPacket"] = loraLastTxPacket;
  doc["loraLastJsonError"] = loraLastJsonError;
  doc["loraLastAnyRxAgeMs"] = loraLastAnyRxMs ? (millis() - loraLastAnyRxMs) : 0;
  bool loraGpsFreshForJson = loraGpsValid && loraLastRxMs > 0 && (millis() - loraLastRxMs < 10000UL);
  bool retainedGpsForJson = hasGoodGpsWithinTimeout();
  doc["gpsValid"] = retainedGpsForJson;
  doc["gpsRetainedForMap"] = retainedGpsForJson;
  doc["gpsOperationalFresh"] = hasOperationalGpsSample(millis());
  doc["gpsOperationalTimeoutMs"] = GPS_OPERATION_TIMEOUT_MS;
  doc["loraAuthorizedRecent"] = loraAcceptedMs > 0 &&
      (uint32_t)(millis() - loraAcceptedMs) <= 10000UL;
  doc["gpsFresh"] = loraGpsFreshForJson;
  doc["gpsPositionAvailable"] = retainedGpsForJson;
  doc["gpsSource"] = loraGpsFreshForJson ? "lora" :
                     (retainedGpsForJson ? "lora_retida" : "none");
  doc["gpsLat"] = retainedGpsForJson ? lastGoodGpsLat : 0.0;
  doc["gpsLon"] = retainedGpsForJson ? lastGoodGpsLon : 0.0;
  doc["gpsPositionAgeMs"] = retainedGpsForJson ? (uint32_t)(millis() - lastGoodGpsMs) : 0;
  doc["gpsAltMeters"] = 0.0;
  doc["gpsSatellites"] = gpsSatellites;
  doc["gpsHdop"] = gpsHdop;
  doc["gpsCourseDeg"] = gpsCourseDeg;
  doc["gpsSpeedKmph"] = gpsSpeedKmph;
  doc["centerLat"] = currentCmd.centerLat;
  doc["centerLon"] = currentCmd.centerLon;
  doc["radiusMeters"] = currentCmd.radiusMeters;
  doc["areaHectares"] = currentCmd.areaHectares;
  doc["geometryReady"] = currentCmd.geometryReady;
  doc["localProfileEnabled"] = cfg.localProfileEnabled;
  doc["localProfileGeometryValid"] = localPivotGeometryValid();
  doc["gpsCorrectionDeg"] = currentCmd.gpsCorrectionDeg;
  doc["pivotAngleValid"] = pivotAngleValid;
  doc["pivotAngleDeg"] = pivotAngleValid ? pivotAngleDeg : 0.0;
  doc["distanceFromCenterM"] = pivotAngleValid ? distanceFromCenterM : 0.0;
  doc["localTestMode"] = localTestMode;
  doc["localDirectMode"] = localDirectMode;
  doc["localTimeoutRemainingSec"] = localTestMode ? ((LOCAL_TEST_TIMEOUT_MS > (millis() - localLastActivityMs)) ? (LOCAL_TEST_TIMEOUT_MS - (millis() - localLastActivityMs)) / 1000UL : 0) : 0;

  addStructuredEventsToHeartbeat(doc);

  if (doc.overflowed()) {
    Serial.println("[JSON] /status overflow");
    server.send(507, "application/json",
                "{\"error\":\"status_json_overflow\",\"firmware\":\"REV47.16\"}");
    return;
  }
  String out;
  serializeJson(doc, out);
  server.send(200, "application/json", out);
}

void setupWebServer() {
  if (webServerStarted) return;
  server.on("/", HTTP_GET, handleRoot);
  server.on("/root.css", HTTP_GET, handleRootCSS);
  server.on("/root.js", HTTP_GET, handleRootJS);
  server.on("/save", HTTP_POST, handleSave);
  server.on("/lorachannel", HTTP_POST, handleSaveLoraChannel);
  server.on("/loraread", HTTP_POST, handleReadLoraChannel);
  server.on("/setcenter", HTTP_POST, handleSetCenter);
  server.on("/resetwifi", HTTP_POST, handleResetWifi);
  server.on("/reboot", HTTP_POST, handleReboot);
  server.on("/status", HTTP_GET, handleJsonStatus);
  server.on("/local", HTTP_GET, handleLocalPanel);
  server.on("/local/command", HTTP_POST, handleLocalCommand);
  server.on("/local/relay", HTTP_POST, handleLocalRelay);
  server.on("/local/stop", HTTP_POST, handleLocalStop);
  server.on("/maintenance/cancel-saved-command", HTTP_POST, handleReportSavedCommandCancellation);
  server.on("/local/disable", HTTP_POST, handleLocalDisable);
  server.begin();
  webServerStarted = true;
}

void startConfigPortal() {
  ensureConfigPortalRunning(true);
}

// ======================================================
// HTTP / SERVIDOR
// ======================================================

class PortalBodySink : public Stream {
 public:
  String body;PortalBodyPolicy::Budget budget;
  PortalBodySink():budget(millis()) {}
  size_t write(uint8_t b) override {return write(&b,1);}
  size_t write(const uint8_t* data,size_t count) override {
    if(!budget.append(count,millis()))return 0;
    if(!body.concat(reinterpret_cast<const char*>(data),count)){budget.failed=true;return 0;}
    return count;
  }
  int available() override{return 0;}int read() override{return -1;}int peek() override{return -1;}void flush() override{}
};
bool readPortalBody(HTTPClient& http,String& out) {
  if(http.getSize()>int(PortalBodyPolicy::Budget::MAX_BYTES))return false;
  PortalBodySink sink;
  int received=http.writeToStream(&sink);
  if(received<0||sink.budget.failed||size_t(received)!=sink.body.length()||
      (http.getSize()>=0&&received!=http.getSize()))return false;
  out=std::move(sink.body);return true;
}

bool httpBeginWithAutoClient(HTTPClient& http, const String& url) {
  if (!url.startsWith("https://") || !tlsClockValid()) return false;
  http.end(); // limpa cabecalhos; preserva apenas conexao reutilizavel confirmada
  const uint32_t now=millis();
  int authorityEnd=url.indexOf('/',8),queryStart=url.indexOf('?',8);
  if(authorityEnd<0 || (queryStart>=0 && queryStart<authorityEnd))authorityEnd=queryStart;
  String origin=authorityEnd<0?url:url.substring(0,authorityEnd);origin.toLowerCase();
  const int network=int(activeUplink);
  const uint32_t localIp=activeUplink==UplinkPolicy::CABLE?uint32_t(ETH.localIP()):uint32_t(WiFi.localIP());
  if(portalKeepAliveFallbackActive && uint32_t(now-portalKeepAliveFallbackStartedMs)>=60000UL)portalKeepAliveFallbackActive=false;
  const bool keepAlive=!portalKeepAliveFallbackActive;
  const bool reuse=keepAlive && portalSecureClient.available()==0 &&
      PortalLatencyPolicy::reuseAllowed(portalSecureClient.connected(),origin==portalTlsOrigin,
        network==portalTlsNetwork && localIp==portalTlsIp,uint32_t(now-portalTlsUsedMs),uint32_t(now-portalTlsOpenedMs));
  portalRequestReused=reuse;
  if(!reuse){
    resetPortalConnection();++portalTlsReconnectAttempts;
    portalSecureClient.setCACert(PROSERVICE_ROOT_CA);portalSecureClient.setHandshakeTimeout(5);
    portalTlsOrigin=origin;portalTlsNetwork=network;portalTlsIp=localIp;portalTlsOpenedMs=now;
  }else ++portalTlsReuseAttempts;
  http.setReuse(keepAlive);
  return http.begin(portalSecureClient,url);
}

bool fetchCommandFromServer(PivotCommand& cmd) {
  if(benchCloudPaused)return false;
  if (httpInFlight) return false;
  if (!networkReady()) return false;
  if (!cfg.backendEnabled) return false;
  if (buildCommandUrl().isEmpty()) return false;
  if (cfg.pivotId.isEmpty()) return false;
  if (bootGuardActive()) return false;

  serviceControlBeforeNetwork();
  HTTPClient& http=portalHttpClient;
  PortalRequestScope requestTiming(true);
  portalGetOutcome="iniciando"; portalGetRootKeys=""; portalGetItemCount=0;
  String url = buildCommandUrl();
  url += (url.indexOf('?') >= 0 ? "&" : "?");
  url += "pivot_id=" + urlEncode(cfg.pivotId) + "&token=" + urlEncode(cfg.deviceToken);

  httpInFlight = true;
  if (!httpBeginWithAutoClient(http, url)) {
    httpInFlight = false;
    portalGetOutcome="falha_inicio_https";
    Serial.println("[HTTP] Falha begin GET comando");
    notePortalCommunicationFailure("Falha ao iniciar GET de comando");
    return false;
  }

  http.setConnectTimeout(HTTP_CONNECT_TIMEOUT_MS);
  http.setTimeout(HTTP_TIMEOUT_MS);
  http.setReuse(!portalKeepAliveFallbackActive);
  http.addHeader("Connection", portalKeepAliveFallbackActive?"close":"keep-alive");
  http.addHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  http.addHeader("Pragma", "no-cache");
  if (cfg.deviceToken.length()) http.addHeader("X-Device-Token", cfg.deviceToken);
  if (cfg.hardwareName.length()) http.addHeader("X-Hardware-Name", cfg.hardwareName);

  int code = http.GET();
  requestTiming.responseCode=code; requestTiming.transportResponseCode=code;
  if (code != 200) {
    portalGetOutcome="falha_http_"+String(code);
    resetPortalConnection(); // nao reutilizar corpo parcial/erro de transporte
    Serial.print("[HTTP] GET comando falhou. Code=");
    Serial.println(code);
    http.end();
    httpInFlight = false;
    notePortalCommunicationFailure("GET comando HTTP=" + String(code));
    return false;
  }

  String payload;
  if(!readPortalBody(http,payload)) {
    resetPortalConnection(); http.end(); httpInFlight=false;
    requestTiming.responseCode=HTTPC_ERROR_CONNECTION_LOST;
    portalGetOutcome="resposta_incompleta";
    notePortalCommunicationFailure("Resposta de comando incompleta"); return false;
  }
  http.end();
  httpInFlight = false;

  DynamicJsonDocument doc(6144);
  DeserializationError err = deserializeJson(doc, payload);
  if (err) {
    portalGetOutcome="json_invalido"; ++portalGetJsonErrorCount;
    Serial.print("[JSON] Erro comando: ");
    Serial.println(err.c_str());
    notePortalCommunicationFailure("JSON de comando invalido: " + String(err.c_str()));
    return false;
  }

  for (JsonPair kv : doc.as<JsonObject>()) {
    if(portalGetRootKeys.length()>160) break;
    if(portalGetRootKeys.length()) portalGetRootKeys+=",";
    portalGetRootKeys+=String(kv.key().c_str()).substring(0,32);
  }
  if (!PortalResponsePolicy::commandEnvelope(doc.as<JsonVariantConst>())) {
    portalGetOutcome="envelope_invalido";
    notePortalCommunicationFailure("Envelope de comando invalido ou success=false");
    return false;
  }

  // Só uma resposta HTTP e JSON semanticamente válida renova o último contato.
  // Isso impede que uma página de erro com HTTP 200 esconda uma queda real.
  lastServerOkMs = millis();
  portalOfflineLatch = false;
  notePortalCommunicationRecovered();

  JsonArray items = doc["items"].as<JsonArray>();
  portalGetItemCount=items.isNull()?0:items.size();
  if (items.isNull() || items.size() == 0) {
    portalGetOutcome=items.isNull()?"items_ausente_ou_tipo_invalido":"items_vazio";
    ++portalGetEmptyCount; return false;
  }
  portalGetOutcome="item_em_validacao";

  JsonObject item = items[0];

  String directionRaw = "";
  if (item["direction"].is<const char*>()) {
    directionRaw = String(item["direction"].as<const char*>());
  } else if (item["direction"].is<int>() || item["direction"].is<long>() ||
             item["direction"].is<unsigned int>() || item["direction"].is<unsigned long>()) {
    directionRaw = String(item["direction"].as<long>());
  }

  String validatedDirection;
  if (!tryNormalizePortalDirection(directionRaw, validatedDirection)) {
    String rejectedCommandId = String((const char*)(item["commandId"] | ""));
    if (rejectedCommandId.isEmpty()) {
      rejectedCommandId = String((const char*)(item["command_id"] | ""));
    }
    if (rejectedCommandId.length() > COMMAND_ID_MAX_LENGTH) {
      rejectedCommandId = rejectedCommandId.substring(0, COMMAND_ID_MAX_LENGTH);
    }
    commandExecution.payloadHash = 0;
    setCommandExecution(rejectedCommandId, directionRaw, "", "rejected",
                        "Direcao invalida ou ausente", true);
    invalidPortalCommandCount++;
    String rejectedDirection = directionRaw.length() ? directionRaw : "(ausente)";
    bool logRejected = rejectedDirection != lastInvalidPortalDirection ||
        lastInvalidPortalEventMs == 0 ||
        millis() - lastInvalidPortalEventMs >= EVENT_FAILURE_REPEAT_MS;
    lastInvalidPortalDirection = rejectedDirection;
    Serial.println("[CMD] Direcao invalida");
    if (logRejected) {
      lastInvalidPortalEventMs = millis();
      appendDiagnosticHistory("COMANDO", "REJEITADO",
          "Direcao invalida/ausente: " + lastInvalidPortalDirection);
    }
    return false;
  }

  // Preserve a ultima geometria valida. Respostas de comando incompletas ou
  // momentaneamente zeradas nao podem apagar centro/raio usados no mapa e na
  // parada angular.
  const bool previousGeometryValid = pivotGeometryValuesValid(
      currentCmd.centerLat, currentCmd.centerLon, currentCmd.radiusMeters);
  const double previousCenterLat = currentCmd.centerLat;
  const double previousCenterLon = currentCmd.centerLon;
  const float previousRadiusMeters = currentCmd.radiusMeters;
  const float previousAreaHectares = currentCmd.areaHectares;
  const float previousGpsCorrectionDeg = currentCmd.gpsCorrectionDeg;

  cmd = PivotCommand();
  const char* modeStr       = item["mode"] | "sem_agua";
  const char* depthStr      = item["depth"] | "8";
  const char* stopDegreeStr = item["stopDegree"] | "360";

  cmd.mode = normalizeModeStr(String(modeStr));
  cmd.direction = validatedDirection;
  cmd.depth = String(depthStr);
  cmd.stopDegree = String(stopDegreeStr);
  if (item["stopDegree"].is<double>()) {
    const double target = item["stopDegree"].as<double>();
    cmd.stopDegree = isfinite(target) && target >= 0.0 && target <= 360.0 ? String(target, 6) : String("360");
  }
  cmd.canhaoFinal = item["canhaoFinal"] | false;
  cmd.pressureWaitSec = item["pressureWaitSec"] | 240;
  cmd.updatedAt = item["updatedAt"] | 0UL;
  if (item["commandId"].is<const char*>()) {
    cmd.commandId = String(item["commandId"].as<const char*>());
  } else if (item["command_id"].is<const char*>()) {
    cmd.commandId = String(item["command_id"].as<const char*>());
  }
  cmd.commandId.trim();
  if (cmd.commandId.length() > COMMAND_ID_MAX_LENGTH) {
    commandExecution.payloadHash = 0;
    setCommandExecution(cmd.commandId.substring(0, COMMAND_ID_MAX_LENGTH),
                        cmd.direction, cmd.mode, "rejected",
                        "commandId excede 96 caracteres", true);
    appendDiagnosticHistory("COMANDO", "REJEITADO",
        "commandId excede o limite de 96 caracteres");
    return false;
  }
  // Compatibilidade temporária com o portal antigo. O protocolo profissional
  // deve sempre fornecer commandId exclusivo e imutável.
  if (cmd.commandId.isEmpty()) {
    cmd.commandId = "legacy-" + cfg.pivotId + "-" + String(cmd.updatedAt) +
                    "-" + cmd.direction + "-" + cmd.mode;
  }

  cmd.requestedDepthMm = jsonToFloat(item["requestedDepthMm"], jsonToFloat(item["depth"], 8.0f));
  cmd.minDepthMm = jsonToFloat(item["minDepthMm"], 0.0f);
  cmd.safetyFactorPercent = jsonToFloat(item["safetyFactorPercent"], 3.0f);
  cmd.lastTowerPercent = jsonToFloat(item["lastTowerPercent"], -1.0f);
  cmd.lastTowerCycleMs = jsonToUInt32(item["lastTowerCycleMs"], DEFAULT_LAST_TOWER_CYCLE_MS);
  cmd.engineeringReady = item["engineeringReady"] | false;
  cmd.depthClamped = item["depthClamped"] | false;
  cmd.centerLat = jsonToDouble(item["centerLat"], 0.0);
  cmd.centerLon = jsonToDouble(item["centerLon"], 0.0);
  cmd.radiusMeters = jsonToFloat(item["radiusMeters"], 0.0f);
  cmd.areaHectares = jsonToFloat(item["areaHectares"], 0.0f);
  cmd.gpsCorrectionDeg = jsonToFloat(item["gpsCorrectionDeg"], jsonToFloat(item["gpsAngleOffsetDeg"], 0.0f));
  if (cmd.radiusMeters < 10.0f && cmd.areaHectares > 0.0f) {
    cmd.radiusMeters = sqrt((cmd.areaHectares * 10000.0f) / PI);
  }
  if (!pivotGeometryValuesValid(cmd.centerLat, cmd.centerLon, cmd.radiusMeters) &&
      previousGeometryValid) {
    cmd.centerLat = previousCenterLat;
    cmd.centerLon = previousCenterLon;
    cmd.radiusMeters = previousRadiusMeters;
    cmd.areaHectares = previousAreaHectares;
    cmd.gpsCorrectionDeg = previousGpsCorrectionDeg;
  }
  cmd.geometryReady = pivotGeometryValuesValid(
      cmd.centerLat, cmd.centerLon, cmd.radiusMeters);

  if (item["autoRestartAfterPowerEnabled"].is<bool>()) {
    cmd.restartAfterPower = item["autoRestartAfterPowerEnabled"].as<bool>();
  } else {
    cmd.restartAfterPower = item["restartAfterPower"] | false;
  }
  cmd.peakEnabled = item["peakEnabled"] | false;
  cmd.peakStart = String((const char*)(item["peakStart"] | ""));
  cmd.peakEnd = String((const char*)(item["peakEnd"] | ""));
  cmd.serverTimeEpoch = jsonToUInt32(item["serverTimeEpoch"], jsonToUInt32(item["serverEpoch"], jsonToUInt32(item["serverTime"], 0UL)));
  cmd.noGpsMode = item["noGpsMode"] | false;
  if (item["scheduleType"].is<const char*>()) {
    String sched = String(item["scheduleType"].as<const char*>());
    sched.toLowerCase();
    if (sched == "horario") cmd.noGpsMode = true;
    if (sched == "programada") cmd.requiresGps = true;
  }
  if (item["gpsRequired"].is<bool>()) cmd.requiresGps = item["gpsRequired"].as<bool>();
  if (item["requiresGps"].is<bool>()) cmd.requiresGps = item["requiresGps"].as<bool>();

  cmd.peakDaysMask = 0;
  if (item["peakDays"].is<JsonArray>()) {
    for (JsonVariant v : item["peakDays"].as<JsonArray>()) {
      if (v.is<const char*>()) cmd.peakDaysMask |= peakDayLabelToBit(String(v.as<const char*>()));
      else if (v.is<int>()) cmd.peakDaysMask |= peakDayLabelToBit(String(v.as<int>()));
    }
  } else if (item["peakDays"].is<const char*>()) {
    String raw = String(item["peakDays"].as<const char*>());
    int start = 0;
    while (start < raw.length()) {
      int comma = raw.indexOf(',', start);
      if (comma < 0) comma = raw.length();
      cmd.peakDaysMask |= peakDayLabelToBit(raw.substring(start, comma));
      start = comma + 1;
    }
  }

  cmd.mode = normalizeModeStr(cmd.mode);
  cmd.direction = normalizeDirectionStr(cmd.direction);
  if (cmd.direction == "parado") cmd.mode = "sem_agua";
  if (cmd.pressureWaitSec < 10) cmd.pressureWaitSec = 10;
  if (cmd.pressureWaitSec > 30000) cmd.pressureWaitSec = 30000;
  cmd.safetyFactorPercent = clampFloat(cmd.safetyFactorPercent, 0.0f, 30.0f);
  if (cmd.lastTowerPercent >= 0.0f) cmd.lastTowerPercent = clampFloat(cmd.lastTowerPercent, 0.0f, 100.0f);
  if (cmd.lastTowerCycleMs < 1000) cmd.lastTowerCycleMs = 1000;
  if (cmd.lastTowerCycleMs > 60000) cmd.lastTowerCycleMs = 60000;
  if (cmd.minDepthMm > 0.0f && cmd.requestedDepthMm < cmd.minDepthMm) {
    cmd.requestedDepthMm = cmd.minDepthMm;
    cmd.depthClamped = true;
  }

  // Dados físicos específicos desta instalação têm prioridade sobre o backend.
  applyLocalPivotProfile(cmd);
  // applyLocalPivotProfile pode ter sido ignorado por NVS incompleta.
  cmd.geometryReady = pivotGeometryValuesValid(
      cmd.centerLat, cmd.centerLon, cmd.radiusMeters);

  portalGetOutcome="comando_decodificado"; ++portalGetParsedCommandCount;
  return true;
}

bool postHeartbeatToServer() {
  if(benchCloudPaused)return false;
  if (httpInFlight) return false;
  if (!networkReady()) return false;
  if (!cfg.backendEnabled) return false;
  if (buildHeartbeatUrl().isEmpty()) return false;
  if (cfg.pivotId.isEmpty()) return false;
  if(!PortalLatencyPolicy::postAllowed(millis(),portalLastPostFinishedMs,portalPostAttempts>0,
      portalLastPostCode>=200 && portalLastPostCode<300)) return false;

  serviceControlBeforeNetwork();
  const uint32_t postingCommandExecutionRevision = commandExecutionRevision;
  const String postingAckStatus=commandAcknowledgementStatus();
  PortalRequestScope requestTiming(false);
  HTTPClient& http=portalHttpClient;
  String url = buildHeartbeatUrl();

  httpInFlight = true;
  if (!httpBeginWithAutoClient(http, url)) {
    httpInFlight = false;
    Serial.println("[HTTP] Falha begin POST heartbeat");
    notePortalCommunicationFailure("Falha ao iniciar POST heartbeat");
    return false;
  }

  http.setConnectTimeout(HTTP_CONNECT_TIMEOUT_MS);
  http.setTimeout(HTTP_TIMEOUT_MS + 2000);
  http.setReuse(!portalKeepAliveFallbackActive);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("Connection", portalKeepAliveFallbackActive?"close":"keep-alive");
  if (cfg.deviceToken.length()) http.addHeader("X-Device-Token", cfg.deviceToken);
  if (cfg.hardwareName.length()) http.addHeader("X-Hardware-Name", cfg.hardwareName);

  bool pumpOn = outputIsOnLogicLevel(PIN_OUT_PUMP);
  bool rightOn = outputIsOnLogicLevel(PIN_OUT_RIGHT);
  bool leftOn = outputIsOnLogicLevel(PIN_OUT_LEFT);
  bool moveOn = rightOn || leftOn;
  bool cannonOn = outputIsOnLogicLevel(PIN_OUT_CANNON);
  bool lastTowerOn = outputIsOnLogicLevel(PIN_OUT_LAST_TOWER);
  bool currentSafety = safetyOk();
  bool currentPressure = pressureOk();
  String currentState = stateToString(controllerState);

  bool gpsWithinTimeout = hasGoodGpsWithinTimeout();
  bool gpsFresh = gpsWithinTimeout && (lastGoodGpsMs > 0) && (millis() - lastGoodGpsMs <= 60000UL);

  DynamicJsonDocument doc(32768);
  doc["pivot_id"] = cfg.pivotId;
  doc["token"] = cfg.deviceToken;
  doc["online"] = true;
  doc["rssi"] = WiFi.RSSI();
  doc["mode"] = currentCmd.mode;
  doc["direction"] = currentCmd.direction;
  doc["bomba"] = pumpOn;
  doc["canhaoFinal"] = currentCmd.canhaoFinal;
  doc["hardwareName"] = cfg.hardwareName;
  doc["safetyOk"] = currentSafety;
  doc["pressureOk"] = currentPressure;
  doc["controllerState"] = currentState;
  doc["faultMessage"] = faultMessage;
  doc["alertMessage"] = alertMessage;
  doc["pumpOn"] = pumpOn;
  doc["rightOn"] = rightOn;
  doc["leftOn"] = leftOn;
  doc["moveOn"] = moveOn;
  doc["lastTowerOn"] = lastTowerOn;
  doc["cannonOn"] = cannonOn;
  doc["watchdogOn"] = outputIsOnLogicLevel(PIN_OUT_AUX6);
  doc["boardReadyRelayOn"] = outputIsOnLogicLevel(PIN_OUT_AUX6);
  doc["updatedAt"] = (unsigned long)millis();
  doc["requestedDepthMm"] = currentCmd.requestedDepthMm;
  doc["minDepthMm"] = currentCmd.minDepthMm;
  doc["safetyFactorPercent"] = currentCmd.safetyFactorPercent;
  doc["lastTowerPercent"] = currentCmd.lastTowerPercent;
  doc["lastTowerCycleMs"] = currentCmd.lastTowerCycleMs;
  doc["engineeringReady"] = currentCmd.engineeringReady;
  doc["depthClamped"] = currentCmd.depthClamped;
  doc["systemEventLog"] = buildSystemEventLog();

  const bool commandAccepted = portalLegacyReceiptAccepted();
  const bool commandApplied = commandedOutputsAreApplied();
  doc["commandReceived"] = portalCommandReceived;
  doc["commandAccepted"] = commandAccepted;
  doc["commandApplied"] = commandApplied;
  doc["commandConfirmed"] = commandApplied;
  doc["commandSentSuccessfully"] = commandApplied;
  doc["commandAckUpdatedAt"] = lastPortalCommandUpdatedAt;
  doc["commandAckDirection"] = lastPortalCommandDirection;
  doc["commandAckMode"] = lastPortalCommandMode;
  doc["commandAckStatus"] = postingAckStatus;
  doc["commandReceivedAgeMs"] = portalReceiptRecovered ? UINT32_MAX : portalCommandReceived ?
      (uint32_t)(millis() - lastPortalCommandReceivedMs) : 0;
  doc["commandId"] = commandExecution.commandId;
  doc["commandPayloadHash"] = String(commandExecution.payloadHash, HEX);
  JsonObject execution = doc.createNestedObject("commandExecution");
  execution["commandId"] = commandExecution.commandId;
  execution["requestedDirection"] = commandExecution.requestedDirection;
  execution["requestedMode"] = commandExecution.requestedMode;
  execution["result"] = commandExecution.result;
  execution["reason"] = commandExecution.reason;
  execution["receivedUptimeMs"] = commandExecution.receivedAtMs;
  execution["changedUptimeMs"] = commandExecution.changedAtMs;
  execution["completedUptimeMs"] = commandExecution.completedAtMs;
  execution["statusBootCounter"] = commandExecution.statusBootCounter;
  execution["payloadHash"] = String(commandExecution.payloadHash, HEX);
  execution["outputsConfirmed"] = commandExecution.outputsConfirmed;
  JsonObject commandProtocol = doc.createNestedObject("commandProtocol");
  commandProtocol["conflictActive"] = commandIdConflictActive;
  commandProtocol["conflictCount"] = commandIdConflictCount;
  commandProtocol["message"] = commandIdConflictMessage;
  JsonObject journalStatus = doc.createNestedObject("operationJournal");
  journalStatus["active"] = operationJournal.active;
  journalStatus["commandId"] = operationJournal.commandId;
  journalStatus["stage"] = operationJournal.stage;
  journalStatus["restartAfterPower"] = operationJournal.restartAfterPower;
  journalStatus["writePending"] = operationJournalDirty;
  journalStatus["generation"] = operationJournalGeneration;
  journalStatus["repairCount"] = operationJournalRepairCount;
  journalStatus["startInterlock"] = operationJournalStartInterlock;
  journalStatus["interlockReason"] = operationJournalInterlockReason;
  JsonObject autoRestartStatus = doc.createNestedObject("autoRestartAfterPower");
  autoRestartStatus["supported"] = true;
  autoRestartStatus["enabledForCommand"] = currentCmd.restartAfterPower;
  autoRestartStatus["recoveryRequired"] = autoRestart.required;
  autoRestartStatus["pending"] = autoRestart.pending;
  autoRestartStatus["portalAuthorized"] = autoRestart.portalAuthorized;
  autoRestartStatus["minimumDelaySec"] = AUTO_RESTART_MIN_OFF_MS / 1000UL;
  autoRestartStatus["remainingMs"] = autoRestartRemainingMs();
  autoRestartStatus["status"] = autoRestart.status;
  autoRestartStatus["commandId"] = autoRestart.commandId;
  doc["pendingCommandActive"] = pendingCmdValid;
  doc["pendingCommandId"] = pendingCmdValid ? pendingCmd.commandId : "";
  doc["pendingCommandDirection"] = pendingCmdValid ? pendingCmd.direction : "";
  doc["pendingCommandStillAuthorized"] = pendingCommandStillAuthorized();
  JsonObject deviceState = doc.createNestedObject("deviceState");
  deviceState["controllerState"] = currentState;
  deviceState["safetyOk"] = currentSafety;
  deviceState["pressureOk"] = currentPressure;
  deviceState["rightOn"] = rightOn;
  deviceState["leftOn"] = leftOn;
  deviceState["pumpOn"] = pumpOn;
  deviceState["moveOn"] = moveOn;
  deviceState["lastTowerOn"] = lastTowerOn;
  deviceState["cannonOn"] = cannonOn;
  deviceState["boardReadyRelayOn"] = outputIsOnLogicLevel(PIN_OUT_AUX6);

  doc["gpsValid"] = gpsWithinTimeout;
  doc["gpsRetainedForMap"] = gpsWithinTimeout;
  doc["gpsOperationalFresh"] = hasOperationalGpsSample(millis());
  doc["gpsOperationalTimeoutMs"] = GPS_OPERATION_TIMEOUT_MS;
  doc["loraAuthorizedRecent"] = loraAcceptedMs > 0 &&
      (uint32_t)(millis() - loraAcceptedMs) <= 10000UL;
  doc["movementObservationReady"] = movementObservationReady(millis());
  doc["movementProgressM"] = movementGuard.progress;
  doc["movementUsableSampleAgeMs"] = movementLastUsableSampleMs ?
      (uint32_t)(millis() - movementLastUsableSampleMs) : 0;
  doc["gpsFresh"] = gpsFresh;
  doc["gpsPositionAvailable"] = gpsWithinTimeout;
  doc["gpsPositionAgeMs"] = gpsWithinTimeout ? (uint32_t)(millis() - lastGoodGpsMs) : 0;
  doc["loraLinkRecent"] = loraLastAnyRxMs > 0 && (millis() - loraLastAnyRxMs <= 10000UL);
  doc["gpsLat"] = gpsWithinTimeout ? lastGoodGpsLat : 0.0;
  doc["gpsLon"] = gpsWithinTimeout ? lastGoodGpsLon : 0.0;
  doc["gpsAltMeters"] = 0.0;
  doc["gpsSatellites"] = gpsSatellites;
  doc["gpsHdop"] = gpsHdop;
  doc["gpsCourseDeg"] = gpsCourseDeg;
  doc["gpsSpeedKmph"] = gpsSpeedKmph;
  doc["centerLat"] = currentCmd.centerLat;
  doc["centerLon"] = currentCmd.centerLon;
  doc["radiusMeters"] = currentCmd.radiusMeters;
  doc["areaHectares"] = currentCmd.areaHectares;
  doc["geometryReady"] = currentCmd.geometryReady;
  doc["localProfileEnabled"] = cfg.localProfileEnabled;
  doc["localProfileGeometryValid"] = localPivotGeometryValid();
  doc["geometrySource"] = (cfg.localProfileEnabled && localPivotGeometryValid()) ? "local" : "servidor";
  doc["pivotAngleValid"] = gpsWithinTimeout ? lastGoodPivotAngleValid : pivotAngleValid;
  doc["pivotAngleDeg"] = (gpsWithinTimeout && lastGoodPivotAngleValid) ? lastGoodPivotAngleDeg : (pivotAngleValid ? pivotAngleDeg : 0.0);
  doc["distanceFromCenterM"] = (gpsWithinTimeout && lastGoodPivotAngleValid) ? lastGoodDistanceFromCenterM : (pivotAngleValid ? distanceFromCenterM : 0.0);
  // Heartbeat com os campos legados; diagnostico ampliado somente no painel local.
  doc["portalOfflineLatched"] = portalOfflineLatch;
  doc["portalCommunicationConfirmedOffline"] = portalRequestFailureActive;
  doc["portalConsecutiveFailures"] = portalConsecutiveFailures;
  doc["portalConsecutiveSuccesses"] = portalConsecutiveSuccesses;
  doc["portalLastSuccessAgeMs"] = lastServerOkMs ? (uint32_t)(millis() - lastServerOkMs) : 0;
  doc["rightRequested"] = (controllerState == STATE_RUNNING && currentCmd.direction == "direita");
  doc["leftRequested"] = (controllerState == STATE_RUNNING && currentCmd.direction == "esquerda");
  doc["pumpRequested"] = (controllerState == STATE_RUNNING && currentCmd.mode == "com_agua") || controllerState == STATE_WAIT_PRESSURE || controllerState == STATE_WAIT_START;
  doc["cannonRequested"] = currentCmd.canhaoFinal && (controllerState == STATE_RUNNING || controllerState == STATE_WAIT_PRESSURE || controllerState == STATE_WAIT_START);
  doc["rightApplied"] = rightOn;
  doc["leftApplied"] = leftOn;
  doc["pumpApplied"] = pumpOn;
  doc["cannonApplied"] = cannonOn;
  doc["freeHeap"] = ESP.getFreeHeap();
  doc["minFreeHeap"] = ESP.getMinFreeHeap();
  doc["uptimeSec"] = millis() / 1000UL;
  doc["rtcPresent"] = rtcPresent;
  doc["rtcTimeValid"] = rtcTimeValid;
  doc["rtcLostPower"] = rtcLostPowerFlag;
  doc["timeEpoch"] = currentServerEpoch();

  addStructuredEventsToHeartbeat(doc);

  if (doc.overflowed()) {
    Serial.println("[HTTP] Heartbeat JSON cheio");
    http.end();
    httpInFlight = false;
    notePortalCommunicationFailure("Heartbeat JSON excedeu a capacidade interna");
    return false;
  }
  String body;
  serializeJson(doc, body);

  int code = http.POST(body);
  requestTiming.responseCode=code; requestTiming.transportResponseCode=code;
  String response;
  if(!readPortalBody(http,response)) {
    resetPortalConnection(); http.end(); httpInFlight=false;
    requestTiming.responseCode=HTTPC_ERROR_CONNECTION_LOST;
    notePortalCommunicationFailure("Resposta de heartbeat incompleta"); return false;
  }
  http.end();
  httpInFlight = false;

  if (code >= 200 && code < 300) {
    recordPortalPostReply(response);
    JsonDocument validatedReply;
    if(deserializeJson(validatedReply,response)||!PortalResponsePolicy::heartbeatEnvelope(validatedReply.as<JsonVariantConst>())) {
      notePortalCommunicationFailure("Resposta de heartbeat sem confirmacao valida");
      return false;
    }
    processStructuredEventAcks(response);
    lastPostedCommandExecutionRevision = postingCommandExecutionRevision;
    lastPostedControllerState = currentState;
    lastPostedFaultMessage = faultMessage;
    lastPostedAlertMessage = alertMessage;
    lastPostedPumpOn = pumpOn;
    lastPostedRightOn = rightOn;
    lastPostedLeftOn = leftOn;
    lastPostedCannonOn = cannonOn;
    lastPostedSafetyOk = currentSafety;
    lastPostedPressureOk = currentPressure;
    lastPostedCommandApplied = commandApplied;
    lastPostedCommandUpdatedAt = lastPortalCommandUpdatedAt;
    lastPostedCommandAckStatus = postingAckStatus;
    lastPostedDiagEventCounter = diagEventCounter;
    lastPostedLastTowerPercent = currentCmd.lastTowerPercent;
    lastPostedRequestedDepthMm = currentCmd.requestedDepthMm;
    lastPostedMinDepthMm = currentCmd.minDepthMm;
    lastServerOkMs = millis();
    portalOfflineLatch = false;
    notePortalCommunicationRecovered();
    return true;
  }

  resetPortalConnection();
  notePortalCommunicationFailure("POST heartbeat HTTP=" + String(code));

  return false;
}

void forceWifiReconnectIfPortalStale() {
  // Falha de autenticacao/servidor nao e prova de falha do Wi-Fi.
  if (ethernetReady() || httpInFlight || wifiAttemptActive || cfg.wifiSsid.isEmpty()) return;
  if (WiFi.status()!=WL_CONNECTED || WiFi.STA.hasIP()) return;
  uint32_t now=millis();
  if (uint32_t(now-bootStartedMs)<WIFI_ATTEMPT_TIMEOUT_MS ||
      (lastWifiForceReconnectMs && uint32_t(now-lastWifiForceReconnectMs)<WIFI_FORCE_RECONNECT_COOLDOWN_MS)) return;
  lastWifiForceReconnectMs=now;
  beginSavedWifiConnection();
}

// ======================================================
// CONTROLE LOCAL
// ======================================================

void enterSafetyFault() {
  cancelPendingCommand("abertura da cadeia de seguranca");
  bool failedDuringPrestart = controllerState == STATE_WAIT_SAFETY;
  allOutputsOff(); // antes de gravar historico e imprimir
  clearOperationJournal("safety_fault");
  controllerState = STATE_FAULT_SAFETY;
  setCommandExecution(currentCmd.commandId, currentCmd.direction, currentCmd.mode,
                      "failed", "Cadeia de seguranca aberta", true);
  String safetyDetail = failedDuringPrestart
      ? "GPIO23 fechado, mas GPIO36/GPIO39 nao liberaram em 10 segundos"
      : "Cadeia abriu durante partida/operacao apos debounce";
  recordFaultEvent("ABERTURA DA CADEIA DE SEGURANCA", safetyDetail);
  recordStopEvent("ABERTURA DA CADEIA DE SEGURANCA", safetyDetail);
  faultMessage = "Segurança acionada";
  alertMessage = "PROSERVICE INFORMA: Emergência acionada, pivô parado";
  urgentSafetyHeartbeatPending = true;
  requireStopCommandAfterFault = true;
  clearControllerTimers();
  allOutputsOff();
  Serial.println("[CTRL] FALHA SEGURANÇA");
  printControllerSnapshot();
}

void enterLowPressureFault() {
  cancelPendingCommand("falha de pressao");
  allOutputsOff(); // antes de gravar historico e imprimir
  clearOperationJournal("pressure_fault");
  controllerState = STATE_FAULT_LOW_PRESSURE;
  setCommandExecution(currentCmd.commandId, currentCmd.direction, currentCmd.mode,
                      "failed", "Pressao invalida durante operacao", true);
  recordFaultEvent("FALHA DE PRESSAO", "GPIO34 sem pressao valida durante a operacao");
  recordStopEvent("FALHA DE PRESSAO", "Parada automatica pelo pressostato");
  faultMessage = "Baixa pressão";
  alertMessage = "PROSERVICE INFORMA: Falha no pressostato, pivô parado";
  urgentSafetyHeartbeatPending = true;
  requireStopCommandAfterFault = true;
  clearControllerTimers();
  allOutputsOff();
  Serial.println("[CTRL] FALHA BAIXA PRESSÃO");
  printControllerSnapshot();
}

void processUrgentSafetyHeartbeat() {
  if (!urgentSafetyHeartbeatPending) return;
  if (httpInFlight || !networkReady() || !cfg.backendEnabled) return;
  // Executado somente depois que a tarefa rapida ja desligou fisicamente
  // todas as saidas. Nunca faz HTTP dentro da tarefa de seguranca.
  if (postHeartbeatToServer()) {
    urgentSafetyHeartbeatPending = false;
    lastStatusMs = millis();
  }
}

void applyOutputs() {
  // Estado logico independente da saida fisica: permanece verdadeiro durante
  // o corte rapido para que a falta de pressao possa completar o debounce.
  fastPressureProtectionArmed = controllerState == STATE_RUNNING &&
                                currentCmd.mode == "com_agua";
  if (!safetyTaskHandle || fastOutputInhibit || bootGuardActive() || operationJournalStartInterlock ||
      controllerState == STATE_FAULT_PERSISTENCE) {
    allOutputsOff();
    return;
  }

  bool rightOn = false;
  bool leftOn = false;
  bool pumpOn = false;
  bool cannonOn = false;

  if (controllerState == STATE_RUNNING) {
    if (currentCmd.direction == "direita") rightOn = true;
    if (currentCmd.direction == "esquerda") leftOn = true;
    if (currentCmd.mode == "com_agua") pumpOn = true;
    cannonOn = currentCmd.canhaoFinal && FieldControl::cannonReady(
        currentCmd.mode == "com_agua", pressureOk(), pressureDetectedMs, millis(), CANNON_AFTER_PRESSURE_MS);
  } else if (controllerState == STATE_WAIT_PRESSURE || controllerState == STATE_WAIT_START) {
    if (currentCmd.mode == "com_agua") pumpOn = true;
  }

  applyDirectionOutputs(rightOn, leftOn);
  writeOutputIfChanged(PIN_OUT_PUMP, pumpOn);
  uint32_t pulseCycle = constrain(currentCmd.lastTowerCycleMs, 1000UL, 60000UL);
  float pulsePercent = currentCmd.lastTowerPercent;
  if (!isfinite(pulsePercent) || pulsePercent < 0) pulsePercent = 0;
  pulsePercent = clampFloat(pulsePercent, 0.0f, 100.0f);
  uint32_t pulseOnMs = (uint32_t)(pulseCycle * (pulsePercent / 100.0f));
  publishTowerPulse(controllerState == STATE_RUNNING && currentCmd.direction != "parado",
                    false, pulseCycle, pulseOnMs);
  writeOutputIfChanged(PIN_OUT_CANNON, cannonOn);
  if (fastOutputInhibit) fastHardwareOutputsOff();
  processBoardReadyRelay();
}

void startCommandSequence(const PivotCommand& cmd) {
  if (normalizeDirectionStr(cmd.direction) == "parado") {
    cancelPendingCommand("comando PARAR recebido");
    cancelAutoRestart("comando PARAR recebido");
    clearFastSafetyLatch();
  }
  if (bootGuardActive()) {
    allOutputsOff();
    return;
  }

  if (normalizeDirectionStr(cmd.direction) != "parado" &&
      operationJournalStartInterlock) {
    currentCmd = cmd;
    currentCmd.direction = normalizeDirectionStr(currentCmd.direction);
    currentCmd.mode = normalizeModeStr(currentCmd.mode);
    enterPersistenceFault("partida bloqueada: " + operationJournalInterlockReason);
    return;
  }
  if (normalizeDirectionStr(cmd.direction)!="parado" && !safetyTaskHandle) {
    fastOutputInhibit=true; enterSafetyFault(); return;
  }

  if (cmd.direction != "parado" && lastCommandAcceptedMs > 0 && (millis() - lastCommandAcceptedMs) < COMMAND_GUARD_MS && cmd.updatedAt != lastAcceptedUpdatedAt) {
    pendingCmd = cmd;
    pendingCmdValid = true;
    alertMessage = "Comando em espera por proteção de 20s";
    setCommandExecution(cmd.commandId, cmd.direction, cmd.mode, "accepted",
                        "Aguardando intervalo de protecao de 20s", false);
    appendDiagnosticHistory("COMANDO", "AGUARDANDO PROTECAO",
        "updatedAt=" + String(cmd.updatedAt) + " direcao=" + cmd.direction);
    return;
  }

  if (requireStopCommandAfterFault && normalizeDirectionStr(cmd.direction) != "parado") {
    allOutputsOff();
    clearOperationJournal("blocked_after_fault");
    alertMessage = "Falha anterior exige comando PARAR antes de nova operação";
    Serial.println("[CTRL] Falha: requer PARAR");
    setCommandExecution(cmd.commandId, cmd.direction, cmd.mode, "rejected",
                        "Falha anterior exige comando PARAR", true);
    appendDiagnosticHistory("COMANDO", "BLOQUEADO POR FALHA",
        "updatedAt=" + String(cmd.updatedAt) + " direcao=" + cmd.direction);
    return;
  }

  bool continuingOperation =
      (controllerState == STATE_RUNNING || controllerState == STATE_WAIT_SAFETY ||
       controllerState == STATE_WAIT_PRESSURE || controllerState == STATE_WAIT_START) &&
      normalizeDirectionStr(cmd.direction) == currentCmd.direction &&
      normalizeModeStr(cmd.mode) == currentCmd.mode &&
      normalizeDirectionStr(cmd.direction) != "parado";
  // Uma mudanca de geometria durante a operacao exige nova partida deliberada.
  if (continuingOperation &&
      (doubleChanged(cmd.centerLat,currentCmd.centerLat) ||
       doubleChanged(cmd.centerLon,currentCmd.centerLon) ||
       floatChanged(cmd.radiusMeters,currentCmd.radiusMeters) ||
       floatChanged(cmd.gpsCorrectionDeg,currentCmd.gpsCorrectionDeg) ||
       commandRequiresGps(cmd) != commandRequiresGps(currentCmd))) {
    cancelPendingCommand("configuracao GPS alterada durante operacao");
    allOutputsOff();
    clearOperationJournal("geometry_changed");
    controllerState = STATE_IDLE;
    setCommandExecution(cmd.commandId, cmd.direction, cmd.mode, "rejected",
                        "Geometria ou modo GPS alterado durante operacao", true);
    recordStopEvent("CONFIGURACAO GPS ALTERADA", "Geometria/modo GPS alterado durante operacao; exige PARAR");
    requireStopCommandAfterFault = true;
    allOutputsOff();
    alertMessage = "Configuracao GPS alterada: envie PARAR antes de nova partida";
    return;
  }
  currentCmd = cmd;
  currentCmd.mode = normalizeModeStr(currentCmd.mode);
  currentCmd.direction = normalizeDirectionStr(currentCmd.direction);
  syncServerTimeFromCommand(currentCmd);

  bool loraFreshNow = loraGpsValid && (millis() - loraLastRxMs < 300000UL);
  if (loraFreshNow && currentCmd.geometryReady) {
    pivotAngleDeg = applyGpsCorrection(calculateBearingFromCenter(currentCmd.centerLat, currentCmd.centerLon, loraGpsLat, loraGpsLon));
    distanceFromCenterM = TinyGPSPlus::distanceBetween(currentCmd.centerLat, currentCmd.centerLon, loraGpsLat, loraGpsLon);
    pivotAngleValid = isFiniteNumber(distanceFromCenterM) && isFiniteNumber(pivotAngleDeg);
    // Recalcular geometria nao constitui uma nova amostra GPS.
    lastGoodPivotAngleValid = pivotAngleValid;
    if (pivotAngleValid) {
      lastGoodPivotAngleDeg = pivotAngleDeg;
      lastGoodDistanceFromCenterM = distanceFromCenterM;
    }
  }

  if (currentCmd.direction == "parado") currentCmd.mode = "sem_agua";

  if (continuingOperation) {
    lastCommandAcceptedMs = millis();
    lastAcceptedUpdatedAt = currentCmd.updatedAt;
    if (!armOperationJournal(currentCmd, "command_update")) {
      enterPersistenceFault("falha ao persistir atualizacao do comando em operacao");
      return;
    }
    setCommandExecution(currentCmd.commandId, currentCmd.direction, currentCmd.mode,
                        "accepted", "Atualizacao recebida; aguardando confirmacao das saidas", false);
    appendDiagnosticHistory("COMANDO", "ATUALIZADO EM OPERACAO",
        "updatedAt=" + String(currentCmd.updatedAt) +
        " direcao=" + currentCmd.direction + " modo=" + currentCmd.mode);
    // processController aplica segurancas, pressao, horario e destino no mesmo ciclo.
    return;
  }
  faultMessage = "";
  alertMessage = "";
  clearControllerTimers();

  if (currentCmd.depthClamped) alertMessage = "Lâmina ajustada para mínima do pivô";

  if (currentCmd.direction == "parado") {
    allOutputsOff();
    bool journalCleared = clearOperationJournal("stop_command");
    requireStopCommandAfterFault = !journalCleared;
    controllerState = journalCleared ? STATE_IDLE : STATE_FAULT_PERSISTENCE;
    faultMessage = journalCleared ? "" : "Falha de persistencia NVS";
    alertMessage = journalCleared ? "Parado" :
        "PROSERVICE INFORMA: parado, mas memoria NVS ainda nao confirmou o registro";
    allOutputsOff();
    lastCommandAcceptedMs = millis();
    lastAcceptedUpdatedAt = currentCmd.updatedAt;
    setCommandExecution(currentCmd.commandId, currentCmd.direction, currentCmd.mode,
                        journalCleared ? "succeeded" : "failed",
                        journalCleared ? "Todas as saidas de movimento desligadas" :
                        "Saidas desligadas; falha ao confirmar journal NVS", true);
    recordStopEvent("COMANDO PARAR RECEBIDO", "updatedAt recebido=" + String(currentCmd.updatedAt));
    return;
  }

  if (currentCmd.peakEnabled && isPeakActiveNow(currentCmd)) {
    controllerState = STATE_WAIT_PEAK_END;
    alertMessage = "PROSERVICE INFORMA: Horário de ponta ativo";
    allOutputsOff();
    lastCommandAcceptedMs = millis();
    lastAcceptedUpdatedAt = currentCmd.updatedAt;
    setCommandExecution(currentCmd.commandId, currentCmd.direction, currentCmd.mode,
                        "accepted", "Aguardando fim do horario de ponta", false);
    appendDiagnosticHistory("COMANDO", "ACEITO AGUARDANDO HORARIO",
        "updatedAt=" + String(currentCmd.updatedAt));
    return;
  }

  // Nunca energiza uma operacao dependente de GPS sem centro e raio validos.
  if (commandRequiresGps(currentCmd) && !currentCmd.geometryReady) {
    controllerState = STATE_IDLE;
    faultMessage = "Geometria do pivô incompleta";
    alertMessage = "PROSERVICE INFORMA: configure centro e raio antes de operar com GPS";
    allOutputsOff();
    lastCommandAcceptedMs = millis();
    lastAcceptedUpdatedAt = currentCmd.updatedAt;
    setCommandExecution(currentCmd.commandId, currentCmd.direction, currentCmd.mode,
                        "rejected", "Centro/raio invalidos para operacao GPS", true);
    appendDiagnosticHistory("COMANDO", "REJEITADO SEM GEOMETRIA",
        "commandId=" + currentCmd.commandId + " centro/raio invalidos");
    return;
  }

  if (commandRequiresGps(currentCmd) && !gpsReadyForStart()) {
    controllerState = STATE_IDLE;
    faultMessage = "GPS indisponivel para partida";
    alertMessage = "PROSERVICE INFORMA: PARTIDA BLOQUEADA. GPS sem amostra recente/qualidade. Recupere o GPS e envie NOVO comando.";
    allOutputsOff();
    lastCommandAcceptedMs = millis();
    lastAcceptedUpdatedAt = currentCmd.updatedAt;
    setCommandExecution(currentCmd.commandId, currentCmd.direction, currentCmd.mode,
                        "rejected", "GPS indisponivel: nao e permitido partir com GPS ativo sem amostra recente e qualidade. Recupere o GPS e envie NOVO comando.", true);
    appendDiagnosticHistory("COMANDO", "REJEITADO SEM GPS OPERACIONAL",
        "updatedAt=" + String(currentCmd.updatedAt) + " direcao=" + currentCmd.direction);
    return;
  }

  // Persiste e relê a intenção ANTES de energizar GPIO23. Sem confirmação,
  // a partida é abortada e todas as saídas permanecem desligadas.
  if (!armOperationJournal(currentCmd, "prestart")) {
    enterPersistenceFault("falha ao persistir pre-partida");
    return;
  }
  setCommandExecution(currentCmd.commandId, currentCmd.direction, currentCmd.mode,
                      "accepted", "Aguardando seguranca de pre-partida", false);
  appendDiagnosticHistory("COMANDO", "ACEITO EM PRE-PARTIDA",
      "updatedAt=" + String(currentCmd.updatedAt) +
      " direcao=" + currentCmd.direction + " modo=" + currentCmd.mode);
  // Pré-partida: GPIO23 fecha primeiro e dá até 10 s para a cadeia NF liberar.
  controllerState = STATE_WAIT_SAFETY;
  safetyPrestartStartedMs = millis();
  safetyStableSinceMs = 0;
  alertMessage = "PROSERVICE INFORMA: Aguardando segurança de pré-partida";

  applyOutputs();
  lastCommandAcceptedMs = millis();
  lastAcceptedUpdatedAt = currentCmd.updatedAt;
}

void processSafetyLogic() {
  if (bootGuardActive()) {
    allOutputsOff();
    return;
  }

  bool safetyMustBeClosed = controllerState == STATE_WAIT_PRESSURE ||
                            controllerState == STATE_WAIT_START ||
                            controllerState == STATE_RUNNING;
  if (!safetyMustBeClosed) {
    safetyLostSinceMs = 0;
    return;
  }

  if (safetyOk()) {
    safetyLostSinceMs = 0;
    return;
  }

  if (safetyLostSinceMs == 0) {
    safetyLostSinceMs = millis();
  }

  if (millis() - safetyLostSinceMs >= SAFETY_DELAY_MS) {
    if (controllerState != STATE_FAULT_SAFETY) {
      enterSafetyFault();
    }
  }
}

void processController() {
  if (bootGuardActive()) {
    controllerState = STATE_BOOT_HOLD;
    faultMessage = "";
    alertMessage = "Estabilizando boot";
    allOutputsOff();
    return;
  }

  if (operationJournalStartInterlock) {
    enterPersistenceFault(operationJournalInterlockReason.length() ?
        operationJournalInterlockReason : String("journal NVS sem confirmacao"));
    return;
  }

  if (controllerState == STATE_WAIT_SAFETY) {
    if(commandRequiresGps(currentCmd) && !gpsReadyForStart()) {
      enterGpsTimeoutFault("GPS indisponivel na pre-partida; movimento nao autorizado"); return;
    }
    processBoardReadyRelay();
    if (safetyStableReady()) {
      if (currentCmd.mode == "com_agua") {
        controllerState = STATE_WAIT_PRESSURE;
        pressureWaitStartedMs = millis();
        alertMessage = "PROSERVICE INFORMA: Segurança liberada, aguardando pressurização";
      } else {
        controllerState = STATE_RUNNING;
        moveStartedMs = millis();
        resetAngularArrivalTracking();
        movementMonitorStartedMs = millis();
        if (commandRequiresGps(currentCmd)) movementGuard.begin(millis());
        movementLastGeneration = gpsSampleGeneration;
        movementLastUsableSampleMs = 0;
        movementQualityRejects = movementAgeRejects = movementGeometryRejects = 0;
        lastMovementMs = millis();
        movementRefValid = false;
        alertMessage = "PROSERVICE INFORMA: Segurança liberada, pivô em movimento";
      }
      applyOutputs();
      return;
    }
    if (safetyPrestartStartedMs > 0 &&
        millis() - safetyPrestartStartedMs >= SAFETY_PRESTART_TIMEOUT_MS) {
      enterSafetyFault();
      return;
    }
    alertMessage = "PROSERVICE INFORMA: Aguardando GPIO36/GPIO39 por até 10s";
    applyOutputs();
    return;
  }

  processSafetyLogic();

  if (controllerState == STATE_FAULT_SAFETY || controllerState == STATE_FAULT_LOW_PRESSURE ||
      controllerState == STATE_FAULT_GPS_TIMEOUT || controllerState == STATE_FAULT_PORTAL_TIMEOUT ||
      controllerState == STATE_FAULT_NO_MOVEMENT ||
      controllerState == STATE_FAULT_PERSISTENCE) {
    allOutputsOff();
    return;
  }

  bool opActive = (controllerState == STATE_WAIT_SAFETY || controllerState == STATE_WAIT_PRESSURE || controllerState == STATE_WAIT_START || controllerState == STATE_RUNNING || controllerState == STATE_WAIT_PEAK_END);
  uint32_t portalReferenceMs = lastServerOkMs > 0 ? lastServerOkMs : bootStartedMs;
  if (cfg.backendEnabled && !localTestMode && opActive &&
      (uint32_t)(millis() - portalReferenceMs) > PORTAL_OFFLINE_MS) {
    portalOfflineLatch = true;
    enterPortalTimeoutFault();
    return;
  }

  if(requireStopCommandAfterFault&&controllerState==STATE_IDLE&&currentCmd.direction=="parado") {
    allOutputsOff();faultMessage="Rearme necessario apos manutencao/falha";
    alertMessage="Envie um novo PARAR para rearmar";return;
  }
  if (currentCmd.direction == "parado") {
    controllerState = STATE_IDLE;
    faultMessage = "";
    alertMessage = "Parado";
    applyOutputs();
    return;
  }

  if (currentCmd.peakEnabled && isPeakActiveNow(currentCmd)) {
    if (controllerState != STATE_WAIT_PEAK_END) {
      controllerState = STATE_WAIT_PEAK_END;
      alertMessage = "PROSERVICE INFORMA: Horário de ponta ativo";
      allOutputsOff();
    }
  } else if (controllerState == STATE_WAIT_PEAK_END) {
    controllerState = STATE_WAIT_SAFETY;
    safetyPrestartStartedMs = millis();
    safetyStableSinceMs = 0;
    alertMessage = "PROSERVICE INFORMA: Horário de ponta encerrado, aguardando segurança";
    processBoardReadyRelay();
    return;
  }

  if (commandRequiresGps(currentCmd) && opActive && !hasOperationalGpsSample(millis())) {
    enterGpsTimeoutFault("Sem nova amostra GPS/LoRa por 5 minutos");
    return;
  }

  bool pressure = pressureOk();

  switch (controllerState) {
    case STATE_IDLE:
      applyOutputs();
      break;

    case STATE_WAIT_PRESSURE:
      if (!pressure) {
        unsigned long waitedMs = millis() - pressureWaitStartedMs;
        unsigned long limitMs = (uint32_t)currentCmd.pressureWaitSec * 1000UL;
        if (waitedMs >= limitMs) {
          enterLowPressureFault();
          return;
        }
      } else {
        pressureDetectedMs = millis();
        controllerState = STATE_WAIT_START;
        alertMessage = currentCmd.depthClamped ? "Lâmina mínima aplicada, pressurizado, aguardando 10s" : "Pressurizado, aguardando 10s";
      }
      applyOutputs();
      break;

    case STATE_WAIT_START:
      if (!pressure) {
        if (millis() - pressureWaitStartedMs < (uint32_t)currentCmd.pressureWaitSec * 1000UL) {
          controllerState = STATE_WAIT_PRESSURE;
          alertMessage = "Pressão caiu, aguardando repressurização";
          return;
        } else {
          enterLowPressureFault();
          return;
        }
      }
      if (millis() - pressureDetectedMs >= MOVE_AFTER_PRESSURE_MS) {
        if(commandRequiresGps(currentCmd) && !gpsReadyForStart()) {
          enterGpsTimeoutFault("GPS indisponivel antes do movimento; partida interrompida"); return;
        }
        controllerState = STATE_RUNNING;
        moveStartedMs = millis();
        resetAngularArrivalTracking();
        movementMonitorStartedMs = millis();
        if (commandRequiresGps(currentCmd)) movementGuard.begin(millis());
        movementLastGeneration = gpsSampleGeneration;
        movementLastUsableSampleMs = 0;
        movementQualityRejects = movementAgeRejects = movementGeometryRejects = 0;
        lastMovementMs = millis();
        movementRefValid = false;
        alertMessage = currentCmd.mode == "com_agua" ? (currentCmd.depthClamped ? "PROSERVICE INFORMA: Pivô em movimento com água (lâmina mínima)" : "PROSERVICE INFORMA: Pivô em movimento com água") : (currentCmd.depthClamped ? "PROSERVICE INFORMA: Pivô em movimento a seco (lâmina mínima)" : "PROSERVICE INFORMA: Pivô em movimento a seco");
      }
      applyOutputs();
      break;

    case STATE_WAIT_PEAK_END:
      allOutputsOff();
      break;

    case STATE_RUNNING: {
      if (currentCmd.mode == "com_agua" && !pressure) {
        // O monitor rapido ja cortou fisicamente as saidas. A falha so e
        // travada apos SAFETY_DELAY_MS continuo, evitando alarme por ruido.
        allOutputsOff();
        return;
      }

      double stopDeg = 360.0;
      if (commandRequiresGps(currentCmd) && AngularTargetPolicy::parse(currentCmd.stopDegree.c_str(), stopDeg)) {
        const uint32_t angularNow = millis();
        const bool angleSampleFresh = hasOperationalGpsSample(angularNow) &&
            (uint32_t)(angularNow - lastGoodGpsMs) <= 15000UL &&
            lastGoodPivotAngleValid && isFiniteNumber(lastGoodPivotAngleDeg);
        bool arrived = false;
        if (!angleSampleFresh) {
          angularPreviousValid = false;
          angularNearCount = 0;
        } else if (angularCheckedGeneration != gpsSampleGeneration) {
          angularCheckedGeneration = gpsSampleGeneration;
          const double cur = normalizeAngle360(lastGoodPivotAngleDeg);
          const double tgt = normalizeAngle360(stopDeg);
          const int dir = currentCmd.direction == "direita" ? 1 :
                          (currentCmd.direction == "esquerda" ? -1 : 0);
          const double signedError = fmod((tgt - cur) * dir + 360.0, 360.0);
          const double shortestError = fmin(signedError, 360.0 - signedError);
          // Duas amostras consecutivas proximas evitam parada por um unico salto.
          if (dir != 0 && shortestError <= 0.5) {
            if (angularNearCount < 2) ++angularNearCount;
          } else angularNearCount = 0;
          arrived = (angularNearCount >= 2);
          // Ultrapassagem pequena e coerente com o sentido do movimento.
          // Nunca considerar saltos > 2 graus como cruzamento confirmado.
          if (!arrived && dir != 0 && angularPreviousValid &&
              (uint32_t)(lastGoodGpsMs - angularPreviousMs) <= 15000UL) {
            const double advance = fmod((cur - angularPreviousDeg) * dir + 360.0, 360.0);
            const double previousToTarget = fmod((tgt - angularPreviousDeg) * dir + 360.0, 360.0);
            if (advance > 0.0 && advance <= 2.0 && previousToTarget <= advance &&
                previousToTarget <= 2.0) arrived = true;
          }
          angularPreviousDeg = cur;
          angularPreviousMs = lastGoodGpsMs;
          angularPreviousValid = true;
        }
        if (arrived) {
          cancelPendingCommand("destino angular atingido");
          allOutputsOff();
          clearOperationJournal("angular_target_reached");
          controllerState = STATE_IDLE;
          setCommandExecution(currentCmd.commandId, currentCmd.direction, currentCmd.mode,
                              "succeeded", "Destino angular atingido e saidas desligadas", true);
          recordStopEvent("DESTINO ANGULAR ATINGIDO",
                          "angulo=" + String(pivotAngleDeg, 2) + " alvo=" + String(stopDeg, 2));
          currentCmd.direction = "parado";
          alertMessage = "PROSERVICE INFORMA: Pivo chegou ao destino (" + String(pivotAngleDeg, 1) + "deg)";
          allOutputsOff();
          return;
        }
      }

      if (commandRequiresGps(currentCmd)) {
        uint32_t now = millis();
        if (!movementGuard.active) {
          movementGuard.begin(now);
          movementLastGeneration = gpsSampleGeneration;
          movementLastUsableSampleMs = 0;
        }
        // A posicao permanece retida para mapa/portal, mas a operacao exige
        // amostra recente. Perda de observacao nao pode virar falso "parado".
        if (!currentCmd.geometryReady) {
          enterGpsTimeoutFault("Geometria do pivo ficou indisponivel durante a operacao");
          return;
        }
        if (!hasOperationalGpsSample(now)) {
          enterGpsTimeoutFault("Sem nova amostra GPS/LoRa por 5 minutos");
          return;
        }
        uint32_t qualityReference = movementLastUsableSampleMs ?
            movementLastUsableSampleMs : movementGuard.started;
        if ((uint32_t)(now - qualityReference) > GPS_OPERATION_TIMEOUT_MS) {
          enterGpsTimeoutFault("GPS recebido sem qualidade suficiente por 5 minutos");
          return;
        }
        if (movementGuard.expired(now)) {
          // Se o avanço mínimo já apareceu e está sendo confirmado, não gerar
          // SEM MOVIMENTO no meio da janela de 10 s. Isso acontecia quando
          // retransmissões/pequenas perdas espaçavam as amostras: havia avanço
          // real, mas o prazo vencia antes da quinta amostra de confirmação.
          const bool coherentCandidateInProgress = movementGuard.candidate &&
              movementGuard.progress >= MovementGuard::MIN_PROGRESS_M &&
              movementObservationReady(now);
          if (coherentCandidateInProgress) {
            movementGuard.reason = "Avanco detectado; concluindo confirmacao";
          } else if (movementObservationReady(now)) {
            enterNoMovementFault();
            return;
          }
          if (!coherentCandidateInProgress)
            movementGuard.reason = "Observacao insuficiente; aguardando GPS operacional";
        }
        if (now-lastGoodGpsMs > 15000UL) {
          movementGuard.reason = "Sem amostra recente; prazo continua";
        }

      } else {
        movementGuard = MovementGuard();
      }

      alertMessage = currentCmd.mode == "com_agua" ? (currentCmd.depthClamped ? "PROSERVICE INFORMA: Pivô em movimento com água (lâmina mínima)" : "PROSERVICE INFORMA: Pivô em movimento com água") : (currentCmd.depthClamped ? "PROSERVICE INFORMA: Pivô em movimento a seco (lâmina mínima)" : "PROSERVICE INFORMA: Pivô em movimento a seco");
      applyOutputs();
      break;
    }

    case STATE_BOOT_HOLD:
      allOutputsOff();
      break;

    case STATE_FAULT_PERSISTENCE:
      allOutputsOff();
      break;

    default:
      applyOutputs();
      break;
  }
}

// ======================================================
// BOTÃO DE CONFIGURAÇÃO
// ======================================================

void processConfigButton() {
  static bool handled=false;
  bool pressed=digitalRead(PIN_CONFIG_BUTTON)==LOW;
  if(!pressed){configButtonPressedMs=0;handled=false;return;}
  if(!configButtonPressedMs)configButtonPressedMs=millis();
  if(!handled&&uint32_t(millis()-configButtonPressedMs)>=CONFIG_BUTTON_HOLD_MS){handled=true;startConfigPortal();openServiceWindow();}
}

// ======================================================
// SETUP / LOOP
// ======================================================

void setupPins() {
  pinMode(PIN_EMERGENCY_1, INPUT);
  pinMode(PIN_EMERGENCY_2, INPUT);
  pinMode(PIN_PRESSURE, INPUT);
  pinMode(PIN_CONFIG_BUTTON, INPUT_PULLUP);
  pinMode(PIN_DETECTA_MOSFET, OUTPUT);
  digitalWrite(PIN_DETECTA_MOSFET, LOW);
  forceHardwareOutputsOff();
  writeOutputIfChanged(PIN_OUT_AUX6, false);
  attachInterrupt(digitalPinToInterrupt(PIN_EMERGENCY_1), emergencyCutIsr, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_EMERGENCY_2), emergencyCutIsr, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_PRESSURE), pressureCutIsr, FALLING);
  safetyInterruptsConfigured = true;
}

void setupGps() {
  Serial.println("[GPS] Posicao via LoRa");
}

// ======================================================
#if 0 // Código legado Detecta desativado: preservado apenas como referência.
// DETECTA — scan de impedância via INA226 (portado do sketch_feb27a)
// Sem LCD, botão ou sensor de temperatura — controlado pelo portal
// ======================================================

// Tempos de medição (comprovados em campo)
static const unsigned long DETECTA_TEMPO_LIGA_MS   = 5400; // energiza cabo
static const unsigned long DETECTA_TEMPO_AMOSTRA_MS = 2700; // janela de coleta
static const int           DETECTA_NUM_AMOSTRAS     = 25;   // amostras por ciclo
static const int           DETECTA_WARMUP_SKIP      = 6;    // amostras de warm-up descartadas

// Um ciclo: energiza, coleta 25 amostras, desliga, retorna mediana em metros
float detectaMedirCiclo(bool &ok) {
  ok = false;
  writeOutputState(PIN_DETECTA_MOSFET, true);
  delay(DETECTA_TEMPO_LIGA_MS);
  esp_task_wdt_reset();

  // Warm-up: descarta 6 leituras iniciais (transiente)
  for (int w = 0; w < 6; w++) {
    (void)ina226.getBusVoltage_V();
    (void)ina226.getCurrent_mA();
    delay(40);
  }

  const int N = 25;
  float metrosAmostra[N];
  int nValidas = 0;

  for (int k = 0; k < N; k++) {
    float v    = ina226.getBusVoltage_V();
    float shuntmV = ina226.getShuntVoltage_mV();
    float imA  = ina226.getCurrent_mA();
    if (fabsf(imA) < 0.5f && fabsf(shuntmV) > 0.01f) {
      imA = shuntmV / 0.1f; // shunt=0.1ohm → mV/0.1 = mA
    }
    float iA = fabsf(imA) / 1000.0f;
    if (iA > 0.00005f && v > 0.05f && detectaResistPorMetro > 0) {
      float r = v / iA;
      float m = r / detectaResistPorMetro;
      metrosAmostra[nValidas++] = m;
    }
    delay(DETECTA_TEMPO_AMOSTRA_MS / N);
    if (k % 5 == 0) esp_task_wdt_reset();
  }

  writeOutputState(PIN_DETECTA_MOSFET, false);
  esp_task_wdt_reset();

  if (nValidas < 3) return 0;

  std::sort(metrosAmostra, metrosAmostra + nValidas);
  ok = true;
  if (nValidas % 2 == 1) return metrosAmostra[nValidas / 2];
  return (metrosAmostra[nValidas / 2 - 1] + metrosAmostra[nValidas / 2]) / 2.0f;
}

// 1 ciclo para calibração manual — retorna ohms brutos
// Energiza GPIO2 (XY-MOS), lê amostras com INA226, retorna média em ohms (0 = falha)
float detectaMedirUmCiclo() {
  const int N     = DETECTA_NUM_AMOSTRAS;
  const int SKIP  = DETECTA_WARMUP_SKIP;
  const int TOTAL = N + SKIP;

  float amostrasOhm[25];
  int nValidas = 0;

  writeOutputState(PIN_DETECTA_MOSFET, true);
  delay(300); // aguarda estabilização

  // === LOG DIAGNÓSTICO: primeiras 3 leituras brutas ===
  for (int d = 0; d < 3; d++) {
    delay(100);
    float dbusV    = ina226.getBusVoltage_V();
    float dshuntmV = ina226.getShuntVoltage_mV();
    float dcurrA   = ina226.getCurrent_mA();
    float dpowmW   = ina226.getBusPower();
    Serial.printf("[INA226 RAW #%d] bus=%.4fV shunt=%.4fmV curr=%.4fmA pow=%.4fmW\n",
                  d, dbusV, dshuntmV, dcurrA, dpowmW);
    esp_task_wdt_reset();
  }
  // =============================================

  for (int k = 0; k < TOTAL; k++) {
    delay(DETECTA_TEMPO_AMOSTRA_MS / TOTAL);
    if (k % 5 == 0) esp_task_wdt_reset();
    if (k < SKIP) continue; // descarta warm-up
    float v       = ina226.getBusVoltage_V();
    float shuntmV = ina226.getShuntVoltage_mV();
    float imA     = ina226.getCurrent_mA();
    // fallback pelo shunt se getCurrent retornar muito baixo
    if (fabsf(imA) < 0.5f && fabsf(shuntmV) > 0.01f) {
      imA = shuntmV / 0.1f; // shunt=0.1ohm
    }
    float iA = fabsf(imA) / 1000.0f;
    if (iA > 0.00005f && v > 0.05f) {
      amostrasOhm[nValidas++] = v / iA;
    }
  }
  writeOutputState(PIN_DETECTA_MOSFET, false);
  esp_task_wdt_reset();

  Serial.printf("[DETECTA] medirUmCiclo: %d/%d amostras validas\n", nValidas, N);
  if (nValidas < 3) return 0.0f;
  float soma = 0;
  for (int i = 0; i < nValidas; i++) soma += amostrasOhm[i];
  float media = soma / nValidas;
  Serial.printf("[DETECTA] media=%.4f ohms\n", media);
  return media;
}

// 3 ciclos com mediana final — elimina outliers
float detectaMedirMetros() {  bool ok1 = false, ok2 = false, ok3 = false;
  float m1 = detectaMedirCiclo(ok1); esp_task_wdt_reset(); delay(200);
  float m2 = detectaMedirCiclo(ok2); esp_task_wdt_reset(); delay(200);
  float m3 = detectaMedirCiclo(ok3); esp_task_wdt_reset();

  float m[3]; int n = 0;
  if (ok1) m[n++] = m1;
  if (ok2) m[n++] = m2;
  if (ok3) m[n++] = m3;

  if (n == 0) return -1.0f; // falha total

  std::sort(m, m + n);
  if (n % 2 == 1) return m[n / 2];
  return (m[n / 2 - 1] + m[n / 2]) / 2.0f;
}

void runDetectaScan() {
  detectaRunning   = true;
  detectaStatus    = "scanning";
  detectaStartedMs = millis();
  detectaResultMetros = 0.0f;
  detectaResultTorre  = 0;
  detectaMensagem  = "Escaneando...";
  Serial.println("[DETECTA] Scan 3x25");

  if (!detectaInaOk) {
    // INA226 não conectado — pulsa GPIO2 + GPIO2 por 2s para confirmar comando recebido
    Serial.println("[DETECTA] Sem INA226, GPIO2 2s");
    writeOutputState(PIN_DETECTA_MOSFET, true);
    esp_task_wdt_reset();
    delay(2000);
    writeOutputState(PIN_DETECTA_MOSFET, false);
    detectaStatus   = "error";
    detectaMensagem = "INA226 não conectado — GPIO2 ativado como teste";
    detectaRunning  = false;
    Serial.println("[DETECTA] Sem INA226");
    return;
  }

  float metros = detectaMedirMetros();
  Serial.printf("[DETECTA] metros=%.1f\n", metros);

  if (metros < 0) {
    detectaStatus   = "error";
    detectaMensagem = "Corrente insuficiente — verifique conexão do cabo";
    detectaRunning  = false;
    return;
  }

  detectaResultMetros = metros;

  // Identifica torre pela faixa configurada no portal
  detectaResultTorre = 0;
  for (int i = 0; i < detectaNumTorres; i++) {
    if (metros >= detectaTorreInicio[i] && metros <= detectaTorreFim[i]) {
      detectaResultTorre = i + 1;
      break;
    }
  }

  if (detectaResultTorre > 0) {
    detectaStatus   = "done";
    detectaMensagem = "TORRE " + String(detectaResultTorre) + " EM FALHA — " + String(metros, 1) + "m do centro";
  } else {
    detectaStatus   = "done";
    detectaMensagem = "Falha a " + String(metros, 1) + "m — fora das faixas configuradas";
  }

  Serial.println("[DETECTA] " + detectaMensagem);
  detectaRunning = false;
}
#endif

void setup() {
  bootStartedMs = millis();
  setupPins(); // Desligar as saidas antes de inicializar a rede.
  setupFastSafetyTask();

  Serial.begin(115200);
  delay(500);
#if PROSERVICE_VERBOSE_SERIAL
  Serial.println("Firmware: " + String(FIRMWARE_VERSION));
#endif

  setupWiFiBase();
  setupRtc();
  processBoardReadyRelay();
  // Sem GPS local: a posicao da ponta chega exclusivamente via LoRa.
  setupLora();
  writeE220Channel(-1,e220ConfigStatus); // Consulta na partida; nao grava o canal salvo.

  // Detecta/INA226 removido desta compilação.
  digitalWrite(PIN_DETECTA_MOSFET, LOW);
#if ESP_IDF_VERSION_MAJOR >= 5
  esp_task_wdt_config_t twdtConfig = {};
  twdtConfig.timeout_ms = 30000;
  twdtConfig.idle_core_mask = 0;
  twdtConfig.trigger_panic = true;
  esp_err_t wdtInitResult = esp_task_wdt_init(&twdtConfig);
  if (wdtInitResult == ESP_ERR_INVALID_STATE) {
    esp_task_wdt_reconfigure(&twdtConfig);
  }
#else
  esp_task_wdt_init(30, true);
#endif
  esp_task_wdt_add(NULL);
#if PROSERVICE_VERBOSE_SERIAL
  Serial.println("[WDT] 30s");
#endif
  loadConfig();

  controllerState = STATE_BOOT_HOLD;
  currentCmd.direction = "parado";
  currentCmd.mode = "sem_agua";
  currentCmd.depth = "8";
  currentCmd.stopDegree = "360";
  currentCmd.canhaoFinal = false;
  currentCmd.pressureWaitSec = 240;
  currentCmd.updatedAt = 0;
  currentCmd.requestedDepthMm = 8.0f;
  currentCmd.minDepthMm = 0.0f;
  currentCmd.safetyFactorPercent = 3.0f;
  currentCmd.lastTowerPercent = -1.0f;
  currentCmd.lastTowerCycleMs = DEFAULT_LAST_TOWER_CYCLE_MS;
  currentCmd.engineeringReady = false;
  currentCmd.depthClamped = false;
  currentCmd.centerLat = 0.0;
  currentCmd.centerLon = 0.0;
  currentCmd.radiusMeters = 0.0f;
  currentCmd.areaHectares = 0.0f;
  currentCmd.geometryReady = false;
  currentCmd.gpsCorrectionDeg = 0.0f;

  // Carrega os dados físicos deste pivô da memória local.
  applyLocalPivotProfile(currentCmd);

  // Carrega histórico e registra a causa desta inicialização.
  loadDiagnostics();
  loadCommandOrderGuard();

  alertMessage = "PROSERVICE INFORMA: Sistema inicializado";
  faultMessage = "";
  clearControllerTimers();
  allOutputsOff();

#if PROSERVICE_VERBOSE_SERIAL
  Serial.println("======================================");
  Serial.println("ESP32 Pivot Control - proservice");
  Serial.println("======================================");
  Serial.println("Pivot ID: " + cfg.pivotId);
  Serial.println("Hardware: " + cfg.hardwareName);
  Serial.println("Backend remoto: " + String(cfg.backendEnabled ? "ATIVO" : "DESATIVADO"));
  Serial.println("Servidor base: " + cfg.serverUrl);
  Serial.println("URL comando: " + buildCommandUrl());
  Serial.println("URL heartbeat: " + buildHeartbeatUrl());
  Serial.println("SSID salvo: " + cfg.wifiSsid);
  Serial.println("GPS: somente ponta via LoRa");
  Serial.println("Perfil físico local: " + String(cfg.localProfileEnabled ? "ATIVO" : "DESATIVADO"));
  Serial.println("Centro local: " + String(cfg.localCenterLat, 8) + ", " + String(cfg.localCenterLon, 8));
  Serial.println("Raio local: " + String(cfg.localRadiusMeters, 2) + " m / Área: " + String(cfg.localAreaHectares, 4) + " ha");
#endif

  ensureConfigPortalRunning(true);

  // Inicia a conexão em segundo plano. O AP e o portal permanecem ativos.
  beginSavedWifiConnection();
  setupEthernet();
  processNetworkTransport();
#if PROSERVICE_VERBOSE_SERIAL
  Serial.println("[WIFI] AP ativo");
#endif

  allOutputsOff();
  postHeartbeatToServer();
  printControllerSnapshot();
}

bool networkControlServiceActive=false;
void serviceControlBeforeNetwork() {
  if(networkControlServiceActive)return;networkControlServiceActive=true;
  processLoRaRx();processFastSafetyTrips();
  if(localTestMode&&localDirectMode)applyLocalDirectOutputs();else processController();
  processBoardReadyRelay();trackCommandAppliedEvent();
  networkControlServiceActive=false;
}
void loop() {
  processServiceAccess();
  processWifiScan();
  processNetworkTransport();
  esp_task_wdt_reset();
  processFastSafetyTrips();
  processUrgentSafetyHeartbeat();

  if (millis() - lastApRetryMs >= AP_RETRY_INTERVAL_MS) {
    lastApRetryMs = millis();
    ensureConfigPortalRunning(false);
  }

  esp_task_wdt_reset();
  if (webServerStarted) {
    server.handleClient();
  }

  processBoardReadyRelay();
  // GPS local removido.
  processLoRaRx();
  sendLoRaConfig();
  processConfigButton();

  // Detecta desabilitado neste projeto


  processWifiConnection();
  if (WiFi.status() != WL_CONNECTED) {
    if (!wifiAttemptActive && !cfg.wifiSsid.isEmpty() && millis() - lastWifiAttemptMs >= WIFI_RETRY_INTERVAL_MS) {
      lastWifiAttemptMs = millis();
      beginSavedWifiConnection();
#if PROSERVICE_VERBOSE_SERIAL
      Serial.println("[WIFI] Reconectando");
#endif
    }
  } else {
    forceWifiReconnectIfPortalStale();
  }

  esp_task_wdt_reset();
  bool ackDue=shouldForceHeartbeatNow() && millis()-lastStatusMs>=HEARTBEAT_FORCE_MIN_INTERVAL_MS;
  if (!localTestMode && networkReady() && PortalLatencyPolicy::fetchDue(millis(),lastFetchMs,ackDue)) {
    lastFetchMs = millis();
    PivotCommand incoming;
#if PROSERVICE_VERBOSE_SERIAL
    Serial.println("[CMD] Consultando portal");
#endif
    if (fetchCommandFromServer(incoming)) {
      processFastSafetyTrips();
      processLoRaRx(); // consome FIX capturado durante HTTP antes de avaliar nova partida
#if PROSERVICE_VERBOSE_SERIAL
      Serial.printf("[CMD] %s %s\n", incoming.mode.c_str(), incoming.direction.c_str());
#endif
      uint32_t incomingPayloadHash = calculateCommandPayloadHash(incoming);
      // A comparacao precisa ocorrer antes de notePortalCommandReceived(),
      // pois um comando realmente novo atualiza commandExecution.
      bool matchesPersistedCommand = !bootInitialCommandEvaluated &&
          commandExecution.commandId == incoming.commandId &&
          commandExecution.payloadHash != 0 &&
          commandExecution.payloadHash == incomingPayloadHash;
      bool terminalReplayAfterBoot = matchesPersistedCommand && commandExecutionIsTerminal();
      bool interruptedReplayAfterBoot = matchesPersistedCommand && !commandExecutionIsTerminal();
      bool authorizedAutomaticRestart = commandMatchesAutoRestart(incoming);
      bool commandProtocolOk = validateAndPersistCommandOrder(incoming) && notePortalCommandReceived(incoming);
      if (!commandProtocolOk) {
        Serial.println("[CMD] ID conflitante");
      } else if (authorizedAutomaticRestart) {
        bootInitialCommandEvaluated = true;
        authorizeAutoRestart(incoming);
#if PROSERVICE_VERBOSE_SERIAL
        Serial.println("[CMD] Restart autorizado");
#endif
      } else if (!bootInitialCommandEvaluated) {
        bootInitialCommandEvaluated = true;
        if (autoRestart.required || autoRestart.pending) {
          cancelAutoRestart("portal enviou comando diferente ou desativou a opcao");
        }
        if (terminalReplayAfterBoot) {
#if PROSERVICE_VERBOSE_SERIAL
          Serial.println("[CMD] Terminal persistido");
#endif
          appendDiagnosticHistory("COMANDO", "RETRANSMISSAO TERMINAL IGNORADA",
              "commandId=" + incoming.commandId +
              " resultado=" + commandExecution.result);
          currentCmd = incoming;
          currentCmd.direction = "parado";
          currentCmd.mode = "sem_agua";
        } else if (interruptedReplayAfterBoot && !incoming.restartAfterPower &&
                   incoming.direction != "parado") {
#if PROSERVICE_VERBOSE_SERIAL
          Serial.println("[CMD] Restart desativado");
#endif
          setCommandExecution(incoming.commandId, incoming.direction, incoming.mode,
                              "ignored", "Reinicio sem autorizacao restartAfterPower", true);
          appendDiagnosticHistory("COMANDO", "IGNORADO APOS RESET",
              "commandId=" + incoming.commandId + " updatedAt=" + String(incoming.updatedAt) +
              " direcao=" + incoming.direction + " restartAfterPower=0");
          currentCmd = incoming;
          currentCmd.direction = "parado";
          currentCmd.mode = "sem_agua";
        } else if (incoming.commandId == cancelledRestartCommandId &&
                   normalizeDirectionStr(incoming.direction) != "parado") {
#if PROSERVICE_VERBOSE_SERIAL
          Serial.println("[CMD] Cancelado bloqueado");
#endif
        } else if (commandIsNewOrChanged(incoming, currentCmd)) {
#if PROSERVICE_VERBOSE_SERIAL
          Serial.println(matchesPersistedCommand ?
              "[CMD] Retomada autorizada do comando persistido" :
              "[CMD] Novo commandId recebido apos boot; solicitacao deliberada aceita");
#endif
          startCommandSequence(incoming);
        }
      } else {
        bool wasAwaitingRestart = autoRestart.required || autoRestart.pending;
        // Revalidar a autorizacao em CADA consulta bem-sucedida ao portal.
        // Nao cancelar uma espera valida apenas por retransmissao do mesmo comando.
        if (autoRestart.required || autoRestart.pending) {
          if (commandMatchesAutoRestart(incoming)) {
            authorizeAutoRestart(incoming);
          } else {
            cancelAutoRestart("portal mudou comando ou desativou religamento");
          }
        }
        if (!wasAwaitingRestart &&
            !(incoming.commandId == cancelledRestartCommandId &&
              normalizeDirectionStr(incoming.direction) != "parado") &&
            commandIsNewOrChanged(incoming, currentCmd)) {
#if PROSERVICE_VERBOSE_SERIAL
          Serial.println("[CMD] Aceito");
#endif
          startCommandSequence(incoming);
        }
      }
    } else {
#if PROSERVICE_VERBOSE_SERIAL
      Serial.println("[CMD] Sem comando novo");
#endif
    }
  }

  processLoRaRx(); // aproveita pacotes recebidos durante HTTP antes de avaliar timeout
  processFastSafetyTrips();
  processUrgentSafetyHeartbeat();
  processAutoRestart();

  // No modo direto, a pagina controla cada relé. No modo de sequência,
  // o controlador normal continua cuidando de pressão, segurança e temporizações.
  if (localTestMode && localDirectMode) {
    applyLocalDirectOutputs();
  } else {
    processController();
    maybeAcceptPendingCommand();
  }

  // Evita que um teste esquecido mantenha o servidor remoto bloqueado.
  if (localTestMode && (millis() - localLastActivityMs >= LOCAL_TEST_TIMEOUT_MS)) {
    Serial.println("[LOCAL] Tempo limite do teste local atingido");
    stopLocalTest();
  }
  processBoardReadyRelay();
  trackCommandAppliedEvent();
  trackLoraCommunicationEvent();
  if (cfg.backendEnabled && !bootGuardActive() && !networkReady()) {
    notePortalCommunicationFailure("Ethernet e Wi-Fi sem conexao; portal indisponivel");
  }
  servicePersistenceRetries();

  esp_task_wdt_reset();
  if (networkReady()) {
    // Aumenta frequencia de heartbeat quando proxima do destino (< 10 graus)
    uint32_t heartbeatIntervalMs = STATUS_POST_INTERVAL_MS;
    if (controllerState == STATE_RUNNING && pivotAngleValid) {
      float stopDegRun = currentCmd.stopDegree.toFloat();
      if (stopDegRun > 0.0f && stopDegRun < 360.0f) {
        float dRun = fabs(normalizeAngle360(pivotAngleDeg) - normalizeAngle360(stopDegRun));
        if (dRun > 180.0f) dRun = 360.0f - dRun;
        if (dRun < 10.0f) heartbeatIntervalMs = 2000UL; // 2s quando proximo do alvo
      }
    }
    bool forceHeartbeat = shouldForceHeartbeatNow();
    uint32_t sinceLastHeartbeatMs = millis() - lastStatusMs;
    if ((forceHeartbeat && sinceLastHeartbeatMs >= HEARTBEAT_FORCE_MIN_INTERVAL_MS) ||
        sinceLastHeartbeatMs >= heartbeatIntervalMs) {
      lastStatusMs = millis();
      postHeartbeatToServer();
    }
  }

  processBoardReadyRelay();
  delay(5);
}
