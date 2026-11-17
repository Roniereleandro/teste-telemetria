/* REVISAO 25.11 */
#include <WiFi.h>
#include <WebServer.h>
#include "src/GpsParserSafe/TinyGPS++.h"
#include <ArduinoJson.h>
#include <Preferences.h>
#include <esp_task_wdt.h>
#include <esp_idf_version.h>
#include <math.h>
#include <esp_random.h>
#include "RadioProtocolPolicy.h"

static const int GPS_RX = 16;
static const int GPS_TX = 17;
static const int LORA_RX = 26;
static const int LORA_TX = 27;
static const int LORA_M0 = 25;
static const int LORA_M1 = 33;
static const int LORA_AUX = 35;
static const uint32_t GPS_BAUD = 9600;
static const uint32_t LORA_BAUD = 9600;
int radioChannel = -1; // -1 = ainda nao lido do modulo.
String radioStatus = "Ainda nao consultado";
String radioRegisters;
uint32_t gpsBytes = 0, gpsLastByteMs = 0, gpsSentences = 0, gpsLastSentenceMs = 0;
uint32_t radioBytes = 0, rejectedId = 0;
String radioLine;
bool discardRadioLine = false;
bool radioConfigured = false;
bool radioProfileCompatible=false;bool radioTxSeen=false;uint32_t radioLastAck=0,radioRejectedAcks=0,radioTxBusySkips=0,radioTxInvalid=0;
uint32_t radioCheckedMs = 0;
uint32_t sampleSequence = 0;
uint32_t bootSession = 0;
bool gpsEpochSeen = false;
uint32_t lastGpsUtc = 0, duplicateGpsEpochs = 0, invalidGpsTimeCount = 0;
static const uint32_t GPS_SAMPLE_MAX_AGE = 15000UL;
static const uint32_t SEND_INTERVAL = 1800UL;
static const uint32_t RESPONSE_WINDOW = 1200UL;
static const uint32_t AUX_TIMEOUT = 2500UL;

const char *DEFAULT_ID = "GPS-PIVO-01";
const char *DEFAULT_AP_PASS = "proservice123";

HardwareSerial GpsSerial(2);
HardwareSerial LoRaSerial(1);
TinyGPSPlus gps;
Preferences prefs;
WebServer server(80);

String nodeId;
String apSsid;
String activeApSsid;
bool apRenamePending = false;
uint32_t apRenameScheduledMs = 0;
String apMessage;

bool validApSsid(const String &name) {
  if (name.length() < 1 || name.length() > 32) return false;
  for (size_t i=0; i<name.length(); ++i) {
    if ((uint8_t)name[i] < 32 || (uint8_t)name[i] > 126) return false;
  }
  return true;
}

String defaultApSsid() {
  uint64_t mac = ESP.getEfuseMac();
  char suffix[7];
  snprintf(suffix,sizeof(suffix),"%06lX",(unsigned long)((mac >> 24) & 0xFFFFFF));
  return "GPS-PONTA-" + String(suffix);
}
uint32_t sequenceNo = 0;
uint32_t lastSendMs = 0;
uint32_t lastNewGpsMs = 0;
uint32_t lastValidFixMs = 0;
uint32_t lastRxMs = 0;
uint32_t txCount = 0;
uint32_t rxCount = 0;
uint32_t invalidGpsCount = 0;
uint32_t invalidRxCount = 0;
double lastLat = 0.0, lastLon = 0.0;
float lastHdop = 99.9f;
uint32_t lastSat = 0;
bool lastGpsValid = false;
bool radioBusy = false;
String lastRxLine;

bool finiteCoord(double lat, double lon) {
  return isfinite(lat) && isfinite(lon) && lat >= -90.0 && lat <= 90.0 && lon >= -180.0 && lon <= 180.0 && !(lat == 0.0 && lon == 0.0);
}

bool freshGps() {
  return lastGpsValid && lastNewGpsMs != 0 && millis() - lastNewGpsMs <= GPS_SAMPLE_MAX_AGE;
}

bool auxReady() {
  return digitalRead(LORA_AUX) == HIGH;
}

void normalMode() {
  digitalWrite(LORA_M0, LOW);
  digitalWrite(LORA_M1, LOW);
  delay(20);
}

void allPinsSafe() {
  pinMode(LORA_M0, OUTPUT);
  pinMode(LORA_M1, OUTPUT);
  digitalWrite(LORA_M0, LOW);
  digitalWrite(LORA_M1, LOW);
  pinMode(LORA_AUX, INPUT);
}

void readGps() {
  // Consome cada sentenca separadamente para nao atribuir idade nova a um FIX antigo.
  for (int n = 0; n < 512 && GpsSerial.available(); ++n) {
    char c = (char)GpsSerial.read();
    ++gpsBytes;
    gpsLastByteMs = millis();
    if (!gps.encode(c)) continue;
    ++gpsSentences;
    gpsLastSentenceMs = millis();
    lastSat = gps.satellites.isValid() && gps.satellites.age() <= 15000UL ? gps.satellites.value() : 0;
    lastHdop = gps.hdop.isValid() && gps.hdop.age() <= 15000UL ? (float)gps.hdop.hdop() : 99.9f;
    if (!gps.location.isUpdated()) continue;
    double lat = gps.location.lat(), lon = gps.location.lng();
    if (!gps.location.isValid() || !finiteCoord(lat, lon)) { ++invalidGpsCount; continue; }
    // A validade da coordenada nao depende do horario UTC. Alguns NEO-6M
    // entregam FIX valido antes de estabilizar a hora; nao descartar esse FIX.
    if (!gps.time.isValid() || gps.time.age() > 15000UL) ++invalidGpsTimeCount;
    else {
      uint32_t utc = gps.time.value();
      if (gpsEpochSeen && utc == lastGpsUtc) {
        ++duplicateGpsEpochs;
        continue; // mesma época GNSS: não renova idade nem sample
      }
      gpsEpochSeen = true; lastGpsUtc = utc;
    }
    lastLat = lat; lastLon = lon;
    lastNewGpsMs = millis(); lastValidFixMs = lastNewGpsMs;
    lastGpsValid = true;
    if (++sampleSequence == 0) ++sampleSequence;
  }
}

void serviceGpsDelay(uint32_t duration) {
  uint32_t start = millis();
  while (millis() - start < duration) { readGps(); delay(2); }
}

bool waitAux(uint32_t timeout) {
  uint32_t start = millis();
  while (millis() - start < timeout) {
    if (auxReady()) { serviceGpsDelay(3); if (auxReady()) return true; }
    readGps(); delay(2);
  }
  return false;
}

bool readRegisters(uint8_t regs[6]) {
  for (int attempt = 0; attempt < 3; ++attempt) {
    if (!waitAux(1500)) { radioStatus = "AUX LOW antes da leitura"; return false; }
    for (int i=0; i<2048 && LoRaSerial.available(); ++i) LoRaSerial.read();
    const uint8_t command[] = {0xC1, 0x00, 0x06};
    LoRaSerial.write(command, sizeof(command)); LoRaSerial.flush();
    uint8_t response[9]; size_t count = 0, total = 0;
    uint32_t start = millis();
    while (millis() - start < 1000UL) {
      readGps();
      if (!LoRaSerial.available()) { delay(2); continue; }
      uint8_t b = (uint8_t)LoRaSerial.read(); ++total;
      if (count == sizeof(response)) { memmove(response,response+1,sizeof(response)-1); --count; }
      response[count++] = b;
      if (count == sizeof(response) && response[0]==0xC1 && response[1]==0 && response[2]==6) {
        memcpy(regs,response+3,6); return true;
      }
    }
    Serial.printf("[E220] Leitura %d: %u bytes sem resposta valida\n",attempt+1,(unsigned)total);
  }
  radioStatus = "Sem resposta valida do E220 a 9600 8N1; confira Monitor Serial";
  return false;
}

bool configureRadio(int requestedChannel) {
  radioBusy = true;
  lastRxMs=0; rxCount=0; lastRxLine=""; // Nova janela de recepcao apos consulta do radio.
  radioConfigured = false; radioProfileCompatible=false; radioLastAck=0; radioChannel = -1; radioRegisters = "";
  bool ok = false;
  if (!waitAux(2500)) radioStatus = "AUX LOW: radio ocupado ou sinal indisponivel";
  else {
    digitalWrite(LORA_M0,HIGH); digitalWrite(LORA_M1,HIGH);
    serviceGpsDelay(120);
    uint8_t regs[6];
    if (readRegisters(regs)) {
      ok = true;
      if (requestedChannel >= 0 && regs[4] != requestedChannel) {
        ok = waitAux(1500);
        if (ok) {
          // Somente CHAN; nao sobrescreve os registros de criptografia.
          uint8_t command[] = {0xC0,0x04,0x01,(uint8_t)requestedChannel};
          LoRaSerial.write(command,sizeof(command)); LoRaSerial.flush();
          serviceGpsDelay(200);
          ok = readRegisters(regs) && regs[4] == requestedChannel;
        }
      }
      if (ok && regs[4] <= 80) {
        radioChannel = regs[4]; radioCheckedMs = millis();
        char hex[4];
        for (int i=0;i<6;++i) { snprintf(hex,sizeof(hex),"%02X ",regs[i]); radioRegisters += hex; }
        radioConfigured = true;
        radioProfileCompatible=(regs[2]&0xF8)==0x60&&(regs[5]&0xC0)==0;
        radioStatus = "Canal lido e confirmado no modulo";
        if ((regs[2] & 0xF8) != 0x60) radioStatus += "; UART normal diferente de 9600 8N1";
        if (regs[5] & 0x40) radioStatus += "; modo fixo: incompativel com envio transparente deste sketch";
        if (regs[5] & 0x80) radioStatus += "; RSSI anexado pode interferir no JSON";
      } else { ok = false; radioStatus = "Leitura/gravação do canal nao confirmada"; }
    }
  }
  normalMode(); serviceGpsDelay(120);
  if (!waitAux(2500)) { ok=false; radioProfileCompatible=false; radioStatus += "; AUX LOW ao voltar ao modo normal"; }
  for (int i=0;i<2048 && LoRaSerial.available();++i) LoRaSerial.read();
  radioLine = ""; discardRadioLine = false;
  radioBusy = false; lastSendMs = millis();
  Serial.println("[E220] " + radioStatus);
  return ok;
}

void sendGps() {
  const uint32_t now = millis();
  if (!RadioProtocolPolicy::txDue(now,lastSendMs,radioTxSeen,bootSession^sequenceNo)) return;
  if (radioBusy || !auxReady()) {++radioTxBusySkips;return;}
  if(!radioConfigured||!radioProfileCompatible||radioChannel<0){++radioTxInvalid;lastSendMs=now;return;}

  StaticJsonDocument<768> doc;
  doc["t"] = "g";
  doc["id"] = nodeId;
  if (++sequenceNo == 0) ++sequenceNo; // zero e reservado pelo protocolo
  doc["seq"] = sequenceNo;
  doc["ok"] = freshGps() ? 1 : 0;
  doc["la"] = lastGpsValid ? lastLat : 0.0;
  doc["lo"] = lastGpsValid ? lastLon : 0.0;
  doc["sa"] = lastSat;
  doc["hd"] = lastHdop;
  doc["age"] = lastGpsValid && lastNewGpsMs ? (now - lastNewGpsMs) : 0;
  if (radioChannel >= 0) doc["ch"] = radioChannel;
  doc["sample"] = sampleSequence;
  doc["boot"] = bootSession;
  String out;
  serializeJson(doc, out);
  if(doc.overflowed()||out.length()>350){++radioTxInvalid;lastSendMs=now;return;}
  LoRaSerial.print(out);
  LoRaSerial.print('\n');
  LoRaSerial.flush();
  radioTxSeen=true;lastSendMs=millis();
  txCount++;
}

void handleRadioLine(const String &line) {
  if (line.length() == 0 || line.length() > 500) return;
  StaticJsonDocument<512> doc;
  DeserializationError err = deserializeJson(doc, line);
  if (err) { invalidRxCount++; return; }
  const char *type = doc["t"] | "";
  if (strcmp(type, "c") != 0 && strcmp(type, "p") != 0) return;
  const char *dest = doc["id"] | "";
  if (!dest[0] || nodeId != String(dest)) { ++rejectedId; return; }
  // Uma resposta só comprova o enlace deste conjunto quando a central informa
  // o mesmo canal real que foi lido no E220 do GPS.
  if (radioChannel < 0 || !doc["ch"].is<int>() ||
      doc["ch"].as<int>() != radioChannel) {
    ++invalidRxCount;
    return;
  }
  if(!doc["boot"].is<uint32_t>()||!doc["ack"].is<uint32_t>()||
      !RadioProtocolPolicy::validAck(doc["boot"].as<uint32_t>(),bootSession,doc["ack"].as<uint32_t>(),sequenceNo,radioLastAck)) {
    ++radioRejectedAcks;return;
  }
  radioLastAck=doc["ack"].as<uint32_t>();
  lastRxLine = line;
  lastRxMs = millis();
  rxCount++;

}

void readRadio() {
  if (radioBusy) return;
  for (int n=0;n<1024 && LoRaSerial.available();++n) {
    char c = (char)LoRaSerial.read(); ++radioBytes;
    if (c == '\n' || c == '\r') {
      if (!discardRadioLine && radioLine.length()) handleRadioLine(radioLine);
      radioLine = ""; discardRadioLine = false;
    } else if (!discardRadioLine) {
      if ((uint8_t)c < 32 || (uint8_t)c > 126 || radioLine.length() >= 500) {
        radioLine = ""; discardRadioLine = true; ++invalidRxCount;
      } else radioLine += c;
    }
  }
}

String statusJson() {
  StaticJsonDocument<2048> doc;
  doc["firmware"] = "PROSERVICE GPS REV25.11";
  doc["id"] = nodeId;
  doc["apSsid"] = activeApSsid;
  doc["apSavedSsid"] = apSsid;
  doc["apMessage"] = apMessage;
  doc["gpsValid"] = lastGpsValid;
  doc["gpsFresh"] = freshGps();
  doc["duplicateGpsEpochs"] = duplicateGpsEpochs;
  doc["invalidGpsTimeCount"] = invalidGpsTimeCount;
  doc["sample"] = sampleSequence;
  doc["boot"] = bootSession;
  doc["lat"] = lastGpsValid ? lastLat : 0.0;
  doc["lon"] = lastGpsValid ? lastLon : 0.0;
  doc["hdop"] = lastHdop;
  doc["satellites"] = lastSat;
  doc["sampleAgeMs"] = lastGpsValid ? millis() - lastNewGpsMs : 0;
  doc["aux"] = auxReady();
  if (radioChannel >= 0) doc["channel"] = radioChannel;
  else doc["channel"] = nullptr;
  doc["radioStatus"] = radioStatus;
  doc["radioRegisters"] = radioRegisters;
  doc["radioRead"] = radioConfigured;
  doc["radioProfileCompatible"]=radioProfileCompatible;
  doc["radioLastAck"]=radioLastAck;
  doc["radioRejectedAcks"]=radioRejectedAcks;
  doc["radioTxBusySkips"]=radioTxBusySkips;
  doc["radioTxInvalid"]=radioTxInvalid;
  doc["bootSession"]=bootSession;
  doc["lastTxSequence"]=sequenceNo;
  doc["radioCheckAgeMs"] = radioConfigured ? millis()-radioCheckedMs : 0;
  doc["gpsBytes"] = gpsBytes;
  doc["gpsSentences"] = gpsSentences;
  doc["gpsChecksumErrors"] = gps.failedChecksum();
  doc["gpsByteAgeMs"] = gpsBytes ? millis()-gpsLastByteMs : 0;
  doc["radioBytes"] = radioBytes;
  doc["rejectedId"] = rejectedId;
  doc["linkRecent"] = rxCount > 0 && millis()-lastRxMs < 30000UL;
  doc["rxAgeMs"] = rxCount ? millis()-lastRxMs : 0;
  doc["gpsState"] = !gpsBytes ? "Nenhum byte recebido do NEO-6M" :
    millis()-gpsLastByteMs > 5000UL ? "Serial do NEO-6M sem dados recentes" :
    !gpsSentences || millis()-gpsLastSentenceMs > 5000UL ? "Bytes recebidos, sem sentenca NMEA valida recente" :
    freshGps() ? "Posicao valida e recente" : "NMEA recebido; sem posicao valida recente";
  doc["tx"] = txCount;
  doc["rx"] = rxCount;
  doc["invalidGps"] = invalidGpsCount;
  doc["invalidRx"] = invalidRxCount;
  String out; serializeJson(doc, out); return out;
}

const char PAGE[] PROGMEM = R"HTML(<!doctype html><html lang="pt-BR"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>PROSERVICE GPS REV25.11</title><style>
*{box-sizing:border-box}body{margin:0;background:#edf2f5;color:#193447;font:16px system-ui,sans-serif}main{max-width:920px;margin:auto;padding:24px}header{margin-bottom:22px}h1{font-size:28px;margin:8px 0}h2{font-size:19px;margin:0 0 15px}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(270px,1fr));gap:16px}section{background:white;padding:20px;border-radius:12px;box-shadow:0 2px 8px #17334512;margin-bottom:16px}dl{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-bottom:0}dt{color:#516777}dd{margin:0;text-align:right;overflow-wrap:anywhere}.status{padding:12px;background:#e7f1f7;border-radius:7px}.muted{color:#526a78;font-size:14px;line-height:1.5}label{display:block;margin:12px 0 6px}input,button{font:inherit;padding:11px;border:1px solid #b7c7d1;border-radius:6px;max-width:100%}button{background:#145776;color:white;border:0;cursor:pointer;margin-top:10px}button:disabled{opacity:.5}input{width:100%}#result{white-space:pre-wrap}code{overflow-wrap:anywhere}footer{font-size:13px;color:#526a78}</style></head><body><main>
<header><b>PROSERVICE · TELEMETRIA</b><h1>GPS da ponta do pivô</h1><div>REV25.11 · NEO-6M + E220 · <b id="node">...</b></div><p id="connection" class="muted">Carregando diagnóstico...</p></header>
<section><h2>Rádio da PONTA - canal real</h2><h1 id="actualChannel">NÃO CONFIRMADO</h1><p id="channelAge" class="muted">Aguardando consulta</p><p class="muted">Leitura do registro na partida ou pelo botão Ler configuração. O campo de gravação abaixo escolhe um novo canal.</p></section><div class="grid"><section><h2>Posição GPS</h2><p id="gpsState" class="status">Aguardando leitura</p><dl><dt>Latitude</dt><dd id="lat">—</dd><dt>Longitude</dt><dd id="lon">—</dd><dt>Idade da posição</dt><dd id="age">—</dd><dt>Satélites informados</dt><dd id="sat">—</dd><dt>HDOP informado</dt><dd id="hdop">—</dd></dl><p class="muted">Sem posição recente, coordenadas antigas não significam GPS ativo. Satélites e HDOP são os últimos valores recebidos.</p></section>
<section><h2>Comunicação com a central</h2><p id="link" class="status">Sem resposta registrada</p><dl><dt>Envios pela serial</dt><dd id="tx">0</dd><dt>Respostas desde a consulta do rádio</dt><dd id="rx">0</dd><dt>Idade da resposta</dt><dd id="rxAge">—</dd><dt>AUX do E220</dt><dd id="aux">—</dd><dt>Canal REAL do E220 da PONTA</dt><dd id="channel">Não verificado</dd></dl><p class="muted">Envio não confirma recebimento pela central. ID e parâmetros de rádio precisam corresponder ao conjunto.</p></section></div>
<section><h2>Diagnóstico das seriais</h2><dl><dt>Bytes do NEO-6M</dt><dd id="bytes">0</dd><dt>Sentenças NMEA válidas</dt><dd id="sentences">0</dd><dt>Erros de checksum NMEA</dt><dd id="checksum">0</dd><dt>Bytes recebidos do LoRa</dt><dd id="radioBytes">0</dd><dt>Erros de recepção LoRa</dt><dd id="errors">0</dd><dt>Mensagens com outro ID/sem ID</dt><dd id="rejected">0</dd></dl><p class="muted">Sem bytes: confira alimentação, TX do NEO-6M para GPIO16 e GND. Com NMEA válido e sem posição: verifique a antena e teste com céu aberto. Contadores acumulam desde a inicialização.</p></section>
<section><h2>Nome da rede Wi-Fi deste GPS</h2><p>Rede ativa: <b id="apCurrent">...</b></p><p id="apMessage" class="muted"></p><form id="wifiForm"><label for="wifiName">Nome para aparecer no celular</label><input id="wifiName" name="ssid" maxlength="32" required placeholder="GPS-P014-PONTA"><p class="muted">Use de 1 a 32 caracteres, sem acentos. Ao salvar, o Wi-Fi será reiniciado após 3 segundos. Reconecte ao novo nome e abra 192.168.4.1. A senha continua a mesma. Este nome não altera o ID LoRa nem o canal.</p><button>Salvar nome e aplicar Wi-Fi</button></form><p id="wifiResult" role="status"></p></section><section><h2>Configuração do conjunto</h2><p id="radioStatus" class="status">Rádio ainda não consultado</p><p class="muted">Registros 00 a 05: <code id="regs">—</code>. A chave de rádio não é exibida nem alterada.</p><button onclick="action('/radio/read')">Ler configuração do E220</button><form id="idForm"><label for="gpsId">ID do GPS autorizado na central</label><input id="gpsId" name="id" minlength="3" maxlength="32" pattern="[A-Za-z0-9_-]{3,32}" required placeholder="GPS-PIVO-01"><button>Salvar ID</button></form><form id="channelForm"><label for="radioChannel">Novo canal desejado (0 a 80; não é o canal lido)</label><input id="radioChannel" name="channel" type="number" min="0" max="80" step="1" required><p class="muted">Altere em bancada, com o pivô parado. A mudança interrompe o enlace até a central usar o mesmo canal. Não alteramos potência, endereço, velocidade ou chave.</p><button>Gravar e conferir canal</button></form><p id="result" role="status"></p></section>
<section><h2>Ligações desta versão</h2><p>NEO-6M TX → GPIO16; RX → GPIO17.<br>E220 TXD → GPIO26; RXD → GPIO27; M0 → GPIO25; M1 → GPIO33; AUX → GPIO35.<br>GND comum. Pinagem mantida da REV24.</p></section><footer>Diagnóstico local · Atualização a cada 2 segundos · Dados JSON em /status</footer></main><script>
let busy=false;const el=id=>document.getElementById(id);const put=(id,v)=>el(id).textContent=v;const seconds=ms=>(ms/1000).toFixed(1)+' s';
async/*JS*/ function refresh(){if(busy)return;try{const r=await fetch('/status',{cache:'no-store'});if(!r.ok)throw Error();const d=await r.json();put('connection','Página conectada · '+new Date().toLocaleTimeString());put('node',d.id);put('apCurrent',d.apSsid||'Indisponível');put('apMessage',d.apMessage||'');if(!el('wifiName').dataset.loaded){el('wifiName').value=d.apSavedSsid;el('wifiName').dataset.loaded=1;}put('gpsState',d.gpsState);put('lat',d.gpsValid?Number(d.lat).toFixed(6):'—');put('lon',d.gpsValid?Number(d.lon).toFixed(6):'—');put('age',d.gpsValid?seconds(d.sampleAgeMs):'Sem amostra');put('sat',d.satellites);put('hdop',d.hdop);put('link',d.linkRecent?'Resposta recente para este ID':d.rx?'Sem resposta recente':'Nenhuma resposta para este ID');put('tx',d.tx);put('rx',d.rx);put('rxAge',d.rx?seconds(d.rxAgeMs):'—');put('aux',d.aux?'HIGH / pronto':'LOW / ocupado ou indisponível');put('channel',d.channel===null?'Não verificado':d.channel);put('actualChannel',d.channel===null?'CANAL NÃO CONFIRMADO':'CANAL '+d.channel);put('channelAge',d.channel===null?'Sem leitura válida do registro':'Registro lido há '+seconds(d.radioCheckAgeMs));put('bytes',d.gpsBytes);put('sentences',d.gpsSentences);put('checksum',d.gpsChecksumErrors);put('radioBytes',d.radioBytes);put('errors',d.invalidRx);put('rejected',d.rejectedId);put('radioStatus',d.radioStatus);put('regs',d.radioRegisters||'—');if(!el('gpsId').dataset.loaded){el('gpsId').value=d.id;el('gpsId').dataset.loaded=1}}catch(e){put('connection','SEM ATUALIZAÇÃO: todos os valores abaixo são da última consulta.');put('actualChannel','SEM ATUALIZAÇÃO');put('channelAge','Reconecte ao Wi-Fi da ponta para atualizar.');put('gpsState','POSIÇÃO ATUAL NÃO AVALIADA');put('link','LINK ATUAL NÃO AVALIADO');put('age','Dado antigo');put('rxAge','Dado antigo')}}
async/*JS*/ function action(url,data){if(busy)return;busy=true;document.querySelectorAll('button').forEach(b=>b.disabled=true);put('result','Aguarde a consulta ao equipamento...');try{const r=await fetch(url,{method:'POST',body:data});put('result',await r.text())}catch(e){put('result','Resposta não recebida. Consulte o rádio antes de repetir a gravação.')}finally{busy=false;document.querySelectorAll('button').forEach(b=>b.disabled=false);refresh()}}
el('wifiForm').onsubmit=async e=>{e.preventDefault();if(busy)return;busy=true;const b=e.target.querySelector('button');b.disabled=true;try{const r=await fetch('/wifi/name',{method:'POST',body:new URLSearchParams(new FormData(e.target))});put('wifiResult',await r.text())}catch(err){put('wifiResult','Resposta não recebida. Confira se o novo nome apareceu na lista de redes.')}finally{busy=false;b.disabled=false}};el('idForm').onsubmit=e=>{e.preventDefault();action('/id',new URLSearchParams(new FormData(e.target)))};el('channelForm').onsubmit=e=>{e.preventDefault();if(confirm('Pivô parado? Alterar o canal interrompe a comunicação com a central.'))action('/radio/channel',new URLSearchParams(new FormData(e.target)))};refresh();setInterval(refresh,2000);
</script></body></html>)HTML";

bool validNodeId(const String &id) {
  if (id.length()<3 || id.length()>32) return false;
  for (size_t i=0;i<id.length();++i) {
    char c=id[i];
    if (!((c>='A'&&c<='Z')||(c>='a'&&c<='z')||(c>='0'&&c<='9')||c=='-'||c=='_')) return false;
  }
  return true;
}

void setupWeb() {
  WiFi.mode(WIFI_AP);
  String ap = apSsid;
  bool apOk = WiFi.softAP(ap.c_str(), DEFAULT_AP_PASS);
  activeApSsid = apOk ? ap : String("");
  apMessage = apOk ? "Rede ativa" : "Falha ao iniciar Wi-Fi";
  Serial.printf("[AP] %s | %s | IP=%s\n",ap.c_str(),apOk?"ATIVO":"FALHA",WiFi.softAPIP().toString().c_str());
  server.on("/", HTTP_GET, [](){ server.send_P(200,"text/html; charset=utf-8",PAGE); });
  server.on("/status", HTTP_GET, [](){ server.sendHeader("Cache-Control","no-store"); server.send(200,"application/json",statusJson()); });
  server.on("/radio/read", HTTP_POST, [](){
    bool ok=configureRadio(-1); server.send(ok?200:503,"text/plain; charset=utf-8",radioStatus);
  });
  server.on("/radio/channel", HTTP_POST, [](){
    String value=server.arg("channel");
    bool valid=value.length()>0 && value.length()<=2;
    for (size_t i=0;i<value.length();++i) if(value[i]<'0'||value[i]>'9') valid=false;
    int channel=value.toInt();
    if(!valid || channel>80){server.send(400,"text/plain; charset=utf-8","Canal inválido: use um inteiro de 0 a 80.");return;}
    bool ok=configureRadio(channel);server.send(ok?200:503,"text/plain; charset=utf-8",radioStatus);
  });
  server.on("/wifi/name", HTTP_POST, [](){
    String name=server.arg("ssid"); name.trim();
    if(!validApSsid(name)) { server.send(400,"text/plain; charset=utf-8","Use 1 a 32 caracteres sem acentos."); return; }
    if(prefs.putString("ap_ssid",name)==0) { server.send(500,"text/plain; charset=utf-8","Falha ao salvar. Nome não alterado."); return; }
    apSsid=name;
    apRenamePending=true; apRenameScheduledMs=millis();
    apMessage="Nome salvo; aguardando aplicar";
    server.send(200,"text/plain; charset=utf-8","Nome salvo: " + name + ". Em 3 segundos o Wi-Fi será reiniciado. Reconecte a essa rede com a mesma senha e abra http://192.168.4.1.");
  });
  server.on("/id", HTTP_POST, [](){
    String id=server.arg("id");id.trim();
    if(!validNodeId(id)){server.send(400,"text/plain; charset=utf-8","ID: 3 a 32 letras, números, hífen ou sublinhado.");return;}
    if(prefs.putString("id",id)==0){server.send(500,"text/plain; charset=utf-8","Não foi possível salvar o ID.");return;}
    nodeId=id;rxCount=0;lastRxMs=0;lastRxLine="";
    server.send(200,"text/plain; charset=utf-8","ID salvo. Configure o mesmo ID autorizado na central. O nome do Wi-Fi é independente e não foi alterado.");
  });
  server.begin();
}

void setup() {
  Serial.begin(115200);
  do { bootSession = esp_random(); } while (bootSession == 0);
  prefs.begin("gpscfg", false);
  nodeId = prefs.getString("id", DEFAULT_ID);
  if (!validNodeId(nodeId)) nodeId = DEFAULT_ID;
  apSsid = prefs.getString("ap_ssid", "");
  if (!validApSsid(apSsid)) apSsid = defaultApSsid();
  allPinsSafe();
  GpsSerial.setRxBufferSize(2048);
  GpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
  LoRaSerial.setRxBufferSize(2048);
  LoRaSerial.begin(LORA_BAUD, SERIAL_8N1, LORA_RX, LORA_TX);
  normalMode();
  configureRadio(-1); // Leitura somente, nao altera canal ao iniciar.
  setupWeb();
#if ESP_IDF_VERSION_MAJOR >= 5
  esp_task_wdt_config_t wdtConfig = {};
  wdtConfig.timeout_ms = 30000;
  wdtConfig.idle_core_mask = 0;
  wdtConfig.trigger_panic = true;
  esp_err_t wdtResult = esp_task_wdt_init(&wdtConfig);
  if (wdtResult == ESP_ERR_INVALID_STATE) esp_task_wdt_reconfigure(&wdtConfig);
#else
  esp_task_wdt_init(30, true);
#endif
  esp_task_wdt_add(NULL);
  Serial.printf("PROSERVICE GPS REV25.11 | ID=%s | GPS RX/TX=%d/%d | LoRa RX/TX=%d/%d | M0/M1/AUX=%d/%d/%d\n", nodeId.c_str(), GPS_RX, GPS_TX, LORA_RX, LORA_TX, LORA_M0, LORA_M1, LORA_AUX);
}

void processApRename() {
  if (!apRenamePending || millis()-apRenameScheduledMs < 3000UL) return;
  apRenamePending=false;
  String previous=activeApSsid;
  WiFi.softAPdisconnect(false);
  bool ok=WiFi.softAP(apSsid.c_str(),DEFAULT_AP_PASS);
  if(ok) {
    activeApSsid=apSsid; apMessage="Novo nome aplicado";
  } else {
    apMessage="Falha ao aplicar o nome salvo; tente novamente";
    bool restored=previous.length() && WiFi.softAP(previous.c_str(),DEFAULT_AP_PASS);
    activeApSsid=restored ? previous : String("");
  }
  Serial.println("[AP] " + apMessage + " | ativa=" + activeApSsid);
}

void loop() {
  esp_task_wdt_reset();
  server.handleClient();
  processApRename();
  readGps();
  readRadio();
  sendGps();
  delay(2);
}
