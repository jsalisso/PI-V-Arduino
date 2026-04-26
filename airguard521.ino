#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Preferences.h>

#include "config.h"
#include "secrets.h"

// =====================================================
// CONFIG OLED
// =====================================================

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// =====================================================
// PINAGEM
// =====================================================

const int pinoMosfetMQ7 = 18;
const int pinoBuzzer    = 23;
const int pinoPresenca  = 27;
const int pinoMQ7       = 34;
const int pinoMQ4       = 32;

// =====================================================
// CALIBRAÇÃO & COMPENSAÇÃO
// =====================================================

float r0_mq7;
float r0_mq4;

const float COEF_TEMP  = -0.0122;
const float COEF_UMID  = -0.00609;
const float COEF_CONST = 1.7086;

// =====================================================
// OBJETOS GLOBAIS
// =====================================================

Adafruit_BME280 bme;
WiFiClientSecure espClient;
PubSubClient client(espClient);
Preferences preferences;

// =====================================================
// CONTROLE DE FASES
// =====================================================

unsigned long tempoInicioFase = 0;
int faseAtual = 1; // 1 = limpeza / 2 = medição

// =====================================================
// TÓPICOS MQTT
// =====================================================

String baseTopic = String("v1/") + ID_CLIENTE + "/" + UNIDADE + "/" + AMBIENTE + "/" + ID_SENSOR;
String cmdTopic  = baseTopic + "/cmd";
String resTopic  = baseTopic + "/res";

// =====================================================
// CONFIGURAÇÃO DO AIRGUARD
// =====================================================

struct ThresholdConfig {
  float co_alerta;
  float co_perigo;
  float gas_alerta;
  float gas_perigo;
};

struct ProfileConfig {
  String nomePerfil;
  bool buzzerHabilitado;
};

struct AirGuardConfig {
  ThresholdConfig thresholds;
  ProfileConfig profile;
};

AirGuardConfig cfg;

enum EstadoAirGuard {
  ESTADO_LIMPO,
  ESTADO_ALERTA,
  ESTADO_PERIGO
};

EstadoAirGuard estadoAtual = ESTADO_LIMPO;
EstadoAirGuard estadoCandidato = ESTADO_LIMPO;

unsigned long inicioCandidatura = 0;

// tempos de persistência
const unsigned long TEMPO_CONFIRMACAO_ALERTA_MS = 5000;
const unsigned long TEMPO_CONFIRMACAO_PERIGO_MS = 3000;
const unsigned long TEMPO_CONFIRMACAO_LIMPO_MS  = 8000;

// histerese
const float HISTERESE_CO  = 1.0;
const float HISTERESE_GAS = 1.0;

// =====================================================
// FUNÇÕES AUXILIARES
// =====================================================

float calcularFatorCompensacao(float temperatura, float umidade) {
  return (COEF_TEMP * temperatura) + (COEF_UMID * umidade) + COEF_CONST;
}

float calcularRS(float tensaoSensor) {
  float v = (tensaoSensor < 0.1f) ? 0.1f : tensaoSensor;
  return ((5.0f / v) - 1.0f) * RL_VALOR;
}

void publicarResposta(const String& mensagem) {
  client.publish(resTopic.c_str(), mensagem.c_str());
}

String estadoParaString(EstadoAirGuard estado) {
  switch (estado) {
    case ESTADO_LIMPO:  return "LIMPO";
    case ESTADO_ALERTA: return "ALERTA";
    case ESTADO_PERIGO: return "PERIGO";
    default:            return "LIMPO";
  }
}

EstadoAirGuard calcularEstadoDesejado(float co_ppm, float gas_ppm) {
  // Regras para entrar em PERIGO
  bool entrarPerigo =
    (co_ppm > cfg.thresholds.co_perigo) ||
    (gas_ppm > cfg.thresholds.gas_perigo);

  // Regras para sair de PERIGO com histerese
  bool sairPerigo =
    (co_ppm < (cfg.thresholds.co_perigo - HISTERESE_CO)) &&
    (gas_ppm < (cfg.thresholds.gas_perigo - HISTERESE_GAS));

  // Regras para entrar em ALERTA
  bool entrarAlerta =
    (co_ppm > cfg.thresholds.co_alerta) ||
    (gas_ppm > cfg.thresholds.gas_alerta);

  // Regras para sair de ALERTA com histerese
  bool sairAlerta =
    (co_ppm < (cfg.thresholds.co_alerta - HISTERESE_CO)) &&
    (gas_ppm < (cfg.thresholds.gas_alerta - HISTERESE_GAS));

  switch (estadoAtual) {
    case ESTADO_LIMPO:
      if (entrarPerigo) return ESTADO_PERIGO;
      if (entrarAlerta) return ESTADO_ALERTA;
      return ESTADO_LIMPO;

    case ESTADO_ALERTA:
      if (entrarPerigo) return ESTADO_PERIGO;
      if (sairAlerta)   return ESTADO_LIMPO;
      return ESTADO_ALERTA;

    case ESTADO_PERIGO:
      if (!sairPerigo) return ESTADO_PERIGO;
      if (entrarAlerta) return ESTADO_ALERTA;
      return ESTADO_LIMPO;

    default:
      return ESTADO_LIMPO;
  }
}

void atualizarEstadoComPersistencia(float co_ppm, float gas_ppm) {
  EstadoAirGuard desejado = calcularEstadoDesejado(co_ppm, gas_ppm);
  unsigned long agora = millis();

  if (desejado == estadoAtual) {
    estadoCandidato = estadoAtual;
    inicioCandidatura = 0;
    return;
  }

  if (desejado != estadoCandidato) {
    estadoCandidato = desejado;
    inicioCandidatura = agora;
    return;
  }

  unsigned long tempoNecessario = 0;

  if (desejado == ESTADO_ALERTA) {
    tempoNecessario = TEMPO_CONFIRMACAO_ALERTA_MS;
  } else if (desejado == ESTADO_PERIGO) {
    tempoNecessario = TEMPO_CONFIRMACAO_PERIGO_MS;
  } else {
    tempoNecessario = TEMPO_CONFIRMACAO_LIMPO_MS;
  }

  if ((agora - inicioCandidatura) >= tempoNecessario) {
    estadoAtual = desejado;
    estadoCandidato = desejado;
    inicioCandidatura = 0;

    Serial.println(">>> Estado confirmado: " + estadoParaString(estadoAtual));
    publicarResposta("Estado alterado para: " + estadoParaString(estadoAtual));
  }
}

// =====================================================
// PERFIS PADRÃO
// =====================================================

void aplicarPerfilCoworking() {
  cfg.profile.nomePerfil = "COWORKING";
  cfg.profile.buzzerHabilitado = false;

  cfg.thresholds.co_alerta  = 5.0;
  cfg.thresholds.co_perigo  = 15.0;
  cfg.thresholds.gas_alerta = 3.0;
  cfg.thresholds.gas_perigo = 8.0;
}

void aplicarPerfilCozinha() {
  cfg.profile.nomePerfil = "COZINHA";
  cfg.profile.buzzerHabilitado = true;

  cfg.thresholds.co_alerta  = 5.0;
  cfg.thresholds.co_perigo  = 15.0;
  cfg.thresholds.gas_alerta = 5.0;
  cfg.thresholds.gas_perigo = 10.0;
}

void aplicarPerfilAreaServico() {
  cfg.profile.nomePerfil = "AREA_SERVICO";
  cfg.profile.buzzerHabilitado = true;

  cfg.thresholds.co_alerta  = 5.0;
  cfg.thresholds.co_perigo  = 12.0;
  cfg.thresholds.gas_alerta = 4.0;
  cfg.thresholds.gas_perigo = 8.0;
}

// =====================================================
// PERSISTÊNCIA - R0
// =====================================================

void carregarR0sDaFlash() {
  preferences.begin("calib", true);
  r0_mq7 = preferences.getFloat("r0_mq7", R0_MQ7_INICIAL);
  r0_mq4 = preferences.getFloat("r0_mq4", R0_MQ4_INICIAL);
  preferences.end();

  Serial.println(">>> R0 MQ-7 carregado: " + String(r0_mq7, 3));
  Serial.println(">>> R0 MQ-4 carregado: " + String(r0_mq4, 3));
}

void salvarR0MQ7naFlash(float valor) {
  preferences.begin("calib", false);
  preferences.putFloat("r0_mq7", valor);
  preferences.end();

  r0_mq7 = valor;
  Serial.println(">>> R0 MQ-7 salvo: " + String(valor, 3));
}

void salvarR0MQ4naFlash(float valor) {
  preferences.begin("calib", false);
  preferences.putFloat("r0_mq4", valor);
  preferences.end();

  r0_mq4 = valor;
  Serial.println(">>> R0 MQ-4 salvo: " + String(valor, 3));
}

// =====================================================
// PERSISTÊNCIA - CONFIG
// =====================================================

void salvarConfig() {
  preferences.begin("config", false);

  preferences.putString("profile", cfg.profile.nomePerfil);
  preferences.putBool("buzzer", cfg.profile.buzzerHabilitado);

  preferences.putFloat("co_alerta", cfg.thresholds.co_alerta);
  preferences.putFloat("co_perigo", cfg.thresholds.co_perigo);
  preferences.putFloat("gas_alerta", cfg.thresholds.gas_alerta);
  preferences.putFloat("gas_perigo", cfg.thresholds.gas_perigo);

  preferences.end();

  Serial.println(">>> Configuracao salva");
}

void carregarConfig() {
  preferences.begin("config", true);

  String profile = preferences.getString("profile", "");

  if (profile == "COWORKING") {
    aplicarPerfilCoworking();
  } else if (profile == "COZINHA") {
    aplicarPerfilCozinha();
  } else if (profile == "AREA_SERVICO") {
    aplicarPerfilAreaServico();
  } else {
    aplicarPerfilCozinha(); // perfil default
  }

  cfg.thresholds.co_alerta  = preferences.getFloat("co_alerta", cfg.thresholds.co_alerta);
  cfg.thresholds.co_perigo  = preferences.getFloat("co_perigo", cfg.thresholds.co_perigo);
  cfg.thresholds.gas_alerta = preferences.getFloat("gas_alerta", cfg.thresholds.gas_alerta);
  cfg.thresholds.gas_perigo = preferences.getFloat("gas_perigo", cfg.thresholds.gas_perigo);

  cfg.profile.buzzerHabilitado = preferences.getBool("buzzer", cfg.profile.buzzerHabilitado);

  preferences.end();

  Serial.println(">>> Config carregada:");
  Serial.println(">>> Perfil: " + cfg.profile.nomePerfil);
  Serial.println(">>> Buzzer: " + String(cfg.profile.buzzerHabilitado ? "ON" : "OFF"));
  Serial.println(">>> CO alerta: " + String(cfg.thresholds.co_alerta, 2));
  Serial.println(">>> CO perigo: " + String(cfg.thresholds.co_perigo, 2));
  Serial.println(">>> GAS alerta: " + String(cfg.thresholds.gas_alerta, 2));
  Serial.println(">>> GAS perigo: " + String(cfg.thresholds.gas_perigo, 2));
}

// =====================================================
// INTERFACE VISUAL
// =====================================================

void desenhaProgresso(int porcentagem, const String& tarefa) {
  if (porcentagem < 0) porcentagem = 0;
  if (porcentagem > 100) porcentagem = 100;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.println("ID: " + String(ID_SENSOR));
  display.drawLine(0, 10, 128, 10, SSD1306_WHITE);

  display.setCursor(0, 18);
  display.println(tarefa);

  display.drawRect(10, 32, 108, 10, SSD1306_WHITE);
  int largura = map(porcentagem, 0, 100, 0, 104);
  display.fillRect(12, 34, largura, 6, SSD1306_WHITE);

  display.setCursor(0, 52);
  display.print("R7:");
  display.print(r0_mq7, 2);
  display.print(" R4:");
  display.print(r0_mq4, 2);

  display.display();
}

void mostrarLeituraOLED(float co, float metano, const String& status, float temperatura, float umidade) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.print("STATUS: ");
  display.println(status);
  display.drawLine(0, 10, 128, 10, SSD1306_WHITE);

  display.setCursor(0, 20);
  display.print("CO:  ");
  display.print(co, 2);
  display.println(" ppm");

  display.print("GAS: ");
  display.print(metano, 2);
  display.println(" ppm");

  display.drawLine(0, 42, 128, 42, SSD1306_WHITE);

  display.setCursor(0, 46);
  display.print("P:");
  display.print(cfg.profile.nomePerfil);

  display.setCursor(0, 56);
  display.print("T:");
  display.print(temperatura, 1);
  display.print(" H:");
  display.print(umidade, 0);
  display.print("%");

  display.display();
}

// =====================================================
// CALIBRAÇÃO
// =====================================================

void executarCalibracaoMQ7() {
  Serial.println(">>> Iniciando calibracao do MQ-7");

  for (int i = 0; i <= 100; i += 5) {
    desenhaProgresso(i, "CALIBRANDO MQ-7...");

    float temperatura = bme.readTemperature();
    float umidade     = bme.readHumidity();
    float cf          = calcularFatorCompensacao(temperatura, umidade);

    float leituraADC = analogRead(pinoMQ7);
    float tensaoMQ7  = (leituraADC * 3.3f / 4095.0f) * 1.4545f;
    float rs         = calcularRS(tensaoMQ7);

    r0_mq7 = (rs / cf) / 26.1f;

    delay(200);
  }

  salvarR0MQ7naFlash(r0_mq7);
  publicarResposta("Calibracao MQ7 OK. Novo R0 MQ7: " + String(r0_mq7, 3));
}

void executarCalibracaoMQ4() {
  Serial.println(">>> Iniciando calibracao do MQ-4");

  for (int i = 0; i <= 100; i += 5) {
    desenhaProgresso(i, "CALIBRANDO MQ-4...");

    float temperatura = bme.readTemperature();
    float umidade     = bme.readHumidity();
    float cf          = calcularFatorCompensacao(temperatura, umidade);

    float leituraADC = analogRead(pinoMQ4);
    float tensaoMQ4  = (leituraADC * 3.3f / 4095.0f);
    float rs         = calcularRS(tensaoMQ4);

    r0_mq4 = (rs / cf);

    delay(200);
  }

  salvarR0MQ4naFlash(r0_mq4);
  publicarResposta("Calibracao MQ4 OK. Novo R0 MQ4: " + String(r0_mq4, 3));
}

// =====================================================
// MQTT
// =====================================================

void publicarConfigAtual() {
  StaticJsonDocument<256> doc;

  doc["profile"] = cfg.profile.nomePerfil;
  doc["buzzer"]  = cfg.profile.buzzerHabilitado;

  JsonObject th = doc.createNestedObject("thresholds");
  th["co_alerta"]  = cfg.thresholds.co_alerta;
  th["co_perigo"]  = cfg.thresholds.co_perigo;
  th["gas_alerta"] = cfg.thresholds.gas_alerta;
  th["gas_perigo"] = cfg.thresholds.gas_perigo;

  String out;
  serializeJson(doc, out);
  client.publish(resTopic.c_str(), out.c_str());
}

void callback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (unsigned int i = 0; i < length; i++) {
    msg += (char)payload[i];
  }

  Serial.println(">>> Comando recebido em MQTT: " + msg);

  if (msg == "CALIBRAR_MQ7") {
    executarCalibracaoMQ7();
  }
  else if (msg == "CALIBRAR_MQ4") {
    executarCalibracaoMQ4();
  }
  else if (msg.startsWith("SET_R0_MQ7:")) {
    float val = msg.substring(11).toFloat();
    if (val > 0) {
      salvarR0MQ7naFlash(val);
      publicarResposta("R0 MQ7 alterado via comando.");
    } else {
      publicarResposta("Valor invalido para R0 MQ7.");
    }
  }
  else if (msg.startsWith("SET_R0_MQ4:")) {
    float val = msg.substring(11).toFloat();
    if (val > 0) {
      salvarR0MQ4naFlash(val);
      publicarResposta("R0 MQ4 alterado via comando.");
    } else {
      publicarResposta("Valor invalido para R0 MQ4.");
    }
  }
  else if (msg.startsWith("SET_PROFILE:")) {
    String perfil = msg.substring(12);

    if (perfil == "COWORKING") {
      aplicarPerfilCoworking();
      salvarConfig();
      publicarResposta("Perfil aplicado: COWORKING");
    }
    else if (perfil == "COZINHA") {
      aplicarPerfilCozinha();
      salvarConfig();
      publicarResposta("Perfil aplicado: COZINHA");
    }
    else if (perfil == "AREA_SERVICO") {
      aplicarPerfilAreaServico();
      salvarConfig();
      publicarResposta("Perfil aplicado: AREA_SERVICO");
    }
    else {
      publicarResposta("Perfil desconhecido.");
    }
  }
  else if (msg.startsWith("SET_CO_ALERTA:")) {
    cfg.thresholds.co_alerta = msg.substring(14).toFloat();
    salvarConfig();
    publicarResposta("CO alerta atualizado.");
  }
  else if (msg.startsWith("SET_CO_PERIGO:")) {
    cfg.thresholds.co_perigo = msg.substring(14).toFloat();
    salvarConfig();
    publicarResposta("CO perigo atualizado.");
  }
  else if (msg.startsWith("SET_GAS_ALERTA:")) {
    cfg.thresholds.gas_alerta = msg.substring(15).toFloat();
    salvarConfig();
    publicarResposta("Gas alerta atualizado.");
  }
  else if (msg.startsWith("SET_GAS_PERIGO:")) {
    cfg.thresholds.gas_perigo = msg.substring(15).toFloat();
    salvarConfig();
    publicarResposta("Gas perigo atualizado.");
  }
  else if (msg == "SET_BUZZER:ON") {
    cfg.profile.buzzerHabilitado = true;
    salvarConfig();
    publicarResposta("Buzzer habilitado.");
  }
  else if (msg == "SET_BUZZER:OFF") {
    cfg.profile.buzzerHabilitado = false;
    salvarConfig();
    publicarResposta("Buzzer desabilitado.");
  }
  else if (msg == "GET_CONFIG") {
    publicarConfigAtual();
  }
  else {
    publicarResposta("Comando desconhecido.");
  }
}

void conectarMQTT() {
  while (!client.connected()) {
    Serial.print("Conectando ao MQTT... ");

    if (client.connect(ID_SENSOR, MQTT_USER, MQTT_PASS)) {
      Serial.println("conectado");
      client.subscribe(cmdTopic.c_str());
      publicarResposta("Sensor conectado e pronto.");
    } else {
      Serial.print("falhou, rc=");
      Serial.print(client.state());
      Serial.println(" tentando novamente em 5s...");
      delay(5000);
    }
  }
}

// =====================================================
// LÓGICA DE ALARME
// =====================================================

void acionarBuzzer() {
  if (cfg.profile.buzzerHabilitado && estadoAtual == ESTADO_PERIGO) {
    digitalWrite(pinoBuzzer, HIGH);
  } else {
    digitalWrite(pinoBuzzer, LOW);
  }
}

/*void acionarBuzzer(const String& status) {
  if (cfg.profile.buzzerHabilitado && status == "PERIGO") {
    digitalWrite(pinoBuzzer, HIGH);
  } else {
    digitalWrite(pinoBuzzer, LOW);
  }
}
*/

// =====================================================
// AQUISIÇÃO E ENVIO
// =====================================================

void realizarLeituraEEnviar() {
  analogWrite(pinoMosfetMQ7, 0);
  delay(250);

  float temperatura = bme.readTemperature();
  float umidade     = bme.readHumidity();
  float cf          = calcularFatorCompensacao(temperatura, umidade);
  bool presenca     = digitalRead(pinoPresenca);

  long somaMQ7 = 0;
  long somaMQ4 = 0;

  for (int i = 0; i < 20; i++) {
    somaMQ7 += analogRead(pinoMQ7);
    somaMQ4 += analogRead(pinoMQ4);
    delay(5);
  }

  float mediaMQ7 = somaMQ7 / 20.0f;
  float mediaMQ4 = somaMQ4 / 20.0f;

  float tensaoMQ7 = (mediaMQ7 * 3.3f / 4095.0f) * 1.4545f;
  float tensaoMQ4 = (mediaMQ4 * 3.3f / 4095.0f);

  float rsMQ7 = calcularRS(tensaoMQ7);
  float rsMQ4 = calcularRS(tensaoMQ4);

  float co_ppm  = 100.0f  * pow((rsMQ7 / cf) / r0_mq7, -1.53f);
  float ch4_ppm = 1000.0f * pow((rsMQ4 / cf) / r0_mq4, -2.8f);

  // String statusStr;
  // if (co_ppm > cfg.thresholds.co_perigo || ch4_ppm > cfg.thresholds.gas_perigo) {
  //   statusStr = "PERIGO";
  // } else if (co_ppm > cfg.thresholds.co_alerta || ch4_ppm > cfg.thresholds.gas_alerta) {
  //   statusStr = "ALERTA";
  // } else {
  //   statusStr = "LIMPO";
  // }

  // acionarBuzzer(statusStr);
  // mostrarLeituraOLED(co_ppm, ch4_ppm, statusStr, temperatura, umidade);

  atualizarEstadoComPersistencia(co_ppm, ch4_ppm);

  String statusStr = estadoParaString(estadoAtual);

  acionarBuzzer();
  mostrarLeituraOLED(co_ppm, ch4_ppm, statusStr, temperatura, umidade);

  StaticJsonDocument<768> doc;
  doc["sensor_id"] = ID_SENSOR;
  doc["cliente"]   = ID_CLIENTE;
  doc["unidade"]   = UNIDADE;
  doc["ambiente"]  = AMBIENTE;
  doc["profile"]   = cfg.profile.nomePerfil;
  doc["buzzer"]    = cfg.profile.buzzerHabilitado;
  doc["r0_mq7"]    = r0_mq7;
  doc["r0_mq4"]    = r0_mq4;
  doc["status"]    = statusStr;
  doc["presenca"]  = presenca;
  doc["temp_c"]    = temperatura;
  doc["umid_pct"]  = umidade;

  JsonObject thresholds = doc.createNestedObject("thresholds");
  thresholds["co_alerta"]  = cfg.thresholds.co_alerta;
  thresholds["co_perigo"]  = cfg.thresholds.co_perigo;
  thresholds["gas_alerta"] = cfg.thresholds.gas_alerta;
  thresholds["gas_perigo"] = cfg.thresholds.gas_perigo;

  JsonObject leitura = doc.createNestedObject("leitura");
  leitura["co_ppm"]     = co_ppm;
  leitura["metano_ppm"] = ch4_ppm;

  String out;
  serializeJson(doc, out);

  Serial.println(">>> Payload publicado:");
  Serial.println(out);

  bool ok = client.publish(baseTopic.c_str(), out.c_str());

  if (ok) {
    Serial.println(">>> Publish MQTT OK");
  } else {
    Serial.println(">>> Falha no publish MQTT");
  }
}

// =====================================================
// SETUP
// =====================================================

void setup() {
  Serial.begin(115200);
  delay(500);

  Wire.begin();

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("ERRO: Falha ao inicializar OLED");
    while (true) delay(100);
  }

  display.clearDisplay();
  display.display();

  pinMode(pinoBuzzer, OUTPUT);
  pinMode(pinoMosfetMQ7, OUTPUT);
  pinMode(pinoPresenca, INPUT);

  digitalWrite(pinoBuzzer, LOW);

  if (!bme.begin(0x76)) {
    Serial.println("ERRO: BME280 nao encontrado em 0x76");
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("ERRO BME280");
    display.println("Endereco 0x76");
    display.display();
    while (true) delay(100);
  }

  carregarR0sDaFlash();
  carregarConfig();

  desenhaProgresso(0, "SISTEMA INICIANDO");
  delay(1000);

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Conectando ao Wi-Fi");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("Wi-Fi conectado");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  espClient.setInsecure();
  client.setServer(MQTT_SERVER, MQTT_PORT);
  client.setCallback(callback);
  client.setBufferSize(1024);

  conectarMQTT();
}

// =====================================================
// LOOP
// =====================================================

void loop() {
  if (!client.connected()) {
    conectarMQTT();
  }

  client.loop();

  unsigned long agora = millis();

  if (faseAtual == 1) {
    // FASE DE LIMPEZA
    if (tempoInicioFase == 0) {
      tempoInicioFase = agora;
      analogWrite(pinoMosfetMQ7, 255);
    }

    int progresso = map(agora - tempoInicioFase, 0, 60000, 0, 100);
    desenhaProgresso(progresso, "LIMPANDO SENSOR...");

    if (agora - tempoInicioFase >= 60000UL) {
      faseAtual = 2;
      tempoInicioFase = 0;
    }
  }
  else {
    // FASE DE MEDIÇÃO
    if (tempoInicioFase == 0) {
      tempoInicioFase = agora;
      analogWrite(pinoMosfetMQ7, 71);
    }

    int progresso = map(agora - tempoInicioFase, 0, 90000, 0, 100);
    desenhaProgresso(progresso, "ESTABILIZANDO...");

    if (agora - tempoInicioFase >= 88000UL) {
      realizarLeituraEEnviar();
      faseAtual = 1;
      tempoInicioFase = 0;
    }
  }
}