// ==========================================================================
// Bibliotecas
// ==========================================================================
#include <WiFi.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <ESPAsyncWebServer.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "Adafruit_VL53L0X.h"

// ==========================================================================
// Configurações e Mapeamento de Pinos
// ==========================================================================
// --- Rede
const char* WIFI_SSID = "RoboSumoGrupoA";
const char* WIFI_PASS = "caveirao";
const char* OTA_HOSTNAME = "robo-sumo-esp32";

// --- Pinos dos Motores (Ponte H L298N)
const int MOTOR_A_ENA = 32;
const int MOTOR_A_IN1 = 33;
const int MOTOR_A_IN2 = 25;
const int MOTOR_B_ENB = 14;
const int MOTOR_B_IN3 = 26;
const int MOTOR_B_IN4 = 27;

// --- Pinos dos Sensores de Linha (TCRT5000)
const int SENSOR_LINHA_L1 = 4;
const int SENSOR_LINHA_L2 = 5;
const int SENSOR_LINHA_L3 = 15;

// --- Pinos de Controle dos Sensores de Distância (VL53L0X)
const int SENSOR_DIST_XSHUT_1 = 18;
const int SENSOR_DIST_XSHUT_2 = 19;
const int SENSOR_DIST_XSHUT_3 = 23;

// ==========================================================================
// Objetos de Hardware e Rede
// ==========================================================================
AsyncWebServer server(80);
Adafruit_SSD1306 display(128, 64, &Wire, -1);
Adafruit_VL53L0X sensorDist1;
Adafruit_VL53L0X sensorDist2;
Adafruit_VL53L0X sensorDist3;

// ==========================================================================
// Variáveis Globais de Estado
// ==========================================================================
int dist1_mm = 0, dist2_mm = 0, dist3_mm = 0;
char linha1_char = 'B', linha2_char = 'B', linha3_char = 'B';
String direcaoAtual = "Iniciando";

unsigned long proximoUpdate = 0;
const long INTERVALO_UPDATE_MS = 100;

// ==========================================================================
// Página de Controle (HTML)
// ==========================================================================
const char PAGINA_CONTROLE_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head><title>Controle do Robo ESP32</title><meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no"><style>body{font-family:Arial,sans-serif;text-align:center;background:#282c34;color:white}h1{margin-top:20px}.btn-container{display:grid;grid-template-columns:repeat(3, 1fr);gap:10px;max-width:300px;margin:30px auto}.btn{padding:20px;font-size:24px;color:white;background-color:#61dafb;border:none;border-radius:10px;cursor:pointer;user-select:none;-webkit-tap-highlight-color:transparent}.btn:active{background-color:#21a1f0}.placeholder{visibility:hidden}#frente{grid-column:2 / 3}#esquerda{grid-column:1 / 2;grid-row:2 / 3}#parar{grid-column:2 / 3;grid-row:2 / 3;background-color:#e04444}#direita{grid-column:3 / 4;grid-row:2 / 3}#tras{grid-column:2 / 3;grid-row:3 / 4}</style></head><body><h1>Controle do Robo</h1><div class="btn-container"><div class="placeholder"></div><button id="frente" class="btn" onmousedown="sendCommand('frente')" onmouseup="sendCommand('parar')" ontouchstart="sendCommand('frente')" ontouchend="sendCommand('parar')">&#8593;</button><div class="placeholder"></div><button id="esquerda" class="btn" onmousedown="sendCommand('esquerda')" onmouseup="sendCommand('parar')" ontouchstart="sendCommand('esquerda')" ontouchend="sendCommand('parar')">&#8592;</button><button id="parar" class="btn" onclick="sendCommand('parar')">STOP</button><button id="direita" class="btn" onmousedown="sendCommand('direita')" onmouseup="sendCommand('parar')" ontouchstart="sendCommand('direita')" ontouchend="sendCommand('parar')">&#8594;</button><div class="placeholder"></div><button id="tras" class="btn" onmousedown="sendCommand('tras')" onmouseup="sendCommand('parar')" ontouchstart="sendCommand('tras')" ontouchend="sendCommand('parar')">&#8595;</button><div class="placeholder"></div></div><script>function sendCommand(action){fetch('/control?action='+action)}</script></body></html>
)rawliteral";

// ==========================================================================
// Protótipos de Funções
// ==========================================================================
void configurarPinos();
void configurarSensores();
void configurarRede();
void configurarOTA();
void lerSensores();
void atualizarDisplay();
void moverFrente();
void moverTras();
void virarDireita();
void virarEsquerda();
void pararMotores();

// ==========================================================================
// Setup Principal
// ==========================================================================
void setup() {
  Serial.begin(115200);

  configurarPinos();
  configurarSensores();
  configurarRede();
  configurarOTA();

  direcaoAtual = "Parado";
  Serial.println("Robo pronto para controle e atualizacao OTA.");
}

// ==========================================================================
// Loop Principal
// ==========================================================================
void loop() {
  ArduinoOTA.handle();

  if (millis() >= proximoUpdate) {
    proximoUpdate = millis() + INTERVALO_UPDATE_MS;
    lerSensores();
    atualizarDisplay();
  }
}

// ==========================================================================
// Funções de Configuração (chamadas no setup)
// ==========================================================================
void configurarPinos() {
  pinMode(MOTOR_A_ENA, OUTPUT);
  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_B_ENB, OUTPUT);
  pinMode(MOTOR_B_IN3, OUTPUT);
  pinMode(MOTOR_B_IN4, OUTPUT);

  pinMode(SENSOR_LINHA_L1, INPUT);
  pinMode(SENSOR_LINHA_L2, INPUT);
  pinMode(SENSOR_LINHA_L3, INPUT);

  pinMode(SENSOR_DIST_XSHUT_1, OUTPUT);
  pinMode(SENSOR_DIST_XSHUT_2, OUTPUT);
  pinMode(SENSOR_DIST_XSHUT_3, OUTPUT);

  digitalWrite(MOTOR_A_ENA, HIGH);
  digitalWrite(MOTOR_B_ENB, HIGH);
}

void configurarSensores() {
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Iniciando Sensores...");
  display.display();

  digitalWrite(SENSOR_DIST_XSHUT_1, LOW);
  digitalWrite(SENSOR_DIST_XSHUT_2, LOW);
  digitalWrite(SENSOR_DIST_XSHUT_3, LOW);
  delay(50);
  
  Wire.begin(21, 22);

  digitalWrite(SENSOR_DIST_XSHUT_1, HIGH); delay(10);
  if (!sensorDist1.begin(0x30)) { Serial.println(F("Falha DD")); while(1); }

  digitalWrite(SENSOR_DIST_XSHUT_3, HIGH); delay(10);
  if (!sensorDist3.begin(0x31)) { Serial.println(F("Falha DE")); while(1); }

  digitalWrite(SENSOR_DIST_XSHUT_2, HIGH); delay(10);
  if (!sensorDist2.begin()) { Serial.println(F("Falha DF")); while(1); }

  Serial.println(F("Sensores de distancia OK!"));
}

void configurarRede() {
  WiFi.softAP(WIFI_SSID, WIFI_PASS);
  Serial.print("Robo AP IP: ");
  Serial.println(WiFi.softAPIP());

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", PAGINA_CONTROLE_HTML);
  });

  server.on("/control", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("action")) {
      String action = request->getParam("action")->value();
      if (action == "frente") moverFrente();
      else if (action == "tras") moverTras();
      else if (action == "direita") virarDireita();
      else if (action == "esquerda") virarEsquerda();
      else if (action == "parar") pararMotores();
    }
    request->send(200, "text/plain", "OK");
  });

  server.begin();
}

void configurarOTA() {
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.onStart([]() {
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.println("ATUALIZANDO");
    display.display();
  });
  ArduinoOTA.onEnd([]() {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(1);
    display.println("Atualizacao\nConcluida!");
    display.display();
    delay(1000);
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    display.drawRect(14, 32, 100, 10, WHITE);
    display.fillRect(14, 32, progress / (total / 100), 10, WHITE);
    display.display();
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("OTA Error[%u]\n", error);
  });
  ArduinoOTA.begin();
}


// ==========================================================================
// Funções de Lógica e Ação
// ==========================================================================
void lerSensores() {
  VL53L0X_RangingMeasurementData_t measure;
  
  sensorDist1.rangingTest(&measure, false);
  dist1_mm = (measure.RangeStatus != 4) ? measure.RangeMilliMeter : 9999;

  sensorDist2.rangingTest(&measure, false);
  dist2_mm = (measure.RangeStatus != 4) ? measure.RangeMilliMeter : 9999;
  
  sensorDist3.rangingTest(&measure, false);
  dist3_mm = (measure.RangeStatus != 4) ? measure.RangeMilliMeter : 9999;

  linha1_char = (digitalRead(SENSOR_LINHA_L1) == LOW) ? 'B' : 'P';
  linha2_char = (digitalRead(SENSOR_LINHA_L2) == LOW) ? 'B' : 'P';
  linha3_char = (digitalRead(SENSOR_LINHA_L3) == LOW) ? 'B' : 'P';
}

void atualizarDisplay() {
  const int Y_HEADER = 4;
  const int Y_DATA_START = 18;
  const int LINE_HEIGHT = 10;
  const int X_COLUNA_2 = 70;
  
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, Y_HEADER);
  display.print("Distancia");
  display.setCursor(X_COLUNA_2, Y_HEADER);
  display.print("| Linha");

  char buffer[16];
  sprintf(buffer, "DD:%4dmm", dist1_mm);
  display.setCursor(0, Y_DATA_START);
  display.print(buffer);
  
  sprintf(buffer, "| LD: %c", linha1_char);
  display.setCursor(X_COLUNA_2, Y_DATA_START);
  display.print(buffer);

  sprintf(buffer, "DF:%4dmm", dist2_mm);
  display.setCursor(0, Y_DATA_START + LINE_HEIGHT);
  display.print(buffer);
  
  sprintf(buffer, "| LT: %c", linha2_char);
  display.setCursor(X_COLUNA_2, Y_DATA_START + LINE_HEIGHT);
  display.print(buffer);

  sprintf(buffer, "DE:%4dmm", dist3_mm);
  display.setCursor(0, Y_DATA_START + (LINE_HEIGHT * 2));
  display.print(buffer);

  sprintf(buffer, "| LE: %c", linha3_char);
  display.setCursor(X_COLUNA_2, Y_DATA_START + (LINE_HEIGHT * 2));
  display.print(buffer);

  display.drawFastHLine(0, 50, display.width(), SSD1306_WHITE);
  display.setCursor(0, 54);
  display.print("Direcao: ");
  display.print(direcaoAtual);
  
  display.display();
}

void moverFrente() {
  direcaoAtual = "Frente";
  digitalWrite(MOTOR_A_IN1, LOW);
  digitalWrite(MOTOR_A_IN2, HIGH);
  digitalWrite(MOTOR_B_IN3, HIGH);
  digitalWrite(MOTOR_B_IN4, LOW);
}

void moverTras() {
  direcaoAtual = "Tras";
  digitalWrite(MOTOR_A_IN1, HIGH);
  digitalWrite(MOTOR_A_IN2, LOW);
  digitalWrite(MOTOR_B_IN3, LOW);
  digitalWrite(MOTOR_B_IN4, HIGH);
}

void virarDireita() {
  direcaoAtual = "Direita";
  digitalWrite(MOTOR_A_IN1, LOW);
  digitalWrite(MOTOR_A_IN2, HIGH);
  digitalWrite(MOTOR_B_IN3, LOW);
  digitalWrite(MOTOR_B_IN4, HIGH);
}

void virarEsquerda() {
  direcaoAtual = "Esquerda";
  digitalWrite(MOTOR_A_IN1, HIGH);
  digitalWrite(MOTOR_A_IN2, LOW);
  digitalWrite(MOTOR_B_IN3, HIGH);
  digitalWrite(MOTOR_B_IN4, LOW);
}

void pararMotores() {
  direcaoAtual = "Parado";
  digitalWrite(MOTOR_A_IN1, LOW);
  digitalWrite(MOTOR_A_IN2, LOW);
  digitalWrite(MOTOR_B_IN3, LOW);
  digitalWrite(MOTOR_B_IN4, LOW);
}
