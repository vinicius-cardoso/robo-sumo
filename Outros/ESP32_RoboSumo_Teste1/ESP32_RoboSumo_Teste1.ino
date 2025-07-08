// ESP32_RoboSumo
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
// #include <ArduinoJson.h> // Removido: Não mais necessário para enviar JSON

// ==========================================================================
// Configurações e Mapeamento de Pinos
// ==========================================================================
// --- Rede
const char* WIFI_SSID = "RoboSumoGrupoA";
const char* WIFI_PASS = "caveirao";
const char* OTA_HOSTNAME = "robo-sumo-esp32";

// --- Pinos dos Motores (Ponte H L298N)
const int MOTOR_A_ENA = 32; // Pino EN A do L298N (para controle de velocidade do Motor A)
const int MOTOR_A_IN1 = 33;
const int MOTOR_A_IN2 = 25;
const int MOTOR_B_ENB = 14; // Pino EN B do L298N (para controle de velocidade do Motor B)
const int MOTOR_B_IN3 = 26;
const int MOTOR_B_IN4 = 27;

// --- Parâmetros do PWM (para ledcAttach, não usa canais explicitamente aqui)
const int freq = 5000;      // Frequência do PWM (Hz)
const int resolution = 8;   // Resolução do PWM (bits) - 8 bits = 0-255

// --- Pinos dos Sensores de Linha (TCRT5000) - Alterados para portas analógicas
const int SENSOR_LINHA_ESQUERDA_A0 = 34;
const int SENSOR_LINHA_DIREITA_A0 = 35;
const int SENSOR_LINHA_TRASEIRA_A0 = 39;

const int SENSOR_LINHA_ESQUERDA_D0 = 4;
const int SENSOR_LINHA_DIREITA_D0 = 15;
const int SENSOR_LINHA_TRASEIRA_D0 = 5;

// --- Pinos de Controle dos Sensores de Distância (VL53L0X)
const int SENSOR_DIST_XSHUT_ESQUERDA = 23;
const int SENSOR_DIST_XSHUT_DIREITA = 18;
const int SENSOR_DIST_XSHUT_FRONTAL = 19;

// ==========================================================================
// Objetos de Hardware e Rede
// ==========================================================================
AsyncWebServer server(80);
Adafruit_SSD1306 display(128, 64, &Wire, -1);
Adafruit_VL53L0X sensorDistEsquerda;
Adafruit_VL53L0X sensorDistDireita;
Adafruit_VL53L0X sensorDistFrontal;

// ==========================================================================
// Variáveis Globais de Estado
// ==========================================================================
int dist_esquerda_mm = 0;
int dist_direita_mm = 0;
int dist_frontal_mm = 0;

int linha_esquerda_analog = 0;
int linha_direita_analog = 0;
int linha_traseira_analog = 0;

int linha_esquerda_digital = 0;
int linha_direita_digital = 0;
int linha_traseira_digital = 0;

const int LIMITE_BRANCO_DIREITA = 1800;
const int LIMITE_PRETO_DIREITA = 2000;
const int LIMITE_BRANCO_ESQUERDA = 1400;
const int LIMITE_PRETO_ESQUERDA = 1400;
const int LIMITE_BRANCO_TRASEIRA = 1100;
const int LIMITE_PRETO_TRASEIRA = 1100;

// Caracteres para exibição no display
char linha_esquerda_char = 'B';
char linha_direita_char = 'B';
char linha_traseira_char = 'B';

String direcaoAtual = "Iniciando";
int velocidadeAtual = 0; // 0-255

unsigned long proximoUpdate = 0;
const long INTERVALO_UPDATE_MS = 100;

// ==========================================================================
// Página de Controle (HTML)
// ==========================================================================
const char PAGINA_CONTROLE_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head><title>Controle do Robo ESP32</title><meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no"><style>
body{font-family:Arial,sans-serif;text-align:center;background:#282c34;color:white}
h1{margin-top:20px}
/* Layout de 3 colunas para os botões principais */
.btn-container{display:grid;grid-template-columns:repeat(3, 1fr);gap:10px;max-width:300px;margin:30px auto}
.btn{padding:20px;font-size:24px;color:white;background-color:#61dafb;border:none;border-radius:10px;cursor:pointer;user-select:none;-webkit-tap-highlight-color:transparent}
.btn:active{background-color:#21a1f0}
/* Placeholders para preencher espaços vazios no grid */
.placeholder{visibility:hidden}

/* POSICIONAMENTO DOS 5 BOTÕES PRINCIPAIS */
#frente{grid-column:2 / 3; grid-row:1 / 2;}
#esquerda{grid-column:1 / 2; grid-row:2 / 3;}
#parar{grid-column:2 / 3; grid-row:2 / 3; background-color:#e04444}
#direita{grid-column:3 / 4; grid-row:2 / 3;}
#tras{grid-column:2 / 3; grid-row:3 / 4;}

</style></head><body><h1>Controle do Robo</h1><div class="btn-container">
<div class="placeholder"></div> 
<button id="frente" class="btn" onmousedown="sendCommand('frente')" onmouseup="sendCommand('parar')" ontouchstart="sendCommand('frente')" ontouchend="sendCommand('parar')">&#8593;</button>
<div class="placeholder"></div>

<button id="esquerda" class="btn" onmousedown="sendCommand('esquerda')" onmouseup="sendCommand('parar')" ontouchstart="sendCommand('esquerda')" ontouchend="sendCommand('parar')">&#8592;</button>
<button id="parar" class="btn" onclick="sendCommand('parar')">STOP</button>
<button id="direita" class="btn" onmousedown="sendCommand('direita')" onmouseup="sendCommand('parar')" ontouchstart="sendCommand('direita')" ontouchend="sendCommand('parar')">&#8594;</button>

<div class="placeholder"></div>
<button id="tras" class="btn" onmousedown="sendCommand('tras')" onmouseup="sendCommand('parar')" ontouchstart="sendCommand('tras')" ontouchend="sendCommand('parar')">&#8595;</button>
<div class="placeholder"></div>

</div>
<script>
function sendCommand(action){
  fetch('/control?action='+action)
    .then(response => response.text())
    .then(data => console.log(data))
    .catch(error => console.error('Erro ao enviar comando:', error));
}
// Removido: Lógica de atualização de dados de sensor via JS
</script></body></html>
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
void setMotorSpeed(int speed);
void moverFrente();
void moverTras();
void virarDireita();
void virarEsquerda();
void pararMotores();

// Funções de 50% de velocidade removidas dos protótipos
// void moverFrenteLenta();
// void moverTrasLenta();
// void virarDireitaLenta();
// void virarEsquerdaLenta();


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
  velocidadeAtual = 0; // Inicia parado
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
  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_B_IN3, OUTPUT);
  pinMode(MOTOR_B_IN4, OUTPUT);

  // Configura os pinos ENABLE para PWM usando ledcAttach
  // ledcAttach(pino, frequencia, resolucao)
  ledcAttach(MOTOR_A_ENA, freq, resolution);
  ledcAttach(MOTOR_B_ENB, freq, resolution);

  pinMode(SENSOR_DIST_XSHUT_ESQUERDA, OUTPUT);
  pinMode(SENSOR_DIST_XSHUT_DIREITA, OUTPUT);
  pinMode(SENSOR_DIST_XSHUT_FRONTAL, OUTPUT);

  // Inicialmente, os motores estão parados
  pararMotores();
}

void configurarSensores() {
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Iniciando Sensores...");
  display.display();

  digitalWrite(SENSOR_DIST_XSHUT_ESQUERDA, LOW);
  digitalWrite(SENSOR_DIST_XSHUT_DIREITA, LOW);
  digitalWrite(SENSOR_DIST_XSHUT_FRONTAL, LOW);
  delay(50);
  
  Wire.begin(21, 22);

  digitalWrite(SENSOR_DIST_XSHUT_ESQUERDA, HIGH); delay(10);
  if (!sensorDistEsquerda.begin(0x31)) { Serial.println(F("Falha DE")); while(1); }

  digitalWrite(SENSOR_DIST_XSHUT_DIREITA, HIGH); delay(10);
  if (!sensorDistDireita.begin(0x30)) { Serial.println(F("Falha DD")); while(1); }

  digitalWrite(SENSOR_DIST_XSHUT_FRONTAL, HIGH); delay(10);
  if (!sensorDistFrontal.begin()) { Serial.println(F("Falha DF")); while(1); }

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
      else if (action == "esquerda") virarEsquerda();
      else if (action == "direita") virarDireita();
      // Chamadas para funções "Lenta" removidas
      // else if (action == "frenteLenta") moverFrenteLenta();
      // else if (action == "trasLenta") moverTrasLenta();
      // else if (action == "esquerdaLenta") virarEsquerdaLenta();
      // else if (action == "direitaLenta") virarDireitaLenta();
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
    display.println("UPLOADING");
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

  sensorDistEsquerda.rangingTest(&measure, false);
  dist_esquerda_mm = (measure.RangeStatus != 4) ? measure.RangeMilliMeter : 9999;

  sensorDistDireita.rangingTest(&measure, false);
  dist_direita_mm = (measure.RangeStatus != 4) ? measure.RangeMilliMeter : 9999;

  sensorDistFrontal.rangingTest(&measure, false);
  dist_frontal_mm = (measure.RangeStatus != 4) ? measure.RangeMilliMeter : 9999;

  linha_esquerda_analog = analogRead(SENSOR_LINHA_ESQUERDA_A0);
  linha_direita_analog = analogRead(SENSOR_LINHA_DIREITA_A0);
  linha_traseira_analog = analogRead(SENSOR_LINHA_TRASEIRA_A0);

  linha_esquerda_digital = digitalRead(SENSOR_LINHA_ESQUERDA_D0);
  linha_direita_digital = digitalRead(SENSOR_LINHA_DIREITA_D0);
  linha_traseira_digital = digitalRead(SENSOR_LINHA_TRASEIRA_D0);

  // Determinar se é linha preta ou branca
  linha_esquerda_char = (linha_esquerda_analog < LIMITE_BRANCO_ESQUERDA) ? 'B' : ((linha_esquerda_analog > LIMITE_PRETO_ESQUERDA) ? 'P' : '-');
  linha_direita_char = (linha_direita_analog < LIMITE_BRANCO_DIREITA) ? 'B' : ((linha_direita_analog > LIMITE_PRETO_DIREITA) ? 'P' : '-');
  linha_traseira_char = (linha_traseira_analog < LIMITE_BRANCO_TRASEIRA) ? 'B' : ((linha_traseira_analog > LIMITE_PRETO_TRASEIRA) ? 'P' : '-');
}

void atualizarDisplay() {
  const int Y_HEADER = 4;
  const int Y_DATA_START = 18;
  const int LINE_HEIGHT = 10;
  const int X_COLUNA_2 = 55;
  
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, Y_HEADER);
  display.print("Distancia");
  display.setCursor(X_COLUNA_2, Y_HEADER);
  display.print("| Linha");

  char buffer[16];

  sprintf(buffer, "E:%4dmm", dist_esquerda_mm);
  display.setCursor(0, Y_DATA_START);
  display.print(buffer);

  sprintf(buffer, "| E:%4d %d %c", linha_esquerda_analog, linha_esquerda_digital, linha_esquerda_char);
  display.setCursor(X_COLUNA_2, Y_DATA_START);
  display.print(buffer);

  sprintf(buffer, "D:%4dmm", dist_direita_mm);
  display.setCursor(0, Y_DATA_START + LINE_HEIGHT);
  display.print(buffer);

  sprintf(buffer, "| D:%4d %d %c", linha_direita_analog, linha_direita_digital, linha_direita_char);
  display.setCursor(X_COLUNA_2, Y_DATA_START + LINE_HEIGHT);
  display.print(buffer);

  sprintf(buffer, "F:%4dmm", dist_frontal_mm);
  display.setCursor(0, Y_DATA_START + (LINE_HEIGHT * 2));
  display.print(buffer);

  sprintf(buffer, "| T:%4d %d %c", linha_traseira_analog, linha_traseira_digital, linha_traseira_char);
  display.setCursor(X_COLUNA_2, Y_DATA_START + (LINE_HEIGHT * 2));
  display.print(buffer);

  // Linha para mostrar a interpretação P/B dos sensores de linha
  //display.setCursor(0, Y_DATA_START + (LINE_HEIGHT * 3));
  //sprintf(buffer, "LD:%c LT:%c LE:%c", linha_direita_char, linha_traseira_char, linha_esquerda_char);
  //display.print(buffer);

  display.setCursor(0, 54);
  display.print("Dir:");
  display.print(direcaoAtual);
  display.print(" V:");
  display.print(map(velocidadeAtual, 0, (1 << resolution) -1, 0, 100)); // Mapeia para porcentagem
  display.print("%");
  
  display.display();
}

// Esta função define a velocidade para AMBOS os motores.
// Usa ledcWrite com o pino diretamente.
void setMotorSpeed(int speed) {
  velocidadeAtual = speed;
  ledcWrite(MOTOR_A_ENA, velocidadeAtual); // Escreve no pino ENA
  ledcWrite(MOTOR_B_ENB, velocidadeAtual); // Escreve no pino ENB
}

void moverFrente() {
  direcaoAtual = "Frente";
  setMotorSpeed((1 << resolution) - 1);
  digitalWrite(MOTOR_A_IN1, HIGH); 
  digitalWrite(MOTOR_A_IN2, LOW); 
  digitalWrite(MOTOR_B_IN3, HIGH); 
  digitalWrite(MOTOR_B_IN4, LOW); 
}

void moverTras() {
  direcaoAtual = "Tras";
  setMotorSpeed((1 << resolution) - 1);
  digitalWrite(MOTOR_A_IN1, LOW); 
  digitalWrite(MOTOR_A_IN2, HIGH); 
  digitalWrite(MOTOR_B_IN3, LOW); 
  digitalWrite(MOTOR_B_IN4, HIGH); 
}

void virarEsquerda() {
  direcaoAtual = "Esquerda";
  setMotorSpeed((1 << resolution) - 1);
  digitalWrite(MOTOR_A_IN1, LOW); 
  digitalWrite(MOTOR_A_IN2, HIGH); 
  digitalWrite(MOTOR_B_IN3, HIGH); 
  digitalWrite(MOTOR_B_IN4, LOW); 
}

void virarDireita() {
  direcaoAtual = "Direita";
  setMotorSpeed((1 << resolution) - 1);
  digitalWrite(MOTOR_A_IN1, HIGH); 
  digitalWrite(MOTOR_A_IN2, LOW); 
  digitalWrite(MOTOR_B_IN3, LOW); 
  digitalWrite(MOTOR_B_IN4, HIGH); 
}

void pararMotores() {
  direcaoAtual = "Parado";
  setMotorSpeed(0);
  digitalWrite(MOTOR_A_IN1, LOW);
  digitalWrite(MOTOR_A_IN2, LOW);
  digitalWrite(MOTOR_B_IN3, LOW);
  digitalWrite(MOTOR_B_IN4, LOW);
}
