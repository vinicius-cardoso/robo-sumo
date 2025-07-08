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
#include <ArduinoJson.h> // Adicionado para manipulação de JSON

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

// --- Pinos dos Sensores de Linha (TCRT5000) - Alterados para portas analógicas
const int SENSOR_LINHA_DIREITA_PIN = 34;
const int SENSOR_LINHA_ESQUERDA_PIN = 36;
const int SENSOR_LINHA_TRASEIRA_PIN = 39;

// --- Pinos de Controle dos Sensores de Distância (VL53L0X)
const int SENSOR_DIST_XSHUT_DIREITA = 18;
const int SENSOR_DIST_XSHUT_FRONTAL = 19;
const int SENSOR_DIST_XSHUT_ESQUERDA = 23;

// ==========================================================================
// Objetos de Hardware e Rede
// ==========================================================================
AsyncWebServer server(80);
Adafruit_SSD1306 display(128, 64, &Wire, -1);
Adafruit_VL53L0X sensorDistDireita;
Adafruit_VL53L0X sensorDistFrontal;
Adafruit_VL53L0X sensorDistEsquerda;

// ==========================================================================
// Variáveis Globais de Estado
// ==========================================================================
int dist_direita_mm = 0, dist_frontal_mm = 0, dist_esquerda_mm = 0;

// Variáveis para os valores analógicos dos sensores de linha
int linha_direita_analog = 0;
int linha_esquerda_analog = 0;
int linha_traseira_analog = 0;

// Variáveis para a faixa de detecção de linha (ajuste conforme necessário)
const int LIMITE_BRANCO = 1000; // Abaixo deste valor é considerado branco
const int LIMITE_PRETO = 2000;  // Acima deste valor é considerado preto

// Caracteres para exibição no display
char linha_direita_char = 'B';
char linha_traseira_char = 'B';
char linha_esquerda_char = 'B';

String direcaoAtual = "Iniciando";

unsigned long proximoUpdate = 0;
const long INTERVALO_UPDATE_MS = 100;

// ==========================================================================
// Página de Controle (HTML)
// ==========================================================================
// Adicionadas divs com IDs para exibir os valores dos sensores
const char PAGINA_CONTROLE_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head><title>Controle do Robo ESP32</title><meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no"><style>body{font-family:Arial,sans-serif;text-align:center;background:#282c34;color:white}h1{margin-top:20px}.btn-container{display:grid;grid-template-columns:repeat(3, 1fr);gap:10px;max-width:300px;margin:30px auto}.btn{padding:20px;font-size:24px;color:white;background-color:#61dafb;border:none;border-radius:10px;cursor:pointer;user-select:none;-webkit-tap-highlight-color:transparent}.btn:active{background-color:#21a1f0}.placeholder{visibility:hidden}#frente{grid-column:2 / 3}#esquerda{grid-column:1 / 2;grid-row:2 / 3}#parar{grid-column:2 / 3;grid-row:2 / 3;background-color:#e04444}#direita{grid-column:3 / 4;grid-row:2 / 3}#tras{grid-column:2 / 3;grid-row:3 / 4}
.sensor-data{margin-top:20px;text-align:left;max-width:300px;margin-left:auto;margin-right:auto;border:1px solid #61dafb;padding:10px;border-radius:5px;}
.sensor-data div{margin-bottom:5px;}
</style></head><body><h1>Controle do Robo</h1><div class="btn-container"><div class="placeholder"></div><button id="frente" class="btn" onmousedown="sendCommand('frente')" onmouseup="sendCommand('parar')" ontouchstart="sendCommand('frente')" ontouchend="sendCommand('parar')">&#8593;</button><div class="placeholder"></div><button id="esquerda" class="btn" onmousedown="sendCommand('esquerda')" onmouseup="sendCommand('parar')" ontouchstart="sendCommand('esquerda')" ontouchend="sendCommand('parar')">&#8592;</button><button id="parar" class="btn" onclick="sendCommand('parar')">STOP</button><button id="direita" class="btn" onmousedown="sendCommand('direita')" onmouseup="sendCommand('parar')" ontouchstart="sendCommand('direita')" ontouchend="sendCommand('parar')">&#8594;</button><div class="placeholder"></div><button id="tras" class="btn" onmousedown="sendCommand('tras')" onmouseup="sendCommand('parar')" ontouchstart="sendCommand('tras')" ontouchend="sendCommand('parar')">&#8595;</button><div class="placeholder"></div></div>
<div class="sensor-data">
  <h2>Dados dos Sensores</h2>
  <div>Distancia Direita: <span id="distDireita">--</span> mm</div>
  <div>Distancia Frontal: <span id="distFrontal">--</span> mm</div>
  <div>Distancia Esquerda: <span id="distEsquerda">--</span> mm</div>
  <div>Linha Direita: <span id="linhaDireitaVal">--</span> (<span id="linhaDireitaChar">--</span>)</div>
  <div>Linha Traseira: <span id="linhaTraseiraVal">--</span> (<span id="linhaTraseiraChar">--</span>)</div>
  <div>Linha Esquerda: <span id="linhaEsquerdaVal">--</span> (<span id="linhaEsquerdaChar">--</span>)</div>
  <div>Direcao Atual: <span id="direcaoAtual">--</span></div>
</div>
<script>
function sendCommand(action){
  fetch('/control?action='+action)
    .then(response => response.text())
    .then(data => console.log(data))
    .catch(error => console.error('Erro ao enviar comando:', error));
}

function updateSensorData() {
  fetch('/sensors') // Novo endpoint para dados dos sensores
    .then(response => response.json())
    .then(data => {
      document.getElementById('distDireita').innerText = data.dist_direita;
      document.getElementById('distFrontal').innerText = data.dist_frontal;
      document.getElementById('distEsquerda').innerText = data.dist_esquerda;
      document.getElementById('linhaDireitaVal').innerText = data.linha_direita_analog;
      document.getElementById('linhaDireitaChar').innerText = data.linha_direita_char;
      document.getElementById('linhaTraseiraVal').innerText = data.linha_traseira_analog;
      document.getElementById('linhaTraseiraChar').innerText = data.linha_traseira_char;
      document.getElementById('linhaEsquerdaVal').innerText = data.linha_esquerda_analog;
      document.getElementById('linhaEsquerdaChar').innerText = data.linha_esquerda_char;
      document.getElementById('direcaoAtual').innerText = data.direcao_atual;
    })
    .catch(error => console.error('Erro ao buscar dados dos sensores:', error));
}

// Atualiza os dados dos sensores a cada 200ms (ajuste conforme a necessidade)
setInterval(updateSensorData, 200);

// Chama a função uma vez ao carregar a página para exibir os dados iniciais
document.addEventListener('DOMContentLoaded', updateSensorData);

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

  pinMode(SENSOR_DIST_XSHUT_DIREITA, OUTPUT);
  pinMode(SENSOR_DIST_XSHUT_ESQUERDA, OUTPUT);
  pinMode(SENSOR_DIST_XSHUT_FRONTAL, OUTPUT);

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

  digitalWrite(SENSOR_DIST_XSHUT_DIREITA, LOW);
  digitalWrite(SENSOR_DIST_XSHUT_ESQUERDA, LOW);
  digitalWrite(SENSOR_DIST_XSHUT_FRONTAL, LOW);
  delay(50);
  
  Wire.begin(21, 22);

  digitalWrite(SENSOR_DIST_XSHUT_DIREITA, HIGH); delay(10);
  if (!sensorDistDireita.begin(0x30)) { Serial.println(F("Falha DD")); while(1); }

  digitalWrite(SENSOR_DIST_XSHUT_ESQUERDA, HIGH); delay(10);
  if (!sensorDistEsquerda.begin(0x31)) { Serial.println(F("Falha DE")); while(1); }

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
      else if (action == "direita") virarDireita();
      else if (action == "esquerda") virarEsquerda();
      else if (action == "parar") pararMotores();
    }
    request->send(200, "text/plain", "OK");
  });

  // Novo endpoint para enviar dados dos sensores em JSON
  server.on("/sensors", HTTP_GET, [](AsyncWebServerRequest *request) {
    // É importante chamar lerSensores() antes de criar o JSON
    // para garantir que os dados estejam atualizados.
    lerSensores(); 

    StaticJsonDocument<512> doc; // Tamanho do documento JSON (ajuste se precisar de mais dados)
    doc["dist_direita"] = dist_direita_mm;
    doc["dist_frontal"] = dist_frontal_mm;
    doc["dist_esquerda"] = dist_esquerda_mm;
    doc["linha_direita_analog"] = linha_direita_analog;
    doc["linha_esquerda_analog"] = linha_esquerda_analog;
    doc["linha_traseira_analog"] = linha_traseira_analog;
    doc["linha_direita_char"] = String(linha_direita_char);
    doc["linha_esquerda_char"] = String(linha_esquerda_char);
    doc["linha_traseira_char"] = String(linha_traseira_char);
    doc["direcao_atual"] = direcaoAtual;

    String response;
    serializeJson(doc, response);
    request->send(200, "application/json", response);
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
  
  sensorDistDireita.rangingTest(&measure, false);
  dist_direita_mm = (measure.RangeStatus != 4) ? measure.RangeMilliMeter : 9999;

  sensorDistFrontal.rangingTest(&measure, false);
  dist_frontal_mm = (measure.RangeStatus != 4) ? measure.RangeMilliMeter : 9999;

  sensorDistEsquerda.rangingTest(&measure, false);
  dist_esquerda_mm = (measure.RangeStatus != 4) ? measure.RangeMilliMeter : 9999;

  // Leitura dos valores analógicos dos sensores de linha
  linha_direita_analog = analogRead(SENSOR_LINHA_DIREITA_PIN);
  linha_esquerda_analog = analogRead(SENSOR_LINHA_ESQUERDA_PIN);
  linha_traseira_analog = analogRead(SENSOR_LINHA_TRASEIRA_PIN);

  // Determinar se é linha preta ou branca
  linha_direita_char = (linha_direita_analog < LIMITE_BRANCO) ? 'B' : ((linha_direita_analog > LIMITE_PRETO) ? 'P' : '-');
  linha_esquerda_char = (linha_esquerda_analog < LIMITE_BRANCO) ? 'B' : ((linha_esquerda_analog > LIMITE_PRETO) ? 'P' : '-');
  linha_traseira_char = (linha_traseira_analog < LIMITE_BRANCO) ? 'B' : ((linha_traseira_analog > LIMITE_PRETO) ? 'P' : '-');
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
  sprintf(buffer, "DD:%4dmm", dist_direita_mm);
  display.setCursor(0, Y_DATA_START);
  display.print(buffer);
  
  // Exibindo o valor analógico do sensor de linha da direita
  sprintf(buffer, "| LD:%4d", linha_direita_analog);
  display.setCursor(X_COLUNA_2, Y_DATA_START);
  display.print(buffer);

  sprintf(buffer, "DF:%4dmm", dist_frontal_mm);
  display.setCursor(0, Y_DATA_START + LINE_HEIGHT);
  display.print(buffer);
  
  // Exibindo o valor analógico do sensor de linha traseiro
  sprintf(buffer, "| LT:%4d", linha_traseira_analog);
  display.setCursor(X_COLUNA_2, Y_DATA_START + LINE_HEIGHT);
  display.print(buffer);

  sprintf(buffer, "DE:%4dmm", dist_esquerda_mm);
  display.setCursor(0, Y_DATA_START + (LINE_HEIGHT * 2));
  display.print(buffer);

  // Exibindo o valor analógico do sensor de linha da esquerda
  sprintf(buffer, "| LE:%4d", linha_esquerda_analog);
  display.setCursor(X_COLUNA_2, Y_DATA_START + (LINE_HEIGHT * 2));
  display.print(buffer);

  // Linha para mostrar a interpretação P/B dos sensores de linha
  display.setCursor(0, Y_DATA_START + (LINE_HEIGHT * 3));
  sprintf(buffer, "L: D:%c T:%c E:%c", linha_direita_char, linha_traseira_char, linha_esquerda_char);
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
