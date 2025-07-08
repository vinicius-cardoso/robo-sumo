#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "Adafruit_VL53L0X.h"

// --- Configurações do Wi-Fi ---
const char* ssid = "RoboSumoGrupoA"; // Nome da rede Wi-Fi que o robô irá criar
const char* password = "caveirao";   // Senha da rede Wi-Fi

// --- Objetos do Servidor, Display e Sensores ---
AsyncWebServer server(80);
Adafruit_SSD1306 display(128, 64, &Wire, -1);
Adafruit_VL53L0X lox_dist1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox_dist2 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox_dist3 = Adafruit_VL53L0X();

// --- Pinos de Controle (XSHUT) dos Sensores de Distância (VL53L0X) ---
#define XSHUT_PIN_D1 18
#define XSHUT_PIN_D2 19
#define XSHUT_PIN_D3 23

// --- Pinos dos Sensores de Linha (TCRT5000) ---
const int TCRT_PIN_L1 = 4;
const int TCRT_PIN_L2 = 5;
const int TCRT_PIN_L3 = 15;

// --- Pinos dos Motores ---
const int ENA = 32; const int ENB = 14;
const int IN1 = 33; const int IN2 = 25;
const int IN3 = 26; const int IN4 = 27;

// --- Variáveis Globais para armazenar o estado ---
int dist1_mm = 0, dist2_mm = 0, dist3_mm = 0;
char linha1_char = 'B', linha2_char = 'B', linha3_char = 'B';
String direcaoAtual = "Iniciando";

// --- Página HTML para o controle (armazenada na memória flash) ---
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <title>Controle do Robo ESP32</title>
  <meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no">
  <style>
    body { font-family: Arial, sans-serif; text-align: center; background: #282c34; color: white; }
    h1 { margin-top: 20px; }
    .btn-container { display: grid; grid-template-columns: repeat(3, 1fr); gap: 10px; max-width: 300px; margin: 30px auto; }
    .btn { padding: 20px; font-size: 24px; color: white; background-color: #61dafb; border: none; border-radius: 10px; cursor: pointer; user-select: none; -webkit-tap-highlight-color: transparent; }
    .btn:active { background-color: #21a1f0; }
    .placeholder { visibility: hidden; }
    #frente { grid-column: 2 / 3; }
    #esquerda { grid-column: 1 / 2; grid-row: 2 / 3; }
    #parar { grid-column: 2 / 3; grid-row: 2 / 3; background-color: #e04444; }
    #direita { grid-column: 3 / 4; grid-row: 2 / 3; }
    #tras { grid-column: 2 / 3; grid-row: 3 / 4; }
  </style>
</head>
<body>
  <h1>Controle do Robo</h1>
  <div class="btn-container">
    <div class="placeholder"></div><button id="frente" class="btn" onmousedown="sendCommand('frente')" onmouseup="sendCommand('parar')" ontouchstart="sendCommand('frente')" ontouchend="sendCommand('parar')">&#8593;</button><div class="placeholder"></div>
    <button id="esquerda" class="btn" onmousedown="sendCommand('esquerda')" onmouseup="sendCommand('parar')" ontouchstart="sendCommand('esquerda')" ontouchend="sendCommand('parar')">&#8592;</button>
    <button id="parar" class="btn" onclick="sendCommand('parar')">STOP</button>
    <button id="direita" class="btn" onmousedown="sendCommand('direita')" onmouseup="sendCommand('parar')" ontouchstart="sendCommand('direita')" ontouchend="sendCommand('parar')">&#8594;</button>
    <div class="placeholder"></div><button id="tras" class="btn" onmousedown="sendCommand('tras')" onmouseup="sendCommand('parar')" ontouchstart="sendCommand('tras')" ontouchend="sendCommand('parar')">&#8595;</button><div class="placeholder"></div>
  </div>
<script>function sendCommand(action){fetch('/control?action='+action);}</script>
</body></html>
)rawliteral";


void inicializarSensoresDistancia();
void lerSensoresDistancia();
void lerSensoresLinha();
void atualizarDisplay();
void acaoFrente();
void acaoTras();
void acaoDireita();
void acaoEsquerda();
void acaoParar();

void setup() {
  Serial.begin(115200);

  // --- Inicialização do Display OLED ---
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("Falha ao iniciar SSD1306"));
    for(;;);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println("Iniciando Robô...");
  display.display();

  // --- Inicialização dos Sensores de Distância ---
  inicializarSensoresDistancia();

  // --- Configuração dos Pinos dos Sensores de Linha ---
  pinMode(TCRT_PIN_L1, INPUT);
  pinMode(TCRT_PIN_L2, INPUT);
  pinMode(TCRT_PIN_L3, INPUT);

  // --- Configuração dos Pinos dos Motores ---
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  digitalWrite(ENA, HIGH); digitalWrite(ENB, HIGH);
  acaoParar(); // Garante que o robô comece parado

  // --- Inicialização do Wi-Fi e Servidor Web ---
  WiFi.softAP(ssid, password);
  Serial.print("Robo AP IP: ");
  Serial.println(WiFi.softAPIP());
  
  // --- Configuração das Rotas do Servidor Web ---
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html);
  });
  server.on("/control", HTTP_GET, [] (AsyncWebServerRequest *request) {
    if (request->hasParam("action")) {
      String action = request->getParam("action")->value();
      if (action == "frente") acaoFrente();
      else if (action == "tras") acaoTras();
      else if (action == "direita") acaoDireita();
      else if (action == "esquerda") acaoEsquerda();
      else if (action == "parar") acaoParar();
    }
    request->send(200, "text/plain", "OK");
  });
  server.begin();
  
  direcaoAtual = "Parado";
}

void loop() {
  lerSensoresDistancia();
  lerSensoresLinha();
  atualizarDisplay();
  delay(100); // Pequeno delay para não sobrecarregar o I2C e o display
}

void atualizarDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // --- Cabeçalho na área AMARELA ---
  display.setCursor(0, 4); // Y=4 posiciona o texto no meio da faixa amarela (0-15)
  display.print("Distancia");
  display.setCursor(70, 4);
  display.print("| Linha");

  // --- Dados dos Sensores na área AZUL ---
  int y_start = 18; // Ponto Y inicial para os dados, abaixo da área amarela
  int line_height = 10; // Espaçamento entre linhas de dados
  char buffer[12];
  
  // Coluna da Esquerda: Distância
  sprintf(buffer, "S1:%4dmm", dist1_mm);
  display.setCursor(0, y_start); display.print(buffer);
  
  sprintf(buffer, "S2:%4dmm", dist2_mm);
  display.setCursor(0, y_start + line_height); display.print(buffer);

  sprintf(buffer, "S3:%4dmm", dist3_mm);
  display.setCursor(0, y_start + (line_height * 2)); display.print(buffer);

  // Coluna da Direita: Linha
  sprintf(buffer, "| S1: %c", linha1_char);
  display.setCursor(70, y_start); display.print(buffer);
  
  sprintf(buffer, "| S2: %c", linha2_char);
  display.setCursor(70, y_start + line_height); display.print(buffer);

  sprintf(buffer, "| S3: %c", linha3_char);
  display.setCursor(70, y_start + (line_height * 2)); display.print(buffer);

  // Seção Inferior: Direção
  display.drawFastHLine(0, 50, display.width(), SSD1306_WHITE);
  display.setCursor(0, 54);
  display.print("Direcao: ");
  display.print(direcaoAtual);
  
  display.display();
}

void lerSensoresDistancia(){
  VL53L0X_RangingMeasurementData_t measure;
  
  lox_dist1.rangingTest(&measure, false);
  if (measure.RangeStatus != 4) dist1_mm = measure.RangeMilliMeter; else dist1_mm = 9999;

  lox_dist2.rangingTest(&measure, false);
  if (measure.RangeStatus != 4) dist2_mm = measure.RangeMilliMeter; else dist2_mm = 9999;
  
  lox_dist3.rangingTest(&measure, false);
  if (measure.RangeStatus != 4) dist3_mm = measure.RangeMilliMeter; else dist3_mm = 9999;
}

void lerSensoresLinha() {
  // Assumindo que 0 (LOW) é preto e 1 (HIGH) é branco
  linha1_char = (digitalRead(TCRT_PIN_L1) == LOW) ? 'P' : 'B';
  linha2_char = (digitalRead(TCRT_PIN_L2) == LOW) ? 'P' : 'B';
  linha3_char = (digitalRead(TCRT_PIN_L3) == LOW) ? 'P' : 'B';
}


// --- Funções de Ação dos Motores ---
void acaoFrente() { direcaoAtual = "Frente"; digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); }
void acaoTras() { direcaoAtual = "Tras"; digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); }
void acaoDireita() { direcaoAtual = "Direita"; digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); }
void acaoEsquerda() { direcaoAtual = "Esquerda"; digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); }
void acaoParar() { direcaoAtual = "Parado"; digitalWrite(IN1, LOW); digitalWrite(IN2, LOW); digitalWrite(IN3, LOW); digitalWrite(IN4, LOW); }

void inicializarSensoresDistancia() {
  pinMode(XSHUT_PIN_D1, OUTPUT); digitalWrite(XSHUT_PIN_D1, LOW);
  pinMode(XSHUT_PIN_D2, OUTPUT); digitalWrite(XSHUT_PIN_D2, LOW);
  pinMode(XSHUT_PIN_D3, OUTPUT); digitalWrite(XSHUT_PIN_D3, LOW);
  delay(50);
  
  Wire.begin(21, 22);

  digitalWrite(XSHUT_PIN_D1, HIGH); delay(10);
  if(!lox_dist1.begin(0x30)) { Serial.println(F("Falha S1")); while(1); }

  digitalWrite(XSHUT_PIN_D3, HIGH); delay(10);
  if(!lox_dist3.begin(0x31)) { Serial.println(F("Falha S3")); while(1); }

  digitalWrite(XSHUT_PIN_D2, HIGH); delay(10);
  if(!lox_dist2.begin()) { Serial.println(F("Falha S2")); while(1); }

  Serial.println(F("Sensores de distancia OK!"));
}
