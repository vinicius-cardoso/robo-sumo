#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// --- Configurações do Wi-Fi ---
const char* ssid = "RoboSumoGrupoA"; // Nome da rede Wi-Fi que o robô irá criar
const char* password = "caveirao";  // Senha da rede Wi-Fi

// --- Cria os objetos do Servidor e Display ---
AsyncWebServer server(80); // Servidor na porta 80 (HTTP)
Adafruit_SSD1306 display(128, 64, &Wire, -1);

// --- Pinos dos Motores ---
const int ENA = 32;
const int ENB = 14;
const int IN1 = 33;
const int IN2 = 25;
const int IN3 = 26;
const int IN4 = 27;

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
    .btn {
      padding: 20px; font-size: 24px; color: white; background-color: #61dafb;
      border: none; border-radius: 10px; cursor: pointer; user-select: none;
      -webkit-tap-highlight-color: transparent; /* Remove o highlight azul no toque em mobile */
    }
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
  function sendCommand(action) {
    fetch('/control?action=' + action);
  }
</script>
</body>
</html>
)rawliteral";


// --- Declaração das Funções ---
void acaoFrente();
void acaoTras();
void acaoDireita();
void acaoEsquerda();
void acaoParar();
void atualizarDisplay(String direcao);

void setup() {
  Serial.begin(115200);

  // --- Inicialização do Display ---
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("Falha ao iniciar SSD1306"));
    for(;;);
  }
  display.clearDisplay();
  
  // --- Configuração dos Pinos dos Motores ---
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  digitalWrite(ENA, HIGH); digitalWrite(ENB, HIGH);
  acaoParar(); // Garante que o robô comece parado

  // --- Inicialização do Wi-Fi em modo Access Point ---
  Serial.print("Criando Ponto de Acesso: ");
  Serial.println(ssid);
  WiFi.softAP(ssid, password);
  Serial.print("Endereco IP do Robo: ");
  Serial.println(WiFi.softAPIP());
  
  atualizarDisplay(WiFi.softAPIP().toString()); // Mostra o IP no display ao iniciar

  // --- Configuração das Rotas do Servidor Web ---
  // Rota principal (/) - envia a página HTML
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html);
  });

  // Rota de controle (/control) - recebe os comandos dos botões
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

  // Inicia o servidor
  server.begin();
}

void loop() {
  // O loop fica vazio, pois o servidor web assíncrono cuida de tudo em background.
}

void atualizarDisplay(String direcao) {
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("Direcao:"));
  display.setTextSize(2);
  display.setCursor(0, 32); // Cursor ajustado para caber textos maiores como o IP
  display.println(direcao);
  display.display();
}

// --- Funções de Ação dos Motores (sem delays) ---
void acaoFrente() {
  atualizarDisplay("Frente");
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); // Motor A: Frente
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); // Motor B: Frente
}

void acaoTras() {
  atualizarDisplay("Tras");
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);  // Motor A: Trás
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);  // Motor B: Trás
}

void acaoDireita() {
  atualizarDisplay("Direita");
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); // Motor A: Frente
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);  // Motor B: Trás (gira para a direita)
}

void acaoEsquerda() {
  atualizarDisplay("Esquerda");
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);  // Motor A: Trás
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); // Motor B: Frente (gira para a esquerda)
}

void acaoParar() {
  atualizarDisplay("Parado");
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}
