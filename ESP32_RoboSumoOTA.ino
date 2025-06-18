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
#include <ArduinoJson.h> // Adicione esta biblioteca para facilitar a criação de JSON

// ==========================================================================
// Variáveis de tempo
// ==========================================================================
const int TEMPO_RE = 500;
const int TEMPO_MANOBRA = 700;

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

// --- Pinos dos Sensores de Linha (TCRT5000) - Agora usados como ANALÓGICOS
const int SENSOR_LINHA_DIREITA = 4;  // GPIO4 - ADC1_CH0
const int SENSOR_LINHA_TRASEIRA = 5; // GPIO5 - ADC1_CH1
const int SENSOR_LINHA_ESQUERDA = 15; // GPIO15 - ADC2_CH3

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

// Variáveis para leituras analógicas dos sensores de linha
int leitura_linha_direita_analog = 0;
int leitura_linha_traseira_analog = 0;
int leitura_linha_esquerda_analog = 0;

char linha_direita_char = '?', linha_traseira_char = '?', linha_esquerda_char = '?'; // 'B' para branco, 'P' para preto, '?' para não determinado
String direcaoAtual = "Iniciando";

unsigned long proximoUpdate = 0;
const long INTERVALO_UPDATE_MS = 100;

// Variáveis para o modo automático
bool modoAutomatico = false;
unsigned long tempoAcaoAuto = 0;
enum EstadoAuto {
    FRENTE,
    RE,
    GIRAR_ALEATORIO
};
EstadoAuto estadoAtualAuto = FRENTE;

// --- Limiares para detecção de linha (ajuste esses valores!) ---
// O valor ideal depende da iluminação, altura do sensor, e da cor da superfície.
// Valores mais altos tendem a ser para preto, mais baixos para branco,
// mas isso pode variar dependendo da fiação (pull-up/pull-down).
// Faça testes com analogRead para encontrar os valores apropriados.
int limiarPretoBranco = 1000; // Exemplo: Valores abaixo de 1000 = BRANCO, acima = PRETO
                              // O ESP32 tem ADC de 12 bits, então as leituras vão de 0 a 4095.
                              // Ajuste este limiar após testar seu robô no ringue.

// ==========================================================================
// Página de Controle (HTML)
// ==========================================================================
const char PAGINA_CONTROLE_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head><title>Controle do Robo ESP32</title><meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no"><style>body{font-family:Arial,sans-serif;text-align:center;background:#282c34;color:white}h1{margin-top:20px}.btn-container{display:grid;grid-template-columns:repeat(3, 1fr);gap:10px;max-width:300px;margin:30px auto}.btn{padding:20px;font-size:24px;color:white;background-color:#61dafb;border:none;border-radius:10px;cursor:pointer;user-select:none;-webkit-tap-highlight-color:transparent}.btn:active{background-color:#21a1f0}.placeholder{visibility:hidden}#frente{grid-column:2 / 3}#esquerda{grid-column:1 / 2;grid-row:2 / 3}#parar{grid-column:2 / 3;grid-row:2 / 3;background-color:#e04444}#direita{grid-column:3 / 4;grid-row:2 / 3}#tras{grid-column:2 / 3;grid-row:3 / 4}
.mode-buttons{margin-top:20px} .mode-btn{padding:15px 30px;font-size:20px;margin:0 10px;border-radius:8px;cursor:pointer;color:white;border:none;}.mode-btn.auto{background-color:#28a745;}.mode-btn.manual{background-color:#ffc107;}
.threshold-control{margin-top:20px;} .threshold-control label{font-size:16px;} .threshold-control input[type="range"]{width:200px;}.threshold-control span{font-size:16px; margin-left:10px;}
.sensor-data{margin-top:20px; text-align:left; max-width:300px; margin-left:auto; margin-right:auto; padding:10px; border:1px solid #444; border-radius:8px;}
.sensor-data p{margin:5px 0;}
</style></head><body><h1>Controle do Robo</h1><div class="mode-buttons"><button class="mode-btn auto" onclick="setMode('auto')">AUTO</button><button class="mode-btn manual" onclick="setMode('manual')">MANUAL</button></div>
<div class="btn-container"><div class="placeholder"></div><button id="frente" class="btn" onmousedown="sendCommand('frente')" onmouseup="sendCommand('parar')" ontouchstart="sendCommand('frente')" ontouchend="sendCommand('parar')">&#8593;</button><div class="placeholder"></div><button id="esquerda" class="btn" onmousedown="sendCommand('esquerda')" onmouseup="sendCommand('parar')" ontouchstart="sendCommand('esquerda')" ontouchend="sendCommand('parar')">&#8592;</button><button id="parar" class="btn" onclick="sendCommand('parar')">STOP</button><button id="direita" class="btn" onmousedown="sendCommand('direita')" onmouseup="sendCommand('parar')" ontouchstart="sendCommand('direita')" ontouchend="sendCommand('parar')">&#8594;</button><div class="placeholder"></div><button id="tras" class="btn" onmousedown="sendCommand('tras')" onmouseup="sendCommand('parar')" ontouchstart="sendCommand('tras')" ontouchend="sendCommand('parar')">&#8595;</button><div class="placeholder"></div></div>
<div class="threshold-control">
    <label for="limiar">Limiar Preto/Branco:</label>
    <input type="range" id="limiar" min="0" max="4095" value="1000" onchange="setThreshold(this.value)">
    <span id="limiarValue">1000</span>
</div>
<div class="sensor-data">
    <h2>Dados dos Sensores:</h2>
    <p>Distância Direita: <span id="distD">--</span> mm</p>
    <p>Distância Frontal: <span id="distF">--</span> mm</p>
    <p>Distância Esquerda: <span id="distE">--</span> mm</p>
    <p>Linha Direita: <span id="linhaD">--</span> (<span id="linhaDA">--</span>)</p>
    <p>Linha Traseira: <span id="linhaT">--</span> (<span id="linhaTA">--</span>)</p>
    <p>Linha Esquerda: <span id="linhaE">--</span> (<span id="linhaEA">--</span>)</p>
    <p>Modo: <span id="currentMode">--</span></p>
    <p>Direção: <span id="currentDirection">--</span></p>
</div>
<script>
function sendCommand(action){fetch('/control?action='+action)}
function setMode(mode){fetch('/mode?set='+mode).then(response => { if(response.ok) { console.log('Modo alterado para: ' + mode); } else { console.error('Erro ao alterar modo.'); } })}
function setThreshold(value){
    document.getElementById('limiarValue').innerText = value;
    fetch('/setThreshold?value='+value);
}
// Pega o valor atual do limiar ao carregar a página
window.onload = function() {
    fetch('/getThreshold').then(response => response.text()).then(data => {
        document.getElementById('limiar').value = data;
        document.getElementById('limiarValue').innerText = data;
    });
    setInterval(updateSensorData, 500); // Atualiza os dados a cada 0.5 segundos
};

function updateSensorData() {
    fetch('/data')
        .then(response => response.json())
        .then(data => {
            document.getElementById('distD').innerText = data.distD;
            document.getElementById('distF').innerText = data.distF;
            document.getElementById('distE').innerText = data.distE;
            document.getElementById('linhaD').innerText = data.linhaD;
            document.getElementById('linhaDA').innerText = data.linhaDA;
            document.getElementById('linhaT').innerText = data.linhaT;
            document.getElementById('linhaTA').innerText = data.linhaTA;
            document.getElementById('linhaE').innerText = data.linhaE;
            document.getElementById('linhaEA').innerText = data.linhaEA;
            document.getElementById('currentMode').innerText = data.mode;
            document.getElementById('currentDirection').innerText = data.direction;
        })
        .catch(error => console.error('Erro ao buscar dados do sensor:', error));
}
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
void controleAutomatico();

// ==========================================================================
// Setup Principal
// ==========================================================================
void setup() {
    Serial.begin(115200);
    randomSeed(analogRead(0)); // Inicializa o gerador de números aleatórios com ruído analógico

    // Configura a resolução do ADC para 12 bits (padrão do ESP32)
    analogReadResolution(12);

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

    if (modoAutomatico) {
        controleAutomatico();
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
        // Ações de controle manual só funcionam se não estiver no modo automático
        if (!modoAutomatico && request->hasParam("action")) {
            String action = request->getParam("action")->value();
            if (action == "frente") moverFrente();
            else if (action == "tras") moverTras();
            else if (action == "direita") virarDireita();
            else if (action == "esquerda") virarEsquerda();
            else if (action == "parar") pararMotores();
        }
        request->send(200, "text/plain", "OK");
    });

    server.on("/mode", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (request->hasParam("set")) {
            String mode = request->getParam("set")->value();
            if (mode == "auto") {
                modoAutomatico = true;
                estadoAtualAuto = FRENTE; // Reseta o estado para começar andando para frente
                pararMotores(); // Garante que os motores parem antes de iniciar o modo AUTO
                Serial.println("Modo automatico ATIVADO");
            } else if (mode == "manual") {
                modoAutomatico = false;
                pararMotores(); // Para os motores ao sair do modo automático
                Serial.println("Modo manual ATIVADO");
            }
        }
        request->send(200, "text/plain", "OK");
    });

    // Endpoint para configurar o limiar do sensor de linha
    server.on("/setThreshold", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (request->hasParam("value")) {
            limiarPretoBranco = request->getParam("value")->value().toInt();
            Serial.printf("Limiar de linha atualizado para: %d\n", limiarPretoBranco);
        }
        request->send(200, "text/plain", "OK");
    });

    // Endpoint para obter o limiar atual
    server.on("/getThreshold", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(200, "text/plain", String(limiarPretoBranco));
    });

    // NOVO: Endpoint para enviar os dados dos sensores
    server.on("/data", HTTP_GET, [](AsyncWebServerRequest *request) {
        StaticJsonDocument<256> doc; // Tamanho do documento JSON (ajuste se precisar de mais dados)
        doc["distD"] = dist_direita_mm;
        doc["distF"] = dist_frontal_mm;
        doc["distE"] = dist_esquerda_mm;
        doc["linhaD"] = String(linha_direita_char);
        doc["linhaDA"] = leitura_linha_direita_analog;
        doc["linhaT"] = String(linha_traseira_char);
        doc["linhaTA"] = leitura_linha_traseira_analog;
        doc["linhaE"] = String(linha_esquerda_char);
        doc["linhaEA"] = leitura_linha_esquerda_analog;
        doc["mode"] = modoAutomatico ? "AUTO" : "MANUAL";
        doc["direction"] = direcaoAtual;

        String jsonResponse;
        serializeJson(doc, jsonResponse);
        request->send(200, "application/json", jsonResponse);
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

    // Leitura analógica dos sensores de linha
    leitura_linha_direita_analog = analogRead(SENSOR_LINHA_DIREITA);
    leitura_linha_traseira_analog = analogRead(SENSOR_LINHA_TRASEIRA);
    leitura_linha_esquerda_analog = analogRead(SENSOR_LINHA_ESQUERDA);

    // Determina se é preto ('P') ou branco ('B') com base no limiar
    // Assumindo que valores MENORES significam BRANCO (mais reflexão) e MAIORES PRETO (menos reflexão)
    // Se seus sensores funcionam ao contrário, INVERTA a comparação ('>')
    linha_direita_char = (leitura_linha_direita_analog < limiarPretoBranco) ? 'B' : 'P';
    linha_traseira_char = (leitura_linha_traseira_analog < limiarPretoBranco) ? 'B' : 'P';
    linha_esquerda_char = (leitura_linha_esquerda_analog < limiarPretoBranco) ? 'B' : 'P';
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
    
    sprintf(buffer, "| LD:%c(%d)", linha_direita_char, leitura_linha_direita_analog);
    display.setCursor(X_COLUNA_2, Y_DATA_START);
    display.print(buffer);

    sprintf(buffer, "DF:%4dmm", dist_frontal_mm);
    display.setCursor(0, Y_DATA_START + LINE_HEIGHT);
    display.print(buffer);
    
    sprintf(buffer, "| LT:%c(%d)", linha_traseira_char, leitura_linha_traseira_analog);
    display.setCursor(X_COLUNA_2, Y_DATA_START + LINE_HEIGHT);
    display.print(buffer);

    sprintf(buffer, "DE:%4dmm", dist_esquerda_mm);
    display.setCursor(0, Y_DATA_START + (LINE_HEIGHT * 2));
    display.print(buffer);

    sprintf(buffer, "| LE:%c(%d)", linha_esquerda_char, leitura_linha_esquerda_analog);
    display.setCursor(X_COLUNA_2, Y_DATA_START + (LINE_HEIGHT * 2));
    display.print(buffer);

    display.drawFastHLine(0, 50, display.width(), SSD1306_WHITE);
    display.setCursor(0, 54);
    display.print("Dir: ");
    display.print(direcaoAtual);
    display.print(" | Modo: ");
    display.print(modoAutomatico ? "A" : "M");
    
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
    digitalWrite(MOTOR_A_IN1, LOW); // Motor A para frente (esquerda do robô)
    digitalWrite(MOTOR_A_IN2, HIGH); 
    digitalWrite(MOTOR_B_IN3, LOW);  // Motor B para trás (direita do robô)
    digitalWrite(MOTOR_B_IN4, HIGH); 
}

void virarEsquerda() {
    direcaoAtual = "Esquerda";
    digitalWrite(MOTOR_A_IN1, HIGH); // Motor A para trás (esquerda do robô)
    digitalWrite(MOTOR_A_IN2, LOW);
    digitalWrite(MOTOR_B_IN3, HIGH); // Motor B para frente (direita do robô)
    digitalWrite(MOTOR_B_IN4, LOW);
}

void pararMotores() {
    direcaoAtual = "Parado";
    digitalWrite(MOTOR_A_IN1, LOW);
    digitalWrite(MOTOR_A_IN2, LOW);
    digitalWrite(MOTOR_B_IN3, LOW);
    digitalWrite(MOTOR_B_IN4, LOW);
}

void controleAutomatico() {
    // Detecta borda: 'B' para branco (limiar atingido/cruzado)
    bool bordaDetectadaFrente = (linha_direita_char == 'B' || linha_esquerda_char == 'B');
    bool bordaDetectadaTras = (linha_traseira_char == 'B');

    switch (estadoAtualAuto) {
        case FRENTE:
            moverFrente();
            if (bordaDetectadaFrente || bordaDetectadaTras) {
                pararMotores();
                tempoAcaoAuto = millis();
                estadoAtualAuto = RE;
                Serial.println("Borda detectada, entrando em RE.");
            }
            break;

        case RE:
            moverTras();
            if (millis() - tempoAcaoAuto >= TEMPO_RE) { // Dá ré pelo tempo definido
                pararMotores();
                tempoAcaoAuto = millis();
                estadoAtualAuto = GIRAR_ALEATORIO;
                Serial.println("Ré concluída, girando aleatoriamente.");
            }
            break;

        case GIRAR_ALEATORIO:
            // Gira para um lado aleatório
            if (random(2) == 0) { // 0 para esquerda, 1 para direita
                virarEsquerda();
                Serial.println("Virando para ESQUERDA.");
            } else {
                virarDireita();
                Serial.println("Virando para DIREITA.");
            }
            
            if (millis() - tempoAcaoAuto >= TEMPO_MANOBRA) { // Gira pelo tempo definido
                pararMotores();
                estadoAtualAuto = FRENTE;
                Serial.println("Giro concluído, voltando para FRENTE.");
            }
            break;
    }
}