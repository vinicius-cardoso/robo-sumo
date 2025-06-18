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
#include <ArduinoJson.h>

// ==========================================================================
// Variáveis de tempo
// ==========================================================================
const int TEMPO_RE = 300; 
const int TEMPO_MANOBRA = 500; 

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

// --- Pinos dos Sensores de Linha (TCRT5000 ou similar com saída DIGITAL)
const int SENSOR_LINHA_DIREITA = 4;
const int SENSOR_LINHA_TRASEIRA = 5;
const int SENSOR_LINHA_ESQUERDA = 15;

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

// Variáveis para leituras dos sensores de linha (agora apenas o caractere)
char linha_direita_char = '?', linha_traseira_char = '?', linha_esquerda_char = '?'; // 'B' para branco, 'P' para preto
String direcaoAtual = "Iniciando";

unsigned long proximoUpdate = 0;
const long INTERVALO_UPDATE_MS = 100;

// Variáveis para o modo automático
bool modoAutomatico = false;
unsigned long tempoAcaoAuto = 0;

const int LIMIAR_DISTANCIA_INIMIGO_MM = 300; 
const int DIFERENCA_MINIMA_LATERAL_AJUSTE_MM = 40;

enum EstadoAuto {
    AVANCAR_PROCURANDO, 
    ATACAR_INIMIGO,     
    EVITAR_BORDA_RE,    
    EVITAR_BORDA_GIRAR  
};
EstadoAuto estadoAtualAuto = AVANCAR_PROCURANDO; 

// A variável limiarPretoBranco não é mais necessária para leitura digital

// ==========================================================================
// Página de Controle (HTML) - Removido controle de limiar
// ==========================================================================
const char PAGINA_CONTROLE_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head><title>Controle do Robo ESP32</title><meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no"><style>body{font-family:Arial,sans-serif;text-align:center;background:#282c34;color:white}h1{margin-top:20px}.btn-container{display:grid;grid-template-columns:repeat(3, 1fr);gap:10px;max-width:300px;margin:30px auto}.btn{padding:20px;font-size:24px;color:white;background-color:#61dafb;border:none;border-radius:10px;cursor:pointer;user-select:none;-webkit-tap-highlight-color:transparent}.btn:active{background-color:#21a1f0}.placeholder{visibility:hidden}#frente{grid-column:2 / 3}#esquerda{grid-column:1 / 2;grid-row:2 / 3}#parar{grid-column:2 / 3;grid-row:2 / 3;background-color:#e04444}#direita{grid-column:3 / 4;grid-row:2 / 3}#tras{grid-column:2 / 3;grid-row:3 / 4}
.mode-buttons{margin-top:20px} .mode-btn{padding:15px 30px;font-size:20px;margin:0 10px;border-radius:8px;cursor:pointer;color:white;border:none;}.mode-btn.auto{background-color:#28a745;}.mode-btn.manual{background-color:#ffc107;}
.sensor-data{margin-top:20px; text-align:left; max-width:300px; margin-left:auto; margin-right:auto; padding:10px; border:1px solid #444; border-radius:8px;}
.sensor-data p{margin:5px 0;}
</style></head><body><h1>Controle do Robo</h1><div class="mode-buttons"><button class="mode-btn auto" onclick="setMode('auto')">AUTO</button><button class="mode-btn manual" onclick="setMode('manual')">MANUAL</button></div>
<div class="btn-container"><div class="placeholder"></div><button id="frente" class="btn" onmousedown="sendCommand('frente')" onmouseup="sendCommand('parar')" ontouchstart="sendCommand('frente')" ontouchend="sendCommand('parar')">↑</button><div class="placeholder"></div><button id="esquerda" class="btn" onmousedown="sendCommand('esquerda')" onmouseup="sendCommand('parar')" ontouchstart="sendCommand('esquerda')" ontouchend="sendCommand('parar')">←</button><button id="parar" class="btn" onclick="sendCommand('parar')">STOP</button><button id="direita" class="btn" onmousedown="sendCommand('direita')" onmouseup="sendCommand('parar')" ontouchstart="sendCommand('direita')" ontouchend="sendCommand('parar')">→</button><div class="placeholder"></div><button id="tras" class="btn" onmousedown="sendCommand('tras')" onmouseup="sendCommand('parar')" ontouchstart="sendCommand('tras')" ontouchend="sendCommand('parar')">↓</button><div class="placeholder"></div></div>
<div class="sensor-data">
    <h2>Dados dos Sensores:</h2>
    <p>Distância Direita: <span id="distD">--</span> mm</p>
    <p>Distância Frontal: <span id="distF">--</span> mm</p>
    <p>Distância Esquerda: <span id="distE">--</span> mm</p>
    <p>Linha Direita: <span id="linhaD">--</span></p>
    <p>Linha Traseira: <span id="linhaT">--</span></p>
    <p>Linha Esquerda: <span id="linhaE">--</span></p>
    <p>Modo: <span id="currentMode">--</span></p>
    <p>Direção: <span id="currentDirection">--</span></p>
</div>
<script>
function sendCommand(action){fetch('/control?action='+action)}
function setMode(mode){fetch('/mode?set='+mode).then(response => { if(response.ok) { console.log('Modo alterado para: ' + mode); } else { console.error('Erro ao alterar modo.'); } })}
// Funções de limiar removidas
window.onload = function() {
    // Não precisa mais buscar o limiar
    setInterval(updateSensorData, 500); 
};
function updateSensorData() {
    fetch('/data')
        .then(response => response.json())
        .then(data => {
            document.getElementById('distD').innerText = data.distD;
            document.getElementById('distF').innerText = data.distF;
            document.getElementById('distE').innerText = data.distE;
            document.getElementById('linhaD').innerText = data.linhaD;
            // Linhas DA, TA, EA (analógicas) removidas da exibição
            document.getElementById('linhaT').innerText = data.linhaT;
            document.getElementById('linhaE').innerText = data.linhaE;
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
    randomSeed(analogRead(0)); // Ruído analógico ainda útil para randomSeed

    // analogReadResolution(12); // Não é mais estritamente necessário se não houver leituras analógicas, mas não prejudica.

    configurarPinos();
    configurarSensores();
    configurarRede();
    configurarOTA();

    modoAutomatico = true; 
    estadoAtualAuto = AVANCAR_PROCURANDO; 
    direcaoAtual = "Iniciando Auto";
    Serial.println("Robo pronto. MODO AUTOMATICO (DIGITAL) ATIVADO PARA TESTE.");
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
// Funções de Configuração
// ==========================================================================
void configurarPinos() {
    pinMode(MOTOR_A_ENA, OUTPUT);
    pinMode(MOTOR_A_IN1, OUTPUT);
    pinMode(MOTOR_A_IN2, OUTPUT);
    pinMode(MOTOR_B_ENB, OUTPUT);
    pinMode(MOTOR_B_IN3, OUTPUT);
    pinMode(MOTOR_B_IN4, OUTPUT);

    // Configura pinos dos sensores de linha como ENTRADA
    pinMode(SENSOR_LINHA_DIREITA, INPUT);
    pinMode(SENSOR_LINHA_TRASEIRA, INPUT);
    pinMode(SENSOR_LINHA_ESQUERDA, INPUT);
    // Considere INPUT_PULLUP se seus sensores puxam para LOW no preto e flutuam no branco.
    // Ex: pinMode(SENSOR_LINHA_DIREITA, INPUT_PULLUP);

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
    Serial.println(F("Sensores de linha configurados para leitura DIGITAL."));
}

void configurarRede() {
    WiFi.softAP(WIFI_SSID, WIFI_PASS);
    Serial.print("Robo AP IP: ");
    Serial.println(WiFi.softAPIP());

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send_P(200, "text/html", PAGINA_CONTROLE_HTML);
    });

    server.on("/control", HTTP_GET, [](AsyncWebServerRequest *request) {
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
                estadoAtualAuto = AVANCAR_PROCURANDO; 
                pararMotores(); 
                Serial.println("Modo automatico (DIGITAL) ATIVADO via Web");
            } else if (mode == "manual") {
                modoAutomatico = false;
                pararMotores();
                Serial.println("Modo manual ATIVADO via Web");
            }
        }
        request->send(200, "text/plain", "OK");
    });

    // Endpoints /setThreshold e /getThreshold removidos pois não são mais necessários

    server.on("/data", HTTP_GET, [](AsyncWebServerRequest *request) {
        StaticJsonDocument<256> doc; 
        doc["distD"] = dist_direita_mm;
        doc["distF"] = dist_frontal_mm;
        doc["distE"] = dist_esquerda_mm;
        doc["linhaD"] = String(linha_direita_char);
        // doc["linhaDA"] = leitura_linha_direita_analog; // Removido
        doc["linhaT"] = String(linha_traseira_char);
        // doc["linhaTA"] = leitura_linha_traseira_analog; // Removido
        doc["linhaE"] = String(linha_esquerda_char);
        // doc["linhaEA"] = leitura_linha_esquerda_analog; // Removido
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
    ArduinoOTA.onStart([]() { /* ... código OTA ... */ });
    ArduinoOTA.onEnd([]() { /* ... código OTA ... */ });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) { /* ... código OTA ... */ });
    ArduinoOTA.onError([](ota_error_t error) { /* ... código OTA ... */ });
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

    // Leitura DIGITAL dos sensores de linha
    // Assumindo: HIGH (1) = Branco (Borda), LOW (0) = Preto (Pista)
    linha_direita_char = digitalRead(SENSOR_LINHA_DIREITA) ? 'B' : 'P';
    linha_traseira_char = digitalRead(SENSOR_LINHA_TRASEIRA) ? 'B' : 'P';
    linha_esquerda_char = digitalRead(SENSOR_LINHA_ESQUERDA) ? 'B' : 'P';
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

    char buffer[16]; // Buffer aumentado ligeiramente para segurança
    sprintf(buffer, "DD:%4dmm", dist_direita_mm);
    display.setCursor(0, Y_DATA_START);
    display.print(buffer);
    
    sprintf(buffer, "| LD:%c", linha_direita_char); // Removido valor analógico
    display.setCursor(X_COLUNA_2, Y_DATA_START);
    display.print(buffer);

    sprintf(buffer, "DF:%4dmm", dist_frontal_mm);
    display.setCursor(0, Y_DATA_START + LINE_HEIGHT);
    display.print(buffer);
    
    sprintf(buffer, "| LT:%c", linha_traseira_char); // Removido valor analógico
    display.setCursor(X_COLUNA_2, Y_DATA_START + LINE_HEIGHT);
    display.print(buffer);

    sprintf(buffer, "DE:%4dmm", dist_esquerda_mm);
    display.setCursor(0, Y_DATA_START + (LINE_HEIGHT * 2));
    display.print(buffer);

    sprintf(buffer, "| LE:%c", linha_esquerda_char); // Removido valor analógico
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

void moverFrente() { /* ... sem alterações ... */ }
void moverTras() { /* ... sem alterações ... */ }
void virarDireita() { /* ... sem alterações ... */ }
void virarEsquerda() { /* ... sem alterações ... */ }
void pararMotores() { /* ... sem alterações ... */ }

// Lógica de controle automático (sem alterações na lógica em si, apenas depende da leitura digital dos sensores de linha)
void controleAutomatico() {
    bool bordaDireitaDetectada = (linha_direita_char == 'B');
    bool bordaEsquerdaDetectada = (linha_esquerda_char == 'B');
    bool bordaTraseiraDetectada = (linha_traseira_char == 'B');
    bool algumaBordaFrontalDetectada = bordaDireitaDetectada || bordaEsquerdaDetectada;
    bool algumaBordaDetectada = algumaBordaFrontalDetectada || bordaTraseiraDetectada;

    bool inimigoDetectadoFrontal = (dist_frontal_mm < LIMIAR_DISTANCIA_INIMIGO_MM && dist_frontal_mm > 0 && dist_frontal_mm != 9999);

    switch (estadoAtualAuto) {
        case AVANCAR_PROCURANDO:
            direcaoAtual = "Avancando";
            moverFrente();

            if (algumaBordaDetectada) {
                Serial.println("AVANCANDO -> Borda! Iniciando evasao.");
                pararMotores(); 
                estadoAtualAuto = EVITAR_BORDA_RE;
                tempoAcaoAuto = millis(); 
            } else if (inimigoDetectadoFrontal) {
                Serial.println("AVANCAR -> Inimigo detectado! Atacando.");
                estadoAtualAuto = ATACAR_INIMIGO;
            }
            break;

        case ATACAR_INIMIGO:
            if (algumaBordaDetectada) {
                Serial.println("ATACANDO -> Borda! Prioridade para evasao.");
                pararMotores();
                estadoAtualAuto = EVITAR_BORDA_RE;
                tempoAcaoAuto = millis();
                break; 
            }

            if (!inimigoDetectadoFrontal) {
                Serial.println("ATACANDO -> Inimigo frontal perdido. Voltando a procurar.");
                pararMotores(); 
                estadoAtualAuto = AVANCAR_PROCURANDO;
                break; 
            }

            int dE_comp = (dist_esquerda_mm == 9999) ? 10000 : dist_esquerda_mm;
            int dD_comp = (dist_direita_mm == 9999) ? 10000 : dist_direita_mm;

            if (dE_comp < (dD_comp - DIFERENCA_MINIMA_LATERAL_AJUSTE_MM) && dE_comp < LIMIAR_DISTANCIA_INIMIGO_MM) {
                direcaoAtual = "Ataca+AjusteEsq";
                Serial.println(direcaoAtual);
                virarEsquerda(); 
            } 
            else if (dD_comp < (dE_comp - DIFERENCA_MINIMA_LATERAL_AJUSTE_MM) && dD_comp < LIMIAR_DISTANCIA_INIMIGO_MM) {
                direcaoAtual = "Ataca+AjusteDir";
                Serial.println(direcaoAtual);
                virarDireita(); 
            } 
            else {
                direcaoAtual = "Atacando Reto";
                Serial.println(direcaoAtual);
                moverFrente(); 
            }
            break; 

        case EVITAR_BORDA_RE:
            direcaoAtual = "Evasao: Re";
            moverTras();
            if (millis() - tempoAcaoAuto >= TEMPO_RE) { 
                pararMotores();
                estadoAtualAuto = EVITAR_BORDA_GIRAR;
                tempoAcaoAuto = millis(); 
                Serial.println("Evasao: Re concluida. Girando.");
            }
            break;

        case EVITAR_BORDA_GIRAR:
            if (bordaDireitaDetectada && !bordaEsquerdaDetectada && !bordaTraseiraDetectada) {
                direcaoAtual = "Evasao: Gira Esq";
                virarEsquerda(); 
                Serial.println("Evasao: Borda Dir, virando Esq.");
            } else if (bordaEsquerdaDetectada && !bordaDireitaDetectada && !bordaTraseiraDetectada) {
                direcaoAtual = "Evasao: Gira Dir";
                virarDireita(); 
                Serial.println("Evasao: Borda Esq, virando Dir.");
            } else {
                if (random(2) == 0) { 
                    direcaoAtual = "Evasao: Gira Esq (Aleat)";
                    virarEsquerda();
                    Serial.println("Evasao: Girando aleatoriamente para ESQUERDA.");
                } else {
                    direcaoAtual = "Evasao: Gira Dir (Aleat)";
                    virarDireita();
                    Serial.println("Evasao: Girando aleatoriamente para DIREITA.");
                }
            }
            
            if (millis() - tempoAcaoAuto >= TEMPO_MANOBRA) { 
                pararMotores();
                estadoAtualAuto = AVANCAR_PROCURANDO; 
                Serial.println("Evasao: Giro concluido, voltando a AVANCAR_PROCURANDO.");
            }
            break;
    }
}