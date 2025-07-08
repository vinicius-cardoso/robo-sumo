#include <WiFi.h>
#include <ArduinoOTA.h>
#include <WebServer.h>
#include <Wire.h> // Para display I2C, como OLED
#include <Adafruit_GFX.h> // Para display OLED
#include <Adafruit_SSD1306.h> // Para display OLED

// --- Configurações do Access Point (AP) e OTA ---
const char* AP_SSID = "RoboSumoGrupoA"; // SSID da rede Wi-Fi que o ESP32 irá criar
const char* AP_PASS = "caveirao";      // Senha da rede Wi-Fi
const IPAddress localIP(192, 168, 4, 2); // IP fixo do ESP32
const IPAddress gateway(192, 168, 4, 1); // Gateway para a rede criada pelo ESP32
const IPAddress subnet(255, 255, 255, 0); // Máscara de sub-rede
const char* OTA_HOSTNAME = "robo-sumo-esp32";

// --- Configurações dos Pinos Analógicos ---
const int pinoAnalogico4 = 4;
const int pinoAnalogico5 = 5;
const int pinoAnalogico15 = 15;

// --- Configuração do Servidor Web ---
WebServer server(80); // Cria um servidor web na porta 80 (HTTP padrão)

// --- Configuração do Display OLED (assumindo um SSD1306 128x64 I2C) ---
#define SCREEN_WIDTH 128 // Largura do display OLED, em pixels
#define SCREEN_HEIGHT 64 // Altura do display OLED, em pixels
#define OLED_RESET -1    // Pino de reset # (ou -1 se compartilhar o pino de reset do Arduino)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// --- Função para configurar o OTA ---
void configurarOTA() {
    ArduinoOTA.setHostname(OTA_HOSTNAME);

    ArduinoOTA.onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH) {
            type = "sketch";
        } else { // U_SPIFFS
            type = "filesystem";
        }
        Serial.println("Iniciando atualização " + type); // Para depuração, mesmo sem monitor serial
        // Atualiza o display OLED
        display.clearDisplay();
        display.setTextSize(2);
        display.setTextColor(WHITE);
        display.setCursor(0, 0);
        display.println("ATUALIZANDO");
        display.display();
    });

    ArduinoOTA.onEnd([]() {
        Serial.println("\nAtualização Concluída!"); // Para depuração
        // Atualiza o display OLED
        display.clearDisplay();
        display.setCursor(0, 0);
        display.setTextSize(1);
        display.setTextColor(WHITE);
        display.println("Atualizacao\nConcluida!");
        display.display();
        delay(1000); // Pequeno atraso para o usuário ver a mensagem
    });

    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progresso: %u%%\r", (progress / (total / 100))); // Para depuração
        // Atualiza a barra de progresso no display OLED
        display.drawRect(14, 32, 100, 10, WHITE); // Desenha o contorno da barra
        // Preenche a barra de acordo com o progresso
        display.fillRect(14, 32, (progress * 100 / total), 10, WHITE);
        display.display();
    });

    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("Erro OTA[%u]: ", error); // Para depuração
        if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR) Serial.println("End Failed");
        
        // Exibe erro no display
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(WHITE);
        display.setCursor(0, 0);
        display.println("ERRO OTA!");
        display.printf("Cod: %u", error);
        display.display();
    });

    ArduinoOTA.begin();
    Serial.println("OTA configurado."); // Para depuração
}

// --- Função para lidar com a requisição da página principal ---
void handleRoot() {
    int valor4 = analogRead(pinoAnalogico4);
    int valor5 = analogRead(pinoAnalogico5);
    int valor15 = analogRead(pinoAnalogico15);

    String html = "<html>";
    html += "<head><meta name='viewport' content='width=device-width, initial-scale=1'>";
    html += "<meta http-equiv='refresh' content='5'>"; // Atualiza a página a cada 5 segundos
    html += "<title>Leituras Analógicas ESP32</title>";
    html += "<style>";
    html += "body { font-family: Arial, sans-serif; text-align: center; margin-top: 50px; background-color: #f0f0f0; }";
    html += ".container { background-color: #fff; padding: 20px; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); display: inline-block; }";
    html += "h1 { color: #333; }";
    html += "p { font-size: 1.2em; color: #555; margin: 10px 0; }";
    html += "</style>";
    html += "</head>";
    html += "<body>";
    html += "<div class='container'>";
    html += "<h1>Leituras Analógicas do RoboSumo</h1>";
    html += "<p><strong>Pino 4:</strong> " + String(valor4) + "</p>";
    html += "<p><strong>Pino 5:</strong> " + String(valor5) + "</p>";
    html += "<p><strong>Pino 15:</strong> " + String(valor15) + "</p>";
    html += "<p><i>Atualizado em: " + String(millis() / 1000) + "s</i></p>"; // Exibe o tempo de uptime
    html += "</div>";
    html += "</body>";
    html += "</html>";

    server.send(200, "text/html", html);
}

void setup() {
    Serial.begin(115200); // Mantenho para depuração, se tiver um serial disponível no futuro

    // --- Inicialização do Display OLED ---
    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Endereço I2C 0x3C ou 0x3D
        Serial.println(F("Falha ao alocar display SSD1306"));
        for(;;); // Não continuar, travar o código
    }
    display.display();
    delay(2000); // Pausa para inicialização
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0,0);
    display.println("Iniciando AP...");
    display.display();

    // --- Configuração do ESP32 como Access Point (AP) ---
    Serial.print("Configurando AP "); // Para depuração
    Serial.println(AP_SSID);

    WiFi.softAP(AP_SSID, AP_PASS); // Cria o Access Point
    WiFi.softAPConfig(localIP, gateway, subnet); // Define o IP fixo para o AP

    IPAddress myIP = WiFi.softAPIP();
    Serial.print("AP IP: "); // Para depuração
    Serial.println(myIP);

    display.clearDisplay();
    display.setCursor(0,0);
    display.println("AP Criado!");
    display.print("IP: ");
    display.println(myIP);
    display.display();

    // --- Configura o OTA ---
    configurarOTA();

    // --- Configura as rotas do servidor web ---
    server.on("/", handleRoot); // Quando acessar a raiz, chama handleRoot
    server.begin(); // Inicia o servidor web
    Serial.println("Servidor Web iniciado."); // Para depuração
    display.println("Servidor Web Ok!");
    display.display();
}

void loop() {
    ArduinoOTA.handle(); // Permite que o OTA processe as requisições
    server.handleClient(); // Permite que o servidor web processe as requisições dos clientes

    // A leitura analógica será feita dentro de handleRoot() quando a página for requisitada.
    // Nenhuma leitura constante é necessária aqui, a menos que você queira atualizar o display OLED em tempo real
    // com os valores dos sensores, independente de uma requisição web.
    // Exemplo:
    /*
    static unsigned long lastOLEDUpdate = 0;
    if (millis() - lastOLEDUpdate > 2000) { // Atualiza o OLED a cada 2 segundos
        display.clearDisplay();
        display.setCursor(0,0);
        display.setTextSize(1);
        display.setTextColor(WHITE);
        display.println("Leituras:");
        display.print("P4: "); display.println(analogRead(pinoAnalogico4));
        display.print("P5: "); display.println(analogRead(pinoAnalogico5));
        display.print("P15: "); display.println(analogRead(pinoAnalogico15));
        display.display();
        lastOLEDUpdate = millis();
    }
    */
}
