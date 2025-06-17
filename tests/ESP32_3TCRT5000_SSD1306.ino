#include <Wire.h>               // Para comunicação I2C
#include <Adafruit_GFX.h>       // Biblioteca principal para gráficos
#include <Adafruit_SSD1306.h>   // Biblioteca para o display SSD1306

// Pinos GPIO do ESP32 para conectar a saída D0 dos sensores TCRT5000
const int SENSOR_PIN_1 = 4;  // Sensor 1 conectado ao GPIO4
const int SENSOR_PIN_2 = 5;  // Sensor 2 conectado ao GPIO5
const int SENSOR_PIN_3 = 15; // Sensor 3 conectado ao GPIO15 (MTDO - atenção se HIGH no boot)

// Configurações do Display OLED SSD1306
#define SCREEN_WIDTH 128 // Largura do OLED em pixels
#define SCREEN_HEIGHT 64 // Altura do OLED em pixels
#define OLED_RESET    -1 // Pino de Reset do OLED (-1 se estiver compartilhando o reset do ESP32)
#define SCREEN_ADDRESS 0x3C // Endereço I2C do SSD1306 (verifique o seu, 0x3C é comum para 128x64)

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
bool displayInitialized = false; // Flag para verificar se o display foi inicializado

void setup() {
  // Inicializa a comunicação Serial a 115200 bps
  Serial.begin(115200);
  Serial.println("\nTeste de 3 Sensores TCRT5000 com ESP32 e Display OLED");
  Serial.println("-------------------------------------------------------");

  // Configura os pinos dos sensores como ENTRADA (INPUT)
  pinMode(SENSOR_PIN_1, INPUT);
  pinMode(SENSOR_PIN_2, INPUT);
  pinMode(SENSOR_PIN_3, INPUT);

  // Inicia a comunicação I2C (para o display OLED)
  // Usando os pinos padrão do ESP32: GPIO21 (SDA), GPIO22 (SCL)
  Wire.begin(21, 22); 

  // Inicializa o display OLED
  Serial.println(F("Inicializando display OLED..."));
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("Falha ao iniciar SSD1306. Verifique conexões/endereço."));
    // displayInitialized permanecerá false
  } else {
    Serial.println(F("Display OLED inicializado."));
    displayInitialized = true;
    display.clearDisplay();
    display.display(); // Aplica a limpeza
  }

  Serial.println("Pinos dos sensores configurados. Iniciando leituras...");
  Serial.print("Formato Serial: S1 (GPIO"); Serial.print(SENSOR_PIN_1);
  Serial.print(") | S2 (GPIO"); Serial.print(SENSOR_PIN_2);
  Serial.print(") | S3 (GPIO"); Serial.print(SENSOR_PIN_3); Serial.println(")");
  Serial.println("-----------------------------------------------------------------");
}

void loop() {
  // Lê o estado digital de cada sensor
  // digitalRead() retorna HIGH (1) ou LOW (0)
  int estadoSensor1 = digitalRead(SENSOR_PIN_1);
  int estadoSensor2 = digitalRead(SENSOR_PIN_2);
  int estadoSensor3 = digitalRead(SENSOR_PIN_3);

  // Mostra os valores no Monitor Serial
  Serial.print("S1: "); Serial.print(estadoSensor1);
  Serial.print(" | S2: "); Serial.print(estadoSensor2);
  Serial.print(" | S3: "); Serial.println(estadoSensor3);

  // Atualiza o Display OLED
  if (displayInitialized) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE); // Pixels acenderão na sua cor nativa (amarelo/azul)

    // Título "Sensores" na parte superior (provavelmente amarela)
    // Texto com tamanho 1 tem 8 pixels de altura.
    // y=4 posiciona bem se a faixa amarela tiver ~16 pixels de altura.
    display.setCursor(0, 4); 
    display.println(F("Sensores"));

    // Leituras dos Sensores na parte azul
    // Assumindo que a parte azul começa em y=16 ou um pouco depois.
    // Uma entrelinha de 10 pixels (8 da altura da fonte + 2 de espaço)

    // Sensor 1
    display.setCursor(0, 18); // Início da "parte azul"
    display.print(F("S1: "));
    display.println(estadoSensor1);

    // Sensor 2
    display.setCursor(0, 28); // (18 + 10)
    display.print(F("S2: "));
    display.println(estadoSensor2);

    // Sensor 3
    display.setCursor(0, 38); // (28 + 10)
    display.print(F("S3: "));
    display.println(estadoSensor3);

    display.display(); // Envia o buffer para o display
  }

  // Aguarda um pouco antes da próxima leitura
  delay(500); // Atualiza a cada 0.5 segundos
}
