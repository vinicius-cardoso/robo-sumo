#include <Wire.h>
#include "Adafruit_VL53L0X.h"
#include <Adafruit_GFX.h>      // Biblioteca principal para gráficos
#include <Adafruit_SSD1306.h>   // Biblioteca para o display SSD1306

// Pinos XSHUT para controlar cada sensor VL53L0X
//#define XSHUT_PIN_S1 14 // GPIO14 para o Sensor 1
//#define XSHUT_PIN_S2 16 // GPIO16 (RX2) para o Sensor 2
//#define XSHUT_PIN_S3 13 // GPIO13 para o Sensor 3
#define XSHUT_PIN_S1 18 // GPIO14 para o Sensor 1
#define XSHUT_PIN_S2 19 // GPIO16 (RX2) para o Sensor 2
#define XSHUT_PIN_S3 23 // GPIO13 para o Sensor 3

// Endereços I2C que usaremos
#define SENSOR1_ADDR 0x30 // Novo endereço para o Sensor 1
#define SENSOR3_ADDR 0x31 // Novo endereço para o Sensor 3
#define SENSOR2_ADDR 0x29 // O Sensor 2 usará o endereço padrão

// Objetos para cada sensor
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox3 = Adafruit_VL53L0X();

// Variáveis para armazenar as leituras de cada sensor
VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;
VL53L0X_RangingMeasurementData_t measure3;

// Configurações do Display OLED SSD1306
#define SCREEN_WIDTH 128 // Largura do OLED em pixels
#define SCREEN_HEIGHT 64 // Altura do OLED em pixels
#define OLED_RESET    -1 // Pino de Reset do OLED (-1 se estiver compartilhando o reset do ESP32)
#define SCREEN_ADDRESS 0x3C // Endereço I2C do SSD1306 (verifique o seu, 0x3C é comum para 128x64)

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
bool displayInitialized = false; // Flag para verificar se o display foi inicializado

void setup() {
  Serial.begin(115200);
  Serial.println(F("Teste com tres sensores VL53L0X no ESP32 e Display OLED"));

  // Inicializa o display OLED
  Serial.println(F("Inicializando display OLED..."));
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("Falha ao iniciar SSD1306. Verifique as conexões e o endereço."));
    // displayInitialized permanecerá false
  } else {
    Serial.println(F("Display OLED inicializado."));
    displayInitialized = true;
    display.clearDisplay(); // Limpa qualquer conteúdo residual
    display.display();      // Aplica a limpeza
    // Não vamos mais colocar um título fixo aqui, o loop cuidará disso.
  }

  // Configura os pinos XSHUT
  pinMode(XSHUT_PIN_S1, OUTPUT);
  digitalWrite(XSHUT_PIN_S1, LOW);
  pinMode(XSHUT_PIN_S2, OUTPUT);
  digitalWrite(XSHUT_PIN_S2, LOW);
  pinMode(XSHUT_PIN_S3, OUTPUT);
  digitalWrite(XSHUT_PIN_S3, LOW);
  delay(50);

  Wire.begin(21, 22); // GPIO21 para SDA, GPIO22 para SCL

  // --- Configurar Sensor 1 ---
  Serial.println(F("Configurando Sensor 1..."));
  digitalWrite(XSHUT_PIN_S1, HIGH); 
  delay(10); 
  if (!lox1.begin()) {
    Serial.println(F("Falha ao iniciar Sensor 1."));
    if (displayInitialized) {display.clearDisplay(); display.setTextSize(1); display.setTextColor(SSD1306_WHITE); display.setCursor(0,0); display.print("Erro S1 Init"); display.display();}
    while (1); 
  }
  Serial.println(F("Sensor 1 iniciado. Mudando endereço..."));
  if (!lox1.setAddress(SENSOR1_ADDR)) {
    Serial.print(F("Falha ao definir endereço S1 (0x")); Serial.print(SENSOR1_ADDR, HEX); Serial.println(F(")"));
    if (displayInitialized) {display.clearDisplay(); display.setTextSize(1); display.setTextColor(SSD1306_WHITE); display.setCursor(0,0); display.print("Erro S1 Addr"); display.display();}
    while(1); 
  }
  Serial.print(F("Sensor 1 OK no endereço 0x")); Serial.println(SENSOR1_ADDR, HEX);

  // --- Configurar Sensor 3 ---
  Serial.println(F("Configurando Sensor 3..."));
  digitalWrite(XSHUT_PIN_S3, HIGH); 
  delay(10); 
  if (!lox3.begin()) {
    Serial.println(F("Falha ao iniciar Sensor 3."));
    if (displayInitialized) {display.clearDisplay(); display.setTextSize(1); display.setTextColor(SSD1306_WHITE); display.setCursor(0,0); display.print("Erro S3 Init"); display.display();}
    while (1); 
  }
  Serial.println(F("Sensor 3 iniciado. Mudando endereço..."));
  if (!lox3.setAddress(SENSOR3_ADDR)) {
    Serial.print(F("Falha ao definir endereço S3 (0x")); Serial.print(SENSOR3_ADDR, HEX); Serial.println(F(")"));
    if (displayInitialized) {display.clearDisplay(); display.setTextSize(1); display.setTextColor(SSD1306_WHITE); display.setCursor(0,0); display.print("Erro S3 Addr"); display.display();}
    while(1); 
  }
  Serial.print(F("Sensor 3 OK no endereço 0x")); Serial.println(SENSOR3_ADDR, HEX);

  // --- Configurar Sensor 2 ---
  Serial.println(F("Configurando Sensor 2..."));
  digitalWrite(XSHUT_PIN_S2, HIGH); 
  delay(10); 
  if (!lox2.begin()) {
    Serial.println(F("Falha ao iniciar Sensor 2."));
    if (displayInitialized) {display.clearDisplay(); display.setTextSize(1); display.setTextColor(SSD1306_WHITE); display.setCursor(0,0); display.print("Erro S2 Init"); display.display();}
    while (1); 
  }
  Serial.print(F("Sensor 2 OK no endereço 0x")); Serial.println(SENSOR2_ADDR, HEX);

  Serial.println(F("Todos os sensores configurados. Iniciando leituras..."));
  Serial.println("------------------------------------------------------------");
}

void loop() {
  int distancia1_mm = 9999; 
  int distancia2_mm = 9999; 
  int distancia3_mm = 9999; 

  lox1.rangingTest(&measure1, false); 
  if (measure1.RangeStatus != 4) {  
    distancia1_mm = measure1.RangeMilliMeter;
  }

  lox2.rangingTest(&measure2, false);
  if (measure2.RangeStatus != 4) { 
    distancia2_mm = measure2.RangeMilliMeter;
  }

  lox3.rangingTest(&measure3, false); 
  if (measure3.RangeStatus != 4) {  
    distancia3_mm = measure3.RangeMilliMeter;
  }

  // --- Imprime as distâncias no Monitor Serial (opcional, para debug) ---
  Serial.print(F("S1: "));
  if (distancia1_mm == 9999) { Serial.print(F("Erro")); } else { Serial.print(distancia1_mm); Serial.print(F("mm")); }
  Serial.print(F(" | S2: "));
  if (distancia2_mm == 9999) { Serial.print(F("Erro")); } else { Serial.print(distancia2_mm); Serial.print(F("mm")); }
  Serial.print(F(" | S3: "));
  if (distancia3_mm == 9999) { Serial.println(F("Erro")); } else { Serial.print(distancia3_mm); Serial.println(F("mm")); }
  
  // --- Atualiza o Display OLED ---
  if (displayInitialized){ 
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE); // Pixels acenderão na sua cor nativa
    
    // Escreve "Sensores" na parte amarela (topo)
    // Texto com tamanho 1 tem 8 pixels de altura.
    // Se a parte amarela tem 16 pixels de altura (y=0 a y=15), y=4 é uma boa posição vertical.
    display.setCursor(0, 4); 
    display.println(F("Sensores"));
    // Opcional: uma linha divisória abaixo do título, ainda na parte amarela
    // display.drawLine(0, 15, SCREEN_WIDTH - 1, 15, SSD1306_WHITE);


    char buffer[20]; // Buffer para formatar a string

    // Leituras dos Sensores na parte azul
    // Assumindo que a parte azul começa em y=16 ou um pouco depois.
    // Vamos dar um espaço do título e da linha divisória (se usada).

    // Sensor 1
    display.setCursor(0, 18); // Início da parte azul
    display.print(F("S1: "));
    if (distancia1_mm == 9999) {
        display.println(F("Erro"));
    } else {
        sprintf(buffer, "%d mm", distancia1_mm);
        display.println(buffer);
    }

    // Sensor 2
    display.setCursor(0, 28); // Próxima linha para S2 (18 + 10)
    display.print(F("S2: "));
    if (distancia2_mm == 9999) {
        display.println(F("Erro"));
    } else {
        sprintf(buffer, "%d mm", distancia2_mm);
        display.println(buffer);
    }

    // Sensor 3
    display.setCursor(0, 38); // Próxima linha para S3 (28 + 10)
    display.print(F("S3: "));
    if (distancia3_mm == 9999) {
        display.println(F("Erro"));
    } else {
        sprintf(buffer, "%d mm", distancia3_mm);
        display.println(buffer);
    }

    display.display(); // Mostra o buffer no display
  }
  
  delay(200); 
}
