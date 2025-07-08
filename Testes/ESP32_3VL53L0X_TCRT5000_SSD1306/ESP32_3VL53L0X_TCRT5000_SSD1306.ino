#include <Wire.h>
#include "Adafruit_VL53L0X.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// --- Pinos e Configurações dos Sensores VL53L0X (Distância) ---
#define XSHUT_PIN_D1 18 // XSHUT para VL53L0X Sensor 1 (Distância)
#define XSHUT_PIN_D2 19 // XSHUT para VL53L0X Sensor 2 (Distância)
#define XSHUT_PIN_D3 23 // XSHUT para VL53L0X Sensor 3 (Distância)

#define DIST_S1_ADDR 0x30 // Novo endereço para VL53L0X Sensor 1
#define DIST_S3_ADDR 0x31 // Novo endereço para VL53L0X Sensor 3
#define DIST_S2_ADDR 0x29 // VL53L0X Sensor 2 usará o endereço padrão

Adafruit_VL53L0X lox_dist1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox_dist2 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox_dist3 = Adafruit_VL53L0X();

VL53L0X_RangingMeasurementData_t measure_dist1;
VL53L0X_RangingMeasurementData_t measure_dist2;
VL53L0X_RangingMeasurementData_t measure_dist3;

// --- Pinos dos Sensores TCRT5000 (Linha) ---
const int TCRT_PIN_L1 = 4;  // TCRT5000 Sensor 1 (Linha)
const int TCRT_PIN_L2 = 5;  // TCRT5000 Sensor 2 (Linha)
const int TCRT_PIN_L3 = 15; // TCRT5000 Sensor 3 (Linha)

// --- Configurações do Display OLED SSD1306 ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define SCREEN_ADDRESS 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
bool displayInitialized = false;

void setup() {
  Serial.begin(115200);
  Serial.println(F("ESP32 - Teste Combinado: 3x VL53L0X & 3x TCRT5000 com OLED v2"));

  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("Falha ao iniciar SSD1306"));
  } else {
    Serial.println(F("Display OLED inicializado."));
    displayInitialized = true;
    display.clearDisplay();
    display.display();
  }

  pinMode(XSHUT_PIN_D1, OUTPUT); digitalWrite(XSHUT_PIN_D1, LOW);
  pinMode(XSHUT_PIN_D2, OUTPUT); digitalWrite(XSHUT_PIN_D2, LOW);
  pinMode(XSHUT_PIN_D3, OUTPUT); digitalWrite(XSHUT_PIN_D3, LOW);
  delay(50);

  pinMode(TCRT_PIN_L1, INPUT);
  pinMode(TCRT_PIN_L2, INPUT);
  pinMode(TCRT_PIN_L3, INPUT);

  Wire.begin(21, 22); // SDA=GPIO21, SCL=GPIO22

  Serial.println(F("Configurando VL53L0X S1 (Distancia)..."));
  digitalWrite(XSHUT_PIN_D1, HIGH); delay(10);
  if (!lox_dist1.begin()) {
    Serial.println(F("Falha S1 VL53L0X Init"));
    if (displayInitialized) { display.clearDisplay(); display.setTextSize(1); display.setTextColor(SSD1306_WHITE); display.setCursor(0,0); display.print("Erro VL53 S1 Init"); display.display(); }
    while (1);
  }
  if (!lox_dist1.setAddress(DIST_S1_ADDR)) {
    Serial.println(F("Falha S1 VL53L0X Addr"));
    if (displayInitialized) { display.clearDisplay(); display.setTextSize(1); display.setTextColor(SSD1306_WHITE); display.setCursor(0,0); display.print("Erro VL53 S1 Addr"); display.display(); }
    while(1);
  }
  Serial.print(F("VL53L0X S1 OK @ 0x")); Serial.println(DIST_S1_ADDR, HEX);

  Serial.println(F("Configurando VL53L0X S3 (Distancia)..."));
  digitalWrite(XSHUT_PIN_D3, HIGH); delay(10);
  if (!lox_dist3.begin()) {
    Serial.println(F("Falha S3 VL53L0X Init"));
    if (displayInitialized) { display.clearDisplay(); display.setTextSize(1); display.setTextColor(SSD1306_WHITE); display.setCursor(0,0); display.print("Erro VL53 S3 Init"); display.display(); }
    while (1);
  }
  if (!lox_dist3.setAddress(DIST_S3_ADDR)) {
    Serial.println(F("Falha S3 VL53L0X Addr"));
    if (displayInitialized) { display.clearDisplay(); display.setTextSize(1); display.setTextColor(SSD1306_WHITE); display.setCursor(0,0); display.print("Erro VL53 S3 Addr"); display.display(); }
    while(1);
  }
  Serial.print(F("VL53L0X S3 OK @ 0x")); Serial.println(DIST_S3_ADDR, HEX);

  Serial.println(F("Configurando VL53L0X S2 (Distancia)..."));
  digitalWrite(XSHUT_PIN_D2, HIGH); delay(10);
  if (!lox_dist2.begin()) {
    Serial.println(F("Falha S2 VL53L0X Init"));
    if (displayInitialized) { display.clearDisplay(); display.setTextSize(1); display.setTextColor(SSD1306_WHITE); display.setCursor(0,0); display.print("Erro VL53 S2 Init"); display.display(); }
    while (1);
  }
  Serial.print(F("VL53L0X S2 OK @ 0x")); Serial.println(DIST_S2_ADDR, HEX);

  Serial.println(F("Todos os sensores configurados. Iniciando leituras..."));
  Serial.println("------------------------------------------------------------");
}

void loop() {
  int dist1_mm = 9999, dist2_mm = 9999, dist3_mm = 9999;
  lox_dist1.rangingTest(&measure_dist1, false);
  if (measure_dist1.RangeStatus != 4) { dist1_mm = measure_dist1.RangeMilliMeter; }
  lox_dist2.rangingTest(&measure_dist2, false);
  if (measure_dist2.RangeStatus != 4) { dist2_mm = measure_dist2.RangeMilliMeter; }
  lox_dist3.rangingTest(&measure_dist3, false);
  if (measure_dist3.RangeStatus != 4) { dist3_mm = measure_dist3.RangeMilliMeter; }

  int linha1_estado = digitalRead(TCRT_PIN_L1);
  int linha2_estado = digitalRead(TCRT_PIN_L2);
  int linha3_estado = digitalRead(TCRT_PIN_L3);

  // Converte estado da linha para P (Preto) ou B (Branco)
  // Assumindo: 0 (LOW) = Preto (P), 1 (HIGH) = Branco (B)
  char l1_char = (linha1_estado == 0) ? 'P' : 'B';
  char l2_char = (linha2_estado == 0) ? 'P' : 'B';
  char l3_char = (linha3_estado == 0) ? 'P' : 'B';

  Serial.print("Dist: S1:"); if(dist1_mm==9999) Serial.print("Err"); else Serial.print(dist1_mm);
  Serial.print(" S2:"); if(dist2_mm==9999) Serial.print("Err"); else Serial.print(dist2_mm);
  Serial.print(" S3:"); if(dist3_mm==9999) Serial.print("Err"); else Serial.print(dist3_mm);
  Serial.print(" | Linha: L1:"); Serial.print(l1_char);
  Serial.print(" L2:"); Serial.print(l2_char);
  Serial.print(" L3:"); Serial.println(l3_char);

  if (displayInitialized) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    // Linha divisória vertical no meio da tela
    display.drawLine(64, 0, 64, SCREEN_HEIGHT - 1, SSD1306_WHITE);

    char buffer[12]; 

    // --- Coluna da Esquerda: Distância ---
    display.setCursor(0, 0); // Título "Distancia" (na parte amarela)
    display.println(F("Distancia"));

    // Leituras de distância (na parte azul)
    // Y=18 para a primeira linha de dados, deixando espaço após o título
    display.setCursor(0, 18); 
    display.print(F("S1:"));
    if (dist1_mm == 9999) display.print(F(" Err")); else { sprintf(buffer, "%4dmm", dist1_mm); display.print(buffer); }
    
    display.setCursor(0, 28); 
    display.print(F("S2:"));
    if (dist2_mm == 9999) display.print(F(" Err")); else { sprintf(buffer, "%4dmm", dist2_mm); display.print(buffer); }

    display.setCursor(0, 38); 
    display.print(F("S3:"));
    if (dist3_mm == 9999) display.print(F(" Err")); else { sprintf(buffer, "%4dmm", dist3_mm); display.print(buffer); }

    // --- Coluna da Direita: Linha ---
    display.setCursor(70, 0); // Título "Linha" (na parte amarela, à direita da divisória)
    display.println(F("Linha"));

    // Leituras de linha (na parte azul)
    display.setCursor(70, 18);
    display.print(F("S1: ")); display.print(l1_char);
    
    display.setCursor(70, 28);
    display.print(F("S2: ")); display.print(l2_char);

    display.setCursor(70, 38);
    display.print(F("S3: ")); display.print(l3_char);

    display.display();
  }
  
  delay(200);
}
