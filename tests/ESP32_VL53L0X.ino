#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_VL53L0X.h>

// Tamanho do display OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// Pinos I2C (ESP32)
#define SDA_PIN 21
#define SCL_PIN 22

// Instâncias
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

void setup() {
  // Inicializa I2C
  Wire.begin(SDA_PIN, SCL_PIN);

  // Inicializa o display
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("Erro ao inicializar display OLED"));
    while (true);
  }

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // Inicializa o sensor de distância
  if (!lox.begin()) {
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println(F("VL53L0X nao detectado!"));
    display.display();
    while (true);
  }

  // Boas-vindas
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("VL53L0X OK!");
  display.display();
  delay(1000);
}

void loop() {
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);

  display.clearDisplay();

  if (measure.RangeStatus != 4) {
    int distancia_m = measure.RangeMilliMeter;

    // Usa a maior fonte possível que caiba no display
    display.setTextSize(3); // 3 = 18px, cabem 4 caracteres de largura
    display.setCursor(0, 24); // verticalmente centralizado
    display.print(distancia_m);
    display.print("mm");
  } else {
    display.setTextSize(2);
    display.setCursor(0, 24);
    display.println("Erro!");
  }

  display.display();
  delay(200);
}
