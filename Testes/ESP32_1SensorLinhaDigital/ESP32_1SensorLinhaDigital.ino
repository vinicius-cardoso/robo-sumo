#include <Wire.h>
const int SENSOR_LINHA_ESQUERDA_D0 = 4;

int linha_esquerda_digital_val = 0;

void lerSensoresLinha();      

void setup() {
  Serial.begin(115200);
}

void loop() {
  lerSensoresLinha();
  delay(500);
}

void lerSensoresLinha() {
    linha_esquerda_digital_val = digitalRead(SENSOR_LINHA_ESQUERDA_D0);

    Serial.printf(
      "DIGITAL: L_E: %d\n", 
      linha_esquerda_digital_val
    );
}
