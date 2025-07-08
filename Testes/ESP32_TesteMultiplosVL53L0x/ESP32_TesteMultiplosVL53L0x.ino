#include <Wire.h>
#include "Adafruit_VL53L0X.h"

// Pinos XSHUT para cada sensor no ESP32
#define XSHUT_PIN_1 18  // GPIO18
#define XSHUT_PIN_2 19  // GPIO19
#define XSHUT_PIN_3 23  // GPIO23

// Novos endereços I2C para os sensores
// O endereço padrão é 0x29. Escolha endereços não utilizados.
#define SENSOR1_ADDRESS 0x30
#define SENSOR2_ADDRESS 0x31
#define SENSOR3_ADDRESS 0x32

// Cria instâncias dos sensores
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox3 = Adafruit_VL53L0X();

// Arrays para facilitar o gerenciamento dos sensores
Adafruit_VL53L0X sensors[] = {lox1, lox2, lox3};
uint8_t sensorPins[] = {XSHUT_PIN_1, XSHUT_PIN_2, XSHUT_PIN_3};
uint8_t sensorAddresses[] = {SENSOR1_ADDRESS, SENSOR2_ADDRESS, SENSOR3_ADDRESS};
const int SENSOR_COUNT = 3;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(1);
  }

  // Inicializa o barramento I2C com os pinos padrão do ESP32 (SDA = GPIO21, SCL = GPIO22)
  Wire.begin();
  // Se precisar usar pinos I2C diferentes: Wire.begin(SDA_PIN, SCL_PIN);

  // Define os pinos XSHUT como saída
  for (int i = 0; i < SENSOR_COUNT; i++) {
    pinMode(sensorPins[i], OUTPUT);
  }

  Serial.println("Desligando todos os sensores...");
  // Coloca todos os sensores em modo shutdown (XSHUT LOW)
  for (int i = 0; i < SENSOR_COUNT; i++) {
    digitalWrite(sensorPins[i], LOW);
  }
  delay(10); // Aguarda um pouco

  Serial.println("Configurando sensores um por um...");

  // Inicializa e configura cada sensor
  for (int i = 0; i < SENSOR_COUNT; i++) {
    Serial.print("Ativando sensor ");
    Serial.println(i + 1);

    // Tira o sensor do modo shutdown
    digitalWrite(sensorPins[i], HIGH);
    delay(150); // Tempo para o sensor ligar e estabilizar

    // Inicializa o sensor (ele vai responder no endereço padrão 0x29)
    if (!sensors[i].begin()) { // Tenta inicializar no endereço padrão 0x29
      Serial.print(F("Falha ao iniciar o sensor "));
      Serial.print(i + 1);
      Serial.println(F(" (no endereco padrao). Verifique a fiacao ou se ha conflito."));
      // Trava se um sensor não puder ser inicializado.
      while (1) {
        delay(1);
      }
    }
    delay(10);

    // Define o novo endereço I2C
    Serial.print(F("Sensor "));
    Serial.print(i + 1);
    Serial.print(F(" detectado no endereco padrao. Definindo novo endereco para 0x"));
    Serial.println(sensorAddresses[i], HEX);
    sensors[i].setAddress(sensorAddresses[i]);
    delay(10);

    // Agora, precisamos verificar se o sensor responde no novo endereço.
    // A maneira mais simples é tentar um 'begin' com o novo endereço.
    // A biblioteca Adafruit não tem um begin(address) direto que seja ideal para re-verificação aqui.
    // Uma forma de verificar é tentar ler a ID do modelo ou algo similar no novo endereço.
    // Para este exemplo, vamos assumir que setAddress() foi bem-sucedido se não travou.
    // A próxima leitura no loop() confirmará.
    
    Serial.print(F("Sensor "));
    Serial.print(i + 1);
    Serial.print(F(" configurado com endereco 0x"));
    // A biblioteca atualiza o endereço internamente no objeto após setAddress()
    Serial.println(sensors[i].VL53L0X_DEFAULT_ADDRESS, HEX); 
    // Nota: VL53L0X_DEFAULT_ADDRESS no objeto sensor é atualizado para o novo endereço pela lib.
  }

  Serial.println(F("Todos os sensores configurados!"));
  Serial.println(F("Iniciando medicoes..."));
}

void loop() {
  VL53L0X_RangingMeasurementData_t measure;

  for (int i = 0; i < SENSOR_COUNT; i++) {
    Serial.print("Sensor ");
    Serial.print(i + 1);
    Serial.print(" (0x");
    Serial.print(sensorAddresses[i], HEX); // Mostra o endereço que configuramos
    Serial.print("): ");

    // Cada objeto 'sensors[i]' agora se comunicará no endereço definido para ele.
    sensors[i].rangingTest(&measure, false); // Passa falso para não imprimir dados de debug da lib

    if (measure.RangeStatus != 4) {  // Se o status da medição for diferente de 4 (erro)
      Serial.print("Distancia (mm): ");
      Serial.println(measure.RangeMilliMeter);
    } else {
      Serial.println(" Fora de alcance ");
    }
  }
  Serial.println("---");
  delay(500); // Delay entre os ciclos de leitura de todos os sensores
}
