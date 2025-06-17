#include <Wire.h>
#include "Adafruit_VL53L0X.h"

// Pinos XSHUT para controlar cada sensor VL53L0X
// Estes são números GPIO. Certifique-se que são adequados para sua placa ESP32.
//#define XSHUT_PIN_S1 14 // GPIO14 para o Sensor 1
//#define XSHUT_PIN_S2 16 // GPIO16 (RX2) para o Sensor 2
//#define XSHUT_PIN_S3 13 // GPIO13 para o Sensor 3
#define XSHUT_PIN_S1 14 // GPIO14 para o Sensor 1
#define XSHUT_PIN_S2 16 // GPIO16 (RX2) para o Sensor 2
#define XSHUT_PIN_S3 13 // GPIO13 para o Sensor 3

// Endereços I2C que usaremos
// O VL53L0X tem o endereço padrão 0x29.
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

void setup() {
  Serial.begin(115200);
  // while (!Serial) { delay(1); } // Opcional: Aguarda a conexão Serial

  Serial.println(F("Teste com tres sensores VL53L0X no ESP32")); // ATUALIZADO para ESP32

  // Configura os pinos XSHUT como SAÍDA e inicialmente BAIXO para manter os sensores em reset
  pinMode(XSHUT_PIN_S1, OUTPUT);
  digitalWrite(XSHUT_PIN_S1, LOW);

  pinMode(XSHUT_PIN_S2, OUTPUT);
  digitalWrite(XSHUT_PIN_S2, LOW);

  pinMode(XSHUT_PIN_S3, OUTPUT);
  digitalWrite(XSHUT_PIN_S3, LOW);
  
  delay(50); // Pequeno delay para garantir que os pinos XSHUT estejam estabelecidos

  // Inicia a comunicação I2C.
  // Para ESP32, os pinos padrão comuns são GPIO21 (SDA) e GPIO22 (SCL).
  Wire.begin(21, 22); // ATUALIZADO: Usando GPIO21 para SDA, GPIO22 para SCL.
                      // Se sua placa ESP32 usa outros pinos I2C padrão ou você quer usar pinos diferentes,
                      // ajuste (SDA_PIN, SCL_PIN) aqui. Wire.begin() sem argumentos também usa os padrões.

  // --- Configurar Sensor 1 ---
  Serial.println(F("Configurando Sensor 1..."));
  digitalWrite(XSHUT_PIN_S1, HIGH); // Tira o Sensor 1 do modo shutdown
  delay(10); // Tempo para o sensor "acordar"
  
  if (!lox1.begin()) { // Inicializa o sensor (ele será encontrado no endereço padrão 0x29)
    Serial.println(F("Falha ao iniciar Sensor 1. Verifique as conexões ou o pino XSHUT."));
    while (1); // Trava se falhar
  }
  Serial.println(F("Sensor 1 iniciado (no endereço padrão 0x29). Mudando endereço..."));
  
  // Muda o endereço I2C do Sensor 1
  if (!lox1.setAddress(SENSOR1_ADDR)) {
    Serial.print(F("Falha ao definir novo endereço (0x")); Serial.print(SENSOR1_ADDR, HEX); Serial.println(F(") para o Sensor 1."));
    while(1); // Trava se falhar
  }
  Serial.print(F("Sensor 1 configurado e operando no endereço 0x")); Serial.println(SENSOR1_ADDR, HEX);

  // --- Configurar Sensor 3 ---
  // (Este é configurado antes do Sensor 2 para que o Sensor 2 possa usar o endereço padrão 0x29)
  Serial.println(F("Configurando Sensor 3..."));
  digitalWrite(XSHUT_PIN_S3, HIGH); // Tira o Sensor 3 do modo shutdown
  delay(10); // Tempo para o sensor "acordar"
  
  if (!lox3.begin()) { // Inicializa o sensor (ele será encontrado no endereço padrão 0x29, pois S1 já foi movido)
    Serial.println(F("Falha ao iniciar Sensor 3. Verifique as conexões ou o pino XSHUT."));
    while (1); // Trava se falhar
  }
  Serial.println(F("Sensor 3 iniciado (no endereço padrão 0x29). Mudando endereço..."));
  
  // Muda o endereço I2C do Sensor 3
  if (!lox3.setAddress(SENSOR3_ADDR)) {
    Serial.print(F("Falha ao definir novo endereço (0x")); Serial.print(SENSOR3_ADDR, HEX); Serial.println(F(") para o Sensor 3."));
    while(1); // Trava se falhar
  }
  Serial.print(F("Sensor 3 configurado e operando no endereço 0x")); Serial.println(SENSOR3_ADDR, HEX);

  // --- Configurar Sensor 2 ---
  // Como o Sensor 1 e o Sensor 3 agora têm endereços diferentes, 
  // o Sensor 2 pode ser inicializado e usado no endereço padrão 0x29 sem conflitos.
  Serial.println(F("Configurando Sensor 2..."));
  digitalWrite(XSHUT_PIN_S2, HIGH); // Tira o Sensor 2 do modo shutdown
  delay(10); // Tempo para o sensor "acordar"

  if (!lox2.begin()) { // Inicializa o Sensor 2 (ele será encontrado no endereço padrão 0x29)
    Serial.println(F("Falha ao iniciar Sensor 2. Verifique as conexões ou o pino XSHUT."));
    while (1); // Trava se falhar
  }
  // O Sensor 2 permanecerá no endereço padrão SENSOR2_ADDR (0x29)
  Serial.print(F("Sensor 2 configurado e operando no endereço 0x")); Serial.println(SENSOR2_ADDR, HEX);

  Serial.println(F("Todos os sensores configurados com sucesso. Iniciando leituras..."));
  Serial.println("------------------------------------------------------------");
}

void loop() {
  int distancia1_mm = 9999; // Valor padrão para erro/fora de alcance
  int distancia2_mm = 9999; // Valor padrão para erro/fora de alcance
  int distancia3_mm = 9999; // Valor padrão para erro/fora de alcance

  // Leitura do Sensor 1 (que está agora em SENSOR1_ADDR)
  lox1.rangingTest(&measure1, false); // O segundo parâmetro 'false' desabilita a impressão de debug da biblioteca

  if (measure1.RangeStatus != 4) {  // Se RangeStatus for diferente de 4, a leitura é válida
    distancia1_mm = measure1.RangeMilliMeter;
  }

  // Leitura do Sensor 2 (que está no endereço padrão SENSOR2_ADDR)
  lox2.rangingTest(&measure2, false);

  if (measure2.RangeStatus != 4) { // Se RangeStatus for diferente de 4, a leitura é válida
    distancia2_mm = measure2.RangeMilliMeter;
  }

  // Leitura do Sensor 3 (que está agora em SENSOR3_ADDR)
  lox3.rangingTest(&measure3, false); 

  if (measure3.RangeStatus != 4) {  // Se RangeStatus for diferente de 4, a leitura é válida
    distancia3_mm = measure3.RangeMilliMeter;
  }

  // Imprime as distâncias
  Serial.print(F("S1 (0x")); Serial.print(SENSOR1_ADDR, HEX); Serial.print(F("): "));
  if (distancia1_mm == 9999) {
    Serial.print(F("Erro"));
  } else {
    Serial.print(distancia1_mm); Serial.print(F("mm"));
  }

  Serial.print(F("   |   S2 (0x")); Serial.print(SENSOR2_ADDR, HEX); Serial.print(F("): "));
  if (distancia2_mm == 9999) {
    Serial.print(F("Erro"));
  } else {
    Serial.print(distancia2_mm); Serial.print(F("mm"));
  }
  
  Serial.print(F("   |   S3 (0x")); Serial.print(SENSOR3_ADDR, HEX); Serial.print(F("): "));
  if (distancia3_mm == 9999) {
    Serial.println(F("Erro"));
  } else {
    Serial.print(distancia3_mm); Serial.println(F("mm"));
  }
  
  delay(200); // Ajuste o delay conforme necessário. Cada rangingTest pode levar algum tempo.
}
