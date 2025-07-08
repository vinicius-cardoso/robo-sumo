// Pinos GPIO do ESP32 para conectar a saída D0 dos sensores TCRT5000
const int SENSOR_PIN_1 = 4; // Sensor 1 conectado ao GPIO4
const int SENSOR_PIN_2 = 5; // Sensor 2 conectado ao GPIO5
const int SENSOR_PIN_3 = 15; // Sensor 3 conectado ao GPIO15

void setup() {
  // Inicializa a comunicação Serial a 115200 bps
  Serial.begin(115200);
  Serial.println("\nTeste de 3 Sensores TCRT5000 com ESP32");
  Serial.println("--------------------------------------");

  // Configura os pinos dos sensores como ENTRADA (INPUT)
  pinMode(SENSOR_PIN_1, INPUT);
  pinMode(SENSOR_PIN_2, INPUT);
  pinMode(SENSOR_PIN_3, INPUT);

  Serial.println("Pinos configurados. Iniciando leituras...");
  Serial.println("Formato: Sensor 1 (GPIO25) | Sensor 2 (GPIO26) | Sensor 3 (GPIO27)");
  Serial.println("-----------------------------------------------------------------");
}

void loop() {
  // Lê o estado digital de cada sensor
  // digitalRead() retorna HIGH (1) ou LOW (0)
  int estadoSensor1 = digitalRead(SENSOR_PIN_1);
  int estadoSensor2 = digitalRead(SENSOR_PIN_2);
  int estadoSensor3 = digitalRead(SENSOR_PIN_3);

  // Mostra os valores no Monitor Serial
  Serial.print("           "); // Espaçamento para alinhar
  Serial.print(estadoSensor1);
  Serial.print("            |      ");
  Serial.print(estadoSensor2);
  Serial.print("            |      ");
  Serial.println(estadoSensor3);

  // Aguarda um pouco antes da próxima leitura para facilitar a visualização
  delay(500); // Atualiza a cada 0.5 segundos
}
