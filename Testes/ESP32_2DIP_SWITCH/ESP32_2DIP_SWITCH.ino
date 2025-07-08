// Definir os pinos GPIO para os DIP switches
// Escolhemos GPIO2 e GPIO12. Você pode ajustar conforme sua preferência.
const int DIP_SWITCH_1_PIN = 2;
const int DIP_SWITCH_2_PIN = 12;

void setup() {
  // Inicializa a comunicação serial a 115200 bits por segundo:
  Serial.begin(9600);
  Serial.println("--------------------------------------------------");
  Serial.println("Monitorando DIP Switches no ESP32");
  Serial.print("DIP Switch 1 no GPIO");
  Serial.println(DIP_SWITCH_1_PIN);
  Serial.print("DIP Switch 2 no GPIO");
  Serial.println(DIP_SWITCH_2_PIN);
  Serial.println("--------------------------------------------------");
  Serial.println("Estado do Switch: 1 = Aberto (HIGH), 0 = Fechado (LOW)");
  Serial.println("--------------------------------------------------");

  // Configura os pinos como entrada com resistor de pull-up interno.
  // Isso significa que, sem conexão, o pino estará HIGH.
  // Quando o switch fechar, ele conectará o pino ao GND, tornando-o LOW.
  pinMode(DIP_SWITCH_1_PIN, INPUT_PULLUP);
  pinMode(DIP_SWITCH_2_PIN, INPUT_PULLUP);
}

void loop() {
  // Lê o estado dos DIP switches
  // digitalRead() retorna HIGH (1) ou LOW (0)
  int estado_dip_1 = digitalRead(DIP_SWITCH_1_PIN);
  int estado_dip_2 = digitalRead(DIP_SWITCH_2_PIN);

  // Imprime o estado atual
  Serial.print("DIP Switch 1: ");
  Serial.print(estado_dip_1);
  Serial.print(" | DIP Switch 2: ");
  Serial.println(estado_dip_2);

  delay(2000); // Espera 500 milissegundos (0.5 segundos) antes da próxima leitura
}
