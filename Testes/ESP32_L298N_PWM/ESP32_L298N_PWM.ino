// Define os pinos do ESP32 conectados ao L298N
const int enablePin = 14;  // Pino EN A ou EN B do L298N (para controle de velocidade)
const int in1Pin = 27;     // Pino IN1 ou IN3 do L298N
const int in2Pin = 26;     // Pino IN2 ou IN4 do L298N

// Pino do ESP32 conectado ao pino central do trimpot (qualquer pino ADC do ESP32)
const int trimpotPin = 34; // Exemplo: GPIO 34. Verifique a documentação do seu ESP32 para pinos ADC disponíveis.

// Parâmetros do PWM
const int freq = 5000;      // Frequência do PWM (Hz)
const int resolution = 8;   // Resolução do PWM (bits) - 8 bits = 0-255

void setup() {
  Serial.begin(115200);

  // Configura os pinos IN como OUTPUT
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);

  // Configura o pino de enable para PWM
  ledcAttach(enablePin, freq, resolution);

  // Define a direção inicial do motor (por exemplo, para frente)
  digitalWrite(in1Pin, HIGH);
  digitalWrite(in2Pin, LOW);

  // O pino do trimpot (GPIO 34 neste caso) já é configurado automaticamente como INPUT por analogRead()
}

void loop() {
  // Lê o valor do trimpot (0-4095 para ESP32 ADC de 12 bits)
  int trimpotValue = analogRead(trimpotPin);

  // Mapeia o valor do trimpot para a faixa de PWM do motor (0 a 255 para 8 bits)
  // Ajuste os valores de 0 a 4095 conforme a resolução ADC do seu ESP32,
  // ou adicione um 'const int adcMax = 4095;' para maior clareza.
  int pwmValue = map(trimpotValue, 0, 4095, 0, (1 << resolution) - 1);

  // Escreve o valor PWM para controlar a velocidade do motor
  ledcWrite(enablePin, pwmValue);

  // Exibe a velocidade no Serial Monitor
  Serial.print("Valor do Trimpot (ADC): ");
  Serial.print(trimpotValue);
  Serial.print(" -> Velocidade (PWM): ");
  Serial.print(map(pwmValue, 0, (1 << resolution) - 1, 0, 100)); // Mostra em porcentagem
  Serial.println("%");

  delay(50); // Pequeno atraso para estabilizar a leitura e não sobrecarregar o Serial Monitor
}
