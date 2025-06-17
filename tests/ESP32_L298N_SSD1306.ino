#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// --- Configurações do Display OLED ---
#define SCREEN_WIDTH 128 // Largura do display em pixels
#define SCREEN_HEIGHT 64 // Altura do display em pixels
#define OLED_RESET    -1 // Pino de reset (-1 se estiver compartilhando o pino de reset do Arduino)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// --- Pinos dos Motores ---
// ENA e ENB ligados como saída digital comum (sem PWM)
const int ENA = 32;
const int ENB = 14;

const int IN1 = 33;
const int IN2 = 25;
const int IN3 = 26;
const int IN4 = 27;

// --- Declaração das Funções ---
void pararMotores();
void andarFrente(int tempo);
void andarTras(int tempo);
void virarDireita(int tempo);
void virarEsquerda(int tempo);
void atualizarDisplay(String direcao);

void setup() {
  // Inicializa a comunicação serial (para debug)
  Serial.begin(115200);

  // --- Inicialização do Display ---
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // 0x3C é o endereço I2C padrão
    Serial.println(F("Falha ao iniciar SSD1306"));
    for(;;); // Loop infinito em caso de falha
  }
  display.clearDisplay();
  display.display();

  // --- Configuração dos Pinos dos Motores ---
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Habilita os motores
  digitalWrite(ENA, HIGH);
  digitalWrite(ENB, HIGH);

  // Garante que o robô comece parado e atualiza o display
  pararMotores();
}

void loop() {
  andarFrente(2000);
  delay(1000);
  virarDireita(2000);
  delay(1000);
  virarEsquerda(2000);
  delay(1000);
  andarTras(2000);
  delay(1000);
}

/**
 * @brief Atualiza o texto no display OLED.
 * @param direcao O texto a ser exibido como o estado atual.
 */
void atualizarDisplay(String direcao) {
  display.clearDisplay();

  // Escreve "Direção:" na seção superior (amarela)
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE); // A cor é definida pela posição no display
  display.setCursor(0, 0);
  display.println(F("Direcao:"));

  // Escreve o estado atual na seção inferior (azul)
  display.setTextSize(2);
  display.setCursor(20, 32); // Posiciona o cursor mais ou menos no centro da área azul
  display.println(direcao);

  display.display(); // Envia o buffer para o display
}

void andarFrente(int tempo) {
  atualizarDisplay("Frente"); // Atualiza o display
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  delay(tempo);
  pararMotores();
}

void andarTras(int tempo) {
  atualizarDisplay("Tras"); // Atualiza o display
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  delay(tempo);
  pararMotores();
}

void virarDireita(int tempo) {
  atualizarDisplay("Direita"); // Atualiza o display
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  delay(tempo);
  pararMotores();
}

void virarEsquerda(int tempo) {
  atualizarDisplay("Esquerda"); // Atualiza o display
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); 
  digitalWrite(IN4, HIGH);
  delay(tempo);
  pararMotores();
}

void pararMotores() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  atualizarDisplay("Parado"); // Atualiza o display para o estado "Parado"
}
