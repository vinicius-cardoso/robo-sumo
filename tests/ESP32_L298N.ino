// ENA e ENB ligados como saída digital comum (sem PWM)
const int ENA = 32;
const int ENB = 14;

const int IN1 = 33;
const int IN2 = 25;
const int IN3 = 26;
const int IN4 = 27;

void pararMotores();
void andarFrente(int tempo);
void andarTras(int tempo);
void virarDireita(int tempo);
void virarEsquerda(int tempo);

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  digitalWrite(ENA, HIGH);  // Habilita motor A
  digitalWrite(ENB, HIGH);  // Habilita motor B

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

void andarFrente(int tempo) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  delay(tempo);
  pararMotores();
}

void andarTras(int tempo) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  delay(tempo);
  pararMotores();
}


void virarDireita(int tempo) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  delay(tempo);
  pararMotores();
}

void virarEsquerda(int tempo) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  delay(tempo);
  pararMotores();
}

void pararMotores() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
