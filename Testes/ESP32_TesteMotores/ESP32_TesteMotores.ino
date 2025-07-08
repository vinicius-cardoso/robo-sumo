#include <Wire.h>

const int MOTOR_B_ENB = 14;
const int MOTOR_B_IN4 = 27;
const int MOTOR_B_IN3 = 26;
const int MOTOR_A_IN2 = 25;
const int MOTOR_A_IN1 = 33;
const int MOTOR_A_ENA = 32;

void moverFrente(); 
void moverTras(); 
void virarDireita(); 
void virarEsquerda(); 
void pararMotores();
void configurarPinos(); // Function prototype added for clarity

void setup() {
  Serial.begin(115200);
  configurarPinos(); // <<< --- CRITICAL FIX: Call configurarPinos() here!
  pararMotores(); 
}

void loop() {
  delay(2000);

  moverFrente();
  delay(2000);

  virarDireita();
  delay(1000);
  virarDireita();
  delay(1000);

  virarEsquerda();
  delay(1000);
  virarEsquerda();
  delay(1000);
  virarEsquerda();
  delay(1000);

  moverTras();
  delay(1000);
  moverTras();
  delay(1000);
  moverTras();
  delay(1000);
  moverTras();
  delay(1000);
  pararMotores(); // Added to stop motors after the sequence in loop
  delay(2000);
}

void configurarPinos() {
  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_B_IN3, OUTPUT);
  pinMode(MOTOR_B_IN4, OUTPUT);
   
  pinMode(MOTOR_A_ENA, OUTPUT);
  digitalWrite(MOTOR_A_ENA, HIGH); // Assuming ENA/ENB enable motors at HIGH
  pinMode(MOTOR_B_ENB, OUTPUT);
  digitalWrite(MOTOR_B_ENB, HIGH); // Assuming ENA/ENB enable motors at HIGH

  pararMotores(); 
}

void moverFrente() {
  digitalWrite(MOTOR_A_IN1, LOW);
  digitalWrite(MOTOR_A_IN2, HIGH);
  digitalWrite(MOTOR_B_IN3, LOW);
  digitalWrite(MOTOR_B_IN4, HIGH);
}

void moverTras() {
  digitalWrite(MOTOR_A_IN1, HIGH);
  digitalWrite(MOTOR_A_IN2, LOW);
  digitalWrite(MOTOR_B_IN3, HIGH);
  digitalWrite(MOTOR_B_IN4, LOW);
}

void virarEsquerda() { 
  digitalWrite(MOTOR_A_IN1, LOW);
  digitalWrite(MOTOR_A_IN2, HIGH);
  digitalWrite(MOTOR_B_IN3, HIGH);
  digitalWrite(MOTOR_B_IN4, LOW);
}

void virarDireita() { 
  digitalWrite(MOTOR_A_IN1, HIGH);
  digitalWrite(MOTOR_A_IN2, LOW);
  digitalWrite(MOTOR_B_IN3, LOW);
  digitalWrite(MOTOR_B_IN4, HIGH);
}

void pararMotores() {
  digitalWrite(MOTOR_A_IN1, LOW);
  digitalWrite(MOTOR_A_IN2, LOW);
  digitalWrite(MOTOR_B_IN3, LOW);
  digitalWrite(MOTOR_B_IN4, LOW);
}
