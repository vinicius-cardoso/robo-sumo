// ESP32_RoboSumo - Versão Completa (SEM DISPLAY)
// ==========================================================================
// Bibliotecas
// ==========================================================================

#include <Wire.h>
#include "Adafruit_VL53L0X.h"

// ==========================================================================
// Configurações e Mapeamento de Pinos
// ==========================================================================

// --- Pinos dos Motores (Ponte H L298N)
const int MOTOR_A_IN1 = 33;
const int MOTOR_A_IN2 = 25;
const int MOTOR_B_IN3 = 26;
const int MOTOR_B_IN4 = 27;

// --- Pinos dos Sensores de Linha (TCRT5000)
const int SENSOR_LINHA_ESQUERDA_A0 = 34;
const int SENSOR_LINHA_DIREITA_A0 = 35;
const int SENSOR_LINHA_TRASEIRA_A0 = 39;

// --- Pinos de Controle dos Sensores de Distância (VL53L0X)
const int SENSOR_DIST_XSHUT_ESQUERDA = 23;
const int SENSOR_DIST_XSHUT_DIREITA = 18;
const int SENSOR_DIST_XSHUT_FRONTAL = 19;

// ==========================================================================
// Objetos de Hardware
// ==========================================================================
Adafruit_VL53L0X sensorDistEsquerda;
Adafruit_VL53L0X sensorDistDireita;
Adafruit_VL53L0X sensorDistFrontal;

// ==========================================================================
// Variáveis Globais e Constantes de Lógica
// ==========================================================================
int dist_esquerda_mm = 0, dist_direita_mm = 0, dist_frontal_mm = 0;
int linha_esquerda_analog = 0, linha_direita_analog = 0, linha_traseira_analog = 0;
char linha_esquerda_char = 'P', linha_direita_char = 'P', linha_traseira_char = 'P';

// --- Limites dos Sensores ---
const int LIMITE_BRANCO_DIREITA = 1800;
const int LIMITE_BRANCO_ESQUERDA = 1400;
const int LIMITE_BRANCO_TRASEIRA = 1100;
const int LIMITE_DETECCAO_FRONTAL = 400; 
const int LIMITE_DETECCAO_LATERAL = 300; 

// --- Tempos de Manobra (em ms) ---
// ** Estes valores são pontos de partida. Você PRECISARÁ ajustá-los! **
const int TEMPO_MEIA_VOLTA = 600;      // AJUSTAR PARA GIRAR 180 GRAUS
const int TEMPO_CORRECAO_BORDA = 450;  // AJUSTAR PARA GIRAR ~90-100 GRAUS
const int TEMPO_RECUPERACAO_TRAS = 400;

// ==========================================================================
// Definições da Máquina de Estados
// ==========================================================================
enum Estados {
  ESPERA_INICIAL,
  BUSCANDO_ADVERSARIO,
  MANOBRA_BORDA
};

Estados estadoAtual = ESPERA_INICIAL;
unsigned long tempoInicial = 0;
const long TEMPO_ESPERA = 5000; // 5 segundos

// ==========================================================================
// Protótipos de Funções
// ==========================================================================
void configurarPinos();
void configurarSensores();
void lerSensores();
void moverFrente();
void moverTras();
void virarDireita();
void virarEsquerda();
void pararMotores();
void manobrar(void (*manobra)(), int tempo);

// ==========================================================================
// Setup Principal
// ==========================================================================
void setup() {
  Serial.begin(115200);
  Serial.println("Inicializando Robo de Sumo...");

  configurarPinos();
  configurarSensores();

  tempoInicial = millis(); // Marca o tempo de inicialização
  estadoAtual = ESPERA_INICIAL;
  Serial.println("Estado: ESPERA_INICIAL");
}

// ==========================================================================
// Loop Principal
// ==========================================================================
void loop() {
  lerSensores();

  // PRIORIDADE MÁXIMA: Se qualquer sensor detectar a borda, entra em estado de manobra.
  if ( (estadoAtual != MANOBRA_BORDA) && (linha_esquerda_char == 'B' || linha_direita_char == 'B' || linha_traseira_char == 'B') ) {
    estadoAtual = MANOBRA_BORDA;
  }

  switch (estadoAtual) {
    case ESPERA_INICIAL:
      pararMotores();
      if (millis() - tempoInicial >= TEMPO_ESPERA) {
        estadoAtual = BUSCANDO_ADVERSARIO;
        Serial.println("Transição -> BUSCANDO_ADVERSARIO");
      }
      break;

    case BUSCANDO_ADVERSARIO:
      if (dist_frontal_mm < LIMITE_DETECCAO_FRONTAL) {
        Serial.println("Alvo na frente! Atacando...");
        moverFrente();
      } else if (dist_direita_mm < LIMITE_DETECCAO_LATERAL) {
        Serial.println("Alvo à direita. Virando para alinhar...");
        virarDireita();
      } else if (dist_esquerda_mm < LIMITE_DETECCAO_LATERAL) {
        Serial.println("Alvo à esquerda. Virando para alinhar...");
        virarEsquerda();
      } else {
        Serial.println("Nenhum alvo. Procurando...");
        virarEsquerda(); // Busca ativa, girando no próprio eixo.
      }
      break;

    case MANOBRA_BORDA:
      Serial.println("Executando MANOBRA_BORDA");
      pararMotores();
      delay(100); // Pausa para estabilizar antes da manobra

      if (linha_esquerda_char == 'B' && linha_direita_char == 'B') {
        Serial.println("Borda Frontal. Recuando e virando...");
        manobrar(moverTras, 200); // Recua um pouco
        manobrar(virarEsquerda, TEMPO_MEIA_VOLTA); // Vira 180 graus
      } else if (linha_esquerda_char == 'B') {
        Serial.println("Borda Esquerda. Corrigindo para a direita...");
        manobrar(moverTras, 150); // Recua um pouco
        manobrar(virarDireita, TEMPO_CORRECAO_BORDA); // Vira para longe da borda
      } else if (linha_direita_char == 'B') {
        Serial.println("Borda Direita. Corrigindo para a esquerda...");
        manobrar(moverTras, 150); // Recua um pouco
        manobrar(virarEsquerda, TEMPO_CORRECAO_BORDA); // Vira para longe da borda
      } else if (linha_traseira_char == 'B') {
        Serial.println("Borda Traseira. Movendo para frente...");
        manobrar(moverFrente, TEMPO_RECUPERACAO_TRAS); // Se afasta da borda
      }
      
      Serial.println("Manobra concluída. Retornando à busca.");
      estadoAtual = BUSCANDO_ADVERSARIO; // Retorna ao estado de busca após a manobra
      break;
  }
}

// ==========================================================================
// Funções de Configuração
// ==========================================================================
void configurarPinos() {
  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_B_IN3, OUTPUT);
  pinMode(MOTOR_B_IN4, OUTPUT);

  pinMode(SENSOR_DIST_XSHUT_ESQUERDA, OUTPUT);
  pinMode(SENSOR_DIST_XSHUT_DIREITA, OUTPUT);
  pinMode(SENSOR_DIST_XSHUT_FRONTAL, OUTPUT);
  pararMotores();
}

void configurarSensores() {
  // Desliga todos os sensores de distância para poder endereçá-los individualmente
  digitalWrite(SENSOR_DIST_XSHUT_ESQUERDA, LOW);
  digitalWrite(SENSOR_DIST_XSHUT_DIREITA, LOW);
  digitalWrite(SENSOR_DIST_XSHUT_FRONTAL, LOW);
  delay(50);

  // Inicializa a comunicação I2C, necessária para os sensores VL53L0X
  Wire.begin(21, 22);

  // Liga e configura cada sensor VL53L0X com um endereço I2C único
  digitalWrite(SENSOR_DIST_XSHUT_ESQUERDA, HIGH); delay(10);
  if (!sensorDistEsquerda.begin(0x31)) { Serial.println(F("Falha sensor de distancia Esquerdo")); while(1); }

  digitalWrite(SENSOR_DIST_XSHUT_DIREITA, HIGH); delay(10);
  if (!sensorDistDireita.begin(0x30)) { Serial.println(F("Falha sensor de distancia Direito")); while(1); }

  digitalWrite(SENSOR_DIST_XSHUT_FRONTAL, HIGH); delay(10);
  if (!sensorDistFrontal.begin()) { Serial.println(F("Falha sensor de distancia Frontal")); while(1); }

  Serial.println(F("Sensores de distancia OK!"));
}

// ==========================================================================
// Funções de Lógica e Ação
// ==========================================================================
void lerSensores() {
  VL53L0X_RangingMeasurementData_t measure;

  sensorDistEsquerda.rangingTest(&measure, false);
  dist_esquerda_mm = (measure.RangeStatus != 4) ? measure.RangeMilliMeter : 9999;

  sensorDistDireita.rangingTest(&measure, false);
  dist_direita_mm = (measure.RangeStatus != 4) ? measure.RangeMilliMeter : 9999;

  sensorDistFrontal.rangingTest(&measure, false);
  dist_frontal_mm = (measure.RangeStatus != 4) ? measure.RangeMilliMeter : 9999;

  linha_esquerda_analog = analogRead(SENSOR_LINHA_ESQUERDA_A0);
  linha_direita_analog = analogRead(SENSOR_LINHA_DIREITA_A0);
  linha_traseira_analog = analogRead(SENSOR_LINHA_TRASEIRA_A0);

  linha_esquerda_char = (linha_esquerda_analog < LIMITE_BRANCO_ESQUERDA) ? 'B' : 'P';
  linha_direita_char = (linha_direita_analog < LIMITE_BRANCO_DIREITA) ? 'B' : 'P';
  linha_traseira_char = (linha_traseira_analog < LIMITE_BRANCO_TRASEIRA) ? 'B' : 'P';
}

void moverFrente() {
  digitalWrite(MOTOR_A_IN1, HIGH);
  digitalWrite(MOTOR_A_IN2, LOW);
  digitalWrite(MOTOR_B_IN3, HIGH);
  digitalWrite(MOTOR_B_IN4, LOW);
}

void moverTras() {
  digitalWrite(MOTOR_A_IN1, LOW);
  digitalWrite(MOTOR_A_IN2, HIGH);
  digitalWrite(MOTOR_B_IN3, LOW);
  digitalWrite(MOTOR_B_IN4, HIGH);
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

// Função auxiliar para executar uma manobra por um tempo específico
void manobrar(void (*manobra)(), int tempo) {
  manobra();
  delay(tempo);
  pararMotores();
}
