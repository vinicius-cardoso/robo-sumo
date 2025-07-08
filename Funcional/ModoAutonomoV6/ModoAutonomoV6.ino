#include <Wire.h>
#include "Adafruit_VL53L0X.h"

const int MOTOR_B_ENB = 14;
const int MOTOR_B_IN4 = 27;
const int MOTOR_B_IN3 = 26;
const int MOTOR_A_IN2 = 25;
const int MOTOR_A_IN1 = 33;
const int MOTOR_A_ENA = 32;

const int SENSOR_LINHA_DIREITA_A0 = 35;
const int SENSOR_LINHA_ESQUERDA_A0 = 34;
const int SENSOR_LINHA_TRASEIRA_A0 = 39;

const int SENSOR_LINHA_DIREITA_D0 = 15;
const int SENSOR_LINHA_ESQUERDA_D0 = 4;
const int SENSOR_LINHA_TRASEIRA_D0 = 5;

const int SENSOR_DIST_XSHUT_DIREITA = 18;
const int SENSOR_DIST_XSHUT_FRONTAL = 19;
const int SENSOR_DIST_XSHUT_ESQUERDA = 23;

const int PINO_SDA = 21;
const int PINO_SCL = 22;

Adafruit_VL53L0X sensorDistEsquerda;
Adafruit_VL53L0X sensorDistDireita;
Adafruit_VL53L0X sensorDistFrontal;

int dist_esquerda_mm = 0;
int dist_direita_mm = 0;
int dist_frontal_mm = 0;

int linha_esquerda_analogico_val = 0;
int linha_direita_analogico_val = 0;
int linha_traseira_analogico_val = 0;

int linha_esquerda_digital_val = 0;
int linha_direita_digital_val = 0;
int linha_traseira_digital_val = 0;

const int LIMITE_BRANCO_DIREITA = 1800;
const int LIMITE_PRETO_DIREITA = 2000;
const int LIMITE_BRANCO_ESQUERDA = 1400;
const int LIMITE_PRETO_ESQUERDA = 2200;
const int LIMITE_BRANCO_TRASEIRA = 1100;
const int LIMITE_PRETO_TRASEIRA = 1300;

char linha_esquerda_char_atual = 'P'; 
char linha_direita_char_atual = 'P';
char linha_traseira_char_atual = 'P';

bool isAnalogico = false;

unsigned long proximoUpdateSensoresDistancia = 0;
const long INTERVALO_UPDATE_SENSORES_DISTANCIA_MS = 100; 
const unsigned long TEMPO_ESPERA_INICIAL_AUTONOMO_MS = 5000; // Tempo de espera após ligar

const unsigned long TEMPO_RECUO_BORDA_AMBOS_MS = 300;       // Recuo para evasão 180 (ambos sensores)
const unsigned long TEMPO_GIRO_180_BORDA_AMBOS_MS = 800;     // Giro 180 para evasão (ambos sensores)
const unsigned long TEMPO_RECUO_BORDA_LATERAL_MS = 200;     // Recuo para evasão 135 (sensor lateral) - Sugestão: menor que 180
const unsigned long TEMPO_GIRO_135_BORDA_LATERAL_MS = 600;   // Giro 135 para evasão (sensor lateral) - Precisa calibrar!

const int DISTANCIA_DETECCAO_INIMIGO_MM = 250;
const int DISTANCIA_MINIMA_OBJETO_MM = 20;
const int DISTANCIA_PERDA_ALVO_MM = 450;
const unsigned long TIMEOUT_PROCURA_RETO_MS = 2500;
const unsigned long TIMEOUT_PROCURA_GIRANDO_MS = 5000;

enum ModoAutonomoState {
  INICIALIZANDO, 
  ESPERANDO_INICIO, 
  PROCURANDO,       
  ALINHANDO,        
  ATACANDO,         
  EVADINDO_RECUANDO, 
  EVADINDO_GIRANDO,  
  ERRO              
};

ModoAutonomoState estadoAutonomoAtual = INICIALIZANDO; 
unsigned long tempoInicioEstadoAutonomo = 0; // Timer para o estado atual

enum EvasionScenario {
  SCENARIO_NONE,
  SCENARIO_AMBOS,     // Ambos os sensores frontais detectaram borda
  SCENARIO_ESQUERDA,  // Apenas o sensor frontal esquerdo detectou
  SCENARIO_DIREITA    // Apenas o sensor frontal direito detectou
};

EvasionScenario currentEvasionScenario = SCENARIO_NONE; // Variável para guardar o cenário de evasão

bool isProcurandoReto = true; // true: anda reto, false: gira
unsigned long procurandoPhaseStartTime = 0; // Timer para as fases de procura

void configurarPinos();
void configurarSensoresVL53L0X();
void lerSensoresDistancia(); 
void lerSensoresLinha();      

void moverFrente(); 
void moverTras(); 
void virarDireita(); 
void virarEsquerda(); 
void pararMotores();

String statusAutonomoDisplay = "Off";
void mudarEstado(ModoAutonomoState novoEstado, String nomeEstadoDisplay);

void setup() {
  Serial.begin(115200);
  randomSeed(micros()); 

  Serial.println("Robo Sumo: Inicializando...");
  configurarPinos();
  configurarSensoresVL53L0X();

  pararMotores(); 

  mudarEstado(ESPERANDO_INICIO, "Esperando 5s");
  Serial.println("Robo Sumo: Aguardando 5 segundos para iniciar modo autonomo...");
}

void loop() {
  lerSensoresLinha();
  bool linhaDetectadaEsq = (linha_esquerda_char_atual == 'B'); 
  bool linhaDetectadaDir = (linha_direita_char_atual == 'B');

  // Leitura dos sensores de DISTÂNCIA e linha TRASEIRA em um intervalo definido
  if (millis() >= proximoUpdateSensoresDistancia) {
    proximoUpdateSensoresDistancia = millis() + INTERVALO_UPDATE_SENSORES_DISTANCIA_MS;
    lerSensoresDistancia();      
  }

  // --- LÓGICA DE PRIORIDADE MÁXIMA: Detecção de Borda Frontal ---
  // Interrompe qualquer estado (exceto evasão) se uma borda frontal for detectada.
  if (estadoAutonomoAtual != EVADINDO_RECUANDO && 
      estadoAutonomoAtual != EVADINDO_GIRANDO &&
      (linhaDetectadaEsq || linhaDetectadaDir)) // Verifica se *alguma* borda frontal foi detectada
  {
      Serial.println("Autonomo: BORDA FRONTAL DETECTADA!");
      pararMotores(); 
      delay(50); // Pequena pausa

      // Determina o cenário de evasão específico
      if (linhaDetectadaEsq && linhaDetectadaDir) {
          currentEvasionScenario = SCENARIO_AMBOS;
          Serial.println("--> Ambos detectaram. Preparando evasao 180.");
      } else if (linhaDetectadaEsq) {
          currentEvasionScenario = SCENARIO_ESQUERDA;
          Serial.println("--> Esquerda detectou. Preparando evasao 135 Direita.");
      } else { // linhaDetectadaDir
          currentEvasionScenario = SCENARIO_DIREITA;
          Serial.println("--> Direita detectou. Preparando evasao 135 Esquerda.");
      }

      // Inicia a primeira fase da evasão
      mudarEstado(EVADINDO_RECUANDO, "Evade Recuando");
      return; // Sai do loop atual para processar o novo estado no próximo ciclo
  }

  // --- MÁQUINA DE ESTADOS PRINCIPAL ---
  bool objFrontal = (dist_frontal_mm < DISTANCIA_DETECCAO_INIMIGO_MM && dist_frontal_mm > DISTANCIA_MINIMA_OBJETO_MM);
  bool objEsquerdo = (dist_esquerda_mm < DISTANCIA_DETECCAO_INIMIGO_MM && dist_esquerda_mm > DISTANCIA_MINIMA_OBJETO_MM);
  bool objDireito = (dist_direita_mm < DISTANCIA_DETECCAO_INIMIGO_MM && dist_direita_mm > DISTANCIA_MINIMA_OBJETO_MM);

  switch (estadoAutonomoAtual) {
    case INICIALIZANDO:
        // Este estado só existe antes do setup. Não deveria ser alcançado aqui.
        Serial.println("ERRO: Estado INICIALIZANDO alcancado no loop.");
        mudarEstado(ERRO, "Erro: Inicializacao");
        break;

    case ESPERANDO_INICIO:
      pararMotores(); 
      if (millis() - tempoInicioEstadoAutonomo >= TEMPO_ESPERA_INICIAL_AUTONOMO_MS) {
        Serial.println("Autonomo: Espera inicial concluida. Entrando no modo Padrao (Procurando).");
        // Transiciona para o estado de procura, iniciando com a fase "reto"
        mudarEstado(PROCURANDO, "Procurando Reta");
        isProcurandoReto = true; // Começa procurando reto
        procurandoPhaseStartTime = millis(); // Inicia o timer da fase de procura
      }
      break;

    case PROCURANDO:
      // Lógica interna para alternar entre procurar reto e girando
      if (isProcurandoReto) {
          moverFrente();
          if (millis() - procurandoPhaseStartTime > TIMEOUT_PROCURA_RETO_MS) {
              // Timeout andando reto, muda para fase girando
              isProcurandoReto = false;
              procurandoPhaseStartTime = millis(); // Reseta timer para fase girando
              Serial.println("Procurando: Timeout reto, mudando para girando.");
              // Não muda de estado principal, apenas muda a fase interna
          }
      } else { // isProcurandoGirando
          virarDireita(); // Gira para a direita para procurar
           if (millis() - procurandoPhaseStartTime > TIMEOUT_PROCURA_GIRANDO_MS) {
              // Timeout girando, volta para fase reta
              isProcurandoReto = true;
              procurandoPhaseStartTime = millis(); // Reseta timer para fase reta
              Serial.println("Procurando: Timeout girando, mudando para reta.");
              // Não muda de estado principal, apenas muda a fase interna
          }
      }

      // Transição para Alinhando se encontrar alvo em qualquer fase de procura
      if (objFrontal || objEsquerdo || objDireito) {
        pararMotores(); delay(50); 
        mudarEstado(ALINHANDO, "Alinhando");
        // Não precisa inicializar nada no ALINHANDO além do timer de estado (já feito em mudarEstado)
      }
      break;

    case ALINHANDO:
      // Prioriza alvo frontal. Se não tiver, usa laterais para alinhar.
      if (objFrontal) {
        // Alvo está na frente, pronto para atacar.
        mudarEstado(ATACANDO, "Atacando");
      } else if (objEsquerdo && !objDireito) {
        virarEsquerda(); 
      } else if (objDireito && !objEsquerdo) {
        virarDireita();
      } else if (objEsquerdo && objDireito) { 
         // Alvo em ambos os lados, gira para o lado onde o alvo está mais próximo.
         if(dist_esquerda_mm <= dist_direita_mm) virarEsquerda();
         else virarDireita();
      } else { 
        // Perdeu o alvo enquanto tentava alinhar. Volta a procurar (iniciando reto).
        Serial.println("Alinhando: Perdeu alvo. Voltando a procurar.");
        pararMotores(); delay(50);
        mudarEstado(PROCURANDO, "Procurando Reta (Perdeu Alvo)");
        isProcurandoReto = true;
        procurandoPhaseStartTime = millis();
      }
      break;

    case ATACANDO:
      moverFrente(); // Ataca em frente
      // Continua atacando enquanto houver alvo na frente ou nos lados para re-alinhar
      if (!objFrontal && (dist_frontal_mm > DISTANCIA_PERDA_ALVO_MM || dist_frontal_mm == 9999 || (!objEsquerdo && !objDireito)) ) { 
        // Alvo frontal perdido E não está mais detectado nos lados. Volta a procurar.
        Serial.println("Atacando: Perdeu alvo completo. Voltando a procurar.");
        pararMotores(); delay(50);
        mudarEstado(PROCURANDO, "Procurando Reta (Perdeu Alvo Atacando)");
        isProcurandoReto = true;
        procurandoPhaseStartTime = millis();
      } else if (!objFrontal && (objEsquerdo || objDireito)) { 
        // Alvo frontal perdido, mas ainda detectado nos lados. Volta a alinhar.
        Serial.println("Atacando: Perdeu alvo frontal, mas ainda lateral. Re-alinhando.");
        pararMotores(); delay(50); 
        mudarEstado(ALINHANDO, "Re-Alinhando");
      }
      // Se objFrontal é true, ele continua neste estado e chama moverFrente() novamente no próximo ciclo.
      break;
    
    case EVADINDO_RECUANDO: { // Use {} para definir escopo para variáveis locais
        // Determina o tempo de recuo com base no cenário de evasão
        unsigned long tempoRecuoNecessario = (currentEvasionScenario == SCENARIO_AMBOS) ? 
                                            TEMPO_RECUO_BORDA_AMBOS_MS : 
                                            TEMPO_RECUO_BORDA_LATERAL_MS;

        moverTras();
        if (millis() - tempoInicioEstadoAutonomo >= tempoRecuoNecessario) {
          pararMotores();
          delay(50); // Pequena pausa antes de girar
          mudarEstado(EVADINDO_GIRANDO, "Evade Girando");
          // Não reseta currentEvasionScenario ainda, precisa dele no estado EVADINDO_GIRANDO
        }
      } break; // Fim do escopo e break

    case EVADINDO_GIRANDO: { // Use {} para definir escopo para variáveis locais
        unsigned long tempoGiroNecessario;

        // Determina a direção e o tempo de giro com base no cenário
        if (currentEvasionScenario == SCENARIO_AMBOS) {
            virarDireita(); // Gira 180 para a direita (ajuste se necessário)
            tempoGiroNecessario = TEMPO_GIRO_180_BORDA_AMBOS_MS;
            Serial.println("Evade Girando 180 Direita.");
        } else if (currentEvasionScenario == SCENARIO_ESQUERDA) {
            virarDireita(); // Se borda na esquerda, gira para a direita 135
            tempoGiroNecessario = TEMPO_GIRO_135_BORDA_LATERAL_MS;
             Serial.println("Evade Girando 135 Direita.");
        } else { // SCENARIO_DIREITA
            virarEsquerda(); // Se borda na direita, gira para a esquerda 135
            tempoGiroNecessario = TEMPO_GIRO_135_BORDA_LATERAL_MS;
             Serial.println("Evade Girando 135 Esquerda.");
        }
        
        if (millis() - tempoInicioEstadoAutonomo >= tempoGiroNecessario) {
          pararMotores(); 
          delay(100);    // Pausa um pouco mais após o giro antes de tentar se mover
          
          currentEvasionScenario = SCENARIO_NONE; // Reseta o cenário de evasão

          // Após a evasão, volta para o estado de procura, iniciando com a fase "reto"
          mudarEstado(PROCURANDO, "Procurando Reta (Pos Evasao)");
          isProcurandoReto = true;
          procurandoPhaseStartTime = millis();
        }
      } break;

    case ERRO:
        pararMotores();
        break;

    default:  
      // Caso o estado atual seja um valor inesperado (erro de programação)
      Serial.println("Autonomo: Estado default inesperado! Parando.");
      mudarEstado(ERRO, "Erro: Estado Default");
      break;
  }
}

void mudarEstado(ModoAutonomoState novoEstado, String nomeEstadoDisplay) {
    if (estadoAutonomoAtual != novoEstado || statusAutonomoDisplay != nomeEstadoDisplay) { 
        Serial.print(millis()); Serial.print("ms | Mudando para estado: "); Serial.println(nomeEstadoDisplay);
    }
    estadoAutonomoAtual = novoEstado;
    statusAutonomoDisplay = nomeEstadoDisplay;
    tempoInicioEstadoAutonomo = millis(); // Reseta o timer do estado a cada mudança
}

void configurarPinos() {
  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_B_IN3, OUTPUT);
  pinMode(MOTOR_B_IN4, OUTPUT);
  
  pinMode(MOTOR_A_ENA, OUTPUT);
  digitalWrite(MOTOR_A_ENA, HIGH);
  pinMode(MOTOR_B_ENB, OUTPUT);
  digitalWrite(MOTOR_B_ENB, HIGH);

  pinMode(SENSOR_DIST_XSHUT_ESQUERDA, OUTPUT);
  pinMode(SENSOR_DIST_XSHUT_DIREITA, OUTPUT);
  pinMode(SENSOR_DIST_XSHUT_FRONTAL, OUTPUT);

  pararMotores(); 
}

void configurarSensoresVL53L0X() {
  Wire.begin(PINO_SDA, PINO_SCL); 

  digitalWrite(SENSOR_DIST_XSHUT_ESQUERDA, LOW);
  digitalWrite(SENSOR_DIST_XSHUT_DIREITA, LOW);
  digitalWrite(SENSOR_DIST_XSHUT_FRONTAL, LOW);
  delay(50);
  
  Serial.println("Configurando sensor Esquerda...");
  pinMode(SENSOR_DIST_XSHUT_ESQUERDA, INPUT); delay(10); 
  if (!sensorDistEsquerda.begin(0x31, &Wire)) { Serial.println(F("Falha ao inicializar sensor de distancia ESQUERDA!"));  while(1); }
  sensorDistEsquerda.setMeasurementTimingBudgetMicroSeconds(20000); 
  sensorDistEsquerda.startRangeContinuous(); 

  Serial.println("Configurando sensor Direita...");
  pinMode(SENSOR_DIST_XSHUT_DIREITA, INPUT); delay(10);
  if (!sensorDistDireita.begin(0x30, &Wire)) { Serial.println(F("Falha ao inicializar sensor de distancia DIREITA!"));  while(1); }
  sensorDistDireita.setMeasurementTimingBudgetMicroSeconds(20000);
  sensorDistDireita.startRangeContinuous();

  Serial.println("Configurando sensor Frontal...");
  pinMode(SENSOR_DIST_XSHUT_FRONTAL, INPUT); delay(10); 
  if (!sensorDistFrontal.begin(0x29, &Wire)) { Serial.println(F("Falha ao inicializar sensor de distancia FRONTAL!"));  while(1); } 
  sensorDistFrontal.setMeasurementTimingBudgetMicroSeconds(20000); 
  sensorDistFrontal.startRangeContinuous();

  Serial.println(F("Sensores de distancia VL53L0X OK!"));
}

void lerSensoresLinha() {
    if (isAnalogico) {
        linha_esquerda_analogico_val = analogRead(SENSOR_LINHA_ESQUERDA_A0);
        linha_direita_analogico_val = analogRead(SENSOR_LINHA_DIREITA_A0);
        linha_traseira_analogico_val = analogRead(SENSOR_LINHA_TRASEIRA_A0);

        linha_esquerda_char_atual = (linha_esquerda_analogico_val < LIMITE_BRANCO_ESQUERDA) ? 'B' : ((linha_esquerda_analogico_val > LIMITE_PRETO_ESQUERDA) ? 'P' : '-');
        linha_direita_char_atual = (linha_direita_analogico_val < LIMITE_BRANCO_DIREITA) ? 'B' : ((linha_direita_analogico_val > LIMITE_PRETO_DIREITA) ? 'P' : '-');
        linha_traseira_char_atual = (linha_traseira_analogico_val < LIMITE_BRANCO_TRASEIRA) ? 'B' : ((linha_traseira_analogico_val > LIMITE_PRETO_TRASEIRA) ? 'P' : '-');

        /*Serial.printf(
            "LINHA (ANALOGICO): L_E: %d (%c) | L_D: %d (%c) | L_T: %d (%c)\n",
            linha_esquerda_analogico_val,
            linha_esquerda_char_atual,
            linha_direita_analogico_val,
            linha_direita_char_atual,
            linha_traseira_analogico_val,
            linha_traseira_char_atual
        );*/
    } else {
        linha_esquerda_digital_val = digitalRead(SENSOR_LINHA_ESQUERDA_D0);
        linha_direita_digital_val = digitalRead(SENSOR_LINHA_DIREITA_D0);
        linha_traseira_digital_val = digitalRead(SENSOR_LINHA_TRASEIRA_D0);

        linha_esquerda_char_atual = (linha_esquerda_digital_val == 0) ? 'B' : ((linha_esquerda_digital_val == 1) ? 'P' : '-');
        linha_direita_char_atual = (linha_direita_digital_val == 0) ? 'B' : ((linha_direita_digital_val == 1) ? 'P' : '-');
        linha_traseira_char_atual = (linha_traseira_digital_val == 0) ? 'B' : ((linha_traseira_digital_val == 1) ? 'P' : '-');

        /*Serial.printf(
            "LINHA (DIGITAL): L_E: %d (%c) | L_D: %d (%c) | L_T: %d (%c)\n",
            linha_esquerda_digital_val,
            linha_esquerda_char_atual,
            linha_direita_digital_val,
            linha_direita_char_atual,
            linha_traseira_digital_val,
            linha_traseira_char_atual
        );*/
    }
}

void lerSensoresDistancia() { 
  VL53L0X_RangingMeasurementData_t measure;

  sensorDistEsquerda.rangingTest(&measure, false);
  dist_esquerda_mm = (measure.RangeStatus != 4) ? measure.RangeMilliMeter : 9999;

  sensorDistDireita.rangingTest(&measure, false);
  dist_direita_mm = (measure.RangeStatus != 4) ? measure.RangeMilliMeter : 9999;
  
  sensorDistFrontal.rangingTest(&measure, false);
  dist_frontal_mm = (measure.RangeStatus != 4) ? measure.RangeMilliMeter : 9999;

  /*Serial.printf(
    "DISTANCIA (mm): D_E: %d | D_D: %d | D_F: %d\n", 
    dist_esquerda_mm, 
    dist_direita_mm, 
    dist_frontal_mm
  );*/
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
