// ESP32_RoboSumo - VERSÃO FINAL COMBINADA
// ==========================================================================
// Bibliotecas
// ==========================================================================
#include <WiFi.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <ESPAsyncWebServer.h>
#include <Wire.h>

#include "Adafruit_VL53L0X.h"

// ==========================================================================
// Configurações e Mapeamento de Pinos
// ==========================================================================
// --- Rede
const char* WIFI_SSID = "RoboSumoGrupoA";
const char* WIFI_PASS = "caveirao";
const char* OTA_HOSTNAME = "robo-sumo-esp32";

// --- Pinos dos Motores (Ponte H L298N)
const int MOTOR_A_ENA = 32;
const int MOTOR_A_IN1 = 33;
const int MOTOR_A_IN2 = 25;
const int MOTOR_B_ENB = 14;
const int MOTOR_B_IN3 = 26;
const int MOTOR_B_IN4 = 27;

// --- Parâmetros do PWM
const int freq = 5000;
const int resolution = 8;
const int PWM_MAX_DUTY = (1 << resolution) - 1;

// --- Velocidade para Modo Autônomo
const int VELOCIDADE_PERCENTUAL_AUTONOMO = 150; // Porcentagem da velocidade máxima (0-100%) - CALIBRAR

// --- Pinos dos Sensores de Linha (TCRT5000)
const int SENSOR_LINHA_ESQUERDA_A0 = 34;
const int SENSOR_LINHA_DIREITA_A0 = 35;
const int SENSOR_LINHA_TRASEIRA_A0 = 39;

const int SENSOR_LINHA_ESQUERDA_D0 = 4;
const int SENSOR_LINHA_DIREITA_D0 = 15;
const int SENSOR_LINHA_TRASEIRA_D0 = 5;

// --- Pinos de Controle dos Sensores de Distância (VL53L0X)
const int SENSOR_DIST_XSHUT_ESQUERDA = 23;
const int SENSOR_DIST_XSHUT_DIREITA = 18;
const int SENSOR_DIST_XSHUT_FRONTAL = 19;

// ==========================================================================
// Objetos de Hardware e Rede
// ==========================================================================
AsyncWebServer server(80);
Adafruit_VL53L0X sensorDistEsquerda;
Adafruit_VL53L0X sensorDistDireita;
Adafruit_VL53L0X sensorDistFrontal;

// ==========================================================================
// Variáveis Globais de Estado
// ==========================================================================
int dist_esquerda_mm = 0;
int dist_direita_mm = 0;
int dist_frontal_mm = 0;

int linha_esquerda_analog = 0;
int linha_direita_analog = 0;
int linha_traseira_analog = 0;

int linha_esquerda_digital = 0;
int linha_direita_digital = 0;
int linha_traseira_digital = 0;

const int LIMITE_BRANCO_DIREITA = 1800;
const int LIMITE_PRETO_DIREITA = 2000;
const int LIMITE_BRANCO_ESQUERDA = 1400;
const int LIMITE_PRETO_ESQUERDA = 2200;
const int LIMITE_BRANCO_TRASEIRA = 1100;
const int LIMITE_PRETO_TRASEIRA = 1100;

char linha_esquerda_char = 'B';
char linha_direita_char = 'B';
char linha_traseira_char = 'B';

String direcaoAtual = "Iniciando";
int velocidadeAtual = 0; // 0-PWM_MAX_DUTY

unsigned long proximoUpdate = 0;
const long INTERVALO_UPDATE_MS = 500; // Alterado para 500ms

// ==========================================================================
// Configurações e Estados para o Modo Autônomo (Lógica Combinada)
// ==========================================================================
const unsigned long TEMPO_ESPERA_INICIAL_AUTONOMO_MS = 5000; // 5 segundos
const unsigned long TEMPO_GIRO_180_AUTONOMO_MS = 1000;      // CALIBRAR: Tempo para giro de 180 graus (evasão linha)

const int DISTANCIA_DETECCAO_OBJETO_MM = 700; // Distância máxima para considerar um objeto
const int DISTANCIA_MINIMA_OBJETO_MM = 20;    // Limiar mínimo para leitura de distância válida

// Constantes Modo Tourada
const int DISTANCIA_DETECCAO_PROXIMA_TOURADA_MM = 100; // Menos de 10cm para desvio
const unsigned long TEMPO_GIRO_DESVIO_TOURADA_MS = 600;        // CALIBRAR: Tempo para girar ~70 graus
const unsigned long TEMPO_AVANCO_TOURADA_POS_DESVIO_MS = 700; // CALIBRAR: Tempo para avançar ~15cm

// Constantes Modo Diagonal
const unsigned long TEMPO_GIRO_45_GRAUS_DIAGONAL_MS = 300;        // CALIBRAR: Tempo para girar ~45 graus
const unsigned long TEMPO_AVANCO_DIAGONAL_FINTA_MS = 700;        // CALIBRAR: Tempo para avançar ~15cm
const unsigned long TEMPO_GIRO_135_GRAUS_DIAGONAL_MS = 900;      // CALIBRAR: Tempo para girar ~135 graus
const unsigned long TIMEOUT_RECALCULO_ALVO_DIAGONAL_MS = 3000; // Timeout para re-procurar alvo na diagonal

enum ModoOperacaoAutonomo {
  MODO_AUTONOMO_NENHUM,
  MODO_AUTONOMO_PADRAO,
  MODO_AUTONOMO_TOURADA,
  MODO_AUTONOMO_DIAGONAL
};
ModoOperacaoAutonomo modoOperacaoAutonomoAtual = MODO_AUTONOMO_NENHUM;

enum ModoAutonomoState {
  AUTONOMO_DESATIVADO,            // Comum
  AUTONOMO_ESPERANDO_INICIO,        // Comum

  // Estados Modo Padrão (e comuns de ataque)
  PADRAO_PROCURANDO_ALVO,
  PADRAO_ALINHANDO_ESQUERDA,
  PADRAO_ALINHANDO_DIREITA,
  PADRAO_ATACANDO_FRENTE,

  // Estados Modo Tourada
  TOURADA_RECUA_ATE_BORDA,
  TOURADA_AGUARDANDO_PROXIMIDADE,
  TOURADA_GIRANDO_DESVIO,
  TOURADA_AVANCANDO_POS_DESVIO,        // Transita para PADRAO_PROCURANDO_ALVO

  // Estados Modo Diagonal
  DIAGONAL_PROCURANDO_ALVO_INICIAL,    // Primeira procura antes da finta
  DIAGONAL_FINTA_GIRO_45,
  DIAGONAL_FINTA_AVANCO,
  DIAGONAL_FINTA_GIRO_135,
  DIAGONAL_RECALCULANDO_ALVO_POS_FINTA, // Re-procura/alinha para PADRAO_ATACANDO_FRENTE

  // Estado Comum de Evasão de Linha
  AUTONOMO_EVASAO_LINHA_GIRANDO        // Comum
};
ModoAutonomoState estadoAutonomoAtual = AUTONOMO_DESATIVADO;
unsigned long tempoInicioEstadoAutonomo = 0;
String statusAutonomoDisplay = "Off";

// Variáveis auxiliares para lógicas específicas
bool touradaDesviarParaEsquerda = false;
bool diagonalFintaGiroInicialFoiEsquerda = false;

// ==========================================================================
// Página de Controle (HTML) - MODIFICADO: Adicionados 3 botões de modo
// ==========================================================================
const char PAGINA_CONTROLE_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head><title>Controle do Robo ESP32</title><meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no"><style>
body{font-family:Arial,sans-serif;text-align:center;background:#282c34;color:white}
h1{margin-top:20px}
.btn-container{display:grid;grid-template-columns:repeat(3, 1fr);gap:10px;max-width:300px;margin:30px auto}
.btn{padding:20px;font-size:24px;color:white;background-color:#61dafb;border:none;border-radius:10px;cursor:pointer;user-select:none;-webkit-tap-highlight-color:transparent}
.btn:active{background-color:#21a1f0}
.placeholder{visibility:hidden}
#frente{grid-column:2 / 3; grid-row:1 / 2;}
#esquerda{grid-column:1 / 2; grid-row:2 / 3;}
#parar{grid-column:2 / 3; grid-row:2 / 3; background-color:#e04444}
#direita{grid-column:3 / 4; grid-row:2 / 3;}
#tras{grid-column:2 / 3; grid-row:3 / 4;}
.modo-btn-container{display:flex;justify-content:center;gap:10px;margin-top:20px;margin-bottom:20px}
.modo-btn{padding:15px 20px;font-size:16px;background-color:#ff9800}
</style></head><body><h1>Controle do Robo</h1><div class="btn-container">
<div class="placeholder"></div> 
<button id="frente" class="btn" onmousedown="sendCommand('frente')" onmouseup="sendCommand('parar')" ontouchstart="sendCommand('frente')" ontouchend="sendCommand('parar')">↑</button>
<div class="placeholder"></div>
<button id="esquerda" class="btn" onmousedown="sendCommand('esquerda')" onmouseup="sendCommand('parar')" ontouchstart="sendCommand('esquerda')" ontouchend="sendCommand('parar')">←</button>
<button id="parar" class="btn" onclick="sendCommand('parar')">STOP</button>
<button id="direita" class="btn" onmousedown="sendCommand('direita')" onmouseup="sendCommand('parar')" ontouchstart="sendCommand('direita')" ontouchend="sendCommand('parar')">→</button>
<div class="placeholder"></div>
<button id="tras" class="btn" onmousedown="sendCommand('tras')" onmouseup="sendCommand('parar')" ontouchstart="sendCommand('tras')" ontouchend="sendCommand('parar')">↓</button>
<div class="placeholder"></div>
</div>

<div class="modo-btn-container">
  <button class="btn modo-btn" onclick="sendCommand('autonomo_padrao')">Modo Padrao</button>
  <button class="btn modo-btn" onclick="sendCommand('autonomo_tourada')">Modo Tourada</button>
  <button class="btn modo-btn" onclick="sendCommand('autonomo_diagonal')">Modo Diagonal</button>
</div>

<script>
function sendCommand(action){
  fetch('/control?action='+action)
    .then(response => response.text())
    .then(data => console.log(data))
    .catch(error => console.error('Erro ao enviar comando:', error));
}
</script></body></html>
)rawliteral";

// ==========================================================================
// Protótipos de Funções
// ==========================================================================
void configurarPinos();
void configurarSensores();
void configurarRede();
void configurarOTA();
void lerSensores();

void setMotorSpeed(int speed);
void moverFrente(int velocidade);
void moverTras(int velocidade);
void virarDireita(int velocidade);
void virarEsquerda(int velocidade);
void pararMotores();
void iniciarModoAutonomo(ModoOperacaoAutonomo novoModo); // Nova função auxiliar

// ==========================================================================
// Setup Principal
// ==========================================================================
void setup() {
  Serial.begin(115200);
  randomSeed(micros()); // Inicializa o gerador de números aleatórios

  configurarPinos();
  configurarSensores();
  configurarRede();
  configurarOTA();

  direcaoAtual = "Parado";
  velocidadeAtual = 0;
  Serial.println("Robo pronto para controle e atualizacao OTA.");
  Serial.print("Velocidade Autonomo: "); Serial.print(VELOCIDADE_PERCENTUAL_AUTONOMO); Serial.println("%");
}

// ==========================================================================
// Loop Principal - MÁQUINA DE ESTADOS AUTÔNOMA COMBINADA
// ==========================================================================
void loop() {
  ArduinoOTA.handle();

  if (modoOperacaoAutonomoAtual != MODO_AUTONOMO_NENHUM) { // Se algum modo autônomo está ativo
    int velocidadeAutonomoCalculada = map(VELOCIDADE_PERCENTUAL_AUTONOMO, 0, 100, 0, PWM_MAX_DUTY);

    // Variáveis de detecção (atualizadas pela função lerSensores() no timer global)
    bool objFrontal = (dist_frontal_mm < DISTANCIA_DETECCAO_OBJETO_MM && dist_frontal_mm > DISTANCIA_MINIMA_OBJETO_MM);
    bool objEsquerdo = (dist_esquerda_mm < DISTANCIA_DETECCAO_OBJETO_MM && dist_esquerda_mm > DISTANCIA_MINIMA_OBJETO_MM);
    bool objDireito = (dist_direita_mm < DISTANCIA_DETECCAO_OBJETO_MM && dist_direita_mm > DISTANCIA_MINIMA_OBJETO_MM);
    
    bool linhaDetectadaEsq = (linha_esquerda_char == 'B'); 
    bool linhaDetectadaDir = (linha_direita_char == 'B');
    bool linhaTraseiraDetectada = (linha_traseira_char == 'B');

    // --- LÓGICA DE PRIORIDADE: Detecção de Linha Frontal ---
    // (Não se aplica durante recuo na tourada, espera inicial ou durante a própria evasão)
    if (estadoAutonomoAtual != AUTONOMO_ESPERANDO_INICIO &&
        estadoAutonomoAtual != AUTONOMO_EVASAO_LINHA_GIRANDO &&
        estadoAutonomoAtual != TOURADA_RECUA_ATE_BORDA) { // Não interrompe o recuo para a borda
      
      if (linhaDetectadaEsq || linhaDetectadaDir) {
        Serial.println("Autonomo: LINHA FRONTAL DETECTADA! Iniciando evasao.");
        statusAutonomoDisplay = "Evasao Lin";
        estadoAutonomoAtual = AUTONOMO_EVASAO_LINHA_GIRANDO;
        tempoInicioEstadoAutonomo = millis();
        pararMotores();
        // O switch case AUTONOMO_EVASAO_LINHA_GIRANDO cuidará do giro
      }
    }

    // --- MÁQUINA DE ESTADOS PRINCIPAL ---
    // Só executa se não estiver em evasão de linha (a menos que o estado seja a própria evasão)
    if (estadoAutonomoAtual != AUTONOMO_EVASAO_LINHA_GIRANDO || 
       (estadoAutonomoAtual == AUTONOMO_EVASAO_LINHA_GIRANDO && (linhaDetectadaEsq || linhaDetectadaDir) ) ) {
      // A segunda condição acima é para permitir que o estado de evasão execute mesmo que a linha ainda seja detectada
      // Na verdade, a lógica de evasão já está no switch, então essa condição complexa pode não ser necessária,
      // a verificação de prioridade acima já deve ter direcionado para AUTONOMO_EVASAO_LINHA_GIRANDO se necessário.
      // Vou simplificar: o switch sempre roda, mas a prioridade da linha acima pode mudar o estado ANTES do switch.
    }


    switch (estadoAutonomoAtual) {
      // ================= ESTADOS COMUNS =================
      case AUTONOMO_ESPERANDO_INICIO:
        statusAutonomoDisplay = "Espera 5s";
        pararMotores(); 
        if (millis() - tempoInicioEstadoAutonomo >= TEMPO_ESPERA_INICIAL_AUTONOMO_MS) {
          Serial.println("Autonomo: Espera concluida.");
          // Determina o próximo estado baseado no modo de operação
          if (modoOperacaoAutonomoAtual == MODO_AUTONOMO_PADRAO) {
            Serial.println("Iniciando Modo Padrao: Procurando Alvo.");
            estadoAutonomoAtual = PADRAO_PROCURANDO_ALVO;
          } else if (modoOperacaoAutonomoAtual == MODO_AUTONOMO_TOURADA) {
            Serial.println("Iniciando Modo Tourada: Recuando ate borda.");
            estadoAutonomoAtual = TOURADA_RECUA_ATE_BORDA;
          } else if (modoOperacaoAutonomoAtual == MODO_AUTONOMO_DIAGONAL) {
            Serial.println("Iniciando Modo Diagonal: Procurando Alvo Inicial.");
            estadoAutonomoAtual = DIAGONAL_PROCURANDO_ALVO_INICIAL;
          }
          tempoInicioEstadoAutonomo = millis(); // Reinicia timer para o próximo estado
        }
        break;

      case AUTONOMO_EVASAO_LINHA_GIRANDO:
        statusAutonomoDisplay = "Evasao Lin";
        virarDireita(velocidadeAutonomoCalculada); // Gira 180 graus (ou para o lado desejado)

        if (millis() - tempoInicioEstadoAutonomo >= TEMPO_GIRO_180_AUTONOMO_MS) {
          Serial.println("Autonomo: Giro de evasao concluido.");
          pararMotores(); 
          delay(200);      // Pequena pausa para estabilizar
          // Volta para o estado de procura apropriado ao modo atual
          if (modoOperacaoAutonomoAtual == MODO_AUTONOMO_PADRAO || modoOperacaoAutonomoAtual == MODO_AUTONOMO_TOURADA) {
              // Tourada, após desvio, também entra na procura padrão
            Serial.println("Retornando a PADRAO_PROCURANDO_ALVO.");
            estadoAutonomoAtual = PADRAO_PROCURANDO_ALVO;
          } else if (modoOperacaoAutonomoAtual == MODO_AUTONOMO_DIAGONAL) {
            Serial.println("Retornando a DIAGONAL_PROCURANDO_ALVO_INICIAL.");
            estadoAutonomoAtual = DIAGONAL_PROCURANDO_ALVO_INICIAL; // Reinicia a sequência diagonal
          }
           tempoInicioEstadoAutonomo = millis();
        }
        break;

      // ================= ESTADOS MODO PADRÃO (E ATAQUE COMUM) =================
      case PADRAO_PROCURANDO_ALVO:
        statusAutonomoDisplay = "Procura";
        virarDireita(velocidadeAutonomoCalculada); // Gira para um lado para procurar

        if (objFrontal) {
          Serial.println("Padrao/Comum (Procura): Obj frontal. Atacando.");
          pararMotores(); delay(50);
          estadoAutonomoAtual = PADRAO_ATACANDO_FRENTE;
        } else if (objEsquerdo && !objDireito) { // Prioriza esquerda se só esquerdo
          Serial.println("Padrao/Comum (Procura): Obj esquerdo. Alinhando Esq.");
          pararMotores(); delay(50);
          estadoAutonomoAtual = PADRAO_ALINHANDO_ESQUERDA;
        } else if (objDireito && !objEsquerdo) { // Prioriza direita se só direito
          Serial.println("Padrao/Comum (Procura): Obj direito. Alinhando Dir.");
          pararMotores(); delay(50);
          estadoAutonomoAtual = PADRAO_ALINHANDO_DIREITA;
        } else if (objEsquerdo && objDireito) { // Se ambos, escolhe um (ex: esquerda)
          Serial.println("Padrao/Comum (Procura): Obj Esq&Dir. Alinhando Esq.");
          pararMotores(); delay(50);
          estadoAutonomoAtual = PADRAO_ALINHANDO_ESQUERDA;
        }
        break;

      case PADRAO_ALINHANDO_ESQUERDA:
        statusAutonomoDisplay = "Alinha Esq";
        virarEsquerda(velocidadeAutonomoCalculada);
        if (objFrontal) {
          Serial.println("Padrao/Comum (Alinha Esq): Alinhado frontal. Atacando.");
          pararMotores(); delay(50);
          estadoAutonomoAtual = PADRAO_ATACANDO_FRENTE;
        } else if (!objEsquerdo && !objFrontal) { // Perdeu o alvo
          Serial.println("Padrao/Comum (Alinha Esq): Perdeu alvo. Procurando.");
          estadoAutonomoAtual = PADRAO_PROCURANDO_ALVO;
           tempoInicioEstadoAutonomo = millis();
        }
        break;

      case PADRAO_ALINHANDO_DIREITA:
        statusAutonomoDisplay = "Alinha Dir";
        virarDireita(velocidadeAutonomoCalculada);
        if (objFrontal) {
          Serial.println("Padrao/Comum (Alinha Dir): Alinhado frontal. Atacando.");
          pararMotores(); delay(50);
          estadoAutonomoAtual = PADRAO_ATACANDO_FRENTE;
        } else if (!objDireito && !objFrontal) { // Perdeu o alvo
          Serial.println("Padrao/Comum (Alinha Dir): Perdeu alvo. Procurando.");
          estadoAutonomoAtual = PADRAO_PROCURANDO_ALVO;
          tempoInicioEstadoAutonomo = millis();
        }
        break;

      case PADRAO_ATACANDO_FRENTE:
        statusAutonomoDisplay = "Ataque";
        if (objFrontal) {
          moverFrente(velocidadeAutonomoCalculada);
        } else { // Perdeu o objeto da frente
          Serial.println("Padrao/Comum (Ataque): Perdeu obj frontal. Procurando.");
          pararMotores();
          estadoAutonomoAtual = PADRAO_PROCURANDO_ALVO;
          tempoInicioEstadoAutonomo = millis();
        }
        break;

      // ================= ESTADOS MODO TOURADA =================
      case TOURADA_RECUA_ATE_BORDA:
        statusAutonomoDisplay = "Tourada Recua";
        moverTras(velocidadeAutonomoCalculada);
        if (linhaTraseiraDetectada) {
          Serial.println("Tourada: Borda traseira detectada. Aguardando proximidade.");
          pararMotores();
          estadoAutonomoAtual = TOURADA_AGUARDANDO_PROXIMIDADE;
        }
        break;

      case TOURADA_AGUARDANDO_PROXIMIDADE:
        statusAutonomoDisplay = "Tourada AgProx";
        pararMotores();
        // Usa sensor frontal para detectar oponente se aproximando
        if (dist_frontal_mm < DISTANCIA_DETECCAO_PROXIMA_TOURADA_MM && dist_frontal_mm > DISTANCIA_MINIMA_OBJETO_MM) {
          Serial.println("Tourada: Oponente proximo! Iniciando desvio.");
          touradaDesviarParaEsquerda = (random(0, 2) == 0);
          estadoAutonomoAtual = TOURADA_GIRANDO_DESVIO;
          tempoInicioEstadoAutonomo = millis();
        }
        break;

      case TOURADA_GIRANDO_DESVIO:
        statusAutonomoDisplay = touradaDesviarParaEsquerda ? "Tourada GiraE" : "Tourada GiraD";
        if (touradaDesviarParaEsquerda) {
          virarEsquerda(velocidadeAutonomoCalculada);
        } else {
          virarDireita(velocidadeAutonomoCalculada);
        }
        if (millis() - tempoInicioEstadoAutonomo >= TEMPO_GIRO_DESVIO_TOURADA_MS) {
          Serial.println("Tourada: Giro de desvio concluido. Avancando.");
          pararMotores(); delay(50);
          estadoAutonomoAtual = TOURADA_AVANCANDO_POS_DESVIO;
          tempoInicioEstadoAutonomo = millis();
        }
        break;

      case TOURADA_AVANCANDO_POS_DESVIO:
        statusAutonomoDisplay = "Tourada Avanca";
        moverFrente(velocidadeAutonomoCalculada);
        if (millis() - tempoInicioEstadoAutonomo >= TEMPO_AVANCO_TOURADA_POS_DESVIO_MS) {
          Serial.println("Tourada: Avanco pos-desvio concluido. Entrando em modo de procura padrao.");
          pararMotores();
          estadoAutonomoAtual = PADRAO_PROCURANDO_ALVO; // Transita para a lógica padrão de procura
          tempoInicioEstadoAutonomo = millis();
        }
        break;

      // ================= ESTADOS MODO DIAGONAL =================
      case DIAGONAL_PROCURANDO_ALVO_INICIAL:
        statusAutonomoDisplay = "Diag Procura";
        virarDireita(velocidadeAutonomoCalculada); 

        if (objFrontal || objEsquerdo || objDireito) {
          Serial.println("Diagonal: Alvo inicial detectado. Iniciando finta (ataque diagonal).");
          pararMotores(); delay(100);
          diagonalFintaGiroInicialFoiEsquerda = (random(0, 2) == 0);
          estadoAutonomoAtual = DIAGONAL_FINTA_GIRO_45;
          tempoInicioEstadoAutonomo = millis();
        }
        break;

      case DIAGONAL_FINTA_GIRO_45:
        statusAutonomoDisplay = diagonalFintaGiroInicialFoiEsquerda ? "Diag G45E" : "Diag G45D";
        if (diagonalFintaGiroInicialFoiEsquerda) {
          virarEsquerda(velocidadeAutonomoCalculada);
        } else {
          virarDireita(velocidadeAutonomoCalculada);
        }
        if (millis() - tempoInicioEstadoAutonomo >= TEMPO_GIRO_45_GRAUS_DIAGONAL_MS) {
          Serial.println("Diagonal: Finta - Giro 45 concluido. Avancando.");
          pararMotores(); delay(100);
          estadoAutonomoAtual = DIAGONAL_FINTA_AVANCO;
          tempoInicioEstadoAutonomo = millis();
        }
        break;

      case DIAGONAL_FINTA_AVANCO:
        statusAutonomoDisplay = "Diag Avanco";
        moverFrente(velocidadeAutonomoCalculada);
        if (millis() - tempoInicioEstadoAutonomo >= TEMPO_AVANCO_DIAGONAL_FINTA_MS) {
          Serial.println("Diagonal: Finta - Avanco concluido. Girando 135 oposto.");
          pararMotores(); delay(100);
          estadoAutonomoAtual = DIAGONAL_FINTA_GIRO_135;
          tempoInicioEstadoAutonomo = millis();
        }
        break;
      
      case DIAGONAL_FINTA_GIRO_135:
        statusAutonomoDisplay = diagonalFintaGiroInicialFoiEsquerda ? "Diag G135D" : "Diag G135E";
        if (diagonalFintaGiroInicialFoiEsquerda) { // Giro de 45 foi Esquerda, então 135 é Direita
          virarDireita(velocidadeAutonomoCalculada);
        } else { // Giro de 45 foi Direita, então 135 é Esquerda
          virarEsquerda(velocidadeAutonomoCalculada);
        }
        if (millis() - tempoInicioEstadoAutonomo >= TEMPO_GIRO_135_GRAUS_DIAGONAL_MS) {
          Serial.println("Diagonal: Finta - Giro 135 concluido. Recalculando alvo para ataque.");
          pararMotores(); delay(100);
          estadoAutonomoAtual = DIAGONAL_RECALCULANDO_ALVO_POS_FINTA;
          tempoInicioEstadoAutonomo = millis();
        }
        break;

      case DIAGONAL_RECALCULANDO_ALVO_POS_FINTA:
        statusAutonomoDisplay = "Diag Recalc";
        // Gira lentamente para o lado que o alvo provavelmente estaria
        if (diagonalFintaGiroInicialFoiEsquerda) {  
           virarEsquerda(velocidadeAutonomoCalculada / 2);
        } else {  
           virarDireita(velocidadeAutonomoCalculada / 2);
        }
        
        if (objFrontal) {
          Serial.println("Diagonal (Recalc): Alvo frontal. Atacando (logica padrao).");
          pararMotores(); delay(50);
          estadoAutonomoAtual = PADRAO_ATACANDO_FRENTE; // Transita para ataque padrão
        } else if (objEsquerdo && !objDireito) {
          Serial.println("Diagonal (Recalc): Alvo esquerdo. Alinhando (logica padrao).");
            pararMotores(); delay(50);
          estadoAutonomoAtual = PADRAO_ALINHANDO_ESQUERDA;
        } else if (objDireito && !objEsquerdo) {
          Serial.println("Diagonal (Recalc): Alvo direito. Alinhando (logica padrao).");
            pararMotores(); delay(50);
          estadoAutonomoAtual = PADRAO_ALINHANDO_DIREITA;
        } else if (objEsquerdo && objDireito) {
            Serial.println("Diagonal (Recalc): Alvo Esq&Dir. Alinhando Esq (logica padrao).");
            pararMotores(); delay(50);
            estadoAutonomoAtual = PADRAO_ALINHANDO_ESQUERDA;
        } else if (millis() - tempoInicioEstadoAutonomo > TIMEOUT_RECALCULO_ALVO_DIAGONAL_MS) {
            Serial.println("Diagonal (Recalc): Timeout. Voltando a procurar alvo inicial para nova finta.");
            estadoAutonomoAtual = DIAGONAL_PROCURANDO_ALVO_INICIAL; // Reinicia a sequencia diagonal
            tempoInicioEstadoAutonomo = millis();
        }
        break;

      default:  
        // Se cair aqui, algo deu errado, desativa o modo autônomo
        Serial.println("ERRO: Estado autonomo desconhecido. Desativando.");
        modoOperacaoAutonomoAtual = MODO_AUTONOMO_NENHUM;
        estadoAutonomoAtual = AUTONOMO_DESATIVADO;
        statusAutonomoDisplay = "Off (Err)";
        pararMotores();
        break;
    }
  }

  if (millis() >= proximoUpdate) {
    proximoUpdate = millis() + INTERVALO_UPDATE_MS;
    lerSensores();      // Atualiza as leituras dos sensores
    // Adicionado: Printa os valores dos sensores no Serial Monitor
    Serial.print("Dist Esquerda: "); Serial.print(dist_esquerda_mm); Serial.print("mm | ");
    Serial.print("Dist Direita: "); Serial.print(dist_direita_mm); Serial.print("mm | ");
    Serial.print("Dist Frontal: "); Serial.print(dist_frontal_mm); Serial.print("mm | ");
    Serial.print("Linha Esq (A/D/C): "); Serial.print(linha_esquerda_analog); Serial.print("/"); Serial.print(linha_esquerda_digital); Serial.print("/"); Serial.print(linha_esquerda_char); Serial.print(" | ");
    Serial.print("Linha Dir (A/D/C): "); Serial.print(linha_direita_analog); Serial.print("/"); Serial.print(linha_direita_digital); Serial.print("/"); Serial.print(linha_direita_char); Serial.print(" | ");
    Serial.print("Linha Tras (A/D/C): "); Serial.print(linha_traseira_analog); Serial.print("/"); Serial.print(linha_traseira_digital); Serial.print("/"); Serial.println(linha_traseira_char);
    

  }
}

// ==========================================================================
// Funções de Configuração (chamadas no setup)
// ==========================================================================
void configurarPinos() {
  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_B_IN3, OUTPUT);
  pinMode(MOTOR_B_IN4, OUTPUT);

  ledcAttach(MOTOR_A_ENA, freq, resolution);
  ledcAttach(MOTOR_B_ENB, freq, resolution);

  pinMode(SENSOR_DIST_XSHUT_ESQUERDA, OUTPUT);
  pinMode(SENSOR_DIST_XSHUT_DIREITA, OUTPUT);
  pinMode(SENSOR_DIST_XSHUT_FRONTAL, OUTPUT);

  pinMode(SENSOR_LINHA_ESQUERDA_D0, INPUT);
  pinMode(SENSOR_LINHA_DIREITA_D0, INPUT);
  pinMode(SENSOR_LINHA_TRASEIRA_D0, INPUT); 

  pararMotores();
}



void configurarSensores() {
  Wire.begin(21, 22); // Inicia a comunicação I2C antes de tentar inicializar o display

  

  digitalWrite(SENSOR_DIST_XSHUT_ESQUERDA, LOW);
  digitalWrite(SENSOR_DIST_XSHUT_DIREITA, LOW);
  digitalWrite(SENSOR_DIST_XSHUT_FRONTAL, LOW);
  delay(50);
  
  digitalWrite(SENSOR_DIST_XSHUT_ESQUERDA, HIGH); delay(10);
  if (!sensorDistEsquerda.begin(0x31)) { Serial.println(F("Falha DE"));  while(1); }

  digitalWrite(SENSOR_DIST_XSHUT_DIREITA, HIGH); delay(10);
  if (!sensorDistDireita.begin(0x30)) { Serial.println(F("Falha DD"));  while(1); }

  digitalWrite(SENSOR_DIST_XSHUT_FRONTAL, HIGH); delay(10);
  if (!sensorDistFrontal.begin()) { Serial.println(F("Falha DF"));  while(1); }

  Serial.println(F("Sensores de distancia OK!"));
  
}

// NOVO: Função auxiliar para iniciar um modo autônomo específico
void iniciarModoAutonomo(ModoOperacaoAutonomo novoModoOperacao, const char* nomeModo) {
    if (modoOperacaoAutonomoAtual == MODO_AUTONOMO_NENHUM) { // Só inicia se estiver desativado
        Serial.print("Comando '"); Serial.print(nomeModo); Serial.println("' recebido. Iniciando espera de 5s.");
        modoOperacaoAutonomoAtual = novoModoOperacao;
        estadoAutonomoAtual = AUTONOMO_ESPERANDO_INICIO;
        tempoInicioEstadoAutonomo = millis();
        statusAutonomoDisplay = "Espera 5s";
        pararMotores(); 
    } else if (modoOperacaoAutonomoAtual == novoModoOperacao) { // Se clicar no mesmo modo novamente, desativa
        Serial.print("Comando '"); Serial.print(nomeModo); Serial.println("' recebido, mas ja estava ativo. Desativando.");
        modoOperacaoAutonomoAtual = MODO_AUTONOMO_NENHUM;
        estadoAutonomoAtual = AUTONOMO_DESATIVADO;
        statusAutonomoDisplay = "Off";
        pararMotores(); 
    } else { // Se estava em um modo e clicou em outro, muda para o novo modo
        Serial.print("Mudando de modo autonomo para '"); Serial.print(nomeModo); Serial.println("'. Iniciando espera de 5s.");
        modoOperacaoAutonomoAtual = novoModoOperacao;
        estadoAutonomoAtual = AUTONOMO_ESPERANDO_INICIO;
        tempoInicioEstadoAutonomo = millis();
        statusAutonomoDisplay = "Espera 5s";
        pararMotores();
    }
}


void configurarRede() {
  WiFi.softAP(WIFI_SSID, WIFI_PASS);
  Serial.print("Robo AP IP: ");
  Serial.println(WiFi.softAPIP());

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", PAGINA_CONTROLE_HTML);
  });

  server.on("/control", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("action")) {
      String action = request->getParam("action")->value();
      
      // Se qualquer comando manual for recebido, desativa qualquer modo autônomo
      if (action != "autonomo_padrao" && action != "autonomo_tourada" && action != "autonomo_diagonal") {
          if(modoOperacaoAutonomoAtual != MODO_AUTONOMO_NENHUM) {
            Serial.println("Modo Autonomo desativado por comando manual.");
            modoOperacaoAutonomoAtual = MODO_AUTONOMO_NENHUM;
            estadoAutonomoAtual = AUTONOMO_DESATIVADO;
            statusAutonomoDisplay = "Off";
            // A ação manual (frente, parar, etc.) será executada em seguida
          }
      }

      if (action == "frente") moverFrente(PWM_MAX_DUTY);
      else if (action == "tras") moverTras(PWM_MAX_DUTY);
      else if (action == "esquerda") virarEsquerda(PWM_MAX_DUTY);
      else if (action == "direita") virarDireita(PWM_MAX_DUTY);
      else if (action == "parar") pararMotores();
      else if (action == "autonomo_padrao") { 
        iniciarModoAutonomo(MODO_AUTONOMO_PADRAO, "autonomo_padrao");
      }
      else if (action == "autonomo_tourada") { 
        iniciarModoAutonomo(MODO_AUTONOMO_TOURADA, "autonomo_tourada");
      }
      else if (action == "autonomo_diagonal") { 
        iniciarModoAutonomo(MODO_AUTONOMO_DIAGONAL, "autonomo_diagonal");
      }
    }
    request->send(200, "text/plain", "OK");
  });

  server.begin();
}

void configurarOTA() {
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.onStart([]() {
    
  });
  ArduinoOTA.onEnd([]() {
    
    delay(1000);
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("OTA Error[%u]\n", error);
  });
  ArduinoOTA.begin();
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

  linha_esquerda_digital = digitalRead(SENSOR_LINHA_ESQUERDA_D0);
  linha_direita_digital = digitalRead(SENSOR_LINHA_DIREITA_D0);
  linha_traseira_digital = digitalRead(SENSOR_LINHA_TRASEIRA_D0);

  linha_esquerda_char = (linha_esquerda_analog < LIMITE_BRANCO_ESQUERDA) ? 'B' : ((linha_esquerda_analog > LIMITE_PRETO_ESQUERDA) ? 'P' : '-');
  linha_direita_char = (linha_direita_analog < LIMITE_BRANCO_DIREITA) ? 'B' : ((linha_direita_analog > LIMITE_PRETO_DIREITA) ? 'P' : '-');
  linha_traseira_char = (linha_traseira_analog < LIMITE_BRANCO_TRASEIRA) ? 'B' : ((linha_traseira_analog > LIMITE_PRETO_TRASEIRA) ? 'P' : '-');
}



void setMotorSpeed(int speed) {
  velocidadeAtual = constrain(speed, 0, PWM_MAX_DUTY);  
  ledcWrite(MOTOR_A_ENA, velocidadeAtual);
  ledcWrite(MOTOR_B_ENB, velocidadeAtual);
}

void moverFrente(int velocidade) {
  direcaoAtual = "Frente";
  setMotorSpeed(velocidade);  
  digitalWrite(MOTOR_A_IN1, HIGH);  
  digitalWrite(MOTOR_A_IN2, LOW);  
  digitalWrite(MOTOR_B_IN3, HIGH);  
  digitalWrite(MOTOR_B_IN4, LOW);  
}

void moverTras(int velocidade) {
  direcaoAtual = "Tras";
  setMotorSpeed(velocidade);
  digitalWrite(MOTOR_A_IN1, LOW);  
  digitalWrite(MOTOR_A_IN2, HIGH);  
  digitalWrite(MOTOR_B_IN3, LOW);  
  digitalWrite(MOTOR_B_IN4, HIGH);  
}

void virarEsquerda(int velocidade) {
  direcaoAtual = "Esquerda";
  setMotorSpeed(velocidade);
  digitalWrite(MOTOR_A_IN1, LOW);  
  digitalWrite(MOTOR_A_IN2, HIGH);  
  digitalWrite(MOTOR_B_IN3, HIGH);  
  digitalWrite(MOTOR_B_IN4, LOW);  
}

void virarDireita(int velocidade) {
  direcaoAtual = "Direita";
  setMotorSpeed(velocidade);
  digitalWrite(MOTOR_A_IN1, HIGH);  
  digitalWrite(MOTOR_A_IN2, LOW);  
  digitalWrite(MOTOR_B_IN3, LOW);  
  digitalWrite(MOTOR_B_IN4, HIGH);  
}

void pararMotores() {
  direcaoAtual = "Parado";
  setMotorSpeed(0);
  digitalWrite(MOTOR_A_IN1, LOW);
  digitalWrite(MOTOR_A_IN2, LOW);
  digitalWrite(MOTOR_B_IN3, LOW);
  digitalWrite(MOTOR_B_IN4, LOW);
}
