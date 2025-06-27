// ESP32_RoboSumo - VERSÃO FINAL COMBINADA (REVISÃO 2)
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

// --- Pinos dos Sensores de Linha (TCRT5000)
const int SENSOR_LINHA_ESQUERDA_A0 = 34;
const int SENSOR_LINHA_DIREITA_A0 = 35;
const int SENSOR_LINHA_TRASEIRA_A0 = 39;

const int SENSOR_LINHA_ESQUERDA_D0 = 4; // Não usado na lógica atual, mas mantido
const int SENSOR_LINHA_DIREITA_D0 = 15; // Não usado na lógica atual, mas mantido
const int SENSOR_LINHA_TRASEIRA_D0 = 5; // Não usado na lógica atual, mas mantido

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

int linha_esquerda_analog_val = 0; // Renomeado para evitar conflito com variável local no loop
int linha_direita_analog_val = 0;  // Renomeado
int linha_traseira_analog_val = 0; // Renomeado

// Limiares para sensores de linha - **CALIBRAR CUIDADOSAMENTE**
const int LIMITE_BRANCO_DIREITA = 1800; 
const int LIMITE_PRETO_DIREITA = 2000;   
const int LIMITE_BRANCO_ESQUERDA = 1400;
const int LIMITE_PRETO_ESQUERDA = 2200;  
const int LIMITE_BRANCO_TRASEIRA = 1100;
const int LIMITE_PRETO_TRASEIRA = 1300;  

char linha_esquerda_char_atual = 'P'; // Inicializado como Preto
char linha_direita_char_atual = 'P';
char linha_traseira_char_atual = 'P';

String direcaoAtual = "Iniciando";
int velocidadeAtual = 0; 

unsigned long proximoUpdateSensoresDistancia = 0;
// Intervalo para leitura dos sensores de DISTÂNCIA e linha TRASEIRA. Linhas frontais são lidas a cada ciclo.
const long INTERVALO_UPDATE_SENSORES_DISTANCIA_MS = 200; // Reduzido para melhor reatividade. Reverter para 500 se necessário.

// ==========================================================================
// Configurações e Estados para o Modo Autônomo (Lógica Combinada) - REVISADO
// ==========================================================================
const unsigned long TEMPO_ESPERA_INICIAL_AUTONOMO_MS = 5000; 
const unsigned long TEMPO_GIRO_180_EVASAO_MS = 800;      // CALIBRAR
const unsigned long TEMPO_RECUO_10CM_BORDA_MS = 300;        // CALIBRAR

const int DISTANCIA_DETECCAO_INIMIGO_MM = 400; // Ajustado para maior confiabilidade. CALIBRAR.
const int DISTANCIA_MINIMA_OBJETO_MM = 20;    
const int DISTANCIA_PERDA_ALVO_MM = 450;        // Ajustado. CALIBRAR.
const unsigned long TIMEOUT_PROCURA_RETO_MS = 2500; // Reduzido. CALIBRAR.
const unsigned long TIMEOUT_PROCURA_GIRANDO_MS = 5000; // Timeout para sair do giro infinito e tentar andar reto.

// Constantes Modo Tourada
const int DISTANCIA_INIMIGO_PROXIMO_TOURADA_MM = 149;    
const unsigned long TEMPO_GIRO_45_DESVIO_TOURADA_MS = 300; // CALIBRAR
const unsigned long TEMPO_GIRO_135_POS_BORDA_TOURADA_MS = 750; // CALIBRAR
const unsigned long TIMEOUT_ESPERA_INIMIGO_TOURADA_MS = 10000; // Se inimigo não aparecer, volta ao modo padrão

// Constantes Modo Diagonal
const unsigned long TEMPO_GIRO_45_GRAUS_DIAGONAL_MS = 300;    // CALIBRAR
const unsigned long TEMPO_AVANCO_40CM_DIAGONAL_MS = 800;  // CALIBRAR
const unsigned long TEMPO_GIRO_BUSCA_DIAGONAL_MS = 600;      // CALIBRAR: Tempo para girar (e.g. 90 graus)
const unsigned long TIMEOUT_BUSCA_POS_DIAGONAL_MS = 3000;    

enum ModoOperacaoAutonomo {
  MODO_AUTONOMO_NENHUM,
  MODO_AUTONOMO_PADRAO,
  MODO_AUTONOMO_TOURADA,
  MODO_AUTONOMO_DIAGONAL
};
ModoOperacaoAutonomo modoOperacaoAutonomoAtual = MODO_AUTONOMO_NENHUM;

enum ModoAutonomoState {
  AUTONOMO_DESATIVADO,
  AUTONOMO_ESPERANDO_INICIO,

  AUTONOMO_EVASAO_BORDA_RECUANDO,
  AUTONOMO_EVASAO_BORDA_GIRANDO_180,

  PADRAO_PROCURANDO_ALVO_RETO,
  PADRAO_PROCURANDO_ALVO_GIRANDO,
  PADRAO_ALINHANDO_COM_ALVO,
  PADRAO_ATACANDO_ALVO,

  TOURADA_RECUA_ATE_BORDA_TRASEIRA,
  TOURADA_PARADO_ESPERA_INIMIGO,
  TOURADA_DECIDE_LADO_DESVIO,
  TOURADA_GIRA_45_DESVIO,
  TOURADA_AVANCA_ATE_BORDA_DESVIO, 
  TOURADA_GIRA_135_POS_BORDA,       

  DIAGONAL_PROCURANDO_ALVO_INICIAL, 
  DIAGONAL_ESCOLHE_LADO_E_GIRA_45,
  DIAGONAL_AVANCA_40CM,
  DIAGONAL_GIRA_BUSCA_POS_AVANCO,   
  DIAGONAL_REINICIANDO_BUSCA      
};
ModoAutonomoState estadoAutonomoAtual = AUTONOMO_DESATIVADO;
unsigned long tempoInicioEstadoAutonomo = 0;
String statusAutonomoDisplay = "Off"; 

// Variáveis auxiliares
bool touradaGatilhoFrontal = false;
bool touradaGatilhoEsquerda = false;
bool touradaGatilhoDireita = false;
bool touradaDesviarParaEsquerda = false;

bool diagonalAtaqueParaEsquerda = false;

// ==========================================================================
// Página de Controle (HTML) - Adicionado DIV para status
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
#statusRobo{margin-top:10px;margin-bottom:10px;font-size:18px;color:#ffeb3b; min-height: 22px;}
</style></head><body><h1>Controle do Robo</h1>
<div id="statusRobo">Estado: Aguardando...</div>
<div class="btn-container">
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
    .then(data => console.log('Comando enviado:', action, 'Resposta:', data))
    .catch(error => console.error('Erro ao enviar comando:', error));
}
setInterval(function() {
  fetch('/status')
    .then(response => response.text())
    .then(data => {
      document.getElementById('statusRobo').innerText = 'Estado: ' + data;
    })
    .catch(error => { /* console.error('Erro ao buscar status:', error); */ });
}, 1000); // Atualiza a cada 1 segundo
</script></body></html>
)rawliteral";

// ==========================================================================
// Protótipos de Funções
// ==========================================================================
void configurarPinos();
void configurarSensoresVL53L0X();
void configurarRede();
void configurarOTA();
void lerSensoresDistanciaTraseira(); // Função separada para sensores no timer
void lerSensoresLinhaFrontal();      // Função para ler linhas frontais a cada ciclo

void moverFrente(); // Velocidade fixa
void moverTras(); // Velocidade fixa
void virarDireita(); // Velocidade fixa
void virarEsquerda(); // Velocidade fixa
void pararMotores();
void iniciarModoAutonomo(ModoOperacaoAutonomo novoModo, const char* nomeModo);
void mudarEstado(ModoAutonomoState novoEstado, String nomeEstadoDisplay);


// ==========================================================================
// Setup Principal
// ==========================================================================
void setup() {
  Serial.begin(115200);
  randomSeed(micros()); 

  configurarPinos();
  configurarSensoresVL53L0X(); // Renomeado para clareza
  configurarRede();
  configurarOTA();

  direcaoAtual = "Parado";
  velocidadeAtual = 0;
  mudarEstado(AUTONOMO_DESATIVADO, "Desativado"); // Define estado inicial
  Serial.println("Robo pronto para controle e atualizacao OTA.");
}

// ==========================================================================
// Loop Principal - MÁQUINA DE ESTADOS AUTÔNOMA REVISADA
// ==========================================================================
void loop() {
  ArduinoOTA.handle();

  // Leitura dos sensores de linha FRONTAIS a cada ciclo para evasão rápida
  lerSensoresLinhaFrontal();
  bool linhaDetectadaEsq = (linha_esquerda_char_atual == 'B'); 
  bool linhaDetectadaDir = (linha_direita_char_atual == 'B');

  if (modoOperacaoAutonomoAtual != MODO_AUTONOMO_NENHUM) {

    bool objFrontal = (dist_frontal_mm < DISTANCIA_DETECCAO_INIMIGO_MM && dist_frontal_mm > DISTANCIA_MINIMA_OBJETO_MM);
    bool objEsquerdo = (dist_esquerda_mm < DISTANCIA_DETECCAO_INIMIGO_MM && dist_esquerda_mm > DISTANCIA_MINIMA_OBJETO_MM);
    bool objDireito = (dist_direita_mm < DISTANCIA_DETECCAO_INIMIGO_MM && dist_direita_mm > DISTANCIA_MINIMA_OBJETO_MM);
    bool linhaTraseiraDetectada = (linha_traseira_char_atual == 'B');

    // --- LÓGICA DE PRIORIDADE MÁXIMA: Detecção de Borda Frontal ---
    bool realizarEvasaoPrioritaria = true;

    if (estadoAutonomoAtual == AUTONOMO_ESPERANDO_INICIO ||
        estadoAutonomoAtual == AUTONOMO_EVASAO_BORDA_RECUANDO ||
        estadoAutonomoAtual == AUTONOMO_EVASAO_BORDA_GIRANDO_180) {
      realizarEvasaoPrioritaria = false;
    }
    // Exceção para o modo Tourada em estados específicos onde a borda é tratada de forma diferente ou não deve interromper
    if (modoOperacaoAutonomoAtual == MODO_AUTONOMO_TOURADA) {
        if (estadoAutonomoAtual == TOURADA_RECUA_ATE_BORDA_TRASEIRA || // Não interrompe recuo para borda traseira
            estadoAutonomoAtual == TOURADA_AVANCA_ATE_BORDA_DESVIO) { // Este estado trata sua própria borda frontal
            realizarEvasaoPrioritaria = false;
        }
    }

    if (realizarEvasaoPrioritaria && (linhaDetectadaEsq || linhaDetectadaDir)) {
      Serial.println("Autonomo: BORDA FRONTAL PRIORITARIA! Iniciando evasao.");
      pararMotores(); 
      delay(50); // Pequena pausa para garantir que os motores pararam antes de reverter
      mudarEstado(AUTONOMO_EVASAO_BORDA_RECUANDO, "Evasao Re");
    } else {
      // --- MÁQUINA DE ESTADOS PRINCIPAL ---
      switch (estadoAutonomoAtual) {
        case AUTONOMO_ESPERANDO_INICIO:
          pararMotores(); 
          if (millis() - tempoInicioEstadoAutonomo >= TEMPO_ESPERA_INICIAL_AUTONOMO_MS) {
            Serial.println("Autonomo: Espera concluida.");
            if (modoOperacaoAutonomoAtual == MODO_AUTONOMO_PADRAO) {
              mudarEstado(PADRAO_PROCURANDO_ALVO_RETO, "Padrao Reta");
            } else if (modoOperacaoAutonomoAtual == MODO_AUTONOMO_TOURADA) {
              mudarEstado(TOURADA_RECUA_ATE_BORDA_TRASEIRA, "Tourada RecuaBorda");
            } else if (modoOperacaoAutonomoAtual == MODO_AUTONOMO_DIAGONAL) {
              mudarEstado(DIAGONAL_PROCURANDO_ALVO_INICIAL, "Diag ProcuraIni");
            }
          }
          break;

        case AUTONOMO_EVASAO_BORDA_RECUANDO:
          moverTras();
          if (millis() - tempoInicioEstadoAutonomo >= TEMPO_RECUO_10CM_BORDA_MS) {
            pararMotores();
            delay(50);
            mudarEstado(AUTONOMO_EVASAO_BORDA_GIRANDO_180, "Evasao Gira180");
          }
          break;

        case AUTONOMO_EVASAO_BORDA_GIRANDO_180:
          virarDireita(); 
          if (millis() - tempoInicioEstadoAutonomo >= TEMPO_GIRO_180_EVASAO_MS) {
            pararMotores(); 
            delay(100);    
            if (modoOperacaoAutonomoAtual == MODO_AUTONOMO_PADRAO || 
                (modoOperacaoAutonomoAtual == MODO_AUTONOMO_TOURADA && estadoAutonomoAtual != TOURADA_AVANCA_ATE_BORDA_DESVIO) ) { // Se Tourada não estava em avanço de desvio
              mudarEstado(PADRAO_PROCURANDO_ALVO_RETO, "Padrao Reta");
            } else if (modoOperacaoAutonomoAtual == MODO_AUTONOMO_DIAGONAL) {
              mudarEstado(DIAGONAL_PROCURANDO_ALVO_INICIAL, "Diag ProcuraIni");
            } else { // Caso específico da Tourada após desvio e evasão genérica (fallback, não deveria acontecer idealmente)
                mudarEstado(PADRAO_PROCURANDO_ALVO_RETO, "Padrao Reta (Fallback Tourada)");
            }
          }
          break;

        // ================= ESTADOS MODO PADRÃO =================
        case PADRAO_PROCURANDO_ALVO_RETO:
          moverFrente();
          if (objFrontal || objEsquerdo || objDireito) {
            pararMotores(); delay(50);
            mudarEstado(PADRAO_ALINHANDO_COM_ALVO, "Padrao Alinha");
          } else if (millis() - tempoInicioEstadoAutonomo > TIMEOUT_PROCURA_RETO_MS) {
            pararMotores(); delay(50);
            mudarEstado(PADRAO_PROCURANDO_ALVO_GIRANDO, "Padrao Gira");
          }
          break;

        case PADRAO_PROCURANDO_ALVO_GIRANDO:
          virarDireita(); // Gira mais devagar para procurar
          if (objFrontal || objEsquerdo || objDireito) {
            pararMotores(); delay(50);
            mudarEstado(PADRAO_ALINHANDO_COM_ALVO, "Padrao Alinha");
          } else if (millis() - tempoInicioEstadoAutonomo > TIMEOUT_PROCURA_GIRANDO_MS) {
            // Se girar por muito tempo e não achar, tenta andar reto novamente
            mudarEstado(PADRAO_PROCURANDO_ALVO_RETO, "Padrao Reta (Timeout Giro)");
          }
          break;

        case PADRAO_ALINHANDO_COM_ALVO:
          if (objFrontal) {
            // Não precisa parar e dar delay se já vai atacar
            mudarEstado(PADRAO_ATACANDO_ALVO, "Padrao Ataca");
          } else if (objEsquerdo && !objDireito) {
            virarEsquerda(); 
          } else if (objDireito && !objEsquerdo) {
            virarDireita();
          } else if (objEsquerdo && objDireito) { 
             if(dist_esquerda_mm <= dist_direita_mm) virarEsquerda();
             else virarDireita();
          } else { 
            pararMotores(); delay(50);
            mudarEstado(PADRAO_PROCURANDO_ALVO_GIRANDO, "Padrao Gira (Perdeu Alvo)");
          }
          break;

        case PADRAO_ATACANDO_ALVO:
          moverFrente();
          if (!objFrontal && (dist_frontal_mm > DISTANCIA_PERDA_ALVO_MM || (!objEsquerdo && !objDireito)) ) { 
            pararMotores();
            mudarEstado(PADRAO_PROCURANDO_ALVO_GIRANDO, "Padrao Gira (Perdeu Alvo Atacando)");
          } else if (!objFrontal && (objEsquerdo || objDireito)) { 
              pararMotores(); delay(50);
              mudarEstado(PADRAO_ALINHANDO_COM_ALVO, "Padrao Re-Alinha");
          }
          break;

        // ================= ESTADOS MODO TOURADA =================
        case TOURADA_RECUA_ATE_BORDA_TRASEIRA:
          moverTras();
          if (linhaTraseiraDetectada) {
            pararMotores();
            mudarEstado(TOURADA_PARADO_ESPERA_INIMIGO, "Tourada EsperaAdv");
          }
          break;

        case TOURADA_PARADO_ESPERA_INIMIGO:
          pararMotores(); 
          touradaGatilhoFrontal = (dist_frontal_mm < DISTANCIA_INIMIGO_PROXIMO_TOURADA_MM && dist_frontal_mm > DISTANCIA_MINIMA_OBJETO_MM);
          touradaGatilhoEsquerda = (dist_esquerda_mm < DISTANCIA_INIMIGO_PROXIMO_TOURADA_MM && dist_esquerda_mm > DISTANCIA_MINIMA_OBJETO_MM);
          touradaGatilhoDireita = (dist_direita_mm < DISTANCIA_INIMIGO_PROXIMO_TOURADA_MM && dist_direita_mm > DISTANCIA_MINIMA_OBJETO_MM);

          if (touradaGatilhoFrontal || touradaGatilhoEsquerda || touradaGatilhoDireita) {
            mudarEstado(TOURADA_DECIDE_LADO_DESVIO, "Tourada DecideDesvio");
          } else if (millis() - tempoInicioEstadoAutonomo > TIMEOUT_ESPERA_INIMIGO_TOURADA_MS) {
            Serial.println("Tourada: Timeout esperando inimigo. Mudando para modo Padrão.");
            modoOperacaoAutonomoAtual = MODO_AUTONOMO_PADRAO; // Muda para modo padrão
            mudarEstado(PADRAO_PROCURANDO_ALVO_RETO, "Padrao Reta (Timeout Tourada)");
          }
          break;

        case TOURADA_DECIDE_LADO_DESVIO:
          if (touradaGatilhoFrontal) {
            touradaDesviarParaEsquerda = (random(0, 2) == 0); 
          } else if (touradaGatilhoEsquerda) {
            touradaDesviarParaEsquerda = false; 
          } else if (touradaGatilhoDireita) {
            touradaDesviarParaEsquerda = true;  
          }
          mudarEstado(TOURADA_GIRA_45_DESVIO, touradaDesviarParaEsquerda ? "Tourada GiraE45" : "Tourada GiraD45");
          break;

        case TOURADA_GIRA_45_DESVIO:
          if (touradaDesviarParaEsquerda) virarEsquerda();
          else virarDireita();
          
          if (millis() - tempoInicioEstadoAutonomo >= TEMPO_GIRO_45_DESVIO_TOURADA_MS) {
            pararMotores(); delay(50);
            mudarEstado(TOURADA_AVANCA_ATE_BORDA_DESVIO, "Tourada AvancaBorda");
          }
          break;

        case TOURADA_AVANCA_ATE_BORDA_DESVIO:
          moverFrente();
          // A checagem de borda frontal (linhaDetectadaEsq || linhaDetectadaDir) é feita no início do loop.
          // Este estado confia que a leitura de linha frontal o tirará daqui.
          if (linhaDetectadaEsq || linhaDetectadaDir) { // Tratamento específico da borda para esta manobra
              pararMotores(); delay(50);
              mudarEstado(TOURADA_GIRA_135_POS_BORDA, touradaDesviarParaEsquerda ? "Tourada GiraD135" : "Tourada GiraE135");
          }
          // Adicionar um timeout aqui para caso não encontre a borda? E.g., após 3-4 segundos.
          // if (millis() - tempoInicioEstadoAutonomo > 4000) { ... mudarEstado(PADRAO_PROCURANDO_ALVO_RETO, "Timeout AvancaBorda"); }
          break;
        
        case TOURADA_GIRA_135_POS_BORDA:
          if (touradaDesviarParaEsquerda) { 
              virarDireita(); 
          } else { 
              virarEsquerda(); 
          }

          if (millis() - tempoInicioEstadoAutonomo >= TEMPO_GIRO_135_POS_BORDA_TOURADA_MS) {
            pararMotores(); delay(50);
            modoOperacaoAutonomoAtual = MODO_AUTONOMO_PADRAO; // Transita para modo padrão após manobra
            mudarEstado(PADRAO_PROCURANDO_ALVO_RETO, "Padrao Reta (PosTourada)");
          }
          break;

        // ================= ESTADOS MODO DIAGONAL =================
        case DIAGONAL_PROCURANDO_ALVO_INICIAL:
          virarDireita(); 
          if (objFrontal || objEsquerdo || objDireito) {
            pararMotores(); delay(100);
            mudarEstado(DIAGONAL_ESCOLHE_LADO_E_GIRA_45, "Diag EscolheLado");
          }
          // Adicionar timeout para não girar indefinidamente
          else if (millis() - tempoInicioEstadoAutonomo > TIMEOUT_PROCURA_GIRANDO_MS) {
              mudarEstado(DIAGONAL_REINICIANDO_BUSCA, "Diag Reinicia (Timeout ProcuraIni)"); // Ou tentar andar um pouco
          }
          break;

        case DIAGONAL_ESCOLHE_LADO_E_GIRA_45:
          diagonalAtaqueParaEsquerda = (random(0, 2) == 0);
          // Mudei: O giro é feito aqui, e a transição é para avançar
          if (diagonalAtaqueParaEsquerda) virarEsquerda();
          else virarDireita();
          
          if (millis() - tempoInicioEstadoAutonomo >= TEMPO_GIRO_45_GRAUS_DIAGONAL_MS) {
            pararMotores(); delay(100);
            mudarEstado(DIAGONAL_AVANCA_40CM, "Diag Avanca40cm");
          }
          break; // Certifique-se de que o estado DIAGONAL_ESCOLHE_LADO_E_GIRA_45 tenha seu próprio timer

        case DIAGONAL_AVANCA_40CM:
          // Este estado agora apenas avança. O giro de 45 foi feito no estado anterior.
          moverFrente();
          if (millis() - tempoInicioEstadoAutonomo >= TEMPO_AVANCO_40CM_DIAGONAL_MS) {
            pararMotores(); delay(100);
            mudarEstado(DIAGONAL_GIRA_BUSCA_POS_AVANCO, "Diag GiraBusca");
          }
          break;
        
        case DIAGONAL_GIRA_BUSCA_POS_AVANCO:
          if (diagonalAtaqueParaEsquerda) { 
            virarDireita(); // Gira para Direita (oposto ao 45E inicial)
          } else { 
            virarEsquerda(); // Gira para Esquerda (oposto ao 45D inicial)
          }

          if (objFrontal || objEsquerdo || objDireito) { 
              pararMotores(); delay(50);
              mudarEstado(PADRAO_ALINHANDO_COM_ALVO, "Padrao Alinha (PosDiag)"); 
          } else if (millis() - tempoInicioEstadoAutonomo >= TEMPO_GIRO_BUSCA_DIAGONAL_MS) { 
            pararMotores(); delay(100);
            mudarEstado(DIAGONAL_REINICIANDO_BUSCA, "Diag Reinicia (Nao Achou)"); 
          }
          break;
        
        case DIAGONAL_REINICIANDO_BUSCA:
          // Simplesmente volta a procurar alvo para tentar nova finta diagonal
          mudarEstado(DIAGONAL_PROCURANDO_ALVO_INICIAL, "Diag ProcuraIni (Reiniciou)");
          break;

        default:  
          pararMotores();
          mudarEstado(AUTONOMO_DESATIVADO, "Off (Erro Estado)");
          modoOperacaoAutonomoAtual = MODO_AUTONOMO_NENHUM;
          break;
      }
    }
  } else { // modoOperacaoAutonomoAtual == MODO_AUTONOMO_NENHUM
      if (estadoAutonomoAtual != AUTONOMO_DESATIVADO) {
          mudarEstado(AUTONOMO_DESATIVADO, "Desativado");
      }
  }


  if (millis() >= proximoUpdateSensoresDistancia) {
    proximoUpdateSensoresDistancia = millis() + INTERVALO_UPDATE_SENSORES_DISTANCIA_MS;
    lerSensoresDistanciaTraseira();      
  }
}

// ==========================================================================
// Funções de Configuração e Lógica
// ==========================================================================
void mudarEstado(ModoAutonomoState novoEstado, String nomeEstadoDisplay) {
    if (estadoAutonomoAtual != novoEstado || statusAutonomoDisplay != nomeEstadoDisplay) { // Atualiza se estado ou nome mudou
        Serial.print(millis()); Serial.print("ms | Mudando para estado: "); Serial.println(nomeEstadoDisplay);
    }
    estadoAutonomoAtual = novoEstado;
    statusAutonomoDisplay = nomeEstadoDisplay;
    tempoInicioEstadoAutonomo = millis(); // Reseta o timer do estado a cada mudança
}

void configurarPinos() {
  pinMode(MOTOR_A_IN1, OUTPUT); pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_B_IN3, OUTPUT); pinMode(MOTOR_B_IN4, OUTPUT);
  
  // Define os pinos ENA/ENB como OUTPUT e seta HIGH para sempre ter velocidade máxima
  pinMode(MOTOR_A_ENA, OUTPUT);
  digitalWrite(MOTOR_A_ENA, HIGH);
  pinMode(MOTOR_B_ENB, OUTPUT);
  digitalWrite(MOTOR_B_ENB, HIGH);

  pinMode(SENSOR_DIST_XSHUT_ESQUERDA, OUTPUT);
  pinMode(SENSOR_DIST_XSHUT_DIREITA, OUTPUT);
  pinMode(SENSOR_DIST_XSHUT_FRONTAL, OUTPUT);

  // Pinos dos sensores de linha já são configurados como INPUT por padrão no analogRead/digitalRead
  pararMotores();
}

void configurarSensoresVL53L0X() {
  Wire.begin(21, 22); 

  digitalWrite(SENSOR_DIST_XSHUT_ESQUERDA, LOW);
  digitalWrite(SENSOR_DIST_XSHUT_DIREITA, LOW);
  digitalWrite(SENSOR_DIST_XSHUT_FRONTAL, LOW);
  delay(50);
  
  Serial.println("Configurando sensor Esquerda...");
  pinMode(SENSOR_DIST_XSHUT_ESQUERDA, INPUT); delay(10); // Release XSHUT
  // A biblioteca Adafruit_VL53L0X mais recente usa 'begin()' para inicializar e configurar o endereço.
  // O parâmetro para 'begin()' pode ser o endereço I2C, e um 'init()' separado não é necessário.
  // setMeasurementTimingBudgetMicroSeconds() e startRangeContinuous() são os nomes corretos.
  if (!sensorDistEsquerda.begin(0x31)) { Serial.println(F("Falha DE"));  while(1); }
  sensorDistEsquerda.setMeasurementTimingBudgetMicroSeconds(20000); 
  sensorDistEsquerda.startRangeContinuous(); // Inicia medição contínua com intervalo padrão

  Serial.println("Configurando sensor Direita...");
  pinMode(SENSOR_DIST_XSHUT_DIREITA, INPUT); delay(10);
  if (!sensorDistDireita.begin(0x30)) { Serial.println(F("Falha DD"));  while(1); }
  sensorDistDireita.setMeasurementTimingBudgetMicroSeconds(20000);
  sensorDistDireita.startRangeContinuous();

  Serial.println("Configurando sensor Frontal...");
  pinMode(SENSOR_DIST_XSHUT_FRONTAL, INPUT); delay(10); // Endereço padrão 0x29
  // Para o sensor frontal, se não estamos mudando o endereço, podemos usar begin() sem argumentos,
  // ou com o endereço padrão (0x29) se a biblioteca exigir.
  if (!sensorDistFrontal.begin()) { Serial.println(F("Falha DF"));  while(1); } 
  sensorDistFrontal.setMeasurementTimingBudgetMicroSeconds(20000); 
  sensorDistFrontal.startRangeContinuous();

  Serial.println(F("Sensores de distancia OK!"));
}

void iniciarModoAutonomo(ModoOperacaoAutonomo novoModoOperacao, const char* nomeModo) {
    pararMotores(); // Garante que o robô para antes de iniciar novo modo/espera
    delay(100);      // Pequena pausa

    if (modoOperacaoAutonomoAtual == MODO_AUTONOMO_NENHUM || modoOperacaoAutonomoAtual != novoModoOperacao) { 
        Serial.print("Iniciando Modo: "); Serial.print(nomeModo); Serial.println(". Espera de 5s.");
        modoOperacaoAutonomoAtual = novoModoOperacao;
        mudarEstado(AUTONOMO_ESPERANDO_INICIO, "Espera 5s");
    } else { // Clicou no mesmo modo que já estava ativo, então desativa
        Serial.print("Modo "); Serial.print(nomeModo); Serial.println(" ja estava ativo. Desativando.");
        modoOperacaoAutonomoAtual = MODO_AUTONOMO_NENHUM;
        mudarEstado(AUTONOMO_DESATIVADO, "Desativado");
    }
}

void configurarRede() {
  WiFi.softAP(WIFI_SSID, WIFI_PASS);
  Serial.print("Robo AP IP: "); Serial.println(WiFi.softAPIP());

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", PAGINA_CONTROLE_HTML);
  });

  server.on("/status", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", statusAutonomoDisplay);
  });

  server.on("/control", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("action")) {
      String action = request->getParam("action")->value();
      
      if (action != "autonomo_padrao" && action != "autonomo_tourada" && action != "autonomo_diagonal") {
          if(modoOperacaoAutonomoAtual != MODO_AUTONOMO_NENHUM) {
            Serial.println("Modo Autonomo desativado por comando manual.");
            modoOperacaoAutonomoAtual = MODO_AUTONOMO_NENHUM;
            mudarEstado(AUTONOMO_DESATIVADO, "Desativado (Manual)");
          }
      }

      if (action == "frente") moverFrente();
      else if (action == "tras") moverTras();
      else if (action == "esquerda") virarEsquerda();
      else if (action == "direita") virarDireita();
      else if (action == "parar") pararMotores();
      else if (action == "autonomo_padrao")    iniciarModoAutonomo(MODO_AUTONOMO_PADRAO, "Padrao");
      else if (action == "autonomo_tourada")  iniciarModoAutonomo(MODO_AUTONOMO_TOURADA, "Tourada");
      else if (action == "autonomo_diagonal") iniciarModoAutonomo(MODO_AUTONOMO_DIAGONAL, "Diagonal");
    }
    request->send(200, "text/plain", "OK");
  });
  server.begin();
}

void configurarOTA() {
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.onStart([]() { Serial.println("OTA: Start"); });
  ArduinoOTA.onEnd([]() { Serial.println("\nOTA: End"); });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) { Serial.printf("OTA Progress: %u%%\r", (progress / (total / 100))); });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("OTA Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
}

void lerSensoresLinhaFrontal() {
    linha_esquerda_analog_val = analogRead(SENSOR_LINHA_ESQUERDA_A0);
    linha_direita_analog_val = analogRead(SENSOR_LINHA_DIREITA_A0);

    linha_esquerda_char_atual = (linha_esquerda_analog_val < LIMITE_BRANCO_ESQUERDA) ? 'B' : ((linha_esquerda_analog_val > LIMITE_PRETO_ESQUERDA) ? 'P' : '-');
    linha_direita_char_atual = (linha_direita_analog_val < LIMITE_BRANCO_DIREITA) ? 'B' : ((linha_direita_analog_val > LIMITE_PRETO_DIREITA) ? 'P' : '-');
}

void lerSensoresDistanciaTraseira() {
  // VL53L0X em modo contínuo, apenas lê o valor
  VL53L0X_RangingMeasurementData_t measure;

  sensorDistEsquerda.rangingTest(&measure, false);
  dist_esquerda_mm = (measure.RangeStatus != 4) ? measure.RangeMilliMeter : 9999;

  sensorDistDireita.rangingTest(&measure, false);
  dist_direita_mm = (measure.RangeStatus != 4) ? measure.RangeMilliMeter : 9999;
  
  sensorDistFrontal.rangingTest(&measure, false);
  dist_frontal_mm = (measure.RangeStatus != 4) ? measure.RangeMilliMeter : 9999;

  linha_traseira_analog_val = analogRead(SENSOR_LINHA_TRASEIRA_A0);
  linha_traseira_char_atual = (linha_traseira_analog_val < LIMITE_BRANCO_TRASEIRA) ? 'B' : ((linha_traseira_analog_val > LIMITE_PRETO_TRASEIRA) ? 'P' : '-');
}

void moverFrente() {
  direcaoAtual = "Frente";
  digitalWrite(MOTOR_A_IN1, HIGH); digitalWrite(MOTOR_A_IN2, LOW);  
  digitalWrite(MOTOR_B_IN3, HIGH); digitalWrite(MOTOR_B_IN4, LOW);  
}

void moverTras() {
  direcaoAtual = "Tras";
  digitalWrite(MOTOR_A_IN1, LOW); digitalWrite(MOTOR_A_IN2, HIGH);  
  digitalWrite(MOTOR_B_IN3, LOW); digitalWrite(MOTOR_B_IN4, HIGH);  
}

void virarEsquerda() {
  direcaoAtual = "Esquerda";
  digitalWrite(MOTOR_A_IN1, LOW);  digitalWrite(MOTOR_A_IN2, HIGH);  
  digitalWrite(MOTOR_B_IN3, HIGH); digitalWrite(MOTOR_B_IN4, LOW);  
}

void virarDireita() {
  direcaoAtual = "Direita";
  digitalWrite(MOTOR_A_IN1, HIGH); digitalWrite(MOTOR_A_IN2, LOW);  
  digitalWrite(MOTOR_B_IN3, LOW);  digitalWrite(MOTOR_B_IN4, HIGH);  
}

void pararMotores() {
  if (direcaoAtual != "Parado") { // Só printa se estava movendo
    Serial.println("Motores Parados");
  }
  direcaoAtual = "Parado";
  digitalWrite(MOTOR_A_IN1, LOW); digitalWrite(MOTOR_A_IN2, LOW);
  digitalWrite(MOTOR_B_IN3, LOW); digitalWrite(MOTOR_B_IN4, LOW);
}
