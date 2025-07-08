// ESP32_RoboSumo - VERSÃO FINAL COMBINADA (REVISÃO 3 - Lógica Padrão Refinada)
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

int linha_esquerda_analog_val = 0;
int linha_direita_analog_val = 0;
int linha_traseira_analog_val = 0;

// Limiares para sensores de linha - CALIBRAR CUIDADOSAMENTE
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
const long INTERVALO_UPDATE_SENSORES_DISTANCIA_MS = 200;

// ==========================================================================
// Configurações e Estados para o Modo Autônomo (Lógica Combinada) - REVISADO
// ==========================================================================
const unsigned long TEMPO_ESPERA_INICIAL_AUTONOMO_MS = 5000;
const unsigned long TEMPO_GIRO_180_EVASAO_MS = 800;
const unsigned long TEMPO_RECUO_10CM_BORDA_MS = 300;
const unsigned long TEMPO_PEQUENO_AVANCO_RECAPTURA_MS = 150; // Novo tempo para pequeno avanço

const int DISTANCIA_DETECCAO_INIMIGO_MM = 400;
const int DISTANCIA_MINIMA_OBJETO_MM = 20;
const int DISTANCIA_PERDA_ALVO_MM = 450;
const unsigned long TIMEOUT_PROCURA_RETO_MS = 2500;
const unsigned long TIMEOUT_PROCURA_GIRANDO_MS = 5000;

// Constantes Modo Tourada
const int DISTANCIA_INIMIGO_PROXIMO_TOURADA_MM = 149;
const unsigned long TEMPO_GIRO_45_DESVIO_TOURADA_MS = 300;
const unsigned long TEMPO_GIRO_135_POS_BORDA_TOURADA_MS = 750;
const unsigned long TIMEOUT_ESPERA_INIMIGO_TOURADA_MS = 10000;

// Constantes Modo Diagonal
const unsigned long TEMPO_GIRO_45_GRAUS_DIAGONAL_MS = 300;
const unsigned long TEMPO_AVANCO_40CM_DIAGONAL_MS = 800;
const unsigned long TEMPO_GIRO_BUSCA_DIAGONAL_MS = 600;
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
PADRAO_TENTANDO_RECAPTURAR, // Novo estado para tentar recapturar o alvo

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
void lerSensores(); // Função unificada para ler todos os sensores
void lerSensoresLinhaFrontal(); // Mantida para leitura rápida no loop

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
configurarSensoresVL53L0X();
configurarRede();
configurarOTA();

direcaoAtual = "Parado";
velocidadeAtual = 0;
mudarEstado(AUTONOMO_DESATIVADO, "Desativado");
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
bool permitirEvasaoPrioritaria = true;
if (estadoAutonomoAtual == AUTONOMO_ESPERANDO_INICIO ||
    estadoAutonomoAtual == AUTONOMO_EVASAO_BORDA_RECUANDO ||
    estadoAutonomoAtual == AUTONOMO_EVASAO_BORDA_GIRANDO_180) {
  permitirEvasaoPrioritaria = false;
}
if (modoOperacaoAutonomoAtual == MODO_AUTONOMO_TOURADA &&
    (estadoAutonomoAtual == TOURADA_RECUA_ATE_BORDA_TRASEIRA ||
     estadoAutonomoAtual == TOURADA_AVANCA_ATE_BORDA_DESVIO)) {
  permitirEvasaoPrioritaria = false;
}

if (permitirEvasaoPrioritaria && (linhaDetectadaEsq || linhaDetectadaDir)) {
  Serial.println("Autonomo: BORDA FRONTAL PRIORITARIA! Iniciando evasao.");
  pararMotores();
  delay(50);
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
            (modoOperacaoAutonomoAtual == MODO_AUTONOMO_TOURADA && estadoAutonomoAtual != TOURADA_AVANCA_ATE_BORDA_DESVIO) ) {
          mudarEstado(PADRAO_PROCURANDO_ALVO_RETO, "Padrao Reta");
        } else if (modoOperacaoAutonomoAtual == MODO_AUTONOMO_DIAGONAL) {
          mudarEstado(DIAGONAL_PROCURANDO_ALVO_INICIAL, "Diag ProcuraIni");
        } else {
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
      virarDireita();
      if (objFrontal || objEsquerdo || objDireito) {
        pararMotores(); delay(50);
        mudarEstado(PADRAO_ALINHANDO_COM_ALVO, "Padrao Alinha");
      } else if (millis() - tempoInicioEstadoAutonomo > TIMEOUT_PROCURA_GIRANDO_MS) {
        mudarEstado(PADRAO_PROCURANDO_ALVO_RETO, "Padrao Reta (Timeout Giro)");
      }
      break;

    case PADRAO_ALINHANDO_COM_ALVO:
      if (objFrontal) {
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
        mudarEstado(PADRAO_TENTANDO_RECAPTURAR, "Padrao Recap");
      }
      break;

    case PADRAO_TENTANDO_RECAPTURAR:
      moverFrente();
      if (objFrontal || objEsquerdo || objDireito) {
        pararMotores(); delay(50);
        mudarEstado(PADRAO_ALINHANDO_COM_ALVO, "Padrao Alinha (Recap)");
      } else if (millis() - tempoInicioEstadoAutonomo > TEMPO_PEQUENO_AVANCO_RECAPTURA_MS) {
        pararMotores(); delay(50);
        mudarEstado(PADRAO_PROCURANDO_ALVO_GIRANDO, "Padrao Gira (Nao Recap)");
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
      if (linhaDetectadaEsq || linhaDetectadaDir) {
          pararMotores(); delay(50);
          mudarEstado(TOURADA_GIRA_135_POS_BORDA, touradaDesviarParaEsquerda ? "Tourada GiraD135" : "Tourada GiraE135");
      }
      break;

    case TOURADA_GIRA_135_POS_BORDA:
      if (touradaDesviarParaEsquerda) {
          virarDireita();
      } else {
          virarEsquerda();
      }

      if (millis() - tempoInicioEstadoAutonomo >= TEMPO_GIRO_135_POS_BORDA_TOURADA_MS) {
        pararMotores(); delay(50);
        modoOperacaoAutonomoAtual = MODO_AUTONOMO_PADRAO;
        mudarEstado(PADRAO_PROCURANDO_ALVO_RETO, "Padrao Reta (PosTourada)");
      }
      break;

    // ================= ESTADOS MODO DIAGONAL =================
    case DIAGONAL_PROCURANDO_ALVO_INICIAL:
      virarDireita();
      if (objFrontal || objEsquerdo || objDireito) {
        pararMotores(); delay(100);
        mudarEstado(DIAGONAL_ESCOLHE_LADO_E_GIRA_45, "Diag EscolheLado");
      } else if (millis() - tempoInicioEstadoAutonomo > TIMEOUT_PROCURA_GIRANDO_MS) {
          mudarEstado(DIAGONAL_REINICIANDO_BUSCA, "Diag Reinicia (Timeout ProcuraIni)");
      }
      break;

    case DIAGONAL_ESCOLHE_LADO_E_GIRA_45:
      diagonalAtaqueParaEsquerda = (random(0, 2) == 0);
      if (diagonalAtaqueParaEsquerda) virarEsquerda();
      else virarDireita();

      if (millis() - tempoInicioEstadoAutonomo >= TEMPO_GIRO_45_GRAUS_DIAGONAL_MS) {
        pararMotores(); delay(100);
        mudarEstado(DIAGONAL_AVANCA_40CM, "Diag Avanca40cm");
      }
      break;

    case DIAGONAL_AVANCA_40CM:
      moverFrente();
      if (millis() - tempoInicioEstadoAutonomo >= TEMPO_AVANCO_40CM_DIAGONAL_MS) {
        pararMotores(); delay(100);
        mudarEstado(DIAGONAL_GIRA_BUSCA_POS_AVANCO, "Diag GiraBusca");
      }
      break;

    case DIAGONAL_GIRA_BUSCA_POS_AVANCO:
      if (diagonalAtaqueParaEsquerda) {
        virarDireita();
      } else {
        virarEsquerda();
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
      mudarEstado(DIAGONAL_PROCURANDO_ALVO_INICIAL, "Diag ProcuraIni (Reiniciou)");
      break;

    default:
      pararMotores();
      mudarEstado(AUTONOMO_DESATIVADO, "Off (Erro Estado)");
      modoOperacaoAutonomoAtual = MODO_AUTONOMO_NENHUM;
      break;
  }
}


} else {
if (estadoAutonomoAtual != AUTONOMO_DESATIVADO) {
mudarEstado(AUTONOMO_DESATIVADO, "Desativado");
}
}

if (millis() >= proximoUpdateSensoresDistancia) {
proximoUpdateSensoresDistancia = millis() + INTERVALO_UPDATE_SENSORES_DISTANCIA_MS;
lerSensores();
}
}

// ==========================================================================
// Funções de Configuração e Lógica
// ==========================================================================
void mudarEstado(ModoAutonomoState novoEstado, String nomeEstadoDisplay) {
if (estadoAutonomoAtual != novoEstado || statusAutonomoDisplay != nomeEstadoDisplay) {
Serial.print(millis()); Serial.print("ms | Mudando para estado: "); Serial.println(nomeEstadoDisplay);
}
estadoAutonomoAtual = novoEstado;
statusAutonomoDisplay = nomeEstadoDisplay;
tempoInicioEstadoAutonomo = millis();
}

void configurarPinos() {
pinMode(MOTOR_A_IN1, OUTPUT); pinMode(MOTOR_A_IN2, OUTPUT);
pinMode(MOTOR_B_IN3, OUTPUT); pinMode(MOTOR_B_IN4, OUTPUT);

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
Wire.begin(21, 22);

digitalWrite(SENSOR_DIST_XSHUT_ESQUERDA, LOW);
digitalWrite(SENSOR_DIST_XSHUT_DIREITA, LOW);
digitalWrite(SENSOR_DIST_XSHUT_FRONTAL, LOW);
delay(50);

Serial.println("Configurando sensor Esquerda...");
pinMode(SENSOR_DIST_XSHUT_ESQUERDA, INPUT); delay(10);
if (!sensorDistEsquerda.begin(0x31)) { Serial.println(F("Falha DE")); while(1); }
sensorDistEsquerda.setMeasurementTimingBudgetMicroSeconds(20000);
sensorDistEsquerda.startRangeContinuous();

Serial.println("Configurando sensor Direita...");
pinMode(SENSOR_DIST_XSHUT_DIREITA, INPUT); delay(10);
if (!sensorDistDireita.begin(0x30)) { Serial.println(F("Falha DD")); while(1); }
sensorDistDireita.setMeasurementTimingBudgetMicroSeconds(20000);
sensorDistDireita.startRangeContinuous();

Serial.println("Configurando sensor Frontal...");
pinMode(SENSOR_DIST_XSHUT_FRONTAL, INPUT); delay(10);
if (!sensorDistFrontal.begin()) { Serial.println(F("Falha DF")); while(1); }
sensorDistFrontal.setMeasurementTimingBudgetMicroSeconds(20000);
sensorDistFrontal.startRangeContinuous();

Serial.println(F("Sensores de distancia OK!"));
}

void iniciarModoAutonomo(ModoOperacaoAutonomo novoModoOperacao, const char* nomeModo) {
pararMotores();
delay(100);

if (modoOperacaoAutonomoAtual == MODO_AUTONOMO_NENHUM || modoOperacaoAutonomoAtual != novoModoOperacao) {
    Serial.print("Iniciando Modo: ");
}
