// ==========================================================================
// Bibliotecas
// ==========================================================================
#include <WiFi.h>           // Para OTA
#include <ESPmDNS.h>        // Para OTA
#include <ArduinoOTA.h>     // Para OTA
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "Adafruit_VL53L0X.h"

// ==========================================================================
// Configurações e Mapeamento de Pinos
// ==========================================================================

// --- Rede (para OTA)
const char* OTA_HOSTNAME = "robo-sumo-auto"; // Novo hostname para o modo autônomo
// Defina suas credenciais de Wi-Fi para OTA. Se não houver, o robô funcionará offline.
const char* WIFI_SSID_OTA = "SUA_REDE_WIFI";    // <<< COLOQUE O SSID DA SUA REDE AQUI
const char* WIFI_PASS_OTA = "SUA_SENHA_WIFI"; // <<< COLOQUE A SENHA DA SUA REDE AQUI

// --- Pinos dos Motores (Ponte H L298N)
const int MOTOR_A_ENA = 32; // Motor Direito (assumindo pela lógica de virar)
const int MOTOR_A_IN1 = 33;
const int MOTOR_A_IN2 = 25;
const int MOTOR_B_ENB = 14; // Motor Esquerdo (assumindo)
const int MOTOR_B_IN3 = 26;
const int MOTOR_B_IN4 = 27;

// --- Canais PWM para controle de velocidade dos motores
const int PWM_MOTOR_A_CANAL = 0; // Canal PWM para motor A (Direito)
const int PWM_MOTOR_B_CANAL = 1; // Canal PWM para motor B (Esquerdo)
const int PWM_FREQUENCIA = 5000; // Frequência em Hz para PWM
const int PWM_RESOLUCAO = 8;     // Resolução em bits (0-255 para 8 bits)

// --- Velocidades (0-255 para resolução de 8 bits)
const int VELOCIDADE_MAXIMA = 255;
const int VELOCIDADE_MEDIA = 180;
const int VELOCIDADE_BAIXA = 120;
const int VELOCIDADE_GIRO = 200;
const int VELOCIDADE_PROCURA = 100;

// --- Pinos dos Sensores de Linha (TCRT5000)
// Baseado na sua máquina de estados e nomes no display teleguiado:
// L1 (LD) - Sensor Frontal Direito
// L3 (LE) - Sensor Frontal Esquerdo
// L2 (LT) - Sensor Traseiro (ou Central, se usado para frente também)
const int SENSOR_LINHA_FD = 4;  // Frontal Direito (era L1)
const int SENSOR_LINHA_FE = 15; // Frontal Esquerdo (era L3)
const int SENSOR_LINHA_TR = 5;  // Traseiro (era L2)

// --- Pinos de Controle dos Sensores de Distância (VL53L0X XSHUT)
const int SENSOR_DIST_XSHUT_DIR = 18; // Sensor Direita (era XSHUT_1)
const int SENSOR_DIST_XSHUT_FRT = 19; // Sensor Frontal (era XSHUT_2)
const int SENSOR_DIST_XSHUT_ESQ = 23; // Sensor Esquerda (era XSHUT_3)

// --- Pinos de Controle do Robô (Botões/Chaves com PULLUP interno)
const int PINO_BOTAO_START = 13;    // Botão para iniciar após seleção de modo
const int PINO_CHAVE_MODO_1 = 12;   // Chave para Modo Padrão
const int PINO_CHAVE_MODO_2 = 17;   // Chave para Modo Tourada
const int PINO_CHAVE_MODO_3 = 16;   // Chave para Modo Diagonal

// ==========================================================================
// Objetos de Hardware
// ==========================================================================
Adafruit_SSD1306 display(128, 64, &Wire, -1);
Adafruit_VL53L0X sensorDistDir; // Sensor de distância Direito
Adafruit_VL53L0X sensorDistFrt; // Sensor de distância Frontal
Adafruit_VL53L0X sensorDistEsq; // Sensor de distância Esquerdo

// ==========================================================================
// Enumerações e Estados Globais
// ==========================================================================
enum EstadoRobo {
    CONFIGURANDO,
    AGUARDANDO_START,    // Aguardando seleção de modo e botão start
    ESPERA_INICIAL_5S,   // Contagem regressiva de 5 segundos após start

    // Estados Modo Padrão
    MP_ANDA_FRENTE,
    MP_GIRO_360,
    MP_PROCURA,

    // Estados Modo Tourada
    MT_RE,
    MT_ESPERA_POS_BORDA, // Espera após detectar borda na ré
    MT_DESVIA,
    // MT_PROCURA (reutiliza MP_PROCURA)

    // Estados Modo Diagonal
    MD_ANDA_DIAGONAL,
    MD_VIRA_PROCURA,
    MD_ANDA_RETO_POS_VIRA, // Anda reto após virar e procurar no modo diagonal
    // MD_GIRO_360 (reutiliza MP_GIRO_360)
    // MD_PROCURA (reutiliza MP_PROCURA)

    ROBO_PARADO // Estado final opcional ou para emergência
};

enum ModoOperacao {
    NENHUM_MODO,
    MODO_PADRAO,
    MODO_TOURADA,
    MODO_DIAGONAL
};

EstadoRobo estadoAtual = CONFIGURANDO;
ModoOperacao modoSelecionado = NENHUM_MODO;
String nomeEstadoAtual = "Config"; // Para debug no display

// --- Variáveis de Sensor (atualizadas periodicamente)
int distDir_mm = 9999, distFrt_mm = 9999, distEsq_mm = 9999;
bool bordaFD_detectada = false, bordaFE_detectada = false, bordaTR_detectada = false;

// --- Temporizadores e Contadores
unsigned long timestampEntradaEstado = 0;   // Registra `millis()` ao entrar em um novo estado
unsigned long proximoUpdateSensoresDisplay = 0;
const long INTERVALO_UPDATE_SENSORES_DISPLAY_MS = 100; // Intervalo para ler sensores e atualizar display

// --- Parâmetros de Comportamento (AJUSTAR EXPERIMENTALMENTE)
const int DISTANCIA_DETECCAO_INIMIGO_MM = 400;     // Limiar para "Detectou Inimigo"
const int DISTANCIA_INIMIGO_PROXIMO_MM = 100;      // Limiar para "Adversário a menos de 10 cm"
const unsigned long TEMPO_GIRO_360_COMPLETO_MS = 1500; // Tempo para um giro de 360 graus
const unsigned long TEMPO_ANDA_DIAGONAL_10CM_MS = 800; // Tempo para andar aprox. 10cm na diagonal
const unsigned long TEMPO_MANOBRA_DESVIO_MS = 1200;    // Duração total da manobra de desvio
const unsigned long TEMPO_PROCURA_MS = 3000;           // Tempo máximo no estado de procura antes de mudar estratégia
const unsigned long TEMPO_ESPERA_TOURADA_MS = 3000;    // Tempo de espera no modo tourada antes de procurar

// ==========================================================================
// Protótipos de Funções
// ==========================================================================

// --- Configuração
void configurarPinosHardware();
void configurarPWMmotores();
void configurarSensoresVL53();
void configurarDisplayInicial();
void configurarOTA();

// --- Leitura de Sensores
void lerTodosSensores();
void lerSensoresDeDistancia();
void lerSensoresDeLinha();

// --- Lógica de Decisão (Abstrações dos sensores)
bool inimigoDetectadoGeral();
bool inimigoMuitoProximoFrente();
bool bordaFrontalDetectada(); // Qualquer um dos sensores frontais
bool bordaTraseiraDetectadaUnica();

// --- Controle dos Motores (Funções de movimento básicas e complexas)
void moverMotoresPWM(int velEsq, int velDir); // Controle base com PWM (-255 a 255)
void pararMotoresCompletamente();
void moverFrentePadrao(int velocidade = VELOCIDADE_MAXIMA);
void moverTrasPadrao(int velocidade = VELOCIDADE_MAXIMA);
void girarNoEixoHorario(int velocidade = VELOCIDADE_GIRO);    // Direita para trás, Esquerda para frente
void girarNoEixoAntiHorario(int velocidade = VELOCIDADE_GIRO); // Direita para frente, Esquerda para trás
void moverEmDiagonalFrenteDireita(int velPrincipal = VELOCIDADE_MEDIA, int velAjuste = VELOCIDADE_BAIXA);
void moverEmDiagonalFrenteEsquerda(int velPrincipal = VELOCIDADE_MEDIA, int velAjuste = VELOCIDADE_BAIXA);

// --- Funções de Estado (Cada estado da máquina terá sua função)
void executarEstadoConfigurando();
void executarEstadoAguardandoStart();
void executarEstadoEsperaInicial5s();
// Modo Padrão
void executarEstadoMpAndaFrente();
void executarEstadoMpGiro360();
void executarEstadoMpProcura();
// Modo Tourada
void executarEstadoMtRe();
void executarEstadoMtEsperaPosBorda();
void executarEstadoMtDesvia();
// Modo Diagonal
void executarEstadoMdAndaDiagonal();
void executarEstadoMdViraProcura();
void executarEstadoMdAndaRetoPosVira();

// --- Utilitários e Display
void atualizarDisplayAutonomoInfo();
void mudarParaEstado(EstadoRobo novoEstado, const String& nomeNovoEstado);
String obterNomeModoOperacao(ModoOperacao modo);

// ==========================================================================
// Setup Principal
// ==========================================================================
void setup() {
  Serial.begin(115200);
  Serial.println("\n###################################");
  Serial.println("Iniciando Robo de Sumo Autonomo");
  Serial.println("###################################");

  mudarParaEstado(CONFIGURANDO, "Configurando");

  configurarPinosHardware();
  configurarPWMmotores();
  configurarDisplayInicial(); // Mostrar mensagens de inicialização
  configurarSensoresVL53();   // Tenta inicializar os sensores
  configurarOTA();            // Configura WiFi e OTA

  pararMotoresCompletamente(); // Garante que os motores estão parados

  Serial.println("Configuracao completa. Aguardando modo e start...");
  mudarParaEstado(AGUARDANDO_START, "Aguard.Start");
}

// ==========================================================================
// Loop Principal
// ==========================================================================
void loop() {
  ArduinoOTA.handle(); // Processa requisições OTA

  // Leitura de sensores e atualização do display em intervalos regulares
  if (millis() >= proximoUpdateSensoresDisplay) {
    proximoUpdateSensoresDisplay = millis() + INTERVALO_UPDATE_SENSORES_DISPLAY_MS;
    lerTodosSensores();
    atualizarDisplayAutonomoInfo();
  }

  // Máquina de Estados Principal
  switch (estadoAtual) {
    case CONFIGURANDO:            executarEstadoConfigurando();           break;
    case AGUARDANDO_START:        executarEstadoAguardandoStart();        break;
    case ESPERA_INICIAL_5S:       executarEstadoEsperaInicial5s();        break;
    
    case MP_ANDA_FRENTE:          executarEstadoMpAndaFrente();           break;
    case MP_GIRO_360:             executarEstadoMpGiro360();              break;
    case MP_PROCURA:              executarEstadoMpProcura();              break;

    case MT_RE:                   executarEstadoMtRe();                   break;
    case MT_ESPERA_POS_BORDA:     executarEstadoMtEsperaPosBorda();       break;
    case MT_DESVIA:               executarEstadoMtDesvia();               break;

    case MD_ANDA_DIAGONAL:        executarEstadoMdAndaDiagonal();         break;
    case MD_VIRA_PROCURA:         executarEstadoMdViraProcura();          break;
    case MD_ANDA_RETO_POS_VIRA:   executarEstadoMdAndaRetoPosVira();      break;
    
    case ROBO_PARADO:
      pararMotoresCompletamente();
      // Fica aqui até ser resetado ou desligado
      break;
    default:
      Serial.println("ERRO: Estado desconhecido! Parando robo.");
      mudarParaEstado(ROBO_PARADO, "Erro Parado");
      break;
  }
  delay(10); // Pequeno delay para estabilidade e permitir que outras tarefas (como OTA) rodem.
}

// ==========================================================================
// Implementações das Funções de Configuração
// ==========================================================================
void configurarPinosHardware() {
  // Motores (INs)
  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_B_IN3, OUTPUT);
  pinMode(MOTOR_B_IN4, OUTPUT);
  // Pinos ENA/ENB serão controlados por PWM, configurados em configurarPWMmotores()

  // Sensores de Linha
  pinMode(SENSOR_LINHA_FD, INPUT);
  pinMode(SENSOR_LINHA_FE, INPUT);
  pinMode(SENSOR_LINHA_TR, INPUT);

  // Sensores de Distância (XSHUT)
  pinMode(SENSOR_DIST_XSHUT_DIR, OUTPUT);
  pinMode(SENSOR_DIST_XSHUT_FRT, OUTPUT);
  pinMode(SENSOR_DIST_XSHUT_ESQ, OUTPUT);

  // Controles do Robô (com pull-up interno, LOW quando ativos)
  pinMode(PINO_BOTAO_START, INPUT_PULLUP);
  pinMode(PINO_CHAVE_MODO_1, INPUT_PULLUP);
  pinMode(PINO_CHAVE_MODO_2, INPUT_PULLUP);
  pinMode(PINO_CHAVE_MODO_3, INPUT_PULLUP);
  Serial.println("Pinos de IO configurados.");
}

void configurarPWMmotores() {
  // Configura Canal 0 para Motor A (Direito)
  ledcSetup(PWM_MOTOR_A_CANAL, PWM_FREQUENCIA, PWM_RESOLUCAO);
  ledcAttachPin(MOTOR_A_ENA, PWM_MOTOR_A_CANAL);

  // Configura Canal 1 para Motor B (Esquerdo)
  ledcSetup(PWM_MOTOR_B_CANAL, PWM_FREQUENCIA, PWM_RESOLUCAO);
  ledcAttachPin(MOTOR_B_ENB, PWM_MOTOR_B_CANAL);

  Serial.println("PWM para motores configurado.");
}

void configurarDisplayInicial() {
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("Falha ao iniciar display SSD1306"));
  } else {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Robo Sumo Autonomo");
    display.println("Inicializando...");
    display.display();
    Serial.println("Display OLED OK.");
  }
}

void configurarSensoresVL53() {
  if (display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Garante que o display está ok para mensagens
    display.setCursor(0, 20); // Posição para mensagens de status dos sensores
    display.print("Sensores Dist: ");
    display.display();
  }

  // Desliga todos os sensores para configurar endereços individualmente
  digitalWrite(SENSOR_DIST_XSHUT_DIR, LOW);
  digitalWrite(SENSOR_DIST_XSHUT_FRT, LOW);
  digitalWrite(SENSOR_DIST_XSHUT_ESQ, LOW);
  delay(50);
  
  Wire.begin(21, 22); // Pinos I2C: SDA=21, SCL=22 (padrão ESP32 Heltec WiFi Kit 32)

  // Configura Sensor Direito (novo endereço 0x30)
  digitalWrite(SENSOR_DIST_XSHUT_DIR, HIGH); delay(10);
  if (!sensorDistDir.begin(0x30)) { 
    Serial.println(F("Falha Sensor Dist DIR (0x30)")); 
    if (display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { display.print("DIR Falha "); display.display(); }
  } else {
    Serial.println(F("Sensor Dist DIR OK (0x30)."));
    if (display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { display.print("DIR OK "); display.display(); }
    sensorDistDir.startContinuous(); // Inicia medições contínuas
  }

  // Configura Sensor Esquerdo (novo endereço 0x31)
  digitalWrite(SENSOR_DIST_XSHUT_ESQ, HIGH); delay(10);
  if (!sensorDistEsq.begin(0x31)) { 
    Serial.println(F("Falha Sensor Dist ESQ (0x31)")); 
    if (display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { display.print("ESQ Falha "); display.display(); }
  } else {
    Serial.println(F("Sensor Dist ESQ OK (0x31)."));
    if (display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { display.print("ESQ OK "); display.display(); }
    sensorDistEsq.startContinuous();
  }

  // Configura Sensor Frontal (endereço padrão 0x29)
  digitalWrite(SENSOR_DIST_XSHUT_FRT, HIGH); delay(10);
  if (!sensorDistFrt.begin()) { 
    Serial.println(F("Falha Sensor Dist FRT (padrao 0x29)")); 
    if (display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { display.print("FRT Falha "); display.display(); }
  } else {
    Serial.println(F("Sensor Dist FRT OK (padrao 0x29)."));
    if (display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { display.print("FRT OK "); display.display(); }
    sensorDistFrt.startContinuous();
  }
  delay(100); 
  if (display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
      display.setCursor(0,30); display.println("Sensores Dist. OK!"); display.display(); delay(500);
  }
}

void configurarOTA() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID_OTA, WIFI_PASS_OTA);
  
  if (display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
      display.setCursor(0,40); display.print("Conectando WiFi OTA"); display.display();
  }
  Serial.print("Conectando ao WiFi para OTA...");

  int tentativasWifi = 0;
  while (WiFi.status() != WL_CONNECTED && tentativasWifi < 20) { // Tenta por ~10 segundos
    delay(500);
    Serial.print(".");
    if (display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { display.print("."); display.display(); }
    tentativasWifi++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi conectado para OTA!");
    Serial.print("IP: "); Serial.println(WiFi.localIP());
    if (display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        display.setCursor(0,50); display.print(WiFi.localIP()); display.display();
    }
    ArduinoOTA.setHostname(OTA_HOSTNAME);
    // Callbacks do OTA (simplificado, pode adicionar mais detalhes como no código teleguiado)
    ArduinoOTA.onStart([]() {
      pararMotoresCompletamente();
      Serial.println("OTA: Iniciando atualizacao...");
      if (display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        display.clearDisplay(); display.setTextSize(2); display.setCursor(0,20);
        display.println("OTA Update"); display.display();
      }
    });
    ArduinoOTA.onEnd([]() { Serial.println("\nOTA: Concluido!"); ESP.restart(); });
    ArduinoOTA.onError([](ota_error_t error) { Serial.printf("OTA Error[%u]\n", error); });
    ArduinoOTA.begin();
    Serial.println("OTA pronto.");
  } else {
    Serial.println("\nFalha ao conectar WiFi para OTA. OTA desabilitado.");
    if (display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        display.setCursor(0,50); display.print("OTA Falhou"); display.display();
    }
  }
  delay(1000); // Tempo para ver a mensagem do OTA no display
}

// ==========================================================================
// Implementações de Leitura de Sensores
// ==========================================================================
void lerTodosSensores() {
    lerSensoresDeDistancia();
    lerSensoresDeLinha();
}

void lerSensoresDeDistancia() {
  if (sensorDistDir.isRangeComplete()) distDir_mm = sensorDistDir.readRange(); else distDir_mm = 9999;
  if (sensorDistFrt.isRangeComplete()) distFrt_mm = sensorDistFrt.readRange(); else distFrt_mm = 9999;
  if (sensorDistEsq.isRangeComplete()) distEsq_mm = sensorDistEsq.readRange(); else distEsq_mm = 9999;
  // A biblioteca VL53L0X da Adafruit pode precisar de `sensor.rangingTest(&measure, false);`
  // e depois `measure.RangeMilliMeter` se `startContinuous()` não for usado ou
  // se você precisar do status da medição. Para simplificar com `startContinuous()`:
  // distDir_mm = sensorDistDir.readRangeContinuousMillimeters(); // Verifique a API exata
  // distFrt_mm = sensorDistFrt.readRangeContinuousMillimeters();
  // distEsq_mm = sensorDistEsq.readRangeContinuousMillimeters();
  // Para ser seguro, vou usar o rangingTest como no seu código teleguiado:
  VL53L0X_RangingMeasurementData_t measure;
  sensorDistDir.rangingTest(&measure, false); distDir_mm = (measure.RangeStatus != 4) ? measure.RangeMilliMeter : 9999;
  sensorDistFrt.rangingTest(&measure, false); distFrt_mm = (measure.RangeStatus != 4) ? measure.RangeMilliMeter : 9999;
  sensorDistEsq.rangingTest(&measure, false); distEsq_mm = (measure.RangeStatus != 4) ? measure.RangeMilliMeter : 9999;
}

void lerSensoresDeLinha() {
  // Lembre-se: Borda PRETA = sensor retorna HIGH (ou LOW, dependendo do sensor e circuito).
  // Assumindo que TCRT5000 com comparador: ALTO para branco, BAIXO para preto.
  // SEU CÓDIGO TELEGUIDADO: (digitalRead(SENSOR_LINHA_L1) == LOW) ? 'B' : 'P';
  // Isso significa que LOW = Branco, HIGH = PRETO (BORDA)
  bordaFD_detectada = (digitalRead(SENSOR_LINHA_FD) == HIGH);
  bordaFE_detectada = (digitalRead(SENSOR_LINHA_FE) == HIGH);
  bordaTR_detectada = (digitalRead(SENSOR_LINHA_TR) == HIGH);
}

// ==========================================================================
// Implementações de Lógica de Decisão
// ==========================================================================
bool inimigoDetectadoGeral() {
  // Considera qualquer um dos 3 sensores para detecção geral
  return (distFrt_mm < DISTANCIA_DETECCAO_INIMIGO_MM || 
          distDir_mm < DISTANCIA_DETECCAO_INIMIGO_MM || 
          distEsq_mm < DISTANCIA_DETECCAO_INIMIGO_MM);
}

bool inimigoMuitoProximoFrente() { // "Adversário a menos de 10 cm"
  return (distFrt_mm < DISTANCIA_INIMIGO_PROXIMO_MM);
}

bool bordaFrontalDetectada() {
  return (bordaFD_detectada || bordaFE_detectada);
}

bool bordaTraseiraDetectadaUnica() {
  return bordaTR_detectada;
}

// ==========================================================================
// Implementações de Controle dos Motores
// ==========================================================================
void moverMotoresPWM(int velEsq, int velDir) {
  // Normaliza velocidades para o range do PWM (0-255)
  velEsq = constrain(velEsq, -VELOCIDADE_MAXIMA, VELOCIDADE_MAXIMA);
  velDir = constrain(velDir, -VELOCIDADE_MAXIMA, VELOCIDADE_MAXIMA);

  // Motor Esquerdo (Motor B)
  if (velEsq > 0) { // Frente
    digitalWrite(MOTOR_B_IN3, HIGH); digitalWrite(MOTOR_B_IN4, LOW);
  } else if (velEsq < 0) { // Trás
    digitalWrite(MOTOR_B_IN3, LOW); digitalWrite(MOTOR_B_IN4, HIGH);
  } else { // Parado
    digitalWrite(MOTOR_B_IN3, LOW); digitalWrite(MOTOR_B_IN4, LOW);
  }
  ledcWrite(PWM_MOTOR_B_CANAL, abs(velEsq));

  // Motor Direito (Motor A)
  if (velDir > 0) { // Frente
    digitalWrite(MOTOR_A_IN1, LOW); digitalWrite(MOTOR_A_IN2, HIGH);
  } else if (velDir < 0) { // Trás
    digitalWrite(MOTOR_A_IN1, HIGH); digitalWrite(MOTOR_A_IN2, LOW);
  } else { // Parado
    digitalWrite(MOTOR_A_IN1, LOW); digitalWrite(MOTOR_A_IN2, LOW);
  }
  ledcWrite(PWM_MOTOR_A_CANAL, abs(velDir));
}

void pararMotoresCompletamente() {
  moverMotoresPWM(0, 0);
}

void moverFrentePadrao(int velocidade) {
  moverMotoresPWM(velocidade, velocidade);
}

void moverTrasPadrao(int velocidade) {
  moverMotoresPWM(-velocidade, -velocidade);
}

void girarNoEixoHorario(int velocidade) { // Robô gira para a direita
  moverMotoresPWM(velocidade, -velocidade); // Esquerdo frente, Direito trás
}

void girarNoEixoAntiHorario(int velocidade) { // Robô gira para a esquerda
  moverMotoresPWM(-velocidade, velocidade); // Esquerdo trás, Direito frente
}

void moverEmDiagonalFrenteDireita(int velPrincipal, int velAjuste) {
  // Para ir para frente-direita, motor esquerdo (B) mais rápido ou motor direito (A) mais lento.
  moverMotoresPWM(velPrincipal, velAjuste); 
}
void moverEmDiagonalFrenteEsquerda(int velPrincipal, int velAjuste) {
  // Para ir para frente-esquerda, motor direito (A) mais rápido ou motor esquerdo (B) mais lento.
  moverMotoresPWM(velAjuste, velPrincipal);
}

// ==========================================================================
// Implementações das Funções de Estado
// ==========================================================================
void executarEstadoConfigurando() {
    // Este estado é mais para o setup. No loop, ele já deve ter passado.
    // Se cair aqui, é um erro, então volta para aguardar.
    mudarParaEstado(AGUARDANDO_START, "Aguard.Start");
}

void executarEstadoAguardandoStart() {
  pararMotoresCompletamente(); // Garante que está parado

  // Ler chaves de modo (LOW = ativo devido ao INPUT_PULLUP)
  if (digitalRead(PINO_CHAVE_MODO_1) == LOW) modoSelecionado = MODO_PADRAO;
  else if (digitalRead(PINO_CHAVE_MODO_2) == LOW) modoSelecionado = MODO_TOURADA;
  else if (digitalRead(PINO_CHAVE_MODO_3) == LOW) modoSelecionado = MODO_DIAGONAL;
  else modoSelecionado = NENHUM_MODO;

  if (modoSelecionado != NENHUM_MODO && digitalRead(PINO_BOTAO_START) == LOW) {
    Serial.print("Modo selecionado: "); Serial.println(obterNomeModoOperacao(modoSelecionado));
    Serial.println("Botao Start pressionado!");
    mudarParaEstado(ESPERA_INICIAL_5S, "Espera 5s");
  }
}

void executarEstadoEsperaInicial5s() {
  pararMotoresCompletamente();
  if (millis() - timestampEntradaEstado >= 5000) {
    Serial.println("Contagem de 5s terminada. Iniciando modo de operacao!");
    switch (modoSelecionado) {
      case MODO_PADRAO:   mudarParaEstado(MP_ANDA_FRENTE, "MP AndaFrt"); break;
      case MODO_TOURADA:  mudarParaEstado(MT_RE, "MT Re"); break;
      case MODO_DIAGONAL: mudarParaEstado(MD_ANDA_DIAGONAL, "MD Diagonal"); break;
      default:
        Serial.println("Erro: Nenhum modo valido selecionado apos espera. Voltando.");
        mudarParaEstado(AGUARDANDO_START, "Aguard.Start");
        break;
    }
  }
}

// --- Funções de Estado: Modo Padrão ---
void executarEstadoMpAndaFrente() {
  moverFrentePadrao(VELOCIDADE_MAXIMA);

  if (bordaFrontalDetectada()) {
    mudarParaEstado(MP_GIRO_360, "MP Giro360");
    return;
  }
  // "Detectou Inimigo" na máquina de estados original mantém em "Anda Para Frente".
  // Se quiser uma lógica mais agressiva, pode adicionar aqui.
  if (inimigoDetectadoGeral()) {
      // Potencialmente aumentar velocidade ou confirmar engajamento
      // Serial.println("MP AndaFrente: Inimigo detectado, mantendo curso.");
  }
}

void executarEstadoMpGiro360() {
  girarNoEixoAntiHorario(VELOCIDADE_GIRO); // Gira para um lado (ex: anti-horário)

  // Condição "Detectou Borda" (durante o giro) -> continua em Giro 360
  // Isso é implícito, pois a ação é continuar girando.
  // A saída é por "Terminou a manobra" (tempo) E NÃO ESTAR MAIS NA BORDA.
  if (millis() - timestampEntradaEstado >= TEMPO_GIRO_360_COMPLETO_MS) {
    if (!bordaFrontalDetectada()) { // Só sai do giro se não estiver mais na borda
        pararMotoresCompletamente(); // Pequena pausa antes de procurar
        delay(100); 
        mudarParaEstado(MP_PROCURA, "MP Procura");
    } else {
        // Tempo do giro esgotou, mas ainda na borda. Pode reiniciar o timer ou mudar estratégia.
        Serial.println("MP Giro360: Tempo esgotado, mas ainda na borda. Reiniciando giro.");
        timestampEntradaEstado = millis(); // Tenta girar mais
    }
  }
}

void executarEstadoMpProcura() {
  girarNoEixoAntiHorario(VELOCIDADE_PROCURA); // Gira lentamente para procurar

  if (inimigoDetectadoGeral()) {
    mudarParaEstado(MP_ANDA_FRENTE, "MP AndaFrt (Atk)");
    return;
  }

  // "Não detectou inimigo" -> Anda Para Frente (para cobrir área)
  // Isso pode ser um timeout no estado de procura.
  if (millis() - timestampEntradaEstado >= TEMPO_PROCURA_MS) {
    Serial.println("MP Procura: Timeout, nao achou. Avancando para procurar.");
    mudarParaEstado(MP_ANDA_FRENTE, "MP AndaFrt (Busca)");
  }
}

// --- Funções de Estado: Modo Tourada ---
void executarEstadoMtRe() {
  moverTrasPadrao(VELOCIDADE_MEDIA);

  if (bordaTraseiraDetectadaUnica()) {
    pararMotoresCompletamente(); // Para imediatamente
    mudarParaEstado(MT_ESPERA_POS_BORDA, "MT Esp.Borda");
    return;
  }
  // Não há transição por detecção de inimigo neste estado na FSM original.
}

void executarEstadoMtEsperaPosBorda() {
  pararMotoresCompletamente(); // Garante que está parado

  // "Adversário a menos de 10 cm" -> Desvia
  if (inimigoMuitoProximoFrente()) {
    mudarParaEstado(MT_DESVIA, "MT Desvia");
    return;
  }
  
  // Timeout se o adversário não se aproximar
  if (millis() - timestampEntradaEstado >= TEMPO_ESPERA_TOURADA_MS) {
      Serial.println("MT Esp.Borda: Timeout, inimigo nao aproximou. Indo procurar.");
      mudarParaEstado(MP_PROCURA, "MP Procura (Tourada)"); // Reutiliza estado de procura
  }
}

void executarEstadoMtDesvia() {
  // Manobra de desvio: pode ser um giro rápido e um pequeno avanço.
  // Exemplo: gira 90 graus e avança.
  unsigned long tempoNoDesvio = millis() - timestampEntradaEstado;

  if (tempoNoDesvio < (TEMPO_MANOBRA_DESVIO_MS / 3)) { // Fase 1: Girar (ex: 1/3 do tempo)
    girarNoEixoHorario(VELOCIDADE_GIRO); 
  } else if (tempoNoDesvio < TEMPO_MANOBRA_DESVIO_MS) { // Fase 2: Avançar (restante do tempo)
    moverFrentePadrao(VELOCIDADE_MEDIA);
  } else { // "Terminou a manobra"
    mudarParaEstado(MP_PROCURA, "MP Procura (Tourada)"); // Reutiliza estado de procura
  }
}

// --- Funções de Estado: Modo Diagonal ---
void executarEstadoMdAndaDiagonal() {
  // Exemplo: Andar para frente-direita. Motor Esquerdo (B) mais rápido.
  moverEmDiagonalFrenteDireita(VELOCIDADE_MEDIA, VELOCIDADE_BAIXA);

  if (bordaFrontalDetectada()) {
    mudarParaEstado(MP_GIRO_360, "MP Giro360 (Diag)"); // Reutiliza
    return;
  }
  if (inimigoDetectadoGeral()) {
    // A FSM original diz "Detectou Inimigo -> Procura".
    // Isso pode ser para se alinhar antes de atacar.
    mudarParaEstado(MP_PROCURA, "MP Procura (Diag)"); // Reutiliza
    return;
  }
  // "Depois de 10 cm" (aproximado por tempo)
  if (millis() - timestampEntradaEstado >= TEMPO_ANDA_DIAGONAL_10CM_MS) {
    mudarParaEstado(MD_VIRA_PROCURA, "MD ViraProcura");
    return;
  }
}

void executarEstadoMdViraProcura() {
  // Ação de "Virar":
  unsigned long tempoNaVirada = millis() - timestampEntradaEstado;
  // Ex: virar 90 graus (1/4 de TEMPO_GIRO_360_COMPLETO_MS)
  if (tempoNaVirada < (TEMPO_GIRO_360_COMPLETO_MS / 4)) {
    girarNoEixoAntiHorario(VELOCIDADE_GIRO); // Vira para um lado
  } else {
    // Após "Virar", começa a "Procurar" (a ação de procurar é contínua neste estado até uma transição)
    girarNoEixoAntiHorario(VELOCIDADE_PROCURA); // Continua girando lentamente para procurar

    if (inimigoDetectadoGeral()) {
      mudarParaEstado(MD_ANDA_RETO_POS_VIRA, "MD AndaReto");
      return;
    }
    if (bordaFrontalDetectada()) {
      mudarParaEstado(MP_GIRO_360, "MP Giro360 (Diag)"); // Reutiliza
      return;
    }
    // Timeout para não ficar preso procurando indefinidamente
    if (tempoNaVirada > (TEMPO_GIRO_360_COMPLETO_MS / 4) + TEMPO_PROCURA_MS) {
        Serial.println("MD ViraProcura: Timeout, voltando a andar diagonal.");
        mudarParaEstado(MD_ANDA_DIAGONAL, "MD Diagonal (Repete)");
    }
  }
}

void executarEstadoMdAndaRetoPosVira() {
  moverFrentePadrao(VELOCIDADE_MAXIMA); // Ataque frontal

  if (bordaFrontalDetectada()) {
    mudarParaEstado(MP_GIRO_360, "MP Giro360 (Diag)"); // Reutiliza
    return;
  }
  // "Não detectou inimigo" (perdeu o alvo) -> Procura
  if (!inimigoDetectadoGeral()) { 
    Serial.println("MD AndaReto: Perdeu inimigo. Indo procurar.");
    mudarParaEstado(MP_PROCURA, "MP Procura (Diag)"); // Reutiliza
    return;
  }
}


// ==========================================================================
// Implementações de Utilitários e Display
// ==========================================================================
void mudarParaEstado(EstadoRobo novoEstado, const String& nomeNovoEstado) {
  if (estadoAtual != novoEstado || nomeEstadoAtual != nomeNovoEstado) { // Evita logs repetidos se chamado sem mudança real
    Serial.print("Mudando estado: "); Serial.print(nomeEstadoAtual);
    Serial.print(" -> "); Serial.println(nomeNovoEstado);
    
    estadoAtual = novoEstado;
    nomeEstadoAtual = nomeNovoEstado;
    timestampEntradaEstado = millis(); // Marca o tempo de entrada no novo estado

    // Ações imediatas ao entrar em alguns estados (ex: parar motores antes de uma espera)
    // Isso pode ser útil para garantir um comportamento limpo entre transições.
    // No entanto, a maioria das ações de "entrada" pode estar no início da função do próprio estado.
  }
}

String obterNomeModoOperacao(ModoOperacao modo) {
  switch (modo) {
    case MODO_PADRAO:   return "Padrao";
    case MODO_TOURADA:  return "Tourada";
    case MODO_DIAGONAL: return "Diagonal";
    default:            return "Nenhum";
  }
}

void atualizarDisplayAutonomoInfo() {
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) return; // Se o display não iniciou

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // Linha 1: Modo e Estado
  display.setCursor(0, 0);
  display.print(obterNomeModoOperacao(modoSelecionado));
  display.setCursor(60,0); // Coluna para estado
  display.print("E:"); display.print(nomeEstadoAtual);

  // Linha 2: Sensores de Distância (Dir, Frt, Esq)
  display.setCursor(0, 10);
  display.printf("D:%3d F:%3d E:%3d", 
                (distDir_mm > 999 ? 999 : distDir_mm), 
                (distFrt_mm > 999 ? 999 : distFrt_mm), 
                (distEsq_mm > 999 ? 999 : distEsq_mm));

  // Linha 3: Sensores de Linha (FE, TR, FD) e Tempo no Estado
  display.setCursor(0, 20);
  display.print("L:");
  display.print(bordaFE_detectada ? "P" : "B"); // Frontal Esquerda
  display.print(bordaTR_detectada ? "P" : "B"); // Traseira
  display.print(bordaFD_detectada ? "P" : "B"); // Frontal Direita
  
  display.setCursor(50, 20); // Coluna para tempo
  if (estadoAtual == ESPERA_INICIAL_5S) {
    long tempoRestante = 5000 - (millis() - timestampEntradaEstado);
    display.printf("T-%lds", (tempoRestante < 0 ? 0 : tempoRestante / 1000));
  } else {
    display.printf("t:%lus", (millis() - timestampEntradaEstado) / 1000);
  }
  
  // Linha 4: IP (se conectado para OTA) ou outra info
  display.setCursor(0, 30);
   if (WiFi.status() == WL_CONNECTED) {
    display.print(WiFi.localIP());
  } else if (String(WIFI_SSID_OTA) != "SUA_REDE_WIFI") { // Só mostra se tentou conectar
    display.print("OTA Offline");
  } else {
    display.print("SumoBot Auto :)");
  }

  display.display();
}