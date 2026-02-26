/**
 * @brief Comunicação entre ESP32 e plataforma Blynk para monitoramento
 *        e controle remoto de um sistema PID de nível e vazão.
 *
 * @details O ESP32 recebe dados do ATmega via UART (Serial2) no formato:
 *          "vazao,nivel,erro,kp_erro,ki_integral,kd_derivada\n"
 *          e os envia para o dashboard Blynk via Wi-Fi.
 *          Os parâmetros Kp, Ki e Kd podem ser ajustados remotamente
 *          pelo dashboard, sendo repassados ao ATmega via Serial2.
 *
 * @note Formato da string recebida do ATmega:
 *       vazao,nivel,erro,kp*erro,ki*integral,kd*derivada
 *       Exemplo: "1.5,23.4,0.8,0.24,0.05,0.03\n"
 *
 * @pinout
 *   GPIO 16 (RX2) -> TX do ATmega (via divisor de tensão 1kΩ/2.2kΩ)
 *   GPIO 17 (TX2) -> RX do ATmega (ligação direta)
 *   GND           -> GND do ATmega (obrigatório para referência comum)
 */


#define BLYNK_TEMPLATE_ID "TMPL2I0JRaVT5"
#define BLYNK_TEMPLATE_NAME "TANQUE"
#define BLYNK_AUTH_TOKEN "6yfjOG9DAbN7_s61uTdV0vbPXVttqeBF"

#include <BlynkSimpleEsp32.h>

/**
 * @brief Nome da rede Wi-Fi (SSID).
 */
char ssid[] = "!!!!!!!!!!!";

/**
 * @brief Senha da rede Wi-Fi.
 */
char pass[] = "!!!!!!!!!!";


//  Mapeamento dos pinos virtuais Blynk

/**
 * @defgroup VirtualPins Pinos Virtuais Blynk
 * @{
 * V0   Vazão (L/min)         somente leitura
 * V1   Nível (cm)            somente leitura
 * V2   Kp                    leitura e escrita (ajuste remoto)
 * V3   Ki                    leitura e escrita (ajuste remoto)
 * V4   Kd                    leitura e escrita (ajuste remoto)
 * V5   Erro                  somente leitura
 * V6   Kp × erro             somente leitura
 * V7   Ki × integral(erro)   somente leitura
 * V8   Kd × derivada(erro)   somente leitura
 * @}
 */


//  Variáveis globais

float vazao = 0;
float nivel = 0;
float erro = 0;
float kp = 1.0;
float ki = 0.1;
float kd = 0.05;


//  Setup

/**
 * @brief Inicializa as comunicações seriais e conecta ao Blynk.
 *
 * @details Serial  (USB)   115200 baud - monitor serial para debug
 *          Serial2 (UART2)  9600 baud   - comunicação com ATmega
 *          Blynk           conecta ao servidor usando Wi-Fi e Auth Token
 */
void setup() {
  Serial.begin(115200);
  Serial.println("Iniciando ESP32...");

  // RX=GPIO16, TX=GPIO17
  Serial2.begin(9600, SERIAL_8N1, 16, 17);
  Serial.println("Serial2 (ATmega) iniciada.");

  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  Serial.println("Conectado ao Blynk!");
}


//  Loop principal

/**
 * @brief Loop principal: mantém conexão Blynk e processa dados do ATmega.
 *
 * @details Aguarda uma linha completa da Serial2 no formato:
 *          "vazao,nivel,erro,kp_erro,ki_integral,kd_derivada\n"
 *          Faz o parse dos valores e os envia aos pinos virtuais do Blynk.
 *
 * @example String recebida: "1.5,23.4,0.8,0.24,0.05,0.03\n"
 *          vazao=1.5, nivel=23.4, erro=0.8,
 *          kp_e=0.24, ki_i=0.05, kd_d=0.03
 */
void loop() {
  Blynk.run(); // Mantém a conexão com o servidor Blynk ativa

  if (Serial2.available()) {
    // Lê uma linha completa até encontrar '\n'
    String dados = Serial2.readStringUntil('\n');
    dados.trim(); // Remove espaços e '\r' extras

    // Encontra as posições das vírgulas para separar os valores
    int i0 = dados.indexOf(',');
    int i1 = dados.indexOf(',', i0 + 1);
    int i2 = dados.indexOf(',', i1 + 1);
    int i3 = dados.indexOf(',', i2 + 1);
    int i4 = dados.indexOf(',', i3 + 1);

    // Extrai e converte cada valor para float
    vazao        = dados.substring(0, i0).toFloat();
    nivel        = dados.substring(i0 + 1, i1).toFloat();
    erro         = dados.substring(i1 + 1, i2).toFloat();
    float kp_e   = dados.substring(i2 + 1, i3).toFloat(); ///< Kp × erro
    float ki_i   = dados.substring(i3 + 1, i4).toFloat(); ///< Ki × integral(erro)
    float kd_d   = dados.substring(i4 + 1).toFloat();     ///< Kd × derivada(erro)

    // Envia os valores para o dashboard Blynk
    Blynk.virtualWrite(V0, vazao);  // Rótulo: Vazão
    Blynk.virtualWrite(V1, nivel);  // Rótulo: Nível
    Blynk.virtualWrite(V2, kp);     // Rótulo: Kp atual
    Blynk.virtualWrite(V3, ki);     // Rótulo: Ki atual
    Blynk.virtualWrite(V4, kd);     // Rótulo: Kd atual
    Blynk.virtualWrite(V5, erro);   // Rótulo: Erro
    Blynk.virtualWrite(V6, kp_e);   // Rótulo: Kp × erro
    Blynk.virtualWrite(V7, ki_i);   // Rótulo: Ki × integral
    Blynk.virtualWrite(V8, kd_d);   // Rótulo: Kd × derivada
  }
}


//  Callbacks Blynk (escrita remota)

/**
 * @brief Callback chamado quando Kp é alterado pelo dashboard.
 * @param param Novo valor de Kp enviado pelo Blynk.
 * @note O valor é repassado ao ATmega via Serial2 no formato "KP:valor\n"
 */
BLYNK_WRITE(V2) {
  kp = param.asFloat();
  Serial2.print("KP:"); Serial2.println(kp);
  Serial.print("Kp atualizado: "); Serial.println(kp);
}

/**
 * @brief Callback chamado quando Ki é alterado pelo dashboard.
 * @param param Novo valor de Ki enviado pelo Blynk.
 * @note O valor é repassado ao ATmega via Serial2 no formato "KI:valor\n"
 */
BLYNK_WRITE(V3) {
  ki = param.asFloat();
  Serial2.print("KI:"); Serial2.println(ki);
  Serial.print("Ki atualizado: "); Serial.println(ki);
}

/**
 * @brief Callback chamado quando Kd é alterado pelo dashboard.
 * @param param Novo valor de Kd enviado pelo Blynk.
 * @note O valor é repassado ao ATmega via Serial2 no formato "KD:valor\n"
 */
BLYNK_WRITE(V4) {
  kd = param.asFloat();
  Serial2.print("KD:"); Serial2.println(kd);
  Serial.print("Kd atualizado: "); Serial.println(kd);
}
