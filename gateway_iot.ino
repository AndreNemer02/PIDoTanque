#define BLYNK_TEMPLATE_ID   "SEU_ID_AQUI"
#define BLYNK_TEMPLATE_NAME "Monitoramento Tanque"
#define BLYNK_AUTH_TOKEN    "SEU_TOKEN_AQUI"

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>

char ssid[] = "SEU_WIFI";
char pass[] = "SUA_SENHA";

// Variáveis para receber os dados do Bare Metal
float nivel, vazao, u_pid;
int setpoint;

void setup() {
  // Serial2 nos pinos 16 (RX2) e 17 (TX2) para falar com o Arduino
  Serial2.begin(9600, SERIAL_8N1, 16, 17); 
  
  // Conecta ao Wi-Fi e ao Blynk Cloud
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
}

void loop() {
  Blynk.run();

  // Se houver dados chegando do Arduino
  if (Serial2.available()) {
    String linha = Serial2.readStringUntil('\n');
    
    // Decompõe a string: "N:15.5|V:10.2|SP:30|U:45.0"
    if (sscanf(linha.c_str(), "N:%f|V:%f|SP:%d|U:%f", &nivel, &vazao, &setpoint, &u_pid) == 4) {
      // Envia os dados para os pinos virtuais do App
      Blynk.virtualWrite(V1, nivel);
      Blynk.virtualWrite(V2, vazao);
      Blynk.virtualWrite(V3, setpoint);
      Blynk.virtualWrite(V4, u_pid);
    }
  }
}
