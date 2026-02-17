#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <stdio.h>

/* * ============================================================================
 * CONFIGURAÇÕES DE SISTEMA E TIMING
 * Aqui definimos a base de tempo para as funções de delay e o cálculo do 
 * Baud Rate da Serial. O UBRR é o registrador que dita a velocidade da USART.
 * ============================================================================
 */
#define F_CPU 16000000UL
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1

/* * ============================================================================
 * MAPEAMENTO DE PINOS (HARDWARE)
 * Definições baseadas nos portos B e D do ATmega328P. 
 * Requisito 6 (Bomba) no pino D13 e Requisito 7 (Servo) no pino D10.
 * ============================================================================
 */
#define LCD_RS PB0      // Digital 8
#define LCD_E  PB1      // Digital 9
#define TRIG_PIN PD7    // Digital 7
#define ECHO_PIN PD6    // Digital 6
#define SERVO_PIN PB2   // Digital 10 (Timer1 OC1B)
#define PUMP_PIN PB5    // Digital 13
#define FLOW_PIN PD3    // Digital 3 (Interrupção INT1)

/* * ============================================================================
 * VARIÁVEIS DE ESTADO E CONTROLE
 * Usamos 'volatile' para variáveis acessadas dentro de interrupções (ISRs).
 * As constantes Kp, Ki e Kd definem o comportamento do seu controlador.
 * ============================================================================
 */
volatile uint8_t flag_pid = 0;
volatile uint32_t pulsos_vazao = 0;
volatile int16_t setpoint_vazao = 30; 

// Variáveis do Algoritmo PID
float Kp = 12.5, Ki = 0.8, Kd = 1.5;
float erro_anterior = 0, integral = 0;
float vazao_real = 0, nivel_real = 0;

/* * ============================================================================
 * DRIVER DO LCD (MODO 4-BITS)
 * Estas funções implementam o protocolo manual para displays HD44780.
 * Enviamos o byte em duas partes (nibbles) para economizar fios no microcontrolador.
 * ============================================================================
 */
void lcd_send_4bits(uint8_t valor) {
    // Máscara para garantir que só os pinos de dados (PD2-PD5) sejam alterados
    PORTD = (PORTD & 0xC3) | ((valor >> 2) & 0x3C);
    PORTB |= (1 << LCD_E); 
    _delay_us(1); 
    PORTB &= ~(1 << LCD_E);
    _delay_us(100);
}

void lcd_command(uint8_t cmd) {
    PORTB &= ~(1 << LCD_RS); // RS=0 para instruções
    lcd_send_4bits(cmd);     
    lcd_send_4bits(cmd << 4);
    _delay_ms(2);
}

void lcd_data(uint8_t d) {
    PORTB |= (1 << LCD_RS);  // RS=1 para caracteres
    lcd_send_4bits(d); 
    lcd_send_4bits(d << 4);
    _delay_us(100);
}

void lcd_init() {
    DDRB |= (1 << LCD_RS) | (1 << LCD_E);
    DDRD |= 0x3C; // Configura pinos de dados como saída
    _delay_ms(50);
    // Sequência de inicialização para habilitar modo 4 bits
    lcd_send_4bits(0x30); _delay_ms(5);
    lcd_send_4bits(0x30); _delay_us(150);
    lcd_send_4bits(0x30); lcd_send_4bits(0x20);
    lcd_command(0x28); // 2 linhas, matriz 5x7
    lcd_command(0x0C); // Display ON, Cursor OFF
    lcd_command(0x01); // Limpa a tela
}

/* * ============================================================================
 * COMUNICAÇÃO USART (UART)
 * Configuração dos registradores para telemetria serial com o ESP32.
 * Habilita a transmissão (TX) e recepção (RX) de frames de 8 bits.
 * ============================================================================
 */
void USART_Init(unsigned int ubrr) {
    UBRR0H = (unsigned char)(ubrr >> 8); 
    UBRR0L = (unsigned char)ubrr;
    UCSR0B = (1 << TXEN0) | (1 << RXEN0);
    UCSR0C = (3 << UCSZ00); // 8-bit data, 1 stop bit
}

void USART_Print(char* str) {
    while (*str) {
        while (!(UCSR0A & (1 << UDRE0))); // Espera buffer de envio esvaziar
        UDR0 = *str++;
    }
}

/* * ============================================================================
 * ROTINAS DE INTERRUPÇÃO (ISRs)
 * INT1: Detecta pulsos do sensor de vazão para o cálculo de fluxo.
 * PCINT0: Lê as variações do Encoder Rotativo (Requisito 9).
 * TIMER1: Garante o tick de 100ms para o cálculo determinístico do PID.
 * ============================================================================
 */
ISR(INT1_vect) { pulsos_vazao++; } // Sensor de Vazão

ISR(PCINT0_vect) { // Encoder em D11/D12
    static uint8_t last_clk = 0;
    uint8_t cur_clk = (PINB & (1 << PB3));
    if (cur_clk != last_clk && cur_clk) {
        if (PINB & (1 << PB4)) setpoint_vazao++; else setpoint_vazao--;
    }
    last_clk = cur_clk;
}

ISR(TIMER1_COMPA_vect) { // Gerador de base de tempo
    static uint8_t count = 0;
    if (++count >= 50) { flag_pid = 1; count = 0; } // 2ms * 50 = 100ms
}

/* * ============================================================================
 * SENSORES E PERIFÉRICOS
 * Medir_Nivel: Controla o Trig/Echo do sensor ultrassônico via registradores.
 * Timer1_Init: Configura o PWM de 50Hz (20ms) para o controle do Servomotor.
 * ============================================================================
 */
float Medir_Nivel() {
    PORTD |= (1 << TRIG_PIN); _delay_us(10); PORTD &= ~(1 << TRIG_PIN);
    uint32_t c = 0; 
    while(!(PIND & (1 << ECHO_PIN)) && c < 30000) c++; // Timeout simples
    c = 0; 
    while((PIND & (1 << ECHO_PIN)) && c < 30000) { c++; _delay_us(1); }
    return (c * 0.034) / 2; // Distância em centímetros
}

void Timer1_Init() {
    // Fast PWM com ICR1 como TOP para definir a frequência exata (50Hz)
    TCCR1A = (1 << COM1B1) | (1 << WGM11);
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11); // Prescaler 8
    ICR1 = 39999;  // Período de 20ms
    OCR1B = 3000;  // Posição inicial do Servo
    TIMSK1 = (1 << OCIE1A); // Habilita interrupção de tempo do sistema
    OCR1A = 3999;           // Tick de 2ms
}

/* * ============================================================================
 * LOOP PRINCIPAL E CONTROLADOR PID
 * Aqui o sistema processa as informações a cada 100ms.
 * Implementa a equação: $$u(k) = K_p e(k) + K_i \sum e(k) \Delta t + K_d \frac{\Delta e(k)}{\Delta t}$$
 * ============================================================================
 */
int main(void) {
    // Configuração de Direção de Dados (DDR)
    DDRD |= (1 << TRIG_PIN); 
    DDRB |= (1 << PUMP_PIN) | (1 << SERVO_PIN);
    DDRD &= ~(1 << ECHO_PIN) & ~(1 << FLOW_PIN);
    
    // Configuração de Interrupções
    EICRA |= (1 << ISC10); EIMSK |= (1 << INT1); // Sensor de Vazão
    PCICR |= (1 << PCIE0); PCMSK0 |= (1 << PCINT3) | (1 << PCINT4); // Encoder
    
    USART_Init(MYUBRR); lcd_init(); Timer1_Init();
    sei(); // Habilita interrupções globais

    char b1[17], b2[17], sn[10], sv[10], tele[64];

    while(1) {
        if(flag_pid) { // Execução determinística a cada 100ms
            // 1. Aquisição de Sinais
            nivel_real = Medir_Nivel();
            vazao_real = (pulsos_vazao * 60.0) / 7.5; // Conversão Hz para L/min
            pulsos_vazao = 0;

            // 2. Cálculo do PID (Vazão -> Servo)
            float erro = (float)setpoint_vazao - vazao_real;
            
            // Termo Integral com Anti-Windup (evita saturação infinita)
            integral += erro * 0.1; 
            if (integral > 1000) integral = 1000; else if (integral < -1000) integral = -1000;
            
            // Termo Derivativo (Taxa de variação do erro)
            float derivada = (erro - erro_anterior) / 0.1;
            
            // Saída do Controlador PID
            float u = (Kp * erro) + (Ki * integral) + (Kd * derivada);
            erro_anterior = erro;

            // Atuação no Servo (Mapeamento de 1ms a 2ms)
            int16_t out_servo = 3000 + (int16_t)u;
            if(out_servo < 2000) out_servo = 2000; if(out_servo > 4000) out_servo = 4000;
            OCR1B = out_servo;

            // 3. Controle do Nível (Ultrassônico -> Bomba)
            // Histerese simples para proteger o MOSFET e a Bomba
            if(nivel_real > 15.0) PORTB |= (1 << PUMP_PIN); 
            else if(nivel_real < 5.0) PORTB &= ~(1 << PUMP_PIN);

            // 4. Interface LCD e Telemetria ESP32
            dtostrf(nivel_real, 4, 1, sn); 
            dtostrf(vazao_real, 4, 1, sv);
            
            lcd_command(0x80); sprintf(b1, "N:%s V:%s ", sn, sv);
            for(int i=0; b1[i]; i++) lcd_data(b1[i]);
            
            lcd_command(0xC0); sprintf(b2, "SP:%d  Err:%.1f", setpoint_vazao, erro);
            for(int i=0; b2[i]; i++) lcd_data(b2[i]);

            // Envia pacote formatado para o Blynk no ESP32
            sprintf(tele, "N:%s|V:%s|SP:%d|U:%.1f\n", sn, sv, setpoint_vazao, u);
            USART_Print(tele);
            
            flag_pid = 0; // Finaliza o ciclo de amostragem
        }
    }
}
