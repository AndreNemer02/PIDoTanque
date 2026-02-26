#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define F_CPU 16000000UL
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1
#define LCD_ADDR 0x27

/* --- MAPEAMENTO DE PINOS RIGOROSO --- */
#define TRIG_PIN PD7    // D7
#define ECHO_PIN PD6    // D6
#define FLOW_PIN PD3    // D3 (Interrupção INT1)
#define SERVO_PIN PB2   // D10 (PWM OC1B)
#define PUMP_PIN PB5    // D13 (Bomba PNP TIP30C)
#define ENC_CLK PB3     // D11
#define ENC_DT  PB4     // D12
#define ENC_SW  PD4     // D4 (Botão do Encoder)

// Variáveis Globais
volatile float Kp = 1.0, Ki = 0.1, Kd = 0.05; // Iniciando com valores do ESP32
volatile float erro = 0; 
volatile uint8_t flag_pid = 0;
volatile uint32_t pulsos_vazao = 0;
float erro_anterior = 0, integral = 0;
float vazao_real = 0, nivel_real = 0;
int16_t setpoint_vazao = 30;
uint8_t param_sel = 0; 

/* --- DRIVER I2C BARE METAL --- */
void I2C_init() { TWSR = 0x00; TWBR = 72; }
void I2C_start() { TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN); while (!(TWCR & (1<<TWINT))); }
void I2C_write(uint8_t data) { TWDR = data; TWCR = (1<<TWINT)|(1<<TWEN); while (!(TWCR & (1<<TWINT))); }
void I2C_stop() { TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO); _delay_us(50); }
void PCF_write(uint8_t data) { I2C_start(); I2C_write(LCD_ADDR << 1); I2C_write(data); I2C_stop(); }

/* --- SEU DRIVER LCD ORIGINAL --- */
void lcd_nibble(uint8_t nibble, uint8_t rs) {
    uint8_t data = (nibble << 4) | (1<<3);
    if (rs) data |= (1<<0);
    PCF_write(data | (1<<2)); _delay_us(1);
    PCF_write(data & ~(1<<2)); _delay_us(50);
}
void lcd_cmd(uint8_t cmd) { lcd_nibble(cmd >> 4, 0); lcd_nibble(cmd & 0x0F, 0); _delay_ms(2); }
void lcd_char(char c) { lcd_nibble((c >> 4) & 0x0F, 1); lcd_nibble(c & 0x0F, 1); }
void lcd_print(const char* s) { while (*s) lcd_char(*s++); }
void lcd_cursor(uint8_t row, uint8_t col) { lcd_cmd(0x80 | ((row ? 0x40 : 0x00) + col)); }

void lcd_init() {
    _delay_ms(50); lcd_nibble(0x03, 0); _delay_ms(5); lcd_nibble(0x03, 0); _delay_ms(1);
    lcd_nibble(0x03, 0); _delay_ms(1); lcd_nibble(0x02, 0); _delay_ms(1);
    lcd_cmd(0x28); lcd_cmd(0x0C); lcd_cmd(0x06); lcd_cmd(0x01);
}

/* --- SUAS FUNÇÕES ORIGINAIS: FTOA E ATUALIZA_LCD --- */
void ftoa(float val, char* buf) {
    int inteiro = (int)val;
    int decimal = (int)((val - inteiro) * 10);
    if (decimal < 0) decimal = -decimal;
    sprintf(buf, "%d.%d", inteiro, decimal);
}

void atualiza_lcd() {
    char buf[8], linha[17];
    lcd_cursor(0, 0);
    ftoa(erro, buf);
    lcd_print("Erro:"); lcd_print(buf); lcd_print("          ");
    lcd_cursor(1, 0);
    if (param_sel == 0) { ftoa(Kp, buf); sprintf(linha, "* Kp: %s        ", buf); }
    else if (param_sel == 1) { ftoa(Ki, buf); sprintf(linha, "* Ki: %s        ", buf); }
    else { ftoa(Kd, buf); sprintf(linha, "* Kd: %s        ", buf); }
    linha[16] = '\0'; lcd_print(linha);
}

/* --- USART (COMUNICAÇÃO COM ESP32) --- */
void USART_Init(unsigned int ubrr) {
    UBRR0H = (unsigned char)(ubrr >> 8); UBRR0L = (unsigned char)ubrr;
    UCSR0B = (1 << TXEN0) | (1 << RXEN0) | (1 << RXCIE0); // Habilita RX, TX e Interrupção de Receção
    UCSR0C = (3 << UCSZ00);
}
void USART_Print(char* str) { while (*str) { while (!(UCSR0A & (1 << UDRE0))); UDR0 = *str++; } }

/* --- INTERRUPÇÕES --- */
ISR(INT1_vect) { pulsos_vazao++; } // Sensor de Vazão no PD3

ISR(USART_RX_vect) { // Recebe ajustes de Kp, Ki, Kd vindos do Blynk/ESP32
    static char rx_buf[20]; static uint8_t i = 0;
    char c = UDR0;
    if (c == '\n') {
        rx_buf[i] = '\0'; i = 0;
        if (strncmp(rx_buf, "KP:", 3) == 0) Kp = atof(&rx_buf[3]);
        else if (strncmp(rx_buf, "KI:", 3) == 0) Ki = atof(&rx_buf[3]);
        else if (strncmp(rx_buf, "KD:", 3) == 0) Kd = atof(&rx_buf[3]);
        atualiza_lcd();
    } else if (i < 19) rx_buf[i++] = c;
}

ISR(TIMER1_COMPA_vect) { // Tick de 100ms para o PID
    static uint8_t count = 0;
    if (++count >= 50) { flag_pid = 1; count = 0; }
}

/* --- HARDWARE E SENSORES --- */
float Medir_Nivel() {
    PORTD |= (1 << TRIG_PIN); _delay_us(10); PORTD &= ~(1 << TRIG_PIN);
    uint32_t c = 0; while(!(PIND & (1 << ECHO_PIN)) && c < 30000) c++;
    c = 0; while((PIND & (1 << ECHO_PIN)) && c < 30000) { c++; _delay_us(1); }
    return (c * 0.034) / 2;
}

void Timer1_Init() {
    TCCR1A = (1 << COM1B1) | (1 << WGM11); 
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);
    ICR1 = 39999; OCR1B = 3000;
    TIMSK1 = (1 << OCIE1A); OCR1A = 3999; 
}

/* --- LOOP PRINCIPAL --- */
int main(void) {
    // Configuração de Direção (DDR) conforme sua tabela
    DDRD |= (1 << TRIG_PIN); 
    DDRB |= (1 << PUMP_PIN) | (1 << SERVO_PIN);
    DDRD &= ~((1 << ECHO_PIN) | (1 << FLOW_PIN) | (1 << ENC_SW));
    DDRB &= ~((1 << ENC_CLK) | (1 << ENC_DT));

    PORTB |= (1 << PUMP_PIN); // Inicia Bomba DESLIGADA (Lógica PNP: 1=OFF)
    PORTD |= (1 << ENC_SW);   // Pull-up no botão do encoder

    EICRA |= (1 << ISC11); EIMSK |= (1 << INT1); // Vazão no PD3
    USART_Init(MYUBRR); I2C_init(); lcd_init(); Timer1_Init(); sei();

    uint8_t last_clk = (PINB & (1 << ENC_CLK));
    uint8_t last_sw = (PIND & (1 << ENC_SW));

    atualiza_lcd();

    while(1) {
        // Leitura Encoder (Polling rápido no Porto B)
        uint8_t clk = (PINB & (1 << ENC_CLK));
        if (clk != last_clk && clk == 0) {
            _delay_ms(5);
            float sinal = (PINB & (1 << ENC_DT)) ? 1.0 : -1.0;
            if (param_sel == 0) Kp += sinal * 0.1;
            else if (param_sel == 1) Ki += sinal * 0.1;
            else Kd += sinal * 0.1;
            if (Kp < 0) Kp = 0; if (Ki < 0) Ki = 0; if (Kd < 0) Kd = 0;
            atualiza_lcd();
        }
        last_clk = clk;

        uint8_t sw = (PIND & (1 << ENC_SW));
        if (!sw && last_sw) {
            _delay_ms(20);
            param_sel = (param_sel + 1) % 3;
            atualiza_lcd();
        }
        last_sw = sw;

        if(flag_pid) { // Executa a cada 100ms
            nivel_real = Medir_Nivel();
            vazao_real = (pulsos_vazao * 60.0) / 7.5; pulsos_vazao = 0;
            
            // Segurança: Bomba PNP (0V = LIGADO, 5V = DESLIGADO)
            if(nivel_real > 15.0) PORTB &= ~(1 << PUMP_PIN); // LIGA
            else if(nivel_real < 5.0) PORTB |= (1 << PUMP_PIN); // DESLIGA

            // Algoritmo PID
            erro = (float)setpoint_vazao - vazao_real; // Valor para sua interface
            integral += erro * 0.1;
            if(integral > 100) integral = 100; else if(integral < -100) integral = -100;
            
            float p_term = Kp * erro;
            float i_term = Ki * integral;
            float d_term = Kd * (erro - erro_anterior) / 0.1;
            float u = p_term + i_term + d_term;
            erro_anterior = erro;

            // Saída para o Servo
            int16_t pulso = 3000 + (int16_t)u;
            if(pulso < 2000) pulso = 2000; if(pulso > 4000) pulso = 4000;
            OCR1B = pulso;

            atualiza_lcd();

            // TELEMETRIA: Envia exatamente o que o seu ESP32 espera
            char tele[80];
            sprintf(tele, "%.1f,%.1f,%.1f,%.1f,%.1f,%.1f\n", 
                    vazao_real, nivel_real, erro, p_term, i_term, d_term);
            USART_Print(tele);

            flag_pid = 0;
        }
    }
} 
--------------------------------------------------------------------------------------------------------------------------------------------------------

/* * PROJETO TANQUE UFES - VERSÃO PURISTA ABSOLUTA 
 * Sem #include <...>
 */

typedef unsigned char  uint8_t;
typedef unsigned int   uint16_t;
typedef unsigned long  uint32_t;
typedef signed int     int16_t;

/* --- DEFINIÇÃO MANUAL DE ENDEREÇOS DE REGISTRADORES --- */
#define DDRB   (*(volatile uint8_t *)(0x24))
#define PORTB  (*(volatile uint8_t *)(0x25))
#define PINB   (*(volatile uint8_t *)(0x23))

#define DDRC   (*(volatile uint8_t *)(0x27))
#define PORTC  (*(volatile uint8_t *)(0x28))
#define PINC   (*(volatile uint8_t *)(0x26))

#define DDRD   (*(volatile uint8_t *)(0x2A))
#define PORTD  (*(volatile uint8_t *)(0x2B))
#define PIND   (*(volatile uint8_t *)(0x29))

// TWI (I2C)
#define TWBR   (*(volatile uint8_t *)(0xB8))
#define TWSR   (*(volatile uint8_t *)(0xB9))
#define TWDR   (*(volatile uint8_t *)(0xBB))
#define TWCR   (*(volatile uint8_t *)(0xBC))

// USART
#define UBRR0H (*(volatile uint8_t *)(0xC5))
#define UBRR0L (*(volatile uint8_t *)(0xC4))
#define UCSR0A (*(volatile uint8_t *)(0xC0))
#define UCSR0B (*(volatile uint8_t *)(0xC1))
#define UCSR0C (*(volatile uint8_t *)(0xC2))
#define UDR0   (*(volatile uint8_t *)(0xC6))

// Timer 1 (16-bit)
#define TCCR1A (*(volatile uint8_t *)(0x80))
#define TCCR1B (*(volatile uint8_t *)(0x81))
#define ICR1   (*(volatile uint16_t *)(0x86))
#define OCR1A  (*(volatile uint16_t *)(0x88))
#define OCR1B  (*(volatile uint16_t *)(0x8A))
#define TIMSK1 (*(volatile uint8_t *)(0x6E))

// Interrupções e Status
#define EICRA  (*(volatile uint8_t *)(0x69))
#define EIMSK  (*(volatile uint8_t *)(0x3D))
#define SREG   (*(volatile uint8_t *)(0x5F))

/* --- FUNÇÕES DE ATRASO MANUAIS (CALIBRADAS PARA 16MHz) --- */
void delay_us(uint16_t us) {
    while (us--) {
        __asm__ volatile ("nop\n\tnop\n\tnop\n\tnop\n\t"); 
    }
}

void delay_ms(uint16_t ms) {
    while (ms--) {
        delay_us(1000);
    }
}

/* --- DRIVER I2C E LCD (SUA INTERFACE) --- */
void i2c_start() {
    TWCR = (1<<7)|(1<<5)|(1<<2); // TWINT, TWSTA, TWEN
    while (!(TWCR & (1<<7)));
}

void i2c_write(uint8_t data) {
    TWDR = data;
    TWCR = (1<<7)|(1<<2); // TWINT, TWEN
    while (!(TWCR & (1<<7)));
}

void pcf_write(uint8_t data) {
    i2c_start();
    i2c_write(0x27 << 1); // Endereço LCD
    i2c_write(data);
    TWCR = (1<<7)|(1<<4)|(1<<2); // TWSTO (STOP)
    delay_us(50);
}

void lcd_nibble(uint8_t nibble, uint8_t rs) {
    uint8_t data = (nibble << 4) | (1<<3); // Backlight ON
    if (rs) data |= (1<<0);
    pcf_write(data | (1<<2)); // EN ON
    delay_us(1);
    pcf_write(data & ~(1<<2)); // EN OFF
    delay_us(50);
}

void lcd_cmd(uint8_t cmd) {
    lcd_nibble(cmd >> 4, 0);
    lcd_nibble(cmd & 0x0F, 0);
    delay_ms(2);
}

void lcd_char(char c) {
    lcd_nibble((c >> 4) & 0x0F, 1);
    lcd_nibble(c & 0x0F, 1);
}

void lcd_print(const char* s) {
    while (*s) lcd_char(*s++);
}

/* --- CONVERSÃO DE DADOS (SEM STDIO.H) --- */
void ftoa_manual(float val, char* buf) {
    if (val < 0) { *buf++ = '-'; val = -val; }
    int int_part = (int)val;
    int dec_part = (int)((val - int_part) * 10);
    
    // Inteiro (max 2 dígitos para interface)
    if (int_part >= 10) *buf++ = (int_part / 10) + '0';
    *buf++ = (int_part % 10) + '0';
    *buf++ = '.';
    *buf++ = dec_part + '0';
    *buf = '\0';
}

float atof_manual(const char* s) {
    float res = 0.0, fact = 1.0;
    if (*s == '-') { s++; fact = -1.0; }
    for (int point_seen = 0; *s; s++) {
        if (*s == '.') { point_seen = 1; continue; }
        int d = *s - '0';
        if (d >= 0 && d <= 9) {
            if (point_seen) fact /= 10.0f;
            res = res * 10.0f + (float)d;
        }
    }
    return res * fact;
}

/* --- VARIÁVEIS GLOBAIS --- */
volatile float Kp = 1.0, Ki = 0.1, Kd = 0.05;
volatile float erro_val = 0;
volatile uint32_t pulsos = 0;
volatile uint8_t flag_pid = 0;
uint8_t param_sel = 0;

/* --- INTERRUPÇÕES MANUAIS (__vector_n) --- */
void __vector_2 (void) __attribute__ ((signal, used, externally_visible)); // INT1 (D3)
void __vector_2 (void) { pulsos++; }

void __vector_11 (void) __attribute__ ((signal, used, externally_visible)); // TIMER1 COMPA
void __vector_11 (void) {
    static uint8_t cnt = 0;
    if (++cnt >= 50) { flag_pid = 1; cnt = 0; }
}

void __vector_18 (void) __attribute__ ((signal, used, externally_visible)); // USART RX
void __vector_18 (void) {
    static char rx_b[16]; static uint8_t rx_i = 0;
    char c = UDR0;
    if (c == '\n') {
        rx_b[rx_i] = '\0'; rx_i = 0;
        if (rx_b[0] == 'K' && rx_b[1] == 'P') Kp = atof_manual(&rx_b[3]);
        else if (rx_b[0] == 'K' && rx_b[1] == 'I') Ki = atof_manual(&rx_b[3]);
        else if (rx_b[0] == 'K' && rx_b[1] == 'D') Kd = atof_manual(&rx_b[3]);
    } else if (rx_i < 15) rx_b[rx_i++] = c;
}

/* --- FUNÇÃO DE INTERFACE ORIGINAL --- */
void atualiza_lcd() {
    char b[8];
    lcd_cmd(0x80); lcd_print("Erro:");
    ftoa_manual(erro_val, b); lcd_print(b); lcd_print("      ");
    
    lcd_cmd(0xC0);
    if (param_sel == 0) { lcd_print("* Kp: "); ftoa_manual(Kp, b); }
    else if (param_sel == 1) { lcd_print("* Ki: "); ftoa_manual(Ki, b); }
    else { lcd_print("* Kd: "); ftoa_manual(Kd, b); }
    lcd_print(b); lcd_print("      ");
}

/* --- MAIN --- */
int main(void) {
    // Configurações DDR conforme Tabela
    DDRD = (1<<7); // PD7 Saída (Trig), PD6/PD3 Entradas
    DDRB = (1<<5) | (1<<2); // PB5 Saída (Bomba), PB2 Saída (Servo), PB3/PB4 Entradas
    PORTB = (1<<5); // Bomba PNP OFF
    PORTD = (1<<4); // Pull-up Encoder SW

    // Inicializações
    TWSR = 0x00; TWBR = 72; // I2C 100kHz
    UBRR0H = 0; UBRR0L = 103; // 9600 baud
    UCSR0B = (1<<4) | (1<<3) | (1<<7); // RXEN, TXEN, RXCIE
    
    // LCD Init
    delay_ms(50);
    lcd_nibble(0x03, 0); delay_ms(5);
    lcd_nibble(0x03, 0); delay_ms(1);
    lcd_nibble(0x03, 0); lcd_nibble(0x02, 0);
    lcd_cmd(0x28); lcd_cmd(0x0C); lcd_cmd(0x01);

    // Timer1 (PWM + Tick)
    TCCR1A = (1<<5) | (1<<1); // COM1B1, WGM11
    TCCR1B = (1<<4) | (1<<3) | (1<<1); // WGM13, WGM12, CS11
    ICR1 = 39999; OCR1B = 3000;
    TIMSK1 = (1<<1); OCR1A = 3999;

    EICRA = (1<<3); EIMSK = (1<<1); // INT1 Borda Descida
    __asm__ volatile ("sei" ::); // Enable Global Interrupts

    uint8_t l_clk = (PINB & (1<<3));
    uint8_t l_sw  = (PIND & (1<<4));

    atualiza_lcd();

    while(1) {
        // Polling Encoder (D11/D12/D4)
        uint8_t clk = (PINB & (1<<3));
        if (clk != l_clk && clk == 0) {
            float s = (PINB & (1<<4)) ? 1.0 : -1.0;
            if (param_sel == 0) Kp += s*0.1;
            else if (param_sel == 1) Ki += s*0.1;
            else Kd += s*0.1;
            atualiza_lcd();
        }
        l_clk = clk;

        uint8_t sw = (PIND & (1<<4));
        if (!sw && l_sw) { param_sel = (param_sel + 1) % 3; atualiza_lcd(); delay_ms(200); }
        l_sw = sw;

        if (flag_pid) {
            // Nível
            PORTD |= (1<<7); delay_us(10); PORTD &= ~(1<<7);
            uint32_t t = 0; while(!(PIND & (1<<6)) && t < 30000) t++;
            t = 0; while((PIND & (1<<6)) && t < 30000) { t++; delay_us(1); }
            float nivel = (t * 0.034) / 2;

            // Bomba PNP
            if (nivel > 15.0) PORTB &= ~(1<<5); else if (nivel < 5.0) PORTB |= (1<<5);

            // PID Simples
            float vazao = (pulsos * 60.0) / 7.5; pulsos = 0;
            erro_val = 30.0 - vazao;
            
            float u = (Kp * erro_val); // Exemplo Proporcional
            int16_t sv = 3000 + (int16_t)u;
            if (sv < 2000) sv = 2000; if (sv > 4000) sv = 4000;
            OCR1B = sv;

            atualiza_lcd();

            // Telemetria Serial (Manual sem sprintf)
            char tmp[10];
            ftoa_manual(vazao, tmp); for(int j=0; tmp[j]; j++){ while(!(UCSR0A & (1<<5))); UDR0 = tmp[j]; }
            while(!(UCSR0A & (1<<5))); UDR0 = ',';
            ftoa_manual(nivel, tmp); for(int j=0; tmp[j]; j++){ while(!(UCSR0A & (1<<5))); UDR0 = tmp[j]; }
            while(!(UCSR0A & (1<<5))); UDR0 = '\n';

            flag_pid = 0;
        }
    }
}
