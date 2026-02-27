/**
 * PROJETO FINAL: Controle de Tanque Baremetal - André
 * FUSÃO: PID (Tiers) + Bomba/Ultrassom + LCD/Encoder + Telemetria ESP32
 * DATA: 11/12 | ATmega328P @ 16MHz
 */

#define REG8(x)  (*(volatile unsigned char *)(x))
#define REG16(x) (*(volatile unsigned int *)(x))

// --- REGISTRADORES MANUAIS ---
#define _PIND   REG8(0x29)
#define _DDRD   REG8(0x2A)
#define _PORTD  REG8(0x2B)
#define _PINB   REG8(0x23)
#define _DDRB   REG8(0x24)
#define _PORTB  REG8(0x25)
#define _SREG   REG8(0x5F)
#define _EICRA  REG8(0x69)
#define _EIMSK  REG8(0x3D)

// Timer1 (Servo no Pino 9)
#define _TCCR1A REG8(0x80)
#define _TCCR1B REG8(0x81)
#define _ICR1   REG16(0x86)
#define _OCR1A  REG16(0x88)

// I2C (LCD)
#define _TWBR   REG8(0xB8)
#define _TWSR   REG8(0xB9)
#define _TWDR   REG8(0xBB)
#define _TWCR   REG8(0xBC)

// UART (Serial)
#define _UDR0   REG8(0xC6)
#define _UCSR0A REG8(0xC0)
#define _UCSR0B REG8(0xC1)
#define _UCSR0C REG8(0xC2)
#define _UBRR0L REG8(0xC4)

// --- CONSTANTES SERVO ---
#define SERVO_STOP 2600 // 1.5ms
#define SERVO_DIR  2000 // 2.0ms
#define SERVO_ESQ  3000 // 1.0ms

// --- VARIÁVEIS GLOBAIS ---
volatile float Kp = 1.0, Ki = 0.1, Kd = 0.01;
volatile float setpoint = 15.0, vazao_atual = 0, erro = 0, nivel = 0;
float erro_anterior = 0, integral = 0;
float kpe = 0, kii = 0, kdd = 0; 

unsigned char param_sel = 0; 
volatile unsigned long pulseCount = 0;
unsigned char c_on = 0, c_off = 0;

// --- FUNÇÕES DE APOIO E DELAY ---
void delay_us(unsigned int us) {
    while(us--) { __asm__ __volatile__ ("nop"); __asm__ __volatile__ ("nop"); }
}

void delay_ms(unsigned int ms) {
    while(ms--) { for(unsigned long i = 0; i < 1600; i++) __asm__ __volatile__ ("nop"); }
}

// ISR do Sensor de Vazão (INT0 no Pino 2)
extern "C" void __vector_1 (void) __attribute__ ((signal, used, externally_visible));
void __vector_1 (void) { pulseCount++; }

// --- UART (Comunicação ESP32) ---
void uart_init() {
    _UBRR0L = 103; // 9600 baud
    _UCSR0B = (1 << 3) | (1 << 4); 
    _UCSR0C = (3 << 1); 
}

void uart_putc(char c) { while (!(_UCSR0A & (1 << 5))); _UDR0 = c; }
void uart_puts(const char* s) { while (*s) uart_putc(*s++); }

void uart_send_float(float f) {
    if (f < 0) { uart_putc('-'); f = -f; }
    int ip = (int)f;
    int dp = (int)((f - ip) * 100);
    char buf[10]; int i = 0;
    if (ip == 0) uart_putc('0');
    while (ip > 0) { buf[i++] = (ip % 10) + '0'; ip /= 10; }
    while (i > 0) uart_putc(buf[--i]);
    uart_putc('.');
    if (dp < 10) uart_putc('0');
    i = 0; if (dp == 0) uart_putc('0');
    while (dp > 0) { buf[i++] = (dp % 10) + '0'; dp /= 10; }
    while (i > 0) uart_putc(buf[--i]);
}

// --- I2C E LCD BAREMETAL ---
void i2c_init() { _TWSR = 0x00; _TWBR = 72; }

void i2c_write(unsigned char data) {
    _TWDR = data; _TWCR = (1 << 7) | (1 << 2);
    while (!(_TWCR & (1 << 7)));
}

void lcd_pcf(unsigned char data) {
    _TWCR = (1 << 7) | (1 << 5) | (1 << 2); while(!(_TWCR & (1 << 7)));
    _TWDR = (0x27 << 1); _TWCR = (1 << 7) | (1 << 2); while(!(_TWCR & (1 << 7)));
    i2c_write(data | 0x08);
    _TWCR = (1 << 7) | (1 << 4) | (1 << 2);
}

void lcd_pulse(unsigned char d) {
    lcd_pcf(d | 0x04); delay_us(1);
    lcd_pcf(d & ~0x04); delay_us(50);
}

void lcd_send(unsigned char val, unsigned char rs) {
    lcd_pulse((val & 0xF0) | rs);
    lcd_pulse(((val << 4) & 0xF0) | rs);
}

// Corrigido: função para imprimir strings sem erro de const char*
void lcd_puts(const char* s) {
    while (*s) {
        lcd_send(*s++, 1);
    }
}

void itoa_simple(int n, char* s) {
    int i = 0, sign = n; if (n < 0) n = -n;
    do { s[i++] = n % 10 + '0'; } while ((n /= 10) > 0);
    if (sign < 0) s[i++] = '-'; s[i] = '\0';
    for (int j = 0, k = i - 1; j < k; j++, k--) { char t = s[j]; s[j] = s[k]; s[k] = t; }
}

void print_float_lcd(float val) {
    char b[10]; int ip = (int)val, dp = (int)((val - ip) * 100);
    if (dp < 0) dp = -dp; itoa_simple(ip, b);
    char *p = b; while(*p) lcd_send(*p++, 1);
    lcd_send('.', 1); if (dp < 10) lcd_send('0', 1);
    itoa_simple(dp, b); p = b; while(*p) lcd_send(*p++, 1);
}

void atualiza_display() {
    lcd_send(0x80, 0); lcd_puts("E:"); print_float_lcd(erro);
    lcd_send(0xC0, 0);
    if (param_sel == 0)      { lcd_puts("Kp:"); print_float_lcd(Kp); }
    else if (param_sel == 1) { lcd_puts("Ki:"); print_float_lcd(Ki); }
    else                     { lcd_puts("Kd:"); print_float_lcd(Kd); }
}

// --- ATUAÇÃO E SENSORES ---
void burst_servo(int dir, float erro_mag) {
    if (erro_mag < 0) erro_mag = -erro_mag;
    unsigned char mult = 1;
    if (erro_mag > 10.0) mult = 10;
    else if (erro_mag > 8.0) mult = 8;
    else if (erro_mag > 6.0) mult = 6;
    else if (erro_mag > 4.0) mult = 4;
    else if (erro_mag > 2.0) mult = 2;
    else mult = 1;

    _OCR1A = (dir == 1) ? SERVO_DIR : SERVO_ESQ;
    delay_ms(25 * mult); 
    _OCR1A = SERVO_STOP;
}

unsigned int medir_nivel() {
    _PORTD |= (1 << 7); delay_us(10); _PORTD &= ~(1 << 7);
    while(!(_PIND & (1 << 6)));
    unsigned int t = 0; 
    while((_PIND & (1 << 6)) && (t < 15000)) { t++; delay_us(1); }
    return t;
}

// --- MAIN ---
int main(void) {
    _DDRB |= (1 << 1) | (1 << 5); _PORTB |= (1 << 5); // Pino 9 e 13 (Bomba PNP)
    _DDRD |= (1 << 7); // Trigger
    _DDRD &= ~((1 << 2)|(1 << 3)|(1 << 4)|(1 << 5)|(1 << 6));
    _PORTD |= (1 << 2)|(1 << 3)|(1 << 4)|(1 << 5);

    _TCCR1A = (1 << 7) | (1 << 1); _TCCR1B = (1 << 4) | (1 << 3) | (1 << 1); 
    _ICR1 = 40000; _OCR1A = SERVO_STOP;

    uart_init(); i2c_init(); 
    // LCD Init Sequência
    delay_ms(50); lcd_pulse(0x30); delay_ms(5); lcd_pulse(0x30); delay_us(200);
    lcd_pulse(0x30); delay_us(200); lcd_pulse(0x20); lcd_send(0x28, 0); 
    lcd_send(0x0C, 0); lcd_send(0x01, 0); delay_ms(2);

    _EICRA = (1 << 1) | (1 << 0); _EIMSK = (1 << 0);
    _SREG |= (1 << 7);

    unsigned long timer_geral = 0;
    unsigned char last_clk = (_PIND & (1 << 3));
    unsigned char last_sw = (_PIND & (1 << 5));

    atualiza_display();

    while(1) {
        timer_geral++;

        // 1. ENCODER
        unsigned char clk = (_PIND & (1 << 3));
        if (clk != last_clk && clk == 0) {
            delay_us(500);
            if ((_PIND & (1 << 4)) != clk) {
                if (param_sel == 0) Kp += 0.1; else if (param_sel == 1) Ki += 0.01; else Kd += 0.01;
            } else {
                if (param_sel == 0) Kp -= 0.1; else if (param_sel == 1) Ki -= 0.01; else Kd -= 0.01;
            }
            if (Kp < 0) Kp = 0; if (Ki < 0) Ki = 0; if (Kd < 0) Kd = 0;
            atualiza_display();
        }
        last_clk = clk;

        unsigned char sw = (_PIND & (1 << 5));
        if (!sw && last_sw) {
            delay_ms(20); param_sel = (param_sel + 1) % 3; atualiza_display();
        }
        last_sw = sw;

        // 2. TAREFAS PERIÓDICAS (A cada ~1 segundo)
        if (timer_geral > 500000) {
            unsigned int d = medir_nivel();
            nivel = (float)d;
            if (d > 240) { c_on++; c_off=0; if(c_on>=5){ _PORTB &= ~(1 << 5); c_on=0; } }
            else if (d < 190) { c_off++; c_on=0; if(c_off>=5){ _PORTB |= (1 << 5); c_off=0; } }

            _SREG &= ~(1 << 7); vazao_atual = (float)pulseCount; pulseCount = 0; _SREG |= (1 << 7);
            erro = setpoint - vazao_atual;
            integral += erro; if(integral > 20) integral = 20;

            kpe = Kp * erro; kii = Ki * integral; kdd = Kd * (erro - erro_anterior);
            float out = kpe + kii + kdd;
            erro_anterior = erro;

            if (out > 1.0) burst_servo(1, erro); else if (out < -1.0) burst_servo(-1, erro);

            // Telemetria para o ESP32
            uart_send_float(vazao_atual); uart_putc(',');
            uart_send_float(nivel);       uart_putc(',');
            uart_send_float(erro);        uart_putc(',');
            uart_send_float(kpe);         uart_putc(',');
            uart_send_float(kii);         uart_putc(',');
            uart_send_float(kdd);         uart_putc('\n');

            atualiza_display();
            timer_geral = 0;
        }
    }
}
