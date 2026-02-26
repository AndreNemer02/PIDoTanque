/* --- ENDEREÇOS MANUAIS DO ATmega328P --- */
#define MY_UDR0    (*(volatile unsigned char *)(0xC6))
#define MY_UCSR0A  (*(volatile unsigned char *)(0xC0))
#define MY_UCSR0B  (*(volatile unsigned char *)(0xC1))
#define MY_UCSR0C  (*(volatile unsigned char *)(0xC2))
#define MY_UBRR0L  (*(volatile unsigned char *)(0xC4))
#define MY_UBRR0H  (*(volatile unsigned char *)(0xC5))

#define MY_DDRB    (*(volatile unsigned char *)(0x24))
#define MY_PORTB   (*(volatile unsigned char *)(0x25))
#define MY_DDRD    (*(volatile unsigned char *)(0x2A))
#define MY_PORTD   (*(volatile unsigned char *)(0x2B))
#define MY_PIND    (*(volatile unsigned char *)(0x29))

#define MY_TCCR1B  (*(volatile unsigned char *)(0x81))
#define MY_TCNT1   (*(volatile unsigned int *)(0x84))

/* --- DRIVER SERIAL --- */
void UART_init() {
    MY_UBRR0H = 0;
    MY_UBRR0L = 103; // 9600 baud @ 16MHz
    MY_UCSR0B = (1 << 3); 
    MY_UCSR0C = (3 << 1); 
}

void UART_send_msg(const char* s) {
    while (*s) {
        while (!(MY_UCSR0A & (1 << 5)));
        MY_UDR0 = *s++;
    }
}

void UART_send_num(unsigned int n) {
    char buf[7]; int i = 0;
    if (n == 0) { while (!(MY_UCSR0A & (1 << 5))); MY_UDR0 = '0'; }
    else {
        while (n > 0) { buf[i++] = (n % 10) + '0'; n /= 10; }
        while (i > 0) { while (!(MY_UCSR0A & (1 << 5))); MY_UDR0 = buf[--i]; }
    }
    while (!(MY_UCSR0A & (1 << 5))); MY_UDR0 = ' ';
}

/* --- FUNÇÃO PRINCIPAL --- */
int main(void) {
    UART_init();
    MY_DDRD |= (1 << 7);  // Trigger (D7) como Saída
    MY_DDRD &= ~(1 << 6); // Echo (D6) como Entrada
    MY_DDRB |= (1 << 5);  // Bomba (D13) como Saída

    MY_PORTB |= (1 << 5); // Inicia DESLIGADA (Lógica PNP)

    // Variáveis de Estabilização
    unsigned char count_on = 0;
    unsigned char count_off = 0;

    while(1) {
        // 1. Pulso de Trigger (10us)
        MY_PORTD |= (1 << 7);
        for(unsigned char i=0; i<40; i++) __asm__ __volatile__ ("nop");
        MY_PORTD &= ~(1 << 7);

        // 2. Medição do Echo
        while(!(MY_PIND & (1 << 6))); 
        MY_TCNT1 = 0;
        MY_TCCR1B = (1 << 1); // Liga Timer1 (Prescaler 8)
        while(MY_PIND & (1 << 6)); 
        unsigned int ticks = MY_TCNT1;
        MY_TCCR1B = 0;

        // 3. Lógica com Contador de Confirmação (5 vezes)
        if (ticks > 850) { 
            count_on++;
            count_off = 0; // Reseta o contador oposto
            
            if (count_on >= 5) {
                MY_PORTB &= ~(1 << 5); // LIGA (0V na base PNP)
                UART_send_msg("LIGADA  | ");
                count_on = 0;
            } else {
                UART_send_msg("CONFIRM.| ");
            }
        } 
        else if (ticks < 700) {
            count_off++;
            count_on = 0; // Reseta o contador oposto
            
            if (count_off >= 5) {
                MY_PORTB |= (1 << 5);  // DESLIGA (5V na base PNP)
                UART_send_msg("DESLIG. | ");
                count_off = 0;
            } else {
                UART_send_msg("CONFIRM.| ");
            }
        } 
        else {
            // Zona morta (Histerese): mantém o estado e reseta filtros
            UART_send_msg("MANTER  | ");
            count_on = 0;
            count_off = 0;
        }

        UART_send_msg("Ticks: ");
        UART_send_num(ticks);
        while (!(MY_UCSR0A & (1 << 5))); MY_UDR0 = '\r';
        while (!(MY_UCSR0A & (1 << 5))); MY_UDR0 = '\n';

        // Delay de estabilização (~200ms) - Essencial para ondas na água
        for(unsigned long i = 0; i < 300000; i++) __asm__ __volatile__ ("nop");
    }
    return 0;
}

void setup() {}
void loop() {}
