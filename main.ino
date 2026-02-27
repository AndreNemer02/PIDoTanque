/**
 * @details:
 *  - Controlador PID de vazão (sensor YF-S201, atuador: servomotor via engrenagem)
 *  - Controle de segurança de nível (sensor HC-SR04, atuador: bomba, TIP30C)
 *  - Interface local via LCD 16x2 I2C e encoder rotativo KY-040
 *  - Telemetria serial UART para gateway ESP32/Blynk
 *  - Recepção de parâmetros Kp, Ki, Kd via UART (ajuste remoto pelo Blynk)
 *
 *
 * @pinout
 *   PD7 (D7)  TRIG do HC-SR04 (saída)
 *   PD6 (D6)  ECHO do HC-SR04 (entrada)
 *   PD3 (D3)  Pulsos do YF-S201 via INT1 (entrada)
 *   PD4 (D4)  Botão SW do encoder KY-040 (entrada, pull-up)
 *   PB2 (D10)  PWM OC1B para o servomotor (saída)
 *   PB5 (D13  Bomba d'água via TIP30C PNP (saída)
 *   PB3 (D11)  CLK do encoder KY-040 (entrada)
 *   PB4 (D12) DT do encoder KY-040 (entrada)
 *   A4  (SDA) Barramento I2C dados (LCD)
 *   A5  (SCL)  Barramento I2C clock (LCD)
 *   D0  (RX)  Recebe KP/KI/KD do ESP32
 *   D1  (TX)  Envia telemetria ao ESP32
 */


typedef unsigned char  uint8_t;
typedef unsigned int   uint16_t;
typedef unsigned long  uint32_t;
typedef signed int     int16_t;


// MAPEAMENTO DE REGISTRADORES I/O (ENDEREÇOS FÍSICOS)

/** @defgroup PortB Registradores da Porta B (Pinos D8-D13)
 *  @{
 */
#define DDRB   (*(volatile uint8_t *)(0x24))  /**< Data Direction Register B: define pinos como entrada (0) ou saída (1) */
#define PORTB  (*(volatile uint8_t *)(0x25))  /**< Port B Data Register: escreve nível lógico nos pinos de saída */
#define PINB   (*(volatile uint8_t *)(0x23))  /**< Port B Input Pins: lê o estado atual dos pinos de entrada */
/** @} */

/** @defgroup PortD Registradores da Porta D (Pinos D0-D7)
 *  @{
 */
#define DDRD   (*(volatile uint8_t *)(0x2A))  /**< Data Direction Register D */
#define PORTD  (*(volatile uint8_t *)(0x2B))  /**< Port D Data Register */
#define PIND   (*(volatile uint8_t *)(0x29))  /**< Port D Input Pins Address */
/** @} */

/** @defgroup TWI Registradores TWI/I2C
 *  @{
 */
#define TWBR   (*(volatile uint8_t *)(0xB8))  /**< TWI Bit Rate Register: define a frequência do barramento I2C */
#define TWSR   (*(volatile uint8_t *)(0xB9))  /**< TWI Status Register: informa o estado atual da transação I2C */
#define TWDR   (*(volatile uint8_t *)(0xBB))  /**< TWI Data Register: byte a ser transmitido ou recebido */
#define TWCR   (*(volatile uint8_t *)(0xBC))  /**< TWI Control Register: controla START, STOP, ACK e habilitação */
/** @} */

/** @defgroup USART Registradores USART (Comunicação Serial)
 *  @{
 */
#define UBRR0H (*(volatile uint8_t *)(0xC5))  /**< USART Baud Rate Register High: byte alto do divisor de baud rate */
#define UBRR0L (*(volatile uint8_t *)(0xC4))  /**< USART Baud Rate Register Low: byte baixo do divisor de baud rate */
#define UCSR0A (*(volatile uint8_t *)(0xC0))  /**< USART Control and Status A: flags de status (UDRE, RXC, TXC) */
#define UCSR0B (*(volatile uint8_t *)(0xC1))  /**< USART Control and Status B: habilita RX, TX e interrupções */
#define UCSR0C (*(volatile uint8_t *)(0xC2))  /**< USART Control and Status C: configura formato do frame (8N1) */
#define UDR0   (*(volatile uint8_t *)(0xC6))  /**< USART I/O Data Register: buffer de transmissão e recepção */
/** @} */

/** @defgroup Timer1 Registradores do Timer 1 (16 bits)
 *  @{
 */
#define TCCR1A (*(volatile uint8_t *)(0x80))  /**< Timer/Counter Control Register A: configura modo PWM e saídas */
#define TCCR1B (*(volatile uint8_t *)(0x81))  /**< Timer/Counter Control Register B: configura prescaler e modo */
#define ICR1   (*(volatile uint16_t *)(0x86)) /**< Input Capture Register: define o valor TOP do PWM (período) */
#define OCR1A  (*(volatile uint16_t *)(0x88)) /**< Output Compare Register A: define o tick de interrupção do PID */
#define OCR1B  (*(volatile uint16_t *)(0x8A)) /**< Output Compare Register B: define a posição do servomotor via PWM */
#define TIMSK1 (*(volatile uint8_t *)(0x6E))  /**< Timer Interrupt Mask Register: habilita interrupções do Timer 1 */
/** @} */

/** @defgroup Interrupts Registradores de Interrupções Externas
 *  @{
 */
#define EICRA  (*(volatile uint8_t *)(0x69))  /**< External Interrupt Control Register A: configura borda de disparo INT0/INT1 */
#define EIMSK  (*(volatile uint8_t *)(0x3D))  /**< External Interrupt Mask Register: habilita INT0 e INT1 */
#define SREG   (*(volatile uint8_t *)(0x5F))  /**< Status Register: bit 7 = habilitação global de interrupções (sei/cli) */
/** @} */


// CONSTANTES E ENDEREÇO DO LCD


/** @brief Endereço I2C padrão do módulo PCF8574 acoplado ao LCD 16x2 */
#define LCD_ADDR 0x27


// VARIÁVEIS GLOBAIS DE CONTROLE

/** @brief Ganho proporcional do controlador PID. Ajustável pelo encoder ou pelo Blynk. */
volatile float Kp = 1.0;

/** @brief Ganho integral do controlador PID. Ajustável pelo encoder ou pelo Blynk. */
volatile float Ki = 0.1;

/** @brief Ganho derivativo do controlador PID. Ajustável pelo encoder ou pelo Blynk. */
volatile float Kd = 0.05;

/** @brief Erro atual do controlador (setpoint - vazão medida). Exibido no LCD e enviado ao Blynk. */
volatile float erro_val = 0;

/** @brief Acumulador do termo integral do PID. */
volatile float integral = 0;

/** @brief Erro do ciclo anterior, utilizado no cálculo do termo derivativo. */
volatile float erro_ant = 0;

/** @brief Contador de pulsos do sensor de vazão YF-S201, incrementado pela ISR INT1. */
volatile uint32_t pulsos = 0;

/**
 * @brief Flag que indica que o ciclo de 100ms ocorreu e o PID deve ser executado.
 * @note Setada pela ISR do Timer 1 (COMPA). Resetada no loop principal após execução do PID.
 */
volatile uint8_t flag_pid = 0;

/** @brief Parâmetro selecionado no menu do encoder: 0=Kp, 1=Ki, 2=Kd. */
uint8_t param_sel = 0;

/**
 * @brief Gera atraso em microssegundos usando instruções NOP.
 * @param us Quantidade de microssegundos (calibrado para F_CPU = 16MHz).
 * @note Cada iteração executa 4 NOPs ≈ 0,25us * 4 = 1us a 16MHz.
 */
void delay_us(uint16_t us) {
    while (us--) {
        __asm__ volatile ("nop\n\tnop\n\tnop\n\tnop\n\t");
    }
}

/**
 * @brief Gera atraso em milissegundos.
 * @param ms Quantidade de milissegundos.
 */
void delay_ms(uint16_t ms) {
    while (ms--) delay_us(1000);
}


// DRIVER I2C (TWI) BARE METAL


/**
 * @brief Envia condição de START no barramento I2C.
 * @details Seta os bits TWINT, TWSTA e TWEN no TWCR e aguarda o hardware confirmar o START.
 */
void i2c_start() {
    TWCR = (1<<7)|(1<<5)|(1<<2); // TWINT=1, TWSTA=1, TWEN=1
    while (!(TWCR & (1<<7)));    // Aguarda TWINT ser setado (operação concluída)
}

/**
 * @brief Escreve um byte no barramento I2C.
 * @param data Byte a ser transmitido (endereço do dispositivo ou dado).
 */
void i2c_write(uint8_t data) {
    TWDR = data;
    TWCR = (1<<7)|(1<<2); // TWINT=1, TWEN=1 (inicia transmissão)
    while (!(TWCR & (1<<7)));
}

/**
 * @brief Envia um byte ao expansor PCF8574 do módulo I2C do LCD.
 * @param data Byte contendo os nibbles de dado e os bits de controle (EN, RS, BL).
 * @details Realiza a sequência completa: START > endereço > dado > STOP.
 */
void pcf_write(uint8_t data) {
    i2c_start();
    i2c_write(LCD_ADDR << 1); // Endereço + bit R/W = 0 (escrita)
    i2c_write(data);
    TWCR = (1<<7)|(1<<4)|(1<<2); // TWINT=1, TWSTO=1, TWEN=1 (STOP)
    delay_us(50);
}


// DRIVER LCD 16x2 (INTERFACE 4 BITS VIA PCF8574)

/**
 * @brief Envia um nibble (4 bits) ao LCD com pulso de Enable.
 * @param nibble Os 4 bits a serem enviados (posicionados nos bits 7:4 do PCF8574).
 * @param rs     Register Select: 0 = comando, 1 = dado (caractere).
 * @details O bit 3 do byte enviado mantém o backlight ligado (BL=1).
 */
void lcd_nibble(uint8_t nibble, uint8_t rs) {
    uint8_t data = (nibble << 4) | (1<<3); // Bit 3 = Backlight ON
    if (rs) data |= (1<<0);                // Bit 0 = RS
    pcf_write(data | (1<<2));              // Enable ON  (bit 2 = EN)
    delay_us(1);
    pcf_write(data & ~(1<<2));             // Enable OFF
    delay_us(50);
}

/**
 * @brief Envia um comando ao LCD (RS=0).
 * @param cmd Byte de comando conforme datasheet HD44780.
 */
void lcd_cmd(uint8_t cmd) {
    lcd_nibble(cmd >> 4, 0);   // Nibble alto
    lcd_nibble(cmd & 0x0F, 0); // Nibble baixo
    delay_ms(2);
}

/**
 * @brief Envia um caractere ao LCD (RS=1).
 * @param c Caractere ASCII a ser exibido.
 */
void lcd_char(char c) {
    lcd_nibble((c >> 4) & 0x0F, 1);
    lcd_nibble(c & 0x0F, 1);
}

/**
 * @brief Envia uma string ao LCD.
 * @param s Ponteiro para string terminada em '\0'.
 */
void lcd_print(const char* s) {
    while (*s) lcd_char(*s++);
}

/**
 * @brief Inicializa o LCD no modo 4 bits conforme sequência do datasheet HD44780.
 */
void lcd_init() {
    delay_ms(50);
    lcd_nibble(0x03, 0); delay_ms(5);
    lcd_nibble(0x03, 0); delay_ms(1);
    lcd_nibble(0x03, 0);
    lcd_nibble(0x02, 0);           // Entra em modo 4 bits
    lcd_cmd(0x28);                 // 2 linhas, fonte 5x8
    lcd_cmd(0x0C);                 // Display ON, cursor OFF
    lcd_cmd(0x01);                 // Limpa display
}


// CONVERSÃO NUMÉRICA (SEM stdio.h / stdlib.h)


/**
 * @brief Converte um valor float para string no formato "X.X".
 * @param val Valor float a ser convertido.
 * @param buf Buffer de saída (mínimo 6 bytes).
 * @note Suporta valores negativos e até 2 dígitos inteiros.
 */
void ftoa_manual(float val, char* buf) {
    if (val < 0) { *buf++ = '-'; val = -val; }
    int int_part = (int)val;
    int dec_part = (int)((val - int_part) * 10);
    if (int_part >= 10) *buf++ = (int_part / 10) + '0';
    *buf++ = (int_part % 10) + '0';
    *buf++ = '.';
    *buf++ = dec_part + '0';
    *buf = '\0';
}

/**
 * @brief Converte uma string para float (substitui atof() da stdlib).
 * @param s String de entrada no formato "[-]inteiro[.decimal]".
 * @return Valor float convertido.
 */
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


// ATUALIZAÇÃO DO LCD


/**
 * @brief Atualiza o conteúdo do LCD com o erro atual e o parâmetro PID selecionado.
 *
 * @details Linha 1: "Erro: X.X"
 *          Linha 2: "* Kp: X.X" ou "* Ki: X.X" ou "* Kd: X.X"
 *          O asterisco (*) indica o parâmetro atualmente selecionado pelo encoder.
 */
void atualiza_lcd() {
    char b[8];
    lcd_cmd(0x80);              // Cursor na linha 1, coluna 0
    lcd_print("Erro:");
    ftoa_manual(erro_val, b);
    lcd_print(b);
    lcd_print("      ");        // Limpa caracteres residuais

    lcd_cmd(0xC0);              // Cursor na linha 2, coluna 0
    if (param_sel == 0) {
        lcd_print("* Kp: ");
        ftoa_manual(Kp, b);
    } else if (param_sel == 1) {
        lcd_print("* Ki: ");
        ftoa_manual(Ki, b);
    } else {
        lcd_print("* Kd: ");
        ftoa_manual(Kd, b);
    }
    lcd_print(b);
    lcd_print("      ");
}


//  VETORES DE INTERRUPÇÃO (MAPEAMENTO DIRETO NO HARDWARE)
 

/**
 * @brief ISR do INT1 (vetor 2) — Contador de pulsos do sensor de vazão YF-S201.
 * @details Disparada na borda de descida do pino PD3 (D3).
 *          Cada pulso corresponde a um determinado volume de líquido.
 *          A vazão é calculada no loop principal: vazao = (pulsos * 60) / 7.5
 */
void __vector_2(void) __attribute__((signal, used, externally_visible));
void __vector_2(void) { pulsos++; }

/**
 * @brief ISR do TIMER1_COMPA (vetor 11) — Base de tempo de 100ms para o PID.
 * @details O Timer 1 dispara a cada 2ms (OCR1A = 3999, prescaler 8, F_CPU 16MHz).
 *          Um contador interno acumula 50 disparos para gerar o ciclo de 100ms,
 *          momento em que a flag_pid é setada para execução do PID no loop principal.
 */
void __vector_11(void) __attribute__((signal, used, externally_visible));
void __vector_11(void) {
    static uint8_t cnt = 0;
    if (++cnt >= 50) { flag_pid = 1; cnt = 0; }
}

/**
 * @brief ISR do USART_RXC (vetor 18) — Recepção de parâmetros Kp, Ki, Kd do ESP32/Blynk.
 * @details Aguarda uma linha completa terminada em '\n'.
 *          Formato esperado: "KP:valor\n", "KI:valor\n" ou "KD:valor\n"
 *          Exemplo: "KP:2.5\n" → atualiza Kp = 2.5
 */
void __vector_18(void) __attribute__((signal, used, externally_visible));
void __vector_18(void) {
    static char rx_b[16];
    static uint8_t rx_i = 0;
    char c = UDR0;
    if (c == '\n') {
        rx_b[rx_i] = '\0'; rx_i = 0;
        if      (rx_b[0] == 'K' && rx_b[1] == 'P') Kp = atof_manual(&rx_b[3]);
        else if (rx_b[0] == 'K' && rx_b[1] == 'I') Ki = atof_manual(&rx_b[3]);
        else if (rx_b[0] == 'K' && rx_b[1] == 'D') Kd = atof_manual(&rx_b[3]);
    } else if (rx_i < 15) {
        rx_b[rx_i++] = c;
    }
}


// LOOP PRINCIPAL
 

/**
 * @brief Função principal — inicializa hardware e executa o loop de controle.
 *
 * @details Sequência de inicialização:
 *  1. Configura direção dos pinos (DDR)
 *  2. Inicializa USART @ 9600 baud
 *  3. Inicializa I2C @ 100kHz e LCD
 *  4. Configura Timer 1: PWM 50Hz para servo + interrupção de 2ms
 *  5. Habilita INT1 (sensor de vazão) e interrupções globais
 *
 * @details Loop principal:
 *  - Polling do encoder para ajuste local de Kp, Ki, Kd
 *  - A cada 100ms (flag_pid): leitura dos sensores, controle bang-bang de nível,
 *    cálculo PID de vazão, acionamento do servo e envio de telemetria ao ESP32
 */
int main(void) {

    /* --- Configuração de Direção dos Pinos --- */
    DDRD = (1<<7);           // PD7 = saída (TRIG); PD6, PD3, PD4 = entradas
    DDRB = (1<<5) | (1<<2);  // PB5 = saída (PUMP), PB2 = saída (SERVO)
                              // PB3, PB4 = entradas (encoder CLK, DT)
    PORTB = (1<<5);          // Bomba PNP OFF: PB5 = HIGH desliga o TIP30C
    PORTD = (1<<4);          // Pull-up interno no SW do encoder (PD4)

    /* --- Inicialização da USART @ 9600 baud --- */
    UBRR0H = 0;
    UBRR0L = 103;                        // UBRR = F_CPU/(16*BAUD) - 1 = 103
    UCSR0B = (1<<4) | (1<<3) | (1<<7);  // RXEN=1, TXEN=1, RXCIE=1 (interrupção RX)

    /* --- Inicialização I2C @ 100kHz e LCD --- */
    TWSR = 0x00; TWBR = 72;  // F_SCL = F_CPU / (16 + 2*TWBR) = 100kHz
    delay_ms(50);
    lcd_nibble(0x03, 0); delay_ms(5);
    lcd_nibble(0x03, 0); delay_ms(1);
    lcd_nibble(0x03, 0);
    lcd_nibble(0x02, 0);
    lcd_cmd(0x28); lcd_cmd(0x0C); lcd_cmd(0x01);

    /* --- Configuração do Timer 1 (PWM Servo + Tick PID) ---
     * Modo 14 (Fast PWM, TOP=ICR1), prescaler 8
     * f_PWM = 16MHz / (8 * (39999+1)) = 50Hz (período 20ms) > ideal para servo
     * OCR1B = 3000 > posição neutra do servo (~1,5ms)
     * OCR1A = 3999 > interrupção COMPA a cada 2ms
     */
    TCCR1A = (1<<5) | (1<<1);           // COM1B1=1 (clear OC1B), WGM11=1
    TCCR1B = (1<<4) | (1<<3) | (1<<1);  // WGM13=1, WGM12=1, CS11=1 (prescaler 8)
    ICR1   = 39999;                      // TOP = 39999 > 50Hz
    OCR1B  = 3000;                       // Posição neutra do servo
    TIMSK1 = (1<<1);                     // Habilita interrupção OCIE1A
    OCR1A  = 3999;                       // Tick a cada 2ms

    /* --- Habilita INT1 (borda de descida, PD3) e interrupções globais --- */
    EICRA = (1<<3);   // ISC11=1: INT1 na borda de descida
    EIMSK = (1<<1);   // INT1 habilitado
    __asm__ volatile ("sei" ::); // Habilita interrupções globais (SREG bit 7)

    uint8_t l_clk = (PINB & (1<<3)); // Estado inicial do CLK do encoder
    uint8_t l_sw  = (PIND & (1<<4)); // Estado inicial do SW do encoder

    atualiza_lcd();

    while(1) {

        /* --- Leitura do Encoder Rotativo KY-040 (Polling) ---
         * Detecta borda de descida no CLK (PB3).
         * O estado do DT (PB4) no momento da borda define o sentido de rotação:
         *   DT=1  sentido horário  incrementa parâmetro
         *   DT=0  sentido anti-horário  decrementa parâmetro
         */
        uint8_t clk = (PINB & (1<<3));
        if (clk != l_clk && clk == 0) {
            float s = (PINB & (1<<4)) ? 1.0f : -1.0f; // Sentido de rotação
            if      (param_sel == 0) Kp += s * 0.1f;
            else if (param_sel == 1) Ki += s * 0.1f;
            else                     Kd += s * 0.1f;
            atualiza_lcd();
        }
        l_clk = clk;

        /* --- Botão SW do Encoder — Alterna parâmetro selecionado ---
         * Ciclo: Kp (0)  Ki (1)  Kd (2)  Kp (0) ...
         */
        uint8_t sw = (PIND & (1<<4));
        if (!sw && l_sw) {
            param_sel = (param_sel + 1) % 3;
            atualiza_lcd();
            delay_ms(200); // Debounce
        }
        l_sw = sw;

        /* --- Ciclo de Controle PID (executa a cada 100ms) --- */
        if (flag_pid) {

            /* Leitura do nível via HC-SR04
             * Pulso TRIG de 10us → mede duração do ECHO → distância = (t * 0.034) / 2
             */
            PORTD |= (1<<7);  delay_us(10);  PORTD &= ~(1<<7);
            uint32_t t = 0;
            while (!(PIND & (1<<6)) && t < 30000) t++;
            t = 0;
            while ( (PIND & (1<<6)) && t < 30000) { t++; delay_us(1); }
            float nivel = (t * 0.034f) / 2.0f; // Distância em cm

            /* Controle bang-bang de nível (segurança independente do PID)
             * TIP30C (PNP): LOW no pino = bomba LIGA | HIGH no pino = bomba DESLIGA
             * nivel > 15cm  tanque vazio  LIGA bomba
             * nivel < 5cm   tanque cheio  DESLIGA bomba
             */
            if      (nivel > 15.0f) PORTB &= ~(1<<5); // LIGA
            else if (nivel <  5.0f) PORTB |=  (1<<5); // DESLIGA

            /* Cálculo da vazão (pulsos acumulados pela ISR INT1 em 100ms)
             * Formula YF-S201: Frequência(Hz) = 7.5 * Q(L/min)
             * → Q = pulsos / (7.5 * 0.1s) = pulsos * 60 / 7.5
             */
            float vazao = (pulsos * 60.0f) / 7.5f;
            pulsos = 0;

            /* Algoritmo PID de vazão
             * Setpoint definido conforme calibração da maquete
             */
            erro_val = 30.0f - vazao; // Setpoint ajustável conforme a aplicação

            integral += erro_val * 0.1f; // Integração trapezoidal (dt = 100ms)
            float derivada = (erro_val - erro_ant) / 0.1f;

            float p_t = Kp * erro_val;
            float i_t = Ki * integral;
            float d_t = Kd * derivada;

            float u = p_t + i_t + d_t; // Sinal de controle total
            erro_ant = erro_val;

            /* Saída para o servomotor via OCR1B
             * Valor de OCR1B mapeado para posição angular da válvula:
             *   2000  posição mínima | 4000  posição máxima
             */
            int16_t sv = 3000 + (int16_t)u;
            if (sv < 2000) sv = 2000;
            if (sv > 4000) sv = 4000;
            OCR1B = sv;

            atualiza_lcd();

            /* Telemetria Serial → ESP32 → Blynk
             * Formato: "vazao,nivel,erro,p_t,i_t,d_t\n"
             * Exemplo: "12.5,8.3,2.1,2.1,0.3,0.1\n"
             */
            char tmp[10];
            #define SEND(val) \
                ftoa_manual(val, tmp); \
                for(int j=0; tmp[j]; j++){ while(!(UCSR0A & (1<<5))); UDR0 = tmp[j]; }

            SEND(vazao)  while(!(UCSR0A & (1<<5))); UDR0 = ',';
            SEND(nivel)  while(!(UCSR0A & (1<<5))); UDR0 = ',';
            SEND(erro_val) while(!(UCSR0A & (1<<5))); UDR0 = ',';
            SEND(p_t)    while(!(UCSR0A & (1<<5))); UDR0 = ',';
            SEND(i_t)    while(!(UCSR0A & (1<<5))); UDR0 = ',';
            SEND(d_t)    while(!(UCSR0A & (1<<5))); UDR0 = '\n';

            flag_pid = 0;
        }
    }
}
