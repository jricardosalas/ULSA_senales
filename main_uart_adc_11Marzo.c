/*
 * File:   main.c
 * Author: JRICARDO
 *
 * Created on 04 March 2021, 19:27
 */

// PIC18F4550 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1L
#pragma config PLLDIV = 1       // PLL Prescaler Selection bits (No prescale (4 MHz oscillator input drives PLL directly))
#pragma config CPUDIV = OSC1_PLL2// System Clock Postscaler Selection bits ([Primary Oscillator Src: /1][96 MHz PLL Src: /2])
#pragma config USBDIV = 1       // USB Clock Selection bit (used in Full-Speed USB mode only; UCFG:FSEN = 1) (USB clock source comes directly from the primary oscillator block with no postscale)

// CONFIG1H
//#pragma config FOSC = INTOSCIO_EC// Oscillator Selection bits (Internal oscillator, port function on RA6, EC used by USB (INTIO))
#pragma config FOSC = INTOSC_EC // Output system clock on RA6  // Oscillator Selection bits (Internal oscillator, port function on RA6, EC used by USB (INTIO))
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOR = ON         // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 2         // Brown-out Reset Voltage bits (Setting 1 2.79V)
#pragma config VREGEN = OFF     // USB Voltage Regulator Enable bit (USB voltage regulator disabled)

// CONFIG2H
#pragma config WDT = OFF        // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = OFF     // CCP2 MUX bit (CCP2 input/output is multiplexed with RB3)
#pragma config PBADEN = OFF     // PORTB A/D Enable bit (PORTB<4:0> pins are configured as digital I/O on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer 1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config ICPRT = OFF      // Dedicated In-Circuit Debug/Programming Port (ICPORT) Enable bit (ICPORT disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-001FFFh) is not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (002000-003FFFh) is not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (004000-005FFFh) is not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (006000-007FFFh) is not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) is not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM is not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-001FFFh) is not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (002000-003FFFh) is not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (004000-005FFFh) is not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (006000-007FFFh) is not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) are not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot block (000000-0007FFh) is not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM is not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-001FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (002000-003FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (004000-005FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (006000-007FFFh) is not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot block (000000-0007FFh) is not protected from table reads executed in other blocks)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdio.h>
#include <string.h>
#include "uart.h"

/*Defines*/
#define REDLED PORTDbits.RD2

// ADC channel selection macros - Example like sensors
#define ADC_channel_Sensor1    0 //AN0
#define ADC_channel_Sensor2    1 //AN1 
#define ADC_channel_Sensor3    2 //AN2

/*main functions prototypes*/
void tsk_UARTmonitor(void);
void printMenuUART(void);
void delay(void);
uint16_t readADC(uint8_t adc_channel);
static void init_UART(void);
static void init_ADC(void);
static void enable_interrupts(void);

/*Global variables*/
volatile char buf_uart[UART_BUF_SIZE];
volatile uint8_t len_uart = 0;

void __interrupt() ISR(void){            
    // UART RX interrupt
    if(RCIF){
        RCIF = 0; //clear flag
        if(len_uart == UART_BUF_SIZE){ // buffer overflow
            RCREG; // read and discard
            buf_uart[len_uart-1] = '\r'; // end of message
        }else{
            buf_uart[len_uart] = RCREG;
            len_uart++;
        }
        putchUART(RCREG); // echo character
        // HW buffer full (2 bytes). Data lost
        if(OERR){ //TODO error
            CREN=0; // Disable receiver to clear OERR flag
            CREN=1; // Re-enable
        }        
    }    
}

void main(void) {        
    PORTDbits.RD2 = 1;
    TRISDbits.RD2 = 0; // SALIDA D2
    init_ADC();
    init_UART();
    uart_en(1);
    enable_interrupts();
    home();
    clrscr();
    printMenuUART();
    while(1){
        tsk_UARTmonitor();
        REDLED = ~REDLED; // Blink LED
        delay();
        
        //getsnUART(buf_uart[0], sizeof(buf_uart));  // Example, polling RX communication ¡blocking!
                    
    }
    return;
}

/*System monitor tasks*/
/* Read PC UART request + take actions + reply
 */
void tsk_UARTmonitor(void){    
    char str[8];  // aux variable for char conversion
    int dataSensor; // dummy variable for sensor data
    //char *p_buf_uart;
    //p_buf_uart = getsnUART(buf_uart, sizeof(buf_uart)); // Block until data\n
        
    if(buf_uart[len_uart-1] == '\r'){ //  \n end of line character
        CREN = 0; // disable receiver module
        if(buf_uart[0] == 'M'){ // Utility commands
            printMenuUART(); //Menu and clear screen            
        }else if(buf_uart[0] == 'E'){ // Enable commands example
            putchsUART("Enable module X\r\n");
        }else if(buf_uart[0] == 'D'){ // Enable commands example
            putchsUART("Disable module X\r\n");           
        }else if(buf_uart[0] == 'R'){ // Read commands example
            if(buf_uart[1]=='1'){ //Hardware
                // example reading routine
                dataSensor = readADC(ADC_channel_Sensor1); 
                sprintf(str, "%u" ,dataSensor); // parse to unsigned int
                putchsUART("Sensor 1: ");
                putchsUART(str);
                putchsUART("\r\n");
            }else if(buf_uart[1]=='2'){
                // example reading routine
                dataSensor = readADC(ADC_channel_Sensor2); 
                sprintf(str, "%u" ,dataSensor); // parse to unsigned int
                putchsUART("Sensor 2: ");
                putchsUART(str);
                putchsUART("\r\n");
            }else if(buf_uart[1]=='3'){ // Read hardware current
                // example reading routine
                dataSensor = readADC(ADC_channel_Sensor3); 
                sprintf(str, "%u" ,dataSensor); // parse to unsigned int
                putchsUART("Sensor 3: ");
                putchsUART(str);
                putchsUART("\r\n");
            }else{
                putchsUART("R_InputError\r\n");
            }
        }else{
            putchsUART("InputError\n");
        }
        memset(buf_uart, '\0', sizeof(buf_uart)); // clear all buffer
        len_uart = 0;
        CREN = 1; // enable receiver
    }
}

void printMenuUART(void){
    // Welcome message
	clrscr(); 	//Terminal cmd
	home(); 	//Terminal cmd
	putchsUART("DEMO serial communication - v05/March/2021\r\n");
    putchsUART("\r\n************************************************\r\n"
        "Menu: \r\nE - Enable X"
              "\r\nD - Disable X"
              "\r\nRX - Read sensor X, X=1 to 3"
              "\r\nM - Menu"
              "\r\n->");  
}

void delay(void){
    int24_t i;
    for(i=0;i<2000;i++);
}

static void init_UART(void){
    // TX/RX UART settings
    // 1. Data direction TRIS    
    TRISC7 = 1; // input
    TRISC6 = 0; // output
    // 2. Transmitter (main control registers TX1STA & BAUD1CON)
    BRGH = 1;
    BRG16 = 1; // Formula BR=FOSC/[4*(n+1)] = 1MHz/(4*(12+1)) = 19230.76 
    SPBRG = 12; // Baud-rate 19200 (max at 1 MHz) TABLE 31-4 (16-bits SP1BRGL and SP1BRGH)
    // Baud rate counter registers SP1BRGH and SP1BRGL
    SPEN = 1; // Serial port enable  (from RC1STA register) 
    SYNC = 0; // Asynchronous operation
    SCKP = 0; // Default polarity; 
    TXEN = 0; // Enables circuitry. Note this set interrupt flag TXIF - start disabled
    // 3. Receiver 
    CREN = 0; // Enables continues receive circuitry - start disabled
}

static void enable_interrupts(void){
    // Interrupts 
    GIE = 1; // Global interrupts enable (from INTCON)
    PEIE = 1; // Peripheral interrupt enabled (from INTCON) 
    RCIE = 1; // UART RX interrupt enabled (from PIE1)  (flag -> RCIF from PIR1)    
    RCIP = 1; // Set RX interrupt priority 
}

static void init_ADC(void){
    // ------ Analogue input settings -------------  
    ADCON1 = 0b00001100; 
    ADCON2 = 0b10111000;
    ADON = 0; // Enable ADC - Start disabled

}

uint16_t readADC(uint8_t adc_channel){
    // Analogue read        
    ADCON0 = adc_channel << 2; // 1. Select channel     
    ADON = 1; // 2. Enable ADC
    delay(); // 3. Wait acquisition time = TACQ calculate EQ 22-1, e.g. 4.62 us. Estimation >5us safe 
    GO_nDONE = 1; // 4. Start conversion -> GO_nDONE = 1; 
    while(GO_nDONE);
    ADON = 0; // Disable ADC
    return (uint16_t) (ADRESH << 8) | ADRESL;
}