/*
 * File:   uart.h
 * Author: JRICARDO
 *
 * Created on 04 March 2021, 19:28
 */

#include <xc.h>
#include <stdint.h>

#define UART_BUF_SIZE	8
#define BACKSPACE		0x08
#define home()  	putchsUART("\x1b[1;1H")
#define clrscr() 	putchsUART("\x1b[2J")

//Function prototypes
void uart_en(char enable);
void putchUART(char data); // send a single char
char putsUART(char *s, char len); // send len characters
char putchsUART(char *s); //Send String
char *getsnUART(char *s, char len); //Receive String

