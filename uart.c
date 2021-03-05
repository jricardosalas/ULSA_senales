/*
 * File:   uart.c
 * Author: JRICARDO
 *
 * Created on 04 March 2021, 19:28
 */

#include "uart.h"

void uart_en(char enable){
    if(enable){
        TXEN = 1; // Enable TX UART
        CREN = 1; // Enable RX UART
    }else{
        TXEN = 0;
        CREN = 0;
    }
}

void putchUART(char data){
    while(!TXIF) // check buffer
        continue; // wait till ready
    TXREG = data; // send data
}

char putsUART(char *s, char len){
    while(len){ // loop until *s == '\0', end of string
		putchUART(*s++); //send char and point to the next one
        len--;
    }
	//putch('\r'); // terminate with a cr / line feed
	//putch('\n');
    return 0;
}

char putchsUART(char *s){
	while(*s) // loop until *s == '\0', end of string
		putchUART(*s++); //send char and point to the next one
	//putch('\r'); // terminate with a cr / line feed
	//putch('\n');
    return 0;
}

//get byte
static char getUUART(void){
    char dataRX;
    while(!RCIF);
    RCIF = 0;
    dataRX = RCREG; 
    if(OERR){ //TODO error
        CREN=0; // Disable receiver to clear OERR flag
        return 0;
    }
	return dataRX;
}

char *getsnUART(char *s, char len){
	char *p = s; // copy the buffer pointer
	// int cc = 0; // character count

	do{         	
        *s = getUUART(); // wait for a new character     

        if (*s=='\n') // line feed, ignore it
			continue;
		if (*s=='\r') // end of line, end loop
			break;
		else{
			putchUART(*s); // echo character
		}

		if ((*s == BACKSPACE) && (s > p)){
			putchUART(' '); // overwrite the last character
			putchUART(BACKSPACE);
			len++;
			s--; // back the pointer
			continue;
		}

		s++; // increment buffer pointer
		len--;
	}while(len>1); // until buffer full
		putchUART('\r'); // echo 'Enter'
		putchUART('\n');
		*s = '\0'; // null terminate the string
	return p; // return buffer pointer
}
