#ifndef __AVR_ATmega328P__
	#define __AVR_ATmega328P__
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "../lib/nRF24L01.h"

#define LED PB0
#define payloadSize  1
uint8_t payload[payloadSize] = {0xFF};


ISR(INT0_vect){
    // toggle LED when logic changes
    PORTB |= (1 << LED);
    _delay_ms(100);  // debouncing
    nRF_ResetIQR();
    nRF_TransmitPayload(payload, payloadSize);
    PORTB &= ~(1 << LED);
}

int main(void)
{
    // setup ext interrupt 0 for button press
    EIMSK |= (1 << INT0);  
    //EICRA |= (1 << ISC00);

    //setup LED and pullup
    DDRB |= (1 << LED);
    PORTD |= (1 << PORTD2);

    //set interrupt
    sei();

    // initialize SPI for nRF
    SPI_MasterInit(); 
    
    // setup TX
    TX_Setup();

    while(1){

    }

    return (0);
}
