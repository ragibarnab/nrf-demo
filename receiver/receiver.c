#ifndef __AVR_ATmega328P__
	#define __AVR_ATmega328P__
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "../lib/nRF24L01.h"

#define LED PB0
#define dataSize 1

uint8_t data[dataSize] = {0x00};


// if IRQ is active (low), INT0 should trigger
ISR(INT0_vect){
    nRF_StopListening();
    nRF_ReceivePayload(data, dataSize);
    nRF_ResetIQR(); 

    /* Check if data is sent correctly,
    should have same value as sent from TX */
    if (data[0] == 0xFF){   
        PORTB |= (1 << LED);
        _delay_ms(100);
        PORTB &= ~(1 << LED);
        data[0] = 0x00;   // reset 
    }
    nRF_StartListening();
}


int main(void)
{

    //setup LED
    DDRB |= (1 << LED);

    //setup interrupts
    EIMSK |= (1 << INT0);
    
    sei();

    // initialize SPI for nRF
    SPI_MasterInit(); 

    // setup RX
    RX_Setup();

    nRF_StartListening();
    nRF_ResetIQR();
    
    while(1){
        
    }
    
    return (0);
}
