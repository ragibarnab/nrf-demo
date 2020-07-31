#ifndef __AVR_ATmega328P__
	#define __AVR_ATmega328P__
#endif

#ifndef BAUD
#define BAUD 9600
#endif

#ifndef F_CPU
#define F_CPU 1000000
#endif

#define USE_2X 1   // this for some reason makes the transmission accurate

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/setbaud.h>
#include "../lib/nRF24L01.h"

#define LED PB0
#define TOGGLE_LED PORTB ^= (1 << LED)

void initUSART(void){
    // set baud rate (util/setbaud.h)
    UBRR0H = UBRRH_VALUE;
    UBRR0L = UBRRL_VALUE;

    #if USE_2X
    UCSR0A |= (1 << U2X0);
    #else
    UCSR0A &= ~(1 << U2X0);
    #endif
    
    // enable transmitter 
    UCSR0B |= (1 << TXEN0);

    // set character size to 8 bit, 1 stop bit  (default)
    UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00);

}

void transmitByte(uint8_t data) {
  loop_until_bit_is_set(UCSR0A, UDRE0);  // wait for empty transmission buffer
  UDR0 = data;  
}


// any address set should be shown on the serial terminal
void testAddress(void){
    //uint8_t address[5] = {0x01, 0x02, 0x03, 0x04, 0x05};  
    //nRF_WriteByteArray(RX_ADDR_P0, address, 5);
    uint8_t ret[5];
    nRF_ReadByteArray(RX_ADDR_P0, ret, 5);
    for (uint8_t i = 0; i < 5; i++){
        transmitByte(ret[i]);
    }
}

/* tests if ART protocol is working by setting off MAX_RT interrupt
    the IRQ pin should be connected to an LED*/

void testART(void){  
    uint8_t payload[1]={0xFF};
    uint8_t data;
    data = nRF_ReadByte(NRF_CONFIG);
    transmitByte(data);
    nRF_ResetIQR();
    data = nRF_ReadByte(NRF_STATUS);
    transmitByte(data);
    nRF_TransmitPayload(payload, 1);
    while(bit_is_clear(data, MAX_RT)){  // loop until MAX_RT interrupt is flagged
        data = nRF_ReadByte(NRF_STATUS);
    }
    data = nRF_ReadByte(NRF_STATUS);
    transmitByte(data);
}


int main(void){

    // PB0 as output
    DDRB |= (1 << LED);

    // initialize USART 0 serial comm
    initUSART();

    // Init SPI
    SPI_MasterInit();

    // Setup TX/RX
    RX_Setup();

    uint8_t data;

    while (1){
        TOGGLE_LED;
        //nRF_WriteByte(, )
        data = nRF_ReadByte(NRF_CONFIG);
        transmitByte(data);
        _delay_ms(500);
    }
    

    
    return (0);
}
