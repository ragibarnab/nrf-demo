#ifndef __AVR_ATmega328P__
	#define __AVR_ATmega328P__
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "nRF24L01.h"



void SPI_MasterInit(void){

    // set MOSI and SCK as output
    DDRB |= (1 << SPI_MOSI) | (1 << SPI_SCK);

    // set CSN and CE as output, PB2 needs to be an output for MSTR
    DDRB |= (1 << CSN) | (1 << CE);

    // enable SPI and set MCU as master, set SCK rate to f_osc / 64 
    SPCR |= (1 << SPE) | (1 << MSTR) | (1 << SPR1);

    // start not selected, nothing to be sent at start
    SLAVE_DESELECT;
}

uint8_t SPI_TradeByte(uint8_t byte){
    SPDR = byte;  // start sending
    loop_until_bit_is_set(SPSR, SPIF);  // wait until done
    return (SPDR);
}

uint8_t nRF_ReadByte(uint8_t reg){
    SLAVE_SELECT;
    SPI_TradeByte(R_REGISTER + reg);  // read instruction + register map address
    SPI_TradeByte(RF24_NOP);  // send dummy byte to clock out a byte
    SLAVE_DESELECT;
    return (SPDR);
}

void nRF_ReadByteArray(uint8_t reg, uint8_t ret[], uint8_t size){
    SLAVE_SELECT;
    SPI_TradeByte(R_REGISTER + reg);  // read instruction + register map address
    for(uint8_t i = 0; i < size; i++){
        ret[i] = SPI_TradeByte(RF24_NOP);  // send dummy byte to clock out a byte
    }
    SLAVE_DESELECT;
}


void nRF_WriteByte(uint8_t reg, uint8_t data){
    SLAVE_SELECT;
    SPI_TradeByte(W_REGISTER + reg);  // write instruction + register map address
    SPI_TradeByte(data);  // send data to write
    SLAVE_DESELECT;
}

void nRF_WriteByteArray(uint8_t reg, uint8_t data[], uint8_t size){
    SLAVE_SELECT;
    SPI_TradeByte(W_REGISTER + reg);  // write instruction + register map address
    for (uint8_t i = 0; i < size; i++){
        SPI_TradeByte(data[i]);  // send the bytes one at a time
    }
    SLAVE_DESELECT;
}

void nRF_TransmitPayload(uint8_t payload[], uint8_t size){
    uint8_t status = nRF_ReadByte(NRF_STATUS);

    //flush TX FIFO
    SLAVE_SELECT;
    SPI_TradeByte(FLUSH_TX);
    SLAVE_DESELECT;
    
    //Write TX payload
    SLAVE_SELECT;
    SPI_TradeByte(W_TX_PAYLOAD);
    for (uint8_t i = 0; i < size; i++){
        SPI_TradeByte(payload[i]);
    }
    SLAVE_DESELECT;

    // transmit
    PORTB |= (1 << CE);
    _delay_us(20);   // start transmit
    while(bit_is_clear(status, MAX_RT)){
        status = nRF_ReadByte(NRF_STATUS);
        if (bit_is_set(status, TX_DS)){
            break;
        }
    }
    PORTB &= ~(1 << CE);

}

void nRF_StartListening(void){
    PORTB |= (1 << CE);  // starts active RX mode
    _delay_us(130);  // standby modes -> TX/RX mode, max 130 µs
}

void nRF_StopListening(void){
    PORTB &= ~(1 << CE);  // stop listening
}

void nRF_ReceivePayload(uint8_t data[], uint8_t size){ 
    SLAVE_SELECT;
    SPI_TradeByte(R_RX_PAYLOAD);
    for (uint8_t i = 0; i < size; i++){
        data[i] = SPI_TradeByte(RF24_NOP);
    }
    SLAVE_DESELECT;
}

void nRF_ResetIQR(void){
    uint8_t status = nRF_ReadByte(NRF_STATUS);
    status |= (1 << RX_DR) | (1 << TX_DS) | (1 << MAX_RT);
    nRF_WriteByte(NRF_STATUS, status);    
}

/*
SETUP AND CUSTOMIZE TX AND RX CONFIGURATIONS HERE,
FOLLOW DATA SHEET AND THE REGISTER MAP

*/

void TX_Setup(void){

    // enable auto-acknowledgment on data pipe 0
    nRF_WriteByte(EN_AA, (1 << ENAA_P0));  

    // enable data pipe 0
    nRF_WriteByte(EN_RXADDR, (1 << ERX_P0));

    // rf address width, set to 5
    nRF_WriteByte(SETUP_AW, 0x03);

    // setup retry, ob00001111=0x0F, default wait 250+86 µs, try 15 times max
    nRF_WriteByte(SETUP_RETR, 0x0F);

    // rf channel, operating frequency to be 2.401 GHz, same as RX
    nRF_WriteByte(RF_CH, 0x01);

    // rf data rate and power setup, default settings, 1 Mbps, -18dBm 
    nRF_WriteByte(RF_SETUP, 0x00);

    uint8_t address[5] = {0x01, 0x02, 0x03, 0x04, 0x05};

    // set address of RX data pipe 0
    nRF_WriteByteArray(RX_ADDR_P0, address, 5);

    // set address TX
    nRF_WriteByteArray(TX_ADDR, address, 5);

    // payload width for data pipe 0, 1 byte to test
    nRF_WriteByte(RX_PW_P0, 0x01);  

    // disable interrupts, power up, then set as tx, 0b01111010=0x7A
    nRF_WriteByte(NRF_CONFIG, 0x0A);

    _delay_us(1500);  // power down -> standby, max 1.5 ms
}


void RX_Setup(void){
    // enable auto-acknowledgment on data pipe 0
    nRF_WriteByte(EN_AA, (1 << ENAA_P0));  

    // enable data pipe 0
    nRF_WriteByte(EN_RXADDR, (1 << ERX_P0));

    // rf address width, set to 5
    nRF_WriteByte(SETUP_AW, 0x03);

    // setup retry, ob00001111=0x0F, default wait 250+86 µs, try 15 times max
    nRF_WriteByte(SETUP_RETR, 0x0F);

    // rf channel, operating frequency to be 2.401 GHz, same as TX
    nRF_WriteByte(RF_CH, 0x01);

    // rf data rate and power setup, default settings, 1 Mbps, -18dBm 
    nRF_WriteByte(RF_SETUP, 0x00);

    uint8_t address[5] = {0x01, 0x02, 0x03, 0x04, 0x05};

    // set address of RX data pipe 0
    nRF_WriteByteArray(RX_ADDR_P0, address, 5);

    // payload width for data pipe 0, 1 byte to test
    nRF_WriteByte(RX_PW_P0, 0x01);  

    // disable TX interrupts, power up, then set as rx, 0b01111011=0x7B
    nRF_WriteByte(NRF_CONFIG, 0x0B);

    _delay_us(1500);  // power down -> standby, max 1.5 ms

}