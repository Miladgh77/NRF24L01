/* Define to prevent recursive inclusion -------------------------------*/
#ifndef __NRF24L01__
#define __NRF24L01__
/* Includes ------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4xx_hal_gpio.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "stdbool.h"

//useful BITS(for shift and etc...)
#define MASK_RX_DR  		6
#define MASK_TX_DS  		5
#define MASK_MAX_RT 		4
#define BIT_EN_CRC      3
#define BIT_CRCO        2
#define BIT_PWR_UP      1
#define BIT_PRIM_RX     0
#define BIT_ENAA_P5     5
#define BIT_ENAA_P4     4
#define BIT_ENAA_P3     3
#define BIT_ENAA_P2     2
#define BIT_ENAA_P1     1
#define BIT_ENAA_P0     0
#define BIT_ERX_P5      5
#define BIT_ERX_P4      4
#define BIT_ERX_P3      3
#define BIT_ERX_P2      2
#define BIT_ERX_P1      1
#define BIT_ERX_P0      0
#define BIT_AW          0
#define BIT_ARD         4
#define BIT_ARC         0
#define BIT_PLL_LOCK    4
#define BIT_RF_DR       3
#define BIT_RF_PWR      6
#define BIT_RX_DR       6
#define BIT_TX_DS       5
#define BIT_MAX_RT      4
#define BIT_RX_P_NO     1
#define BIT_TX_FULL     0
#define BIT_PLOS_CNT    4
#define BIT_ARC_CNT     0
#define BIT_TX_REUSE    6
#define BIT_FIFO_FULL   5
#define BIT_TX_EMPTY    4
#define BIT_RX_FULL     1
#define BIT_RX_EMPTY    0
#define BIT_DPL_P5	    5
#define BIT_DPL_P4	    4
#define BIT_DPL_P3	    3
#define BIT_DPL_P2	    2
#define BIT_DPL_P1	    1
#define BIT_DPL_P0	    0
#define BIT_EN_DPL	    2
#define BIT_EN_ACK_PAY  1
#define BIT_EN_DYN_ACK  0
#define RF_DR				 		3
#define RF_PWR_LOW  		1
#define RF_PWR_HIGH 		2
//command words(according to SPI commands(table16)):
#define CMD_R_REGISTER    0x00
#define CMD_W_REGISTER    0x20
#define CMD_REGISTER_MASK 0x1F
#define CMD_ACTIVATE      0x50
#define CMD_R_RX_PL_WID   0x60
#define CMD_R_RX_PAYLOAD  0x61
#define CMD_W_TX_PAYLOAD  0xA0
#define CMD_W_ACK_PAYLOAD 0xA8
#define CMD_FLUSH_TX      0xE1
#define CMD_FLUSH_RX      0xE2
#define CMD_REUSE_TX_PL   0xE3
#define CMD_NOP           0xFF
//registers and memory map(according to registermap(table24)):
#define REG_CONFIG      0x00
#define REG_EN_AA       0x01
#define REG_EN_RXADDR   0x02
#define REG_SETUP_AW    0x03
#define REG_SETUP_RETR  0x04
#define REG_RF_CH       0x05
#define REG_RF_SETUP    0x06
#define REG_STATUS      0x07
#define REG_OBSERVE_TX  0x08
#define REG_CD          0x09
#define REG_RX_ADDR_P0  0x0A
#define REG_RX_ADDR_P1  0x0B
#define REG_RX_ADDR_P2  0x0C
#define REG_RX_ADDR_P3  0x0D
#define REG_RX_ADDR_P4  0x0E
#define REG_RX_ADDR_P5  0x0F
#define REG_TX_ADDR     0x10
#define REG_RX_PW_P0    0x11
#define REG_RX_PW_P1    0x12
#define REG_RX_PW_P2    0x13
#define REG_RX_PW_P3    0x14
#define REG_RX_PW_P4    0x15
#define REG_RX_PW_P5    0x16
#define REG_FIFO_STATUS 0x17
#define REG_DYNPD	    	0x1C
#define REG_FEATURE	    0x1D

//power struct
typedef enum { 
	power_18dB = 0,
	power_12dB,
	power_6dB,
	power_0dB,
	power_ERROR 
}power ;
//speed struct
typedef enum { 
	speed_1Mbps = 0,
	speed_2Mbps,
}speed;
//pipe address registers
static const uint8_t NRF24_ADDR_REGS[7] = {
		REG_RX_ADDR_P0,
		REG_RX_ADDR_P1,
		REG_RX_ADDR_P2,
		REG_RX_ADDR_P3,
		REG_RX_ADDR_P4,
		REG_RX_ADDR_P5,
		REG_TX_ADDR
};
//RX_PW_Px registers addresses
static const uint8_t RF24_RX_PW_PIPE[6] = {
		REG_RX_PW_P0, 
		REG_RX_PW_P1,
		REG_RX_PW_P2,
		REG_RX_PW_P3,
		REG_RX_PW_P4,
		REG_RX_PW_P5
};

//functions:

//print function
void print(const char* msg);
//usec delay:
void DelayMicroSeconds(uint32_t uSec);
//cs function:
void csn(int state);
//ce function:
void ce(int state);
//reading one byte from a register
uint8_t read_register(uint8_t reg);
//reading multiple bytes from a register
void NRF24_read_registerN(uint8_t reg, uint8_t *buf, uint8_t len);
//write one byte in register function:
void write_register(uint8_t reg, uint8_t value);
//write multiple bytes in register function:
void write_registerN(uint8_t reg, const uint8_t* buf, uint8_t len);
//kernel of the write function to send data
void write_payload(const void* buf, uint8_t len);
//kernel of the read function to receive data
void read_payload(void* buf, uint8_t len);
//flushing TX buffer
void flush_tx(void);
//flushing RX buffer
void flush_rx(void);
//finding the status with status register
uint8_t get_status(void);
//initial function
void init(GPIO_TypeDef *nrf24PORT, uint16_t nrfCSN_Pin, uint16_t nrfCE_Pin, SPI_HandleTypeDef* nrfSPI,UART_HandleTypeDef* nrf24Uart);
//setting the payload size
void setPayloadSize(uint8_t size);
//finding out the payload size
uint8_t getPayloadSize(void);
//active cmd(activating necessary registers)
void ACTIVATE_cmd(void);
//powering up
void powerUp(void);
//powering down
void powerDown(void);
//retrying to send failed datas
void setRetries(uint8_t delay, uint8_t count);
//setting power
void setpower( power level );
//setting speed
void setspeed(speed spd);
//reset status(clearing RX_DR & TX_DS & MAX_RT)
void resetStatus(void);
//setting channel
void setChannel(uint8_t channel);
//disable dynamic payloads
void disableDynamicPayloads(void);
//openning a pipe to listen
void openReadingPipe(uint8_t number, uint64_t address);
//start listening ...
void startListening(void);
//stop listening!
void stopListening(void);
//is data available?!(finding data availability and finding the mode of autoack)
bool available(void);
//reading function
bool read( void* buf, uint8_t len );
//openning a oioe to write and also setting pipe0 for autoack
void openWritingPipe(uint64_t address);
//calling write payload to write
void callwritepayload( const void* buf, uint8_t len);
//writing function
void write( const void* buf, uint8_t len );
//geting the status of sent data,ready data and failed data
void condition(bool *tx_ok,bool *tx_fail,bool *rx_ready);
//getting the remaining payload size
uint8_t getDynamicPayloadSize(void);
//enable/disable autoack for all pipes
void setAutoAckall(bool enable);
//enabling/disabling autoack for a certain pipe
void setAutoAckPipe( uint8_t pipe, bool enable );

#endif
