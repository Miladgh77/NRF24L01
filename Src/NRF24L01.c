#include "NRF24L01.h"

//defining some functions
#define MAX(x, y) (((x) >= (y)) ? (x) : (y))
#define MIN(x, y) (((x) <= (y)) ? (x) : (y))
#define _BOOL(x) (((x)>0) ? 1:0)
#define _BV(x) (1<<(x))

//static variables:
static GPIO_TypeDef* nrf24_PORT;
static uint16_t	nrf24_CSN_PIN;
static uint16_t	nrf24_CE_PIN;
static SPI_HandleTypeDef nrf24_hspi;
static UART_HandleTypeDef nrf24_huart;
static uint8_t payload_size;									//our payload size
static uint64_t pipe0_reading_address;
static bool ack_payload_available;						//waiting ack payloads
static uint8_t waiting_payload_length;						//size of waiting payload

//functions
void print(const char* msg){
	static uint8_t newline = '\n';
	int l = strlen(msg);
	HAL_UART_Transmit(&nrf24_huart, (uint8_t*) msg, l, 100);
	HAL_UART_Transmit(&nrf24_huart, &newline, 1, 100);
}
void DelayMicroSeconds(uint32_t uSec)
{
	uint32_t uSecVar = uSec;
	uSecVar = uSecVar* ((SystemCoreClock/1000000)/3);
	while(uSecVar--);
}
void csn(int state)
{
	if(state) HAL_GPIO_WritePin(nrf24_PORT,nrf24_CSN_PIN, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(nrf24_PORT, nrf24_CSN_PIN, GPIO_PIN_RESET);
}
void ce(int state)
{
	if(state) HAL_GPIO_WritePin(nrf24_PORT, nrf24_CE_PIN, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(nrf24_PORT, nrf24_CE_PIN, GPIO_PIN_RESET);
}
uint8_t read_register(uint8_t reg)
{
	uint8_t spiBuf[3];
	uint8_t retData;
	csn(0);
	spiBuf[0] = reg&0x1F;
	HAL_SPI_Transmit(&nrf24_hspi, spiBuf, 1, 100);
	HAL_SPI_Receive(&nrf24_hspi, &spiBuf[1], 1, 100);
	retData = spiBuf[1];
	csn(1);
	return retData;
}
void NRF24_read_registerN(uint8_t reg, uint8_t *buf, uint8_t len)
{
	uint8_t spiBuf[3];
	csn(0);
	spiBuf[0] = reg&0x1F;
	HAL_SPI_Transmit(&nrf24_hspi, spiBuf, 1, 100);
	HAL_SPI_Receive(&nrf24_hspi, buf, len, 100);
	csn(1);
}
void write_register(uint8_t reg, uint8_t value)
{
	uint8_t spiBuf[3];
	csn(0);
	spiBuf[0] = reg|0x20;
	spiBuf[1] = value;
	HAL_SPI_Transmit(&nrf24_hspi, spiBuf, 2, 100);
	csn(1);
}
void write_registerN(uint8_t reg, const uint8_t* buf, uint8_t len)
{
	uint8_t spiBuf[3];
	csn(0);
	spiBuf[0] = reg|0x20;
	HAL_SPI_Transmit(&nrf24_hspi, spiBuf, 1, 100);
	HAL_SPI_Transmit(&nrf24_hspi, (uint8_t*)buf, len, 100);
	csn(1);
}
void write_payload(const void* buf, uint8_t len)
{
	uint8_t wrPayloadCmd;
	csn(0);
	wrPayloadCmd = CMD_W_TX_PAYLOAD;
	HAL_SPI_Transmit(&nrf24_hspi, &wrPayloadCmd, 1, 100);
	HAL_SPI_Transmit(&nrf24_hspi, (uint8_t *)buf, len, 100);
	csn(1);
}
void read_payload(void* buf, uint8_t len)
{
	uint8_t cmdRxBuf;
	uint8_t data_len = MIN(len, getPayloadSize());
	csn(0);
	cmdRxBuf = CMD_R_RX_PAYLOAD;
	HAL_SPI_Transmit(&nrf24_hspi, &cmdRxBuf, 1, 100);
	HAL_SPI_Receive(&nrf24_hspi, buf, data_len, 100);
	csn(1);
}
void flush_tx(void)
{
	write_register(CMD_FLUSH_TX, 0xFF);
}
void flush_rx(void)
{
	write_register(CMD_FLUSH_RX, 0xFF);
}
uint8_t get_status(void)
{
	uint8_t statReg;
	statReg = read_register(REG_STATUS);
	return statReg;
}
void init(GPIO_TypeDef *nrf24PORT, uint16_t nrfCSN_Pin, uint16_t nrfCE_Pin, SPI_HandleTypeDef nrfSPI,UART_HandleTypeDef nrf24Uart)
{
	memcpy(&nrf24_hspi, &nrfSPI, sizeof(nrfSPI));
	memcpy(&nrf24_huart, &nrf24Uart, sizeof(nrf24Uart));
	nrf24_PORT = nrf24PORT;
	nrf24_CSN_PIN = nrfCSN_Pin;
	nrf24_CE_PIN = nrfCE_Pin;
	csn(1);
	ce(0);
	HAL_Delay(5);

	write_register(0x00, 0x08);
	write_register(0x01, 0x3f);
	write_register(0x02, 0x03);
	write_register(0x03, 0x03);
	write_register(0x04, 0x03);
	write_register(0x05, 0x02);
	write_register(0x06, 0x0f);
	write_register(0x07, 0x0e);
	write_register(0x08, 0x00);
	write_register(0x09, 0x00);
	uint8_t pipeAddrVar[6];
	pipeAddrVar[4]=0xE7; pipeAddrVar[3]=0xE7; pipeAddrVar[2]=0xE7; pipeAddrVar[1]=0xE7; pipeAddrVar[0]=0xE7; 
	write_registerN(0x0A, pipeAddrVar, 5);
	pipeAddrVar[4]=0xC2; pipeAddrVar[3]=0xC2; pipeAddrVar[2]=0xC2; pipeAddrVar[1]=0xC2; pipeAddrVar[0]=0xC2; 
	write_registerN(0x0B, pipeAddrVar, 5);
	write_register(0x0C, 0xC3);
	write_register(0x0D, 0xC4);
	write_register(0x0E, 0xC5);
	write_register(0x0F, 0xC6);
	pipeAddrVar[4]=0xE7; pipeAddrVar[3]=0xE7; pipeAddrVar[2]=0xE7; pipeAddrVar[1]=0xE7; pipeAddrVar[0]=0xE7; 
	write_registerN(0x10, pipeAddrVar, 5);
	write_register(0x11, 0);
	write_register(0x12, 0);
	write_register(0x13, 0);
	write_register(0x14, 0);
	write_register(0x15, 0);
	write_register(0x16, 0);
	write_register(0x17, 0x11);
	ACTIVATE_cmd();
	write_register(0x1c, 0);
	write_register(0x1d, 0);
	//printRadioSettings();
	setRetries(15, 15);
	setpower(power_0dB);
	setspeed(speed_2Mbps);
	//NRF24_setCRCLength(RF24_CRC_16);
	disableDynamicPayloads();
	setPayloadSize(32);
	resetStatus();
	setChannel(76);
	flush_tx();
	flush_rx();
	powerDown();
}
void setPayloadSize(uint8_t size)
{
	const uint8_t max_payload_size = 32;
  payload_size = MIN(size,max_payload_size);
}
uint8_t getPayloadSize(void)
{
	return payload_size;
}
void ACTIVATE_cmd(void)
{
	uint8_t cmdRxBuf[2];
	csn(0);
	cmdRxBuf[0] = CMD_ACTIVATE;
	cmdRxBuf[1] = 0x73;
	HAL_SPI_Transmit(&nrf24_hspi, cmdRxBuf, 2, 100);
	csn(1);
}
void powerUp(void)
{
	write_register(REG_CONFIG,read_register(REG_CONFIG) | _BV(BIT_PWR_UP));
}
void powerDown(void)
{
	write_register(REG_CONFIG,read_register(REG_CONFIG) & ~_BV(BIT_PWR_UP));
}
void setRetries(uint8_t delay, uint8_t count)
{
	write_register(REG_SETUP_RETR,(delay&0xf)<<BIT_ARD | (count&0xf)<<BIT_ARC);
}
void setpower( power level )
{
	uint8_t setup = read_register(REG_RF_SETUP) ;
  setup &= ~(_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH)) ;

  if ( level == power_0dB)
    setup |= (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH)) ;
  else if ( level == power_6dB )
    setup |= _BV(RF_PWR_HIGH) ;
  else if ( level == power_12dB )
    setup |= _BV(RF_PWR_LOW);
  else if ( level == power_ERROR )
    setup |= (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH)) ;
	write_register(REG_RF_SETUP,setup);
}
void setspeed(speed spd)
{
  uint8_t setup = read_register(REG_RF_SETUP) ;
  setup &= ~(_BV(RF_DR)) ;
  if ( spd == speed_2Mbps )
    setup |= _BV(RF_DR);
	write_register(REG_RF_SETUP,setup);
}
void resetStatus(void)
{
	write_register(REG_STATUS,_BV(BIT_RX_DR) | _BV(BIT_TX_DS) | _BV(BIT_MAX_RT) );
}
void setChannel(uint8_t channel)
{
	const uint8_t max_channel = 127;
  write_register(REG_RF_CH,MIN(channel,max_channel));
}
void disableDynamicPayloads(void)
{
	write_register(REG_FEATURE,read_register(REG_FEATURE) &  ~(_BV(BIT_EN_DPL)) );
	write_register(REG_DYNPD,0);
	//dynamic_payloads_enabled = false;
}
void openReadingPipe(uint8_t number, uint64_t address)
{
	if (number == 0)
    pipe0_reading_address = address;
	if(number <= 6)
	{
		if(number < 2)
			write_registerN(NRF24_ADDR_REGS[number], (uint8_t *)(&address), 5);
		else
			write_registerN(NRF24_ADDR_REGS[number], (uint8_t *)(&address), 1);
		write_register(RF24_RX_PW_PIPE[number],payload_size);
		write_register(REG_EN_RXADDR, read_register(REG_EN_RXADDR) | _BV(number));
	}
	startListening();
}
void startListening(void)
{
	write_register(REG_CONFIG, read_register(REG_CONFIG) | (1UL<<1) |(1UL <<0));
	if(pipe0_reading_address)
		write_registerN(REG_RX_ADDR_P0, (uint8_t *)(&pipe0_reading_address), 5);
	flush_tx();
	flush_rx();
	ce(1);
	DelayMicroSeconds(150);
}
void stopListening(void)
{
	ce(0);
	flush_tx();
	flush_rx();
}
bool available()
{
	uint8_t status = get_status();
  bool result = ( status & _BV(BIT_RX_DR) );
  if (result)
  {
    write_register(REG_STATUS,_BV(BIT_RX_DR) );
    if ( status & _BV(BIT_TX_DS) )
      write_register(REG_STATUS,_BV(BIT_TX_DS));
  }
  return result;
}
bool read( void* buf, uint8_t len )
{
	read_payload( buf, len );
	uint8_t rxStatus = read_register(REG_FIFO_STATUS) & _BV(BIT_RX_EMPTY);
	flush_rx();
	//NRF24_getDynamicPayloadSize();
	return rxStatus;
}
void openWritingPipe(uint64_t address)
{
	stopListening();
	write_registerN(REG_RX_ADDR_P0, (uint8_t *)(&address), 5);
  write_registerN(REG_TX_ADDR, (uint8_t *)(&address), 5);
	const uint8_t max_payload_size = 32;
  write_register(REG_RX_PW_P0,MIN(payload_size,max_payload_size));
}
void callwritepayload( const void* buf, uint8_t len)
{
  write_register(REG_CONFIG, ( read_register(REG_CONFIG) | _BV(BIT_PWR_UP) ) & ~_BV(BIT_PRIM_RX) );
  DelayMicroSeconds(150);
  write_payload( buf, len );
  ce(1);
  DelayMicroSeconds(15);
  ce(0);
}
void write( const void* buf, uint8_t len )
{
	bool retStatus;
	bool tx_ok, tx_fail;
	resetStatus();
	callwritepayload(buf,len);
  uint8_t observe_tx;
  uint8_t status;
  uint32_t sent_at = HAL_GetTick();
	const uint32_t timeout = 100;
	do
  {
    NRF24_read_registerN(REG_OBSERVE_TX,&observe_tx,1);
		status = get_status();
  }
  while( ! ( status & ( _BV(BIT_TX_DS) | _BV(BIT_MAX_RT) ) ) && ( HAL_GetTick() - sent_at < timeout ) );
  condition(&tx_ok,&tx_fail, &ack_payload_available);
	retStatus = tx_ok;
	if ( ack_payload_available )
    waiting_payload_length = getDynamicPayloadSize();
	//NRF24_available();
	flush_tx();
	if(!retStatus)
		print("Failed to transmit");
	else
		print("Transmitted successfully");
}
void condition(bool *tx_ok,bool *tx_fail,bool *rx_ready)
{
	uint8_t status = get_status();
	*tx_ok = 0;
	write_register(REG_STATUS,_BV(BIT_RX_DR) | _BV(BIT_TX_DS) | _BV(BIT_MAX_RT) );
  *tx_ok = status & _BV(BIT_TX_DS);
  *tx_fail = status & _BV(BIT_MAX_RT);
  *rx_ready = status & _BV(BIT_RX_DR);
}
uint8_t getDynamicPayloadSize(void)
{
	return read_register(CMD_R_RX_PL_WID);
}
void setAutoAckall(bool enable)
{
	if ( enable )
    write_register(REG_EN_AA, 0x3F);
  else
    write_register(REG_EN_AA, 0x00);
}
void setAutoAckPipe( uint8_t pipe, bool enable )
{
	if ( pipe <= 6 )
  {
    uint8_t en_aa = read_register( REG_EN_AA ) ;
    if( enable )
      en_aa |= _BV(pipe) ;
    else
      en_aa &= ~_BV(pipe) ;
    write_register( REG_EN_AA, en_aa ) ;
  }
}
