/*
 Copyright (C) 2011 J. Coliz <maniacbug@ymail.com>

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.
 */

#include "nRF24L01.h"
#include "RF24_config.h"
#include "RF24.h"

/****************************************************************************/
#define SYMAX_PAYLOADSIZE 10
#define SYMAX_BIND_COUNT 345   // 1.5 seconds
#define FIRST_PACKET_DELAY  12000
#define PACKET_PERIOD        4000     // Timeout for callback in uSec
#define SYMAX_INITIAL_WAIT   500

#define SYMAX_FLAG_FLIP      0x01
//#define SYMAX_FLAG_RATES     0x02
#define SYMAX_FLAG_VIDEO     0x02
#define SYMAX_FLAG_PICTURE   0x04
#define SYMAX_PAYLOADSIZE 10       // receive data pipes set to this size, but unused
#define MAX_PACKET_SIZE 16   // X11,X12,X5C-1 10-byte, X5C 16-byte
#define PPM_MAX 2000
#define PPM_MIN 1000
#define PPM_THROW 1000//PPM max-PPMmin
#define X5C_CHAN2TRIM(X) ((((X) & 0x80 ? 0xff - (X) : 0x80 + (X)) >> 2) + 0x20)
static uint8_t packet[MAX_PACKET_SIZE];
static uint8_t packet_size;
static uint16_t counter;
static uint32_t packet_counter;
//static uint8_t tx_power;
static uint8_t throttle, rudder, elevator, aileron, flags;
static uint8_t rx_tx_addr[5];
static uint8_t multimedia = 0;
const uint8_t VideoBit = 7;
const uint8_t PictureBit = 7;
volatile uint16_t Servo_data[8];
uint8_t mode_select = 0;
byte scale;
enum chan_order {
  AILERON = 0,
  ELEVATOR,
  THROTTLE,
  RUDDER,
  AUX1,
  AUX2,
  AUX3,
  AUX4
};


enum {
  SYMAX_INIT1 = 0,
  SYMAX_BIND2,
  SYMAX_BIND3,
  SYMAX_DATA
};

#define MAX_RF_CHANNELS    17
static uint8_t current_chan;
static uint8_t chans[MAX_RF_CHANNELS];
static uint8_t num_rf_channels;
static uint8_t phase;
bool partial_read = false;
uint8_t data_rx_tx_addr[] = {0x3b,0xb6,0x00,0x00,0xa2};//<<--- is ok

//function decleration
uint8_t convert_channel(uint8_t num);
static uint8_t checksum(uint8_t *data);
static void SYMAX_read_controls(uint8_t* throttle, uint8_t* rudder, uint8_t* elevator, uint8_t* aileron, uint8_t* flags);
bool RF24::writeFast( const void* buf, uint8_t len, const bool multicast );
uint8_t RF24::flush_rx(void);
uint8_t RF24::write_register(uint8_t reg, const uint8_t* buf, uint8_t len);
static void symax_init2();
static void symax_init1();
static void SYMAX_send_packet(uint8_t bind);
void callback();
void  callback() {
    
}

void RF24::csn(bool mode)
{

#if defined (RF24_TINY)
	if (ce_pin != csn_pin) {
		digitalWrite(csn_pin,mode);
	} 
	else {
		if (mode == HIGH) {
			PORTB |= (1<<PINB2);  	// SCK->CSN HIGH
			delayMicroseconds(100); // allow csn to settle.
		} 
		else {
			PORTB &= ~(1<<PINB2);	// SCK->CSN LOW
			delayMicroseconds(11);  // allow csn to settle
		}
	}
	// Return, CSN toggle complete
	return;
	
#elif defined(ARDUINO) && !defined (RF24_SPI_TRANSACTIONS)
	// Minimum ideal SPI bus speed is 2x data rate
	// If we assume 2Mbs data rate and 16Mhz clock, a
	// divider of 4 is the minimum we want.
	// CLK:BUS 8Mhz:2Mhz, 16Mhz:4Mhz, or 20Mhz:5Mhz
	
      #if !defined (SOFTSPI)	
		_SPI.setBitOrder(MSBFIRST);
		_SPI.setDataMode(SPI_MODE0);
		_SPI.setClockDivider(SPI_CLOCK_DIV2);
      #endif
#elif defined (RF24_RPi)
      if(!mode)
	    _SPI.chipSelect(csn_pin);
#endif

#if !defined (RF24_LINUX)
	digitalWrite(csn_pin,mode);
	delayMicroseconds(csDelay);
#endif

}

/****************************************************************************/

void RF24::ce(bool level)
{
  //Allow for 3-pin use on ATTiny
  if (ce_pin != csn_pin) digitalWrite(ce_pin,level);
}

/****************************************************************************/

  inline void RF24::beginTransaction() {
    #if defined (RF24_SPI_TRANSACTIONS)
    _SPI.beginTransaction(SPISettings(RF24_SPI_SPEED, MSBFIRST, SPI_MODE0));
    #endif
    csn(LOW);
  }

/****************************************************************************/

  inline void RF24::endTransaction() {
    csn(HIGH);
	#if defined (RF24_SPI_TRANSACTIONS)
    _SPI.endTransaction();
	#endif
  }

/****************************************************************************/

uint8_t RF24::read_register(uint8_t reg, uint8_t* buf, uint8_t len)
{
  uint8_t status;

  #if defined (RF24_LINUX)
  beginTransaction(); //configures the spi settings for RPi, locks mutex and setting csn low
  uint8_t * prx = spi_rxbuff;
  uint8_t * ptx = spi_txbuff;
  uint8_t size = len + 1; // Add register value to transmit buffer

  *ptx++ = ( R_REGISTER | ( REGISTER_MASK & reg ) );

  while (len--){ *ptx++ = RF24_NOP; } // Dummy operation, just for reading
  
  _SPI.transfernb( (char *) spi_txbuff, (char *) spi_rxbuff, size);
  
  status = *prx++; // status is 1st byte of receive buffer

  // decrement before to skip status byte
  while ( --size ){ *buf++ = *prx++; } 
  endTransaction(); //unlocks mutex and setting csn high

#else

  beginTransaction();
  status = _SPI.transfer( R_REGISTER | ( REGISTER_MASK & reg ) );
  while ( len-- ){
    *buf++ = _SPI.transfer(0xff);
  }
  endTransaction();

#endif

  return status;
}

/****************************************************************************/

uint8_t RF24::read_register(uint8_t reg)
{
  uint8_t result;
  
  #if defined (RF24_LINUX)
	
  beginTransaction();
  
  uint8_t * prx = spi_rxbuff;
  uint8_t * ptx = spi_txbuff;	
  *ptx++ = ( R_REGISTER | ( REGISTER_MASK & reg ) );
  *ptx++ = RF24_NOP ; // Dummy operation, just for reading
  
  _SPI.transfernb( (char *) spi_txbuff, (char *) spi_rxbuff, 2);
  result = *++prx;   // result is 2nd byte of receive buffer
  
  endTransaction();
  #else

  beginTransaction();
  _SPI.transfer( R_REGISTER | ( REGISTER_MASK & reg ) );
  result = _SPI.transfer(0xff);
  endTransaction();

  #endif

  return result;
}

/****************************************************************************/

uint8_t RF24::write_register(uint8_t reg, const uint8_t* buf, uint8_t len)
{
  uint8_t status;
  cout << "hi" <<"endl;
  #if defined (RF24_LINUX) 
  beginTransaction();
  uint8_t * prx = spi_rxbuff;
  uint8_t * ptx = spi_txbuff;
  uint8_t size = len + 1; // Add register value to transmit buffer

  *ptx++ = ( W_REGISTER | ( REGISTER_MASK & reg ) );
  while ( len-- )
    *ptx++ = *buf++;
  
  _SPI.transfernb( (char *) spi_txbuff, (char *) spi_rxbuff, size);
  status = *prx; // status is 1st byte of receive buffer
  endTransaction();
  #else

  beginTransaction();
  status = _SPI.transfer( W_REGISTER | ( REGISTER_MASK & reg ) );
  while ( len-- )
    _SPI.transfer(*buf++);
  endTransaction();

  #endif

  return status;
}

/****************************************************************************/

uint8_t RF24::write_register(uint8_t reg, uint8_t value)
{
  uint8_t status;

  IF_SERIAL_DEBUG(printf_P(PSTR("write_register(%02x,%02x)\r\n"),reg,value));

  #if defined (RF24_LINUX)
    beginTransaction();
	uint8_t * prx = spi_rxbuff;
	uint8_t * ptx = spi_txbuff;
	*ptx++ = ( W_REGISTER | ( REGISTER_MASK & reg ) );
	*ptx = value ;	
  	
	_SPI.transfernb( (char *) spi_txbuff, (char *) spi_rxbuff, 2);
	status = *prx++; // status is 1st byte of receive buffer
	endTransaction();
  #else

  beginTransaction();
  status = _SPI.transfer( W_REGISTER | ( REGISTER_MASK & reg ) );
  _SPI.transfer(value);
  endTransaction();

  #endif

  return status;
}

/****************************************************************************/

uint8_t RF24::write_payload(const void* buf, uint8_t data_len, const uint8_t writeType)
{
  uint8_t status;
  const uint8_t* current = reinterpret_cast<const uint8_t*>(buf);

   data_len = rf24_min(data_len, payload_size);
   uint8_t blank_len = dynamic_payloads_enabled ? 0 : payload_size - data_len;
  
  //printf("[Writing %u bytes %u blanks]",data_len,blank_len);
  IF_SERIAL_DEBUG( printf("[Writing %u bytes %u blanks]\n",data_len,blank_len); );
  
 #if defined (RF24_LINUX)
	beginTransaction();
	uint8_t * prx = spi_rxbuff;
	uint8_t * ptx = spi_txbuff;
    uint8_t size;
	size = data_len + blank_len + 1 ; // Add register value to transmit buffer

	*ptx++ =  writeType;
    while ( data_len-- )
      *ptx++ =  *current++;
    while ( blank_len-- )
	  *ptx++ =  0;
	
	_SPI.transfernb( (char *) spi_txbuff, (char *) spi_rxbuff, size);
	status = *prx; // status is 1st byte of receive buffer
	endTransaction();

  #else

  beginTransaction();
  status = _SPI.transfer( writeType );
  while ( data_len-- ) {
    _SPI.transfer(*current++);
  }
  while ( blank_len-- ) {
    _SPI.transfer(0);
  }  
  endTransaction();

  #endif

  return status;
}

/****************************************************************************/

uint8_t RF24::read_payload(void* buf, uint8_t data_len)
{
  uint8_t status;
  uint8_t* current = reinterpret_cast<uint8_t*>(buf);

  if(data_len > payload_size) data_len = payload_size;
  uint8_t blank_len = dynamic_payloads_enabled ? 0 : payload_size - data_len;
  
  //printf("[Reading %u bytes %u blanks]",data_len,blank_len);

  IF_SERIAL_DEBUG( printf("[Reading %u bytes %u blanks]\n",data_len,blank_len); );
  
  #if defined (RF24_LINUX)
	beginTransaction();
	uint8_t * prx = spi_rxbuff;
	uint8_t * ptx = spi_txbuff;
    uint8_t size;
    size = data_len + blank_len + 1; // Add register value to transmit buffer

	*ptx++ =  R_RX_PAYLOAD;
	while(--size) 
		*ptx++ = RF24_NOP;
		
	size = data_len + blank_len + 1; // Size has been lost during while, re affect
	
	_SPI.transfernb( (char *) spi_txbuff, (char *) spi_rxbuff, size);
	
	status = *prx++; // 1st byte is status	
    
    if (data_len > 0) {
      while ( --data_len ) // Decrement before to skip 1st status byte
          *current++ = *prx++;
		
      *current = *prx;
    }
	endTransaction();
  #else

  beginTransaction();
  status = _SPI.transfer( R_RX_PAYLOAD );
  while ( data_len-- ) {
    *current++ = _SPI.transfer(0xFF);
  }
  while ( blank_len-- ) {
    _SPI.transfer(0xff);
  }
  endTransaction();

  #endif

  return status;
}

/****************************************************************************/

uint8_t RF24::flush_rx(void)
{
  return spiTrans( FLUSH_RX );
}

/****************************************************************************/

uint8_t RF24::flush_tx(void)
{
  return spiTrans( FLUSH_TX );
}

/****************************************************************************/

uint8_t RF24::spiTrans(uint8_t cmd){

  uint8_t status;
  
  beginTransaction();
  status = _SPI.transfer( cmd );
  endTransaction();
  
  return status;
}

/****************************************************************************/

uint8_t RF24::get_status(void)
{
  return spiTrans(RF24_NOP);
}

/****************************************************************************/
#if !defined (MINIMAL)
void RF24::print_status(uint8_t status)
{
  printf_P(PSTR("STATUS\t\t = 0x%02x RX_DR=%x TX_DS=%x MAX_RT=%x RX_P_NO=%x TX_FULL=%x\r\n"),
           status,
           (status & _BV(RX_DR))?1:0,
           (status & _BV(TX_DS))?1:0,
           (status & _BV(MAX_RT))?1:0,
           ((status >> RX_P_NO) & 0x07),
           (status & _BV(TX_FULL))?1:0
          );
}

/****************************************************************************/

void RF24::print_observe_tx(uint8_t value)
{
  printf_P(PSTR("OBSERVE_TX=%02x: POLS_CNT=%x ARC_CNT=%x\r\n"),
           value,
           (value >> PLOS_CNT) & 0x0F,
           (value >> ARC_CNT) & 0x0F
          );
}

/****************************************************************************/

void RF24::print_byte_register(const char* name, uint8_t reg, uint8_t qty)
{
  //char extra_tab = strlen_P(name) < 8 ? '\t' : 0;
  //printf_P(PSTR(PRIPSTR"\t%c ="),name,extra_tab);
  #if defined (RF24_LINUX)
    printf("%s\t =", name);
  #else
    printf_P(PSTR(PRIPSTR"\t ="),name);
  #endif
  while (qty--)
    printf_P(PSTR(" 0x%02x"),read_register(reg++));
  printf_P(PSTR("\r\n"));
}

/****************************************************************************/

void RF24::print_address_register(const char* name, uint8_t reg, uint8_t qty)
{

  #if defined (RF24_LINUX)
    printf("%s\t =",name);
  #else
    printf_P(PSTR(PRIPSTR"\t ="),name);
  #endif
  while (qty--)
  {
    uint8_t buffer[addr_width];
    read_register(reg++,buffer,sizeof buffer);

    printf_P(PSTR(" 0x"));
    uint8_t* bufptr = buffer + sizeof buffer;
    while( --bufptr >= buffer )
      printf_P(PSTR("%02x"),*bufptr);
  }

  printf_P(PSTR("\r\n"));
}
#endif
/****************************************************************************/

RF24::RF24(uint16_t _cepin, uint16_t _cspin):
  ce_pin(_cepin), csn_pin(_cspin), p_variant(false),
  payload_size(32), dynamic_payloads_enabled(false), addr_width(5),csDelay(5)//,pipe0_reading_address(0)
{
  pipe0_reading_address[0]=0;
  for (int i = 0; i < 8; i++) {
    Servo_data[i] = 0 ; //1500;
  }
  Servo_data[THROTTLE] = 255 ; // -127 in sign and magnitude format
  packet_counter = 0;
  flags = 0;
  phase = SYMAX_INIT1;//phase =0
}

/****************************************************************************/

#if defined (RF24_LINUX) && !defined (MRAA)//RPi constructor

RF24::RF24(uint16_t _cepin, uint16_t _cspin, uint32_t _spi_speed):
  ce_pin(_cepin),csn_pin(_cspin),spi_speed(_spi_speed),p_variant(false), payload_size(32), dynamic_payloads_enabled(false),addr_width(5)//,pipe0_reading_address(0) 
{
  pipe0_reading_address[0]=0;
}
#endif

/****************************************************************************/

void RF24::setChannel(uint8_t channel)
{
  const uint8_t max_channel = 125;
  write_register(RF_CH,rf24_min(channel,max_channel));
}

uint8_t RF24::getChannel()
{
  
  return read_register(RF_CH);
}
/****************************************************************************/

void RF24::setPayloadSize(uint8_t size)
{
  payload_size = rf24_min(size,32);
}

/****************************************************************************/

uint8_t RF24::getPayloadSize(void)
{
  return payload_size;
}

/****************************************************************************/

#if !defined (MINIMAL)

static const char rf24_datarate_e_str_0[] PROGMEM = "1MBPS";
static const char rf24_datarate_e_str_1[] PROGMEM = "2MBPS";
static const char rf24_datarate_e_str_2[] PROGMEM = "250KBPS";
static const char * const rf24_datarate_e_str_P[] PROGMEM = {
  rf24_datarate_e_str_0,
  rf24_datarate_e_str_1,
  rf24_datarate_e_str_2,
};
static const char rf24_model_e_str_0[] PROGMEM = "nRF24L01";
static const char rf24_model_e_str_1[] PROGMEM = "nRF24L01+";
static const char * const rf24_model_e_str_P[] PROGMEM = {
  rf24_model_e_str_0,
  rf24_model_e_str_1,
};
static const char rf24_crclength_e_str_0[] PROGMEM = "Disabled";
static const char rf24_crclength_e_str_1[] PROGMEM = "8 bits";
static const char rf24_crclength_e_str_2[] PROGMEM = "16 bits" ;
static const char * const rf24_crclength_e_str_P[] PROGMEM = {
  rf24_crclength_e_str_0,
  rf24_crclength_e_str_1,
  rf24_crclength_e_str_2,
};
static const char rf24_pa_dbm_e_str_0[] PROGMEM = "PA_MIN";
static const char rf24_pa_dbm_e_str_1[] PROGMEM = "PA_LOW";
static const char rf24_pa_dbm_e_str_2[] PROGMEM = "PA_HIGH";
static const char rf24_pa_dbm_e_str_3[] PROGMEM = "PA_MAX";
static const char * const rf24_pa_dbm_e_str_P[] PROGMEM = {
  rf24_pa_dbm_e_str_0,
  rf24_pa_dbm_e_str_1,
  rf24_pa_dbm_e_str_2,
  rf24_pa_dbm_e_str_3,
};

#if defined (RF24_LINUX)
static const char rf24_csn_e_str_0[] = "CE0 (PI Hardware Driven)";
static const char rf24_csn_e_str_1[] = "CE1 (PI Hardware Driven)";
static const char rf24_csn_e_str_2[] = "CE2 (PI Hardware Driven)";
static const char rf24_csn_e_str_3[] = "Custom GPIO Software Driven";
static const char * const rf24_csn_e_str_P[] = {
  rf24_csn_e_str_0,
  rf24_csn_e_str_1,
  rf24_csn_e_str_2,
  rf24_csn_e_str_3,
};
#endif

void RF24::printDetails(void)
{

#if defined (RF24_RPi)
  printf("================ SPI Configuration ================\n" );
  if (csn_pin < BCM2835_SPI_CS_NONE ){
    printf("CSN Pin  \t = %s\n",rf24_csn_e_str_P[csn_pin]);
  }else{
    printf("CSN Pin  \t = Custom GPIO%d%s\n", csn_pin,
    csn_pin==RPI_V2_GPIO_P1_26 ? " (CE1) Software Driven" : "" );
  }
  printf("CE Pin  \t = Custom GPIO%d\n", ce_pin );
  printf("Clock Speed\t = " );
	switch (spi_speed)
	{
		case BCM2835_SPI_SPEED_64MHZ : printf("64 Mhz");	break ;
		case BCM2835_SPI_SPEED_32MHZ : printf("32 Mhz");	break ;
		case BCM2835_SPI_SPEED_16MHZ : printf("16 Mhz");	break ;
		case BCM2835_SPI_SPEED_8MHZ  : printf("8 Mhz");	break ;
		case BCM2835_SPI_SPEED_4MHZ  : printf("4 Mhz");	break ;
		case BCM2835_SPI_SPEED_2MHZ  : printf("2 Mhz");	break ;
		case BCM2835_SPI_SPEED_1MHZ  : printf("1 Mhz");	break ;
		case BCM2835_SPI_SPEED_512KHZ: printf("512 KHz");	break ;
		case BCM2835_SPI_SPEED_256KHZ: printf("256 KHz");	break ;
		case BCM2835_SPI_SPEED_128KHZ: printf("128 KHz");	break ;
		case BCM2835_SPI_SPEED_64KHZ : printf("64 KHz");	break ;
		case BCM2835_SPI_SPEED_32KHZ : printf("32 KHz");	break ;
		case BCM2835_SPI_SPEED_16KHZ : printf("16 KHz");	break ;
		case BCM2835_SPI_SPEED_8KHZ  : printf("8 KHz");	break ;
		default : printf("8 Mhz");	break ;
	}
	printf("\n================ NRF Configuration ================\n");
 
#endif //Linux

  print_status(get_status());

  print_address_register(PSTR("RX_ADDR_P0-1"),RX_ADDR_P0,2);
  print_byte_register(PSTR("RX_ADDR_P2-5"),RX_ADDR_P2,4);
  print_address_register(PSTR("TX_ADDR\t"),TX_ADDR);

  print_byte_register(PSTR("RX_PW_P0-6"),RX_PW_P0,6);
  print_byte_register(PSTR("EN_AA\t"),EN_AA);
  print_byte_register(PSTR("EN_RXADDR"),EN_RXADDR);
  print_byte_register(PSTR("RF_CH\t"),RF_CH);
  print_byte_register(PSTR("RF_SETUP"),RF_SETUP);
  print_byte_register(PSTR("CONFIG\t"),NRF_CONFIG);
  print_byte_register(PSTR("DYNPD/FEATURE"),DYNPD,2);

  printf_P(PSTR("Data Rate\t = " PRIPSTR "\r\n"),pgm_read_word(&rf24_datarate_e_str_P[getDataRate()]));
  printf_P(PSTR("Model\t\t = " PRIPSTR "\r\n"),pgm_read_word(&rf24_model_e_str_P[isPVariant()]));
  printf_P(PSTR("CRC Length\t = " PRIPSTR "\r\n"),pgm_read_word(&rf24_crclength_e_str_P[getCRCLength()]));
  printf_P(PSTR("PA Power\t = " PRIPSTR "\r\n"),  pgm_read_word(&rf24_pa_dbm_e_str_P[getPALevel()]));

}

#endif
/****************************************************************************/

bool RF24::begin(void)
{

  uint8_t setup=0;
  // -----------------------NEW CODE BY OFIR AND JUDY--------------------
  //---------------------------------------------------------------------
  //--------------------------TODO: complete function--------------------
  //---------------------------------------------------------------------
  //---------------------------------------------------------------------
  //---------------------------------------------------------------------
  uint8_t bind_rx_tx_addr_x5c[] = {0x6d, 0x6a, 0x73, 0x73, 0x73};
  cout << "hi";
  read_register(NRF_STATUS);
  write_register(NRF_CONFIG, _BV(EN_CRC) | _BV(CRCO));
  write_register(EN_AA, 0x00);      // No Auto Acknoledgement
  write_register(EN_RXADDR, 0x3F);  // Enable all data pipes (even though not used?)
  write_register(SETUP_AW, 0x03);   // 5-byte RX/TX address
  write_register(SETUP_RETR, 0xff); // 4mS retransmit t/o, 15 tries (retries w/o AA?)
  write_register(RF_CH, 0x08);
  setDataRate(RF24_1MBPS);
  packet_size = 16;
  //NRF24L01_SetPower(4);
  write_register(NRF_STATUS, 0x70);     // Clear data ready, data sent, and retransmit
  write_register(OBSERVE_TX, 0x00);
  write_register(CD, 0x00);
  write_register(RX_ADDR_P2, 0xC3); // LSB byte of pipe 2 receive address
  write_register(RX_ADDR_P3, 0xC4);
  write_register(RX_ADDR_P4, 0xC5);
  write_register(RX_ADDR_P5, 0xC6);
  write_register(RX_PW_P0, SYMAX_PAYLOADSIZE);   // bytes of data payload for pipe 1
  write_register(RX_PW_P1, SYMAX_PAYLOADSIZE);
  write_register(RX_PW_P2, SYMAX_PAYLOADSIZE);
  write_register(RX_PW_P3, SYMAX_PAYLOADSIZE);
  write_register(RX_PW_P4, SYMAX_PAYLOADSIZE);
  write_register(RX_PW_P5, SYMAX_PAYLOADSIZE);
  write_register(FIFO_STATUS, 0x00); // Just in case, no real bits to write here
  write_register(TX_ADDR, bind_rx_tx_addr_x5c,5);

  read_register(NRF_STATUS);
  checkBelken();
  if (read_register(NRF_STATUS & 0x80)) {
    write_register(0x00, (uint8_t *) "\x40\x4B\x01\xE2", 4);
    write_register(0x01, (uint8_t *) "\xC0\x4B\x00\x00", 4);
    write_register(0x02, (uint8_t *) "\xD0\xFC\x8C\x02", 4);
    write_register(0x03, (uint8_t *) "\x99\x00\x39\x21", 4);
    write_register(0x04, (uint8_t *) "\xF9\x96\x82\x1B", 4);
    write_register(0x05, (uint8_t *) "\x24\x06\x7F\xA6", 4);
    write_register(0x06, (uint8_t *) "\x00\x00\x00\x00", 4);
    write_register(0x07, (uint8_t *) "\x00\x00\x00\x00", 4);
    write_register(0x08, (uint8_t *) "\x00\x00\x00\x00", 4);
    write_register(0x09, (uint8_t *) "\x00\x00\x00\x00", 4);
    write_register(0x0A, (uint8_t *) "\x00\x00\x00\x00", 4);
    write_register(0x0B, (uint8_t *) "\x00\x00\x00\x00", 4);
    write_register(0x0C, (uint8_t *) "\x00\x12\x73\x00", 4);
    write_register(0x0D, (uint8_t *) "\x46\xB4\x80\x00", 4);
    write_register(0x0E, (uint8_t *) "\x41\x10\x04\x82\x20\x08\x08\xF2\x7D\xEF\xFF", 11);
    write_register(0x04, (uint8_t *) "\xFF\x96\x82\x1B", 4);
    write_register(0x04, (uint8_t *) "\xF9\x96\x82\x1B", 4);
  }
  else {

  }
  checkBelken();
  flush_tx();
  read_register(NRF_STATUS);
  write_register(NRF_CONFIG, 0x0c);
  write_register(NRF_CONFIG, 0x0e);
  phase = SYMAX_INIT1;
  //---------------------------------------------------------------------
  //---------------------------------------------------------------------
  //---------------------------------------------------------------------
  //---------------------------------------------------------------------
  //---------------------------------------------------------------------
  //---------------------------------------------------------------------
  #if defined (RF24_LINUX)

	#if defined (MRAA)
	  GPIO();	
	  gpio.begin(ce_pin,csn_pin);	
	#endif
	
    #ifdef RF24_RPi
	  switch(csn_pin){     //Ensure valid hardware CS pin
	    case 0: break;
	    case 1: break;
	    // Allow BCM2835 enums for RPi
	    case 8: csn_pin = 0; break;
	    case 7: csn_pin = 1; break;
	    default: csn_pin = 0; break;
	  }
    #endif
	
    _SPI.begin(csn_pin);

	pinMode(ce_pin,OUTPUT);
	ce(LOW);    

	delay(100);
  
  #elif defined(LITTLEWIRE)
    pinMode(csn_pin,OUTPUT);
    _SPI.begin();
    csn(HIGH);
  #elif defined(XMEGA_D3)
	if (ce_pin != csn_pin) pinMode(ce_pin,OUTPUT);
	_SPI.begin(csn_pin);
	ce(LOW);
	csn(HIGH);
	delay(200);
  #else
    // Initialize pins
    if (ce_pin != csn_pin) pinMode(ce_pin,OUTPUT);  
  
    #if ! defined(LITTLEWIRE)
      if (ce_pin != csn_pin)
    #endif
        pinMode(csn_pin,OUTPUT);
    
    _SPI.begin();
    ce(LOW);
  	csn(HIGH);
  	#if defined (__ARDUINO_X86__)
		delay(100);
  	#endif
  #endif //Linux

  // Must allow the radio time to settle else configuration bits will not necessarily stick.
  // This is actually only required following power up but some settling time also appears to
  // be required after resets too. For full coverage, we'll always assume the worst.
  // Enabling 16b CRC is by far the most obvious case if the wrong timing is used - or skipped.
  // Technically we require 4.5ms + 14us as a worst case. We'll just call it 5ms for good measure.
  // WARNING: Delay is based on P-variant whereby non-P *may* require different timing.
  delay( 5 ) ;

  // Reset NRF_CONFIG and enable 16-bit CRC.
  write_register( NRF_CONFIG, 0x0C ) ;

  // Set 1500uS (minimum for 32B payload in ESB@250KBPS) timeouts, to make testing a little easier
  // WARNING: If this is ever lowered, either 250KBS mode with AA is broken or maximum packet
  // sizes must never be used. See documentation for a more complete explanation.
  setRetries(5,15);

  // Reset value is MAX
  //setPALevel( RF24_PA_MAX ) ;

  // check for connected module and if this is a p nRF24l01 variant
  //
  if( setDataRate( RF24_250KBPS ) )
  {
    p_variant = true ;
  }
  setup = read_register(RF_SETUP);
  /*if( setup == 0b00001110 )     // register default for nRF24L01P
  {
    p_variant = true ;
  }*/
  
  // Then set the data rate to the slowest (and most reliable) speed supported by all
  // hardware.
  setDataRate( RF24_1MBPS ) ;

  // Initialize CRC and request 2-byte (16bit) CRC
  //setCRCLength( RF24_CRC_16 ) ;

  // Disable dynamic payloads, to match dynamic_payloads_enabled setting - Reset value is 0
  toggle_features();
  write_register(FEATURE,0 );
  write_register(DYNPD,0);
  dynamic_payloads_enabled = false;

  // Reset current status
  // Notice reset and flush is the last thing we do
  write_register(NRF_STATUS,_BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );

  // Set up default configuration.  Callers can always change it later.
  // This channel should be universally safe and not bleed over into adjacent
  // spectrum.
  setChannel(76);

  // Flush buffers
  flush_rx();
  flush_tx();

  powerUp(); //Power up by default when begin() is called

  // Enable PTX, do not write CE high so radio will remain in standby I mode ( 130us max to transition to RX or TX instead of 1500us from powerUp )
  // PTX should use only 22uA of power
  write_register(NRF_CONFIG, ( read_register(NRF_CONFIG) ) & ~_BV(PRIM_RX) );
  symax_callback();
  // if setup is 0 or ff then there was no response from module
  return ( setup != 0 && setup != 0xff );
}

/****************************************************************************/

bool RF24::isChipConnected()
{
  uint8_t setup = read_register(SETUP_AW);
  if(setup >= 1 && setup <= 3)
  {
    return true;
  }

  return false;
}

/****************************************************************************/

void RF24::startListening(void)
{
 #if !defined (RF24_TINY) && ! defined(LITTLEWIRE)
  powerUp();
 #endif
  write_register(NRF_CONFIG, read_register(NRF_CONFIG) | _BV(PRIM_RX));
  write_register(NRF_STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );
  ce(HIGH);
  // Restore the pipe0 adddress, if exists
  if (pipe0_reading_address[0] > 0){
    write_register(RX_ADDR_P0, pipe0_reading_address, addr_width);	
  }else{
	closeReadingPipe(0);
  }

  // Flush buffers
  //flush_rx();
  if(read_register(FEATURE) & _BV(EN_ACK_PAY)){
	flush_tx();
  }

  // Go!
  //delayMicroseconds(100);
}

/****************************************************************************/
static const uint8_t child_pipe_enable[] PROGMEM =
{
  ERX_P0, ERX_P1, ERX_P2, ERX_P3, ERX_P4, ERX_P5
};

void RF24::stopListening(void)
{  
  ce(LOW);

  delayMicroseconds(txDelay);
  
  if(read_register(FEATURE) & _BV(EN_ACK_PAY)){
    delayMicroseconds(txDelay); //200
	flush_tx();
  }
  //flush_rx();
  write_register(NRF_CONFIG, ( read_register(NRF_CONFIG) ) & ~_BV(PRIM_RX) );
 
  #if defined (RF24_TINY) || defined (LITTLEWIRE)
  // for 3 pins solution TX mode is only left with additonal powerDown/powerUp cycle
  if (ce_pin == csn_pin) {
    powerDown();
	powerUp();
  }
  #endif
  write_register(EN_RXADDR,read_register(EN_RXADDR) | _BV(pgm_read_byte(&child_pipe_enable[0]))); // Enable RX on pipe0
  
  //delayMicroseconds(100);

}

/****************************************************************************/

void RF24::powerDown(void)
{
  ce(LOW); // Guarantee CE is low on powerDown
  write_register(NRF_CONFIG,read_register(NRF_CONFIG) & ~_BV(PWR_UP));
}

/****************************************************************************/

//Power up now. Radio will not power down unless instructed by MCU for config changes etc.
void RF24::powerUp(void)
{
   uint8_t cfg = read_register(NRF_CONFIG);

   // if not powered up then power up and wait for the radio to initialize
   if (!(cfg & _BV(PWR_UP))){
      write_register(NRF_CONFIG, cfg | _BV(PWR_UP));

      // For nRF24L01+ to go from power down mode to TX or RX mode it must first pass through stand-by mode.
	  // There must be a delay of Tpd2stby (see Table 16.) after the nRF24L01+ leaves power down mode before
	  // the CEis set high. - Tpd2stby can be up to 5ms per the 1.0 datasheet
      delay(5);
   }
}

/******************************************************************/
#if defined (FAILURE_HANDLING) || defined (RF24_LINUX)
void RF24::errNotify(){
	#if defined (SERIAL_DEBUG) || defined (RF24_LINUX)
	  printf_P(PSTR("RF24 HARDWARE FAIL: Radio not responding, verify pin connections, wiring, etc.\r\n"));
	#endif
	#if defined (FAILURE_HANDLING)
	failureDetected = 1;
	#else
	delay(5000);
	#endif
}
#endif
/******************************************************************/

//Similar to the previous write, clears the interrupt flags
bool RF24::write( const void* buf, uint8_t len, const bool multicast )
{
  for (int i = 0; i<7; i++ ){
    Servo_data[i] = buf[i];
  }
  symax_init1();
	//Start Writing
	startFastWrite(buf,len,multicast);

	//Wait until complete or failed
	#if defined (FAILURE_HANDLING) || defined (RF24_LINUX)
		uint32_t timer = millis();
	#endif 
	
	while( ! ( get_status()  & ( _BV(TX_DS) | _BV(MAX_RT) ))) { 
		#if defined (FAILURE_HANDLING) || defined (RF24_LINUX)
			if(millis() - timer > 95){			
				errNotify();
				#if defined (FAILURE_HANDLING)
				  return 0;		
				#else
				  delay(100);
				#endif
			}
		#endif
	}
    
	ce(LOW);

	uint8_t status = write_register(NRF_STATUS,_BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );

  //Max retries exceeded
  if( status & _BV(MAX_RT)){
  	flush_tx(); //Only going to be 1 packet int the FIFO at a time using this method, so just flush
  	return 0;
  }
	//TX OK 1 or 0
  return 1;
}

bool RF24::write( const void* buf, uint8_t len ){
for (int i = 0; i<7; i++ ){
    Servo_data[i] = buf[i];
  }
	//Start Writing
  symax_init1();
	return write(buf,len,0);
}
/****************************************************************************/

//For general use, the interrupt flags are not important to clear
bool RF24::writeBlocking( const void* buf, uint8_t len, uint32_t timeout )
{
	//Block until the FIFO is NOT full.
	//Keep track of the MAX retries and set auto-retry if seeing failures
	//This way the FIFO will fill up and allow blocking until packets go through
	//The radio will auto-clear everything in the FIFO as long as CE remains high

	uint32_t timer = millis();							  //Get the time that the payload transmission started

	while( ( get_status()  & ( _BV(TX_FULL) ))) {		  //Blocking only if FIFO is full. This will loop and block until TX is successful or timeout

		if( get_status() & _BV(MAX_RT)){					  //If MAX Retries have been reached
			reUseTX();										  //Set re-transmit and clear the MAX_RT interrupt flag
			if(millis() - timer > timeout){ return 0; }		  //If this payload has exceeded the user-defined timeout, exit and return 0
		}
		#if defined (FAILURE_HANDLING) || defined (RF24_LINUX)
			if(millis() - timer > (timeout+95) ){			
				errNotify();
				#if defined (FAILURE_HANDLING)
				return 0;			
                #endif				
			}
		#endif

  	}

  	//Start Writing
	startFastWrite(buf,len,0);								  //Write the payload if a buffer is clear

	return 1;												  //Return 1 to indicate successful transmission
}

/****************************************************************************/

void RF24::reUseTX(){
		write_register(NRF_STATUS,_BV(MAX_RT) );			  //Clear max retry flag
		spiTrans( REUSE_TX_PL );
		ce(LOW);										  //Re-Transfer packet
		ce(HIGH);
}

/****************************************************************************/

bool RF24::writeFast( const void* buf, uint8_t len, const bool multicast )
{
	//Block until the FIFO is NOT full.
	//Keep track of the MAX retries and set auto-retry if seeing failures
	//Return 0 so the user can control the retrys and set a timer or failure counter if required
	//The radio will auto-clear everything in the FIFO as long as CE remains high

	#if defined (FAILURE_HANDLING) || defined (RF24_LINUX)
		uint32_t timer = millis();
	#endif
	
	while( ( get_status()  & ( _BV(TX_FULL) ))) {			  //Blocking only if FIFO is full. This will loop and block until TX is successful or fail

		if( get_status() & _BV(MAX_RT)){
			//reUseTX();										  //Set re-transmit
			write_register(NRF_STATUS,_BV(MAX_RT) );			  //Clear max retry flag
			return 0;										  //Return 0. The previous payload has been retransmitted
															  //From the user perspective, if you get a 0, just keep trying to send the same payload
		}
		#if defined (FAILURE_HANDLING) || defined (RF24_LINUX)
			if(millis() - timer > 95 ){			
				errNotify();
				#if defined (FAILURE_HANDLING)
				return 0;							
				#endif
			}
		#endif
  	}
		     //Start Writing
	startFastWrite(buf,len,multicast);

	return 1;
}

bool RF24::writeFast( const void* buf, uint8_t len ){
	return writeFast(buf,len,0);
}

/****************************************************************************/

//Per the documentation, we want to set PTX Mode when not listening. Then all we do is write data and set CE high
//In this mode, if we can keep the FIFO buffers loaded, packets will transmit immediately (no 130us delay)
//Otherwise we enter Standby-II mode, which is still faster than standby mode
//Also, we remove the need to keep writing the config register over and over and delaying for 150 us each time if sending a stream of data

void RF24::startFastWrite( const void* buf, uint8_t len, const bool multicast, bool startTx){ //TMRh20
  
	//write_payload( buf,len);
	write_payload( buf, len,multicast ? W_TX_PAYLOAD_NO_ACK : W_TX_PAYLOAD ) ;
	if(startTx){
		ce(HIGH);
	}

}

/****************************************************************************/

//Added the original startWrite back in so users can still use interrupts, ack payloads, etc
//Allows the library to pass all tests
void RF24::startWrite( const void* buf, uint8_t len, const bool multicast ){

  // Send the payload

  //write_payload( buf, len );
  write_payload( buf, len,multicast? W_TX_PAYLOAD_NO_ACK : W_TX_PAYLOAD ) ;
  ce(HIGH);
  #if defined(CORE_TEENSY) || !defined(ARDUINO) || defined (RF24_SPIDEV) || defined (RF24_DUE)
	delayMicroseconds(10);
  #endif
  ce(LOW);


}

/****************************************************************************/

bool RF24::rxFifoFull(){
	return read_register(FIFO_STATUS) & _BV(RX_FULL);
}
/****************************************************************************/

bool RF24::txStandBy(){

    #if defined (FAILURE_HANDLING) || defined (RF24_LINUX)
		uint32_t timeout = millis();
	#endif
	while( ! (read_register(FIFO_STATUS) & _BV(TX_EMPTY)) ){
		if( get_status() & _BV(MAX_RT)){
			write_register(NRF_STATUS,_BV(MAX_RT) );
			ce(LOW);
			flush_tx();    //Non blocking, flush the data
			return 0;
		}
		#if defined (FAILURE_HANDLING) || defined (RF24_LINUX)
			if( millis() - timeout > 95){
				errNotify();
				#if defined (FAILURE_HANDLING)
				return 0;	
				#endif
			}
		#endif
	}

	ce(LOW);			   //Set STANDBY-I mode
	return 1;
}

/****************************************************************************/

bool RF24::txStandBy(uint32_t timeout, bool startTx){

    if(startTx){
	  stopListening();
	  ce(HIGH);
	}
	uint32_t start = millis();

	while( ! (read_register(FIFO_STATUS) & _BV(TX_EMPTY)) ){
		if( get_status() & _BV(MAX_RT)){
			write_register(NRF_STATUS,_BV(MAX_RT) );
				ce(LOW);										  //Set re-transmit
				ce(HIGH);
				if(millis() - start >= timeout){
					ce(LOW); flush_tx(); return 0;
				}
		}
		#if defined (FAILURE_HANDLING) || defined (RF24_LINUX)
			if( millis() - start > (timeout+95)){
				errNotify();
				#if defined (FAILURE_HANDLING)
				return 0;	
				#endif
			}
		#endif
	}

	
	ce(LOW);				   //Set STANDBY-I mode
	return 1;

}

/****************************************************************************/

void RF24::maskIRQ(bool tx, bool fail, bool rx){

	uint8_t config = read_register(NRF_CONFIG);
	/* clear the interrupt flags */
	config &= ~(1 << MASK_MAX_RT | 1 << MASK_TX_DS | 1 << MASK_RX_DR);
	/* set the specified interrupt flags */
	config |= fail << MASK_MAX_RT | tx << MASK_TX_DS | rx << MASK_RX_DR;
	write_register(NRF_CONFIG, config);
}

/****************************************************************************/

uint8_t RF24::getDynamicPayloadSize(void)
{
  uint8_t result = 0;

  #if defined (RF24_LINUX)  
  spi_txbuff[0] = R_RX_PL_WID;
  spi_rxbuff[1] = 0xff;
  beginTransaction();
  _SPI.transfernb( (char *) spi_txbuff, (char *) spi_rxbuff, 2);
  result = spi_rxbuff[1];  
  endTransaction();
  #else
  beginTransaction();
  _SPI.transfer( R_RX_PL_WID );
  result = _SPI.transfer(0xff);
  endTransaction();
  #endif

  if(result > 32) { flush_rx(); delay(2); return 0; }
  return result;
}

/****************************************************************************/

bool RF24::available(void)
{
  return available(NULL);
}

/****************************************************************************/

bool RF24::available(uint8_t* pipe_num)
{
  if (!( read_register(FIFO_STATUS) & _BV(RX_EMPTY) )){

    // If the caller wants the pipe number, include that
    if ( pipe_num ){
	  uint8_t status = get_status();
      *pipe_num = ( status >> RX_P_NO ) & 0x07;
  	}
  	return 1;
  }


  return 0;


}

/****************************************************************************/

void RF24::read( void* buf, uint8_t len ){

  // Fetch the payload
  read_payload( buf, len );

  //Clear the two possible interrupt flags with one command
  write_register(NRF_STATUS,_BV(RX_DR) | _BV(MAX_RT) | _BV(TX_DS) );

}

/****************************************************************************/

void RF24::whatHappened(bool& tx_ok,bool& tx_fail,bool& rx_ready)
{
  // Read the status & reset the status in one easy call
  // Or is that such a good idea?
  uint8_t status = write_register(NRF_STATUS,_BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );

  // Report to the user what happened
  tx_ok = status & _BV(TX_DS);
  tx_fail = status & _BV(MAX_RT);
  rx_ready = status & _BV(RX_DR);
}

/****************************************************************************/

void RF24::openWritingPipe(uint64_t value)
{
  // Note that AVR 8-bit uC's store this LSB first, and the NRF24L01(+)
  // expects it LSB first too, so we're good.

  write_register(RX_ADDR_P0, reinterpret_cast<uint8_t*>(&value), addr_width);
  write_register(TX_ADDR, reinterpret_cast<uint8_t*>(&value), addr_width);
  
  
  //const uint8_t max_payload_size = 32;
  //write_register(RX_PW_P0,rf24_min(payload_size,max_payload_size));
  write_register(RX_PW_P0,payload_size);
}

/****************************************************************************/
void RF24::openWritingPipe(const uint8_t *address)
{
  // Note that AVR 8-bit uC's store this LSB first, and the NRF24L01(+)
  // expects it LSB first too, so we're good.

  write_register(RX_ADDR_P0,address, addr_width);
  write_register(TX_ADDR, address, addr_width);

  //const uint8_t max_payload_size = 32;
  //write_register(RX_PW_P0,rf24_min(payload_size,max_payload_size));
  write_register(RX_PW_P0,payload_size);
}

/****************************************************************************/
static const uint8_t child_pipe[] PROGMEM =
{
  RX_ADDR_P0, RX_ADDR_P1, RX_ADDR_P2, RX_ADDR_P3, RX_ADDR_P4, RX_ADDR_P5
};
static const uint8_t child_payload_size[] PROGMEM =
{
  RX_PW_P0, RX_PW_P1, RX_PW_P2, RX_PW_P3, RX_PW_P4, RX_PW_P5
};


void RF24::openReadingPipe(uint8_t child, uint64_t address)
{
  // If this is pipe 0, cache the address.  This is needed because
  // openWritingPipe() will overwrite the pipe 0 address, so
  // startListening() will have to restore it.
  if (child == 0){
    memcpy(pipe0_reading_address,&address,addr_width);
  }

  if (child <= 6)
  {
    // For pipes 2-5, only write the LSB
    if ( child < 2 )
      write_register(pgm_read_byte(&child_pipe[child]), reinterpret_cast<const uint8_t*>(&address), addr_width);
    else
      write_register(pgm_read_byte(&child_pipe[child]), reinterpret_cast<const uint8_t*>(&address), 1);

    write_register(pgm_read_byte(&child_payload_size[child]),payload_size);

    // Note it would be more efficient to set all of the bits for all open
    // pipes at once.  However, I thought it would make the calling code
    // more simple to do it this way.
    write_register(EN_RXADDR,read_register(EN_RXADDR) | _BV(pgm_read_byte(&child_pipe_enable[child])));
  }
}

/****************************************************************************/
void RF24::setAddressWidth(uint8_t a_width){

	if(a_width -= 2){
		write_register(SETUP_AW,a_width%4);
		addr_width = (a_width%4) + 2;
	}else{
        write_register(SETUP_AW,0);
        addr_width = 2;
    }

}

/****************************************************************************/

void RF24::openReadingPipe(uint8_t child, const uint8_t *address)
{
  // If this is pipe 0, cache the address.  This is needed because
  // openWritingPipe() will overwrite the pipe 0 address, so
  // startListening() will have to restore it.
  if (child == 0){
    memcpy(pipe0_reading_address,address,addr_width);
  }
  if (child <= 6)
  {
    // For pipes 2-5, only write the LSB
    if ( child < 2 ){
      write_register(pgm_read_byte(&child_pipe[child]), address, addr_width);
    }else{
      write_register(pgm_read_byte(&child_pipe[child]), address, 1);
	}
    write_register(pgm_read_byte(&child_payload_size[child]),payload_size);

    // Note it would be more efficient to set all of the bits for all open
    // pipes at once.  However, I thought it would make the calling code
    // more simple to do it this way.
    write_register(EN_RXADDR,read_register(EN_RXADDR) | _BV(pgm_read_byte(&child_pipe_enable[child])));

  }
}

/****************************************************************************/

void RF24::closeReadingPipe( uint8_t pipe )
{
  write_register(EN_RXADDR,read_register(EN_RXADDR) & ~_BV(pgm_read_byte(&child_pipe_enable[pipe])));
}

/****************************************************************************/

void RF24::toggle_features(void)
{
    beginTransaction();
	_SPI.transfer( ACTIVATE );
    _SPI.transfer( 0x73 );
	endTransaction();
}
void checkBelken(void)
{
    beginTransaction();
  _SPI.transfer( ACTIVATE );
    _SPI.transfer( 0x53 );
  endTransaction();
}

/****************************************************************************/

void RF24::enableDynamicPayloads(void)
{
  // Enable dynamic payload throughout the system

    //toggle_features();
    write_register(FEATURE,read_register(FEATURE) | _BV(EN_DPL) );


  IF_SERIAL_DEBUG(printf("FEATURE=%i\r\n",read_register(FEATURE)));

  // Enable dynamic payload on all pipes
  //
  // Not sure the use case of only having dynamic payload on certain
  // pipes, so the library does not support it.
  write_register(DYNPD,read_register(DYNPD) | _BV(DPL_P5) | _BV(DPL_P4) | _BV(DPL_P3) | _BV(DPL_P2) | _BV(DPL_P1) | _BV(DPL_P0));

  dynamic_payloads_enabled = true;
}

/****************************************************************************/
void RF24::disableDynamicPayloads(void)
{
  // Disables dynamic payload throughout the system.  Also disables Ack Payloads

  //toggle_features();
  write_register(FEATURE, 0);


  IF_SERIAL_DEBUG(printf("FEATURE=%i\r\n",read_register(FEATURE)));

  // Disable dynamic payload on all pipes
  //
  // Not sure the use case of only having dynamic payload on certain
  // pipes, so the library does not support it.
  write_register(DYNPD, 0);

  dynamic_payloads_enabled = false;
}

/****************************************************************************/

void RF24::enableAckPayload(void)
{
  //
  // enable ack payload and dynamic payload features
  //

    //toggle_features();
    write_register(FEATURE,read_register(FEATURE) | _BV(EN_ACK_PAY) | _BV(EN_DPL) );

  IF_SERIAL_DEBUG(printf("FEATURE=%i\r\n",read_register(FEATURE)));

  //
  // Enable dynamic payload on pipes 0 & 1
  //

  write_register(DYNPD,read_register(DYNPD) | _BV(DPL_P1) | _BV(DPL_P0));
  dynamic_payloads_enabled = true;
}

/****************************************************************************/

void RF24::enableDynamicAck(void){
  //
  // enable dynamic ack features
  //
    //toggle_features();
    write_register(FEATURE,read_register(FEATURE) | _BV(EN_DYN_ACK) );

  IF_SERIAL_DEBUG(printf("FEATURE=%i\r\n",read_register(FEATURE)));


}

/****************************************************************************/

void RF24::writeAckPayload(uint8_t pipe, const void* buf, uint8_t len)
{
  const uint8_t* current = reinterpret_cast<const uint8_t*>(buf);

  uint8_t data_len = rf24_min(len,32);

  #if defined (RF24_LINUX)
    beginTransaction();
    uint8_t * ptx = spi_txbuff;
    uint8_t size = data_len + 1 ; // Add register value to transmit buffer
	*ptx++ =  W_ACK_PAYLOAD | ( pipe & 0x07 );
    while ( data_len-- ){
      *ptx++ =  *current++;
    }
	
    _SPI.transfern( (char *) spi_txbuff, size);
	endTransaction();
  #else
  beginTransaction();
  _SPI.transfer(W_ACK_PAYLOAD | ( pipe & 0x07 ) );

  while ( data_len-- )
    _SPI.transfer(*current++);
  endTransaction();
  	
  #endif  

}

/****************************************************************************/

bool RF24::isAckPayloadAvailable(void)
{
  return ! (read_register(FIFO_STATUS) & _BV(RX_EMPTY));
}

/****************************************************************************/

bool RF24::isPVariant(void)
{
  return p_variant ;
}

/****************************************************************************/

void RF24::setAutoAck(bool enable)
{
  if ( enable )
    write_register(EN_AA, 0x3F);
  else
    write_register(EN_AA, 0);
}

/****************************************************************************/

void RF24::setAutoAck( uint8_t pipe, bool enable )
{
  if ( pipe <= 6 )
  {
    uint8_t en_aa = read_register( EN_AA ) ;
    if( enable )
    {
      en_aa |= _BV(pipe) ;
    }
    else
    {
      en_aa &= ~_BV(pipe) ;
    }
    write_register( EN_AA, en_aa ) ;
  }
}

/****************************************************************************/

bool RF24::testCarrier(void)
{
  return ( read_register(CD) & 1 );
}

/****************************************************************************/

bool RF24::testRPD(void)
{
  return ( read_register(RPD) & 1 ) ;
}

/****************************************************************************/

void RF24::setPALevel(uint8_t level)
{

  uint8_t setup = read_register(RF_SETUP) & 0xF8;

  if(level > 3){  						// If invalid level, go to max PA
	  level = (RF24_PA_MAX << 1) + 1;		// +1 to support the SI24R1 chip extra bit
  }else{
	  level = (level << 1) + 1;	 		// Else set level as requested
  }


  write_register( RF_SETUP, setup |= level ) ;	// Write it to the chip
}

/****************************************************************************/

uint8_t RF24::getPALevel(void)
{

  return (read_register(RF_SETUP) & (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH))) >> 1 ;
}

/****************************************************************************/

bool RF24::setDataRate(rf24_datarate_e speed)
{
  bool result = false;
  uint8_t setup = read_register(RF_SETUP) ;

  // HIGH and LOW '00' is 1Mbs - our default
  setup &= ~(_BV(RF_DR_LOW) | _BV(RF_DR_HIGH)) ;
  
  #if defined(__arm__) || defined (RF24_LINUX) || defined (__ARDUINO_X86__)
    txDelay=250;
  #else //16Mhz Arduino
    txDelay=85;
  #endif
  if( speed == RF24_250KBPS )
  {
    // Must set the RF_DR_LOW to 1; RF_DR_HIGH (used to be RF_DR) is already 0
    // Making it '10'.
    setup |= _BV( RF_DR_LOW ) ;
  #if defined(__arm__) || defined (RF24_LINUX) || defined (__ARDUINO_X86__)
    txDelay=450;
  #else //16Mhz Arduino
	txDelay=155;
  #endif
  }
  else
  {
    // Set 2Mbs, RF_DR (RF_DR_HIGH) is set 1
    // Making it '01'
    if ( speed == RF24_2MBPS )
    {
      setup |= _BV(RF_DR_HIGH);
      #if defined(__arm__) || defined (RF24_LINUX) || defined (__ARDUINO_X86__)
      txDelay=190;
      #else //16Mhz Arduino	  
	  txDelay=65;
	  #endif
    }
  }
  write_register(RF_SETUP,setup);

  // Verify our result
  if ( read_register(RF_SETUP) == setup )
  {
    result = true;
  }
  return result;
}

/****************************************************************************/

rf24_datarate_e RF24::getDataRate( void )
{
  rf24_datarate_e result ;
  uint8_t dr = read_register(RF_SETUP) & (_BV(RF_DR_LOW) | _BV(RF_DR_HIGH));

  // switch uses RAM (evil!)
  // Order matters in our case below
  if ( dr == _BV(RF_DR_LOW) )
  {
    // '10' = 250KBPS
    result = RF24_250KBPS ;
  }
  else if ( dr == _BV(RF_DR_HIGH) )
  {
    // '01' = 2MBPS
    result = RF24_2MBPS ;
  }
  else
  {
    // '00' = 1MBPS
    result = RF24_1MBPS ;
  }
  return result ;
}

/****************************************************************************/

void RF24::setCRCLength(rf24_crclength_e length)
{
  uint8_t config = read_register(NRF_CONFIG) & ~( _BV(CRCO) | _BV(EN_CRC)) ;

  // switch uses RAM (evil!)
  if ( length == RF24_CRC_DISABLED )
  {
    // Do nothing, we turned it off above.
  }
  else if ( length == RF24_CRC_8 )
  {
    config |= _BV(EN_CRC);
  }
  else
  {
    config |= _BV(EN_CRC);
    config |= _BV( CRCO );
  }
  write_register( NRF_CONFIG, config ) ;
}

/****************************************************************************/

rf24_crclength_e RF24::getCRCLength(void)
{
  rf24_crclength_e result = RF24_CRC_DISABLED;
  
  uint8_t config = read_register(NRF_CONFIG) & ( _BV(CRCO) | _BV(EN_CRC)) ;
  uint8_t AA = read_register(EN_AA);
  
  if ( config & _BV(EN_CRC ) || AA)
  {
    if ( config & _BV(CRCO) )
      result = RF24_CRC_16;
    else
      result = RF24_CRC_8;
  }

  return result;
}

/****************************************************************************/

void RF24::disableCRC( void )
{
  uint8_t disable = read_register(NRF_CONFIG) & ~_BV(EN_CRC) ;
  write_register( NRF_CONFIG, disable ) ;
}

/****************************************************************************/
void RF24::setRetries(uint8_t delay, uint8_t count)
{
 write_register(SETUP_RETR,(delay&0xf)<<ARD | (count&0xf)<<ARC);
}
/*
function added by ofir traubas for the syma quadcopter protocol
taken from symax5c_protocol.ino file
*/
uint16_t symax_callback()
{

  switch (phase) {
    case SYMAX_INIT1:
      phase = SYMAX_BIND2;
      symax_init1();
      
      //
      return FIRST_PACKET_DELAY;
      break;

    case SYMAX_BIND2:
      counter = SYMAX_BIND_COUNT;
      phase = SYMAX_BIND3;
      SYMAX_send_packet(1);
      break;

    case SYMAX_BIND3:
      if (counter == 0) {
        symax_init2();
        phase = SYMAX_DATA;
        //PROTOCOL_SetBindState(0);
        //MUSIC_Play(MUSIC_DONE_BINDING);
      } else {
        SYMAX_send_packet(1);
        counter -= 1;
      }
      break;

    case SYMAX_DATA:
      SYMAX_send_packet(0);
      //return SYMAX_INITIAL_WAIT;
      break;
  }
  return PACKET_PERIOD;
}
//this function is called by CALLBACK function above.
static void symax_init1()
{
  // write a strange first packet to RF channel 8 ...
  uint8_t first_packet[] = {0xf9, 0x96, 0x82, 0x1b, 0x20, 0x08, 0x08, 0xf2, 0x7d, 0xef, 0xff, 0x00, 0x00, 0x00, 0x00};
  uint8_t chans_bind[] = {0x4b, 0x30, 0x40, 0x2e};
  uint8_t chans_bind_x5c[] = {0x27, 0x1b, 0x39, 0x28, 0x24, 0x22, 0x2e, 0x36,
                              0x19, 0x21, 0x29, 0x14, 0x1e, 0x12, 0x2d, 0x18
                             };


  //Serial.println("symax_init1");

  flush_tx();
  write_register(RF_CH, 0x08);
  writeFast(first_packet, 15);

  num_rf_channels = sizeof(chans_bind_x5c);
  memcpy(chans, chans_bind_x5c, num_rf_channels);
  current_chan = 0;
  packet_counter = 0;
  symax_callback();
}

static void SYMAX_send_packet(uint8_t bind)
{
  build_packet_x5c(bind);

  // clear packet status bits and TX FIFO
  write_register(NRF_STATUS, 0x70);
  write_register(NRF_CONFIG, 0x2e);
  write_register(RF_CH, chans[current_chan]);
  flush_tx();

  writeFast(packet, packet_size);

  if (packet_counter++ % 2) {   // use each channel twice
    current_chan = (current_chan + 1) % num_rf_channels;
  }

  // Check and adjust transmission power. We do this after
  // transmission to not bother with timeout after power
  // settings change -  we have plenty of time until next
  // packet.
  //NRF24L01_SetPower(7);
  symax_callback();

}

static void symax_init2()
{
  // uint8_t chans_data[] = {0x1d, 0x3d, 0x15, 0x35};
  uint8_t chans_data_x5c[] = {0x1d, 0x2f, 0x26, 0x3d, 0x15, 0x2b, 0x25, 0x24,
                              0x27, 0x2c, 0x1c, 0x3e, 0x39, 0x2d, 0x22
                             };
  //Serial.println("symax_init2");
  num_rf_channels = sizeof(chans_data_x5c);
  memcpy(chans, chans_data_x5c, num_rf_channels);
  current_chan = 0;
  packet_counter = 0;
}

static void build_packet_x5c(uint8_t bind)
{
  if (bind) {
    memset(packet, 0, packet_size);
    packet[7] = 0xae;
    packet[8] = 0xa9;
    packet[14] = 0xc0;
    packet[15] = 0x17;
  } else {
    SYMAX_read_controls(&throttle, &rudder, &elevator, &aileron, &flags);

    packet[0] = throttle;
    cout << throttle << endl;
    packet[1] = rudder;
    packet[2] = elevator ^ 0x80;  // reversed from default
    packet[3] = aileron;
    packet[4] = X5C_CHAN2TRIM(rudder ^ 0x80);// drive trims for extra control range
    packet[5] = X5C_CHAN2TRIM(elevator);
    packet[6] = X5C_CHAN2TRIM(aileron ^ 0x80);
    packet[7] = 0xae;
    packet[8] = 0xa9;
    packet[9] = 0x00;
    packet[10] = 0x00;
    packet[11] = 0x00;
    packet[12] = 0x00;
    packet[13] = 0x00;
    packet[14] = (flags & SYMAX_FLAG_VIDEO   ? 0x10 : 0x00)
                 | (flags & SYMAX_FLAG_PICTURE ? 0x08 : 0x00)
                 | (flags & SYMAX_FLAG_FLIP    ? 0x01 : 0x00)
                 | 0x04;// (flags & SYMAX_FLAG_RATES   ? 0x04 : 0x00);
    packet[15] = checksum(packet);
  }
}
static uint8_t checksum(uint8_t *data)
{
  uint8_t sum = data[0];

  for (int i = 1; i < packet_size - 1; i++)
      sum += data[i];

  return sum + 1;
}
static void SYMAX_read_controls(uint8_t* throttle, uint8_t* rudder, uint8_t* elevator, uint8_t* aileron, uint8_t* flags)
{
  // Protocol is registered AETRF, that is
  // Aileron is channel 1, Elevator - 2, Throttle - 3, Rudder - 4, Flip control - 5

  *aileron  = convert_channel(AILERON); //rool
  *elevator = convert_channel(ELEVATOR);//pitch
  *throttle = convert_channel(THROTTLE);
  *throttle = *throttle & 0x80 ? 0xff - *throttle : 0x80 + *throttle;
  *rudder   = convert_channel(RUDDER);//yaw

  // Channel 5
  if (Servo_data[AUX1] <= 1500)
    *flags &= ~SYMAX_FLAG_FLIP;
  else
    *flags |= SYMAX_FLAG_FLIP;

  // Channel 6
  //if (Servo_data[AUX2] <= 1500)
  //    *flags &= ~SYMAX_FLAG_RATES;
  // else
  //    *flags |= SYMAX_FLAG_RATES;

  // Channel 6
  if (Servo_data[AUX2] <= 128)
    // *flags &= ~SYMAX_FLAG_PICTURE;
    *flags &= ~SYMAX_FLAG_PICTURE;
  else
    *flags |= SYMAX_FLAG_PICTURE;

  // Channel 7
  if (Servo_data[AUX3] <= 128)
    *flags &= ~SYMAX_FLAG_VIDEO;
  else
    *flags |= SYMAX_FLAG_VIDEO;
  //    dbgprintf("ail %5d, ele %5d, thr %5d, rud %5d, flags 0x%x\n",
  //            *aileron, *elevator, *throttle, *rudder, *flags);
}

uint8_t convert_channel(uint8_t num) {

  if (Servo_data[num] < PPM_MIN) {
    Servo_data[num] = PPM_MIN;
  } else if (Servo_data[num] > PPM_MAX) {
    Servo_data[num] = PPM_MAX;
  }
  return (PPM_MIN + ((255) / (PPM_MAX - PPM_MIN)) * (Servo_data[num] - PPM_MIN)) // does the same thing the line below does on the arduino but for the raspberry pi
  //return (uint8_t) (map(Servo_data[num], PPM_MIN, PPM_MAX, 0, 255));
}


//ATTiny support code pulled in from https://github.com/jscrane/RF24

#if defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
// see http://gammon.com.au/spi
#	define DI   0  // D0, pin 5  Data In
#	define DO   1  // D1, pin 6  Data Out (this is *not* MOSI)
#	define USCK 2  // D2, pin 7  Universal Serial Interface clock
#	define SS   3  // D3, pin 2  Slave Select
#elif defined(__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
// these depend on the core used (check pins_arduino.h)
// this is for jeelabs' one (based on google-code core)
#	define DI   4   // PA6
#	define DO   5   // PA5
#	define USCK 6   // PA4
#	define SS   3   // PA7
#elif defined(__AVR_ATtiny2313__) || defined(__AVR_ATtiny4313__)
// these depend on the core used (check pins_arduino.h)
// tested with google-code core
#	define DI   14  // PB5
#	define DO   15  // PB6
#	define USCK 16  // PB7
#	define SS   13  // PB4
#elif defined(__AVR_ATtiny861__)
// these depend on the core used (check pins_arduino.h)
// tested with google-code core
#    define DI   9   // PB0
#    define DO   8   // PB1
#    define USCK 7   // PB2
#    define SS   6   // PB3
#endif

#if defined(RF24_TINY)

void SPIClass::begin() {

  pinMode(USCK, OUTPUT);
  pinMode(DO, OUTPUT);
  pinMode(DI, INPUT);
  USICR = _BV(USIWM0);

}

byte SPIClass::transfer(byte b) {

  USIDR = b;
  USISR = _BV(USIOIF);
  do
    USICR = _BV(USIWM0) | _BV(USICS1) | _BV(USICLK) | _BV(USITC);
  while ((USISR & _BV(USIOIF)) == 0);
  return USIDR;

}

void SPIClass::end() {}
void SPIClass::setDataMode(uint8_t mode){}
void SPIClass::setBitOrder(uint8_t bitOrder){}
void SPIClass::setClockDivider(uint8_t rate){}


#endif
