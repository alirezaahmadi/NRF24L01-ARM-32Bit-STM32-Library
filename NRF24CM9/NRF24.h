/*********************************************************
 * Nrf Library - OpenCm9.04 Compatible
 * Author: Alireza Ahamdi (A2)
 * www.AlirezaAhmadi.xyz
 * Alireza.elecat@gmail.com
 * Release date: 23/05/2017
 *********************************************************/
#ifndef NRF24_H_
#define NRF24_H_

#include <SPI.h>

#include "wirish.h"
#include <math.h>

// Register addresses -----------------
#define CONFIG      0x00
#define EN_AA       0x01
#define EN_RXADDR   0x02
#define SETUP_AW    0x03
#define SETUP_RETR  0x04
#define RF_CH       0x05
#define RF_SETUP    0x06
#define STATUS      0x07
#define OBSERVE_TX  0x08
#define RPD         0x09
#define RX_ADDR_P0  0x0A
#define RX_ADDR_P1  0x0B
#define RX_ADDR_P2  0x0C
#define RX_ADDR_P3  0x0D
#define RX_ADDR_P4  0x0E
#define RX_ADDR_P5  0x0F
#define TX_ADDR     0x10
#define RX_PW_P0    0x11
#define RX_PW_P1    0x12
#define RX_PW_P2    0x13
#define RX_PW_P3    0x14
#define RX_PW_P4    0x15
#define RX_PW_P5    0x16
#define FIFO_STATUS 0x17
#define DYNPD       0x1C
#define FEATURE     0x1D


// Bit Mnemonics ----------------------

// CONFIG
#define MASK_RX_DR  0x40
#define MASK_TX_DS  0x20
#define MASK_MAX_RT 0x10
#define EN_CRC      0x08
#define CRCO        0x04
#define PWR_UP      0x02
#define PRIM_RX     0x01
// EN_AA
#define ENAA_P5     0x20
#define ENAA_P4     0x10
#define ENAA_P3     0x08
#define ENAA_P2     0x04
#define ENAA_P1     0x02
#define ENAA_P0     0x01
// EN_RXADDR
#define ERX_P5      0x20
#define ERX_P4      0x10
#define ERX_P3      0x08
#define ERX_P2      0x04
#define ERX_P1      0x02
#define ERX_P0      0x01
// SETUP_AW
#define AW          0x01
// SETUP_RETR
#define ARD         0x10
#define ARC         0x01
// RF_SETUP
#define CONT_WAVE   0x80
#define RF_DR_LOW   0x20
#define PLL_LOCK    0x10
#define RF_DR_HIGH  0x08
#define RF_PA_HIGH  0x04
#define RF_PA_LOW   0x02
// STATUS
#define RX_DR       0x40
#define TX_DS       0x20
#define MAX_RT      0x10
#define RX_P_NO     0x02
#define TX_FULL     0x01
// OBSERVE_TX
#define PLOS_CNT    0x10
#define ARC_CNT     0x01
// FIFO_STATUS
#define TX_REUSE    0x40
#define TX_FULL_FIFO 0x20 // annoyingly this has the same mnemonic as in STATUS
#define TX_EMPTY    0x10
#define RX_FULL     0x02
#define RX_EMPTY    0x01
// DYNPD
#define DPL_P5      0x20
#define DPL_P4      0x10
#define DPL_P3      0x08
#define DPL_P2      0x04
#define DPL_P1      0x02
#define DPL_P0      0x01
// FEATURE
#define EN_DPL      0x04
#define EN_ACK_PAY  0x02
#define EN_DYN_ACK  0x01


// SPI Commands ----------------------
#define REGISTER_MASK       0x1F
#define R_REGISTER          0x00
#define W_REGISTER          0x20
#define R_RX_PAYLOAD        0x61
#define W_TX_PAYLOAD        0xA0
#define FLUSH_TX            0xE1
#define FLUSH_RX            0xE2
#define REUSE_TX_PL         0xE3
#define ACTIVATE            0x50
#define R_RX_PL_WID         0x60
#define W_ACK_PAYLOAD       0xA8
#define W_TX_PAYLOAD_NO_ACK 0xB0
#define NOP                 0xFF

typedef enum
{
	NRF24_NO_CRC = 0,
	NRF24_CRC_8BIT,
	NRF24_CRC_16BIT
} nrf24_crc_mode_e;

typedef enum
{
	NRF24_PA_MAX = 0,		//  0dBm	/	11.3mA transmit
	NRF24_PA_HIGH,			// -6dBm	/	9mA transmit
	NRF24_PA_MID,			// -12dBm	/	7.5mA transmit
	NRF24_PA_LOW			// -18dBm	/	7.0mA transmit
} nrf24_pa_level_e;

typedef enum
{
	NRF24_250KBPS = 0,
	NRF24_1MBPS,
	NRF24_2MBPS
} nrf24_datarate_e;

typedef enum
{
	NRF24_MODE_POWER_DOWN = 0,
	NRF24_MODE_STANDBY1,
	NRF24_MODE_STANDBY2,
	NRF24_MODE_RX,
	NRF24_MODE_TX
} nrf24_mode_e;

class NRF24
{
	public:
		// Netmask should be something "random"
		// All nodes that talk to eachother need to have the same netmask
		// 10101010.. can continue on preamble and cause missed packets
		// 11110000.. with only one logic transition can cause missed packets
		bool begin(uint8 cePin, uint8 csnPin, uint32 netmask = 0xC2C2C2C2);

		// Logical RF channels
		void setAddress(uint8 address);
		int8 listenToAddress(uint8 address);

		// Physical RF channel
		void setChannel(uint8 channel);
		uint8 getChannel();

		void setDataRate(nrf24_datarate_e dataRate);

		void setPowerAmplificationLevel(nrf24_pa_level_e level);
		nrf24_pa_level_e getPowerAmplificationLevel();

		// Broadcast to any listeners. As multiple may be listening ACK is disabled and there's no way to know if the message was actually received
		// See send() for transmitting more reliably but only to single node
		// returns false if send failed for some reason
		bool broadcast(uint8 *data, uint8 length);
		bool broadcast(char *message);
		//bool broadcast_P(const char FlashStringHelper *message);

		bool send(uint8 targetAddress, uint8 *data, uint8 length, uint8 *numAttempts = NULL);
		int8 send(uint8 targetAddress, uint8 *data, uint8 length, uint8 *responseBuffer, uint8 bufferSize, uint8 *numAttempts = NULL);
		bool send(uint8 targetAddress, char *message);

		bool queueResponse(uint8 *data, uint8 length);

		uint8 available(uint8 *listener = NULL);
		uint8 read(uint8 *buf, uint8 bufferSize);		// raw data
		uint8 read(char *buf, uint8 bufferSize);		// makes sure data is 0 terminated

		void setActive(bool active);
		bool getActive();

		nrf24_mode_e getCurrentMode();

		void startListening();
		void stopListening();

		void setRetries(uint8 delay, uint8 count);
		void setCRCMode(nrf24_crc_mode_e mode);

		void setACKEnabled(bool ack = true);

		uint8 ownAddress;
	
	
		void ceHigh()  { digitalWrite(CePin,HIGH);    };
		void ceLow()   { digitalWrite(CePin,LOW);   };
		bool ceIsHigh(){ return *ceInput & ceBitMask; };
		void csnHigh() { digitalWrite(CsnPin,HIGH); };
		void csnLow()  { digitalWrite(CsnPin,LOW); };

		uint8 readRegister(uint8 reg);
		void writeRegister(uint8 reg, uint8 value);
		void writeRegister(uint8 reg, uint8 *value, uint8 numBytes);

		bool transmit(uint8 targetAddress, uint8 *data, uint8 length, bool ack = true);

		void assembleFullAddress(uint8 address, uint8 buf[5]);

		void flushTX();
		void flushRX();
private:
		
		bool listening;
		uint8 previousTXAddress;
		uint8 previousRXAddress;

		bool ackEnabled;

		uint32 netmask;
		uint8 numPipes;
		int8 previousPipe;


		volatile uint8 *cePort;
		volatile uint8 *ceInput;
		uint8 ceBitMask;
		volatile uint8 *csnPort;
		uint8 csnBitMask;

		uint8 CePin;
		uint8 CsnPin;
};

extern NRF24 radio;

#endif // NRF24_H_