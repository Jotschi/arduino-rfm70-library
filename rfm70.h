/***************************************************
 *	rfm70.h
 *	author: Heye Everts
 *	heye.everts.1@gmail.com
 * 	Copyright 2013 Heye Everts
 *
 *	This file is part of the megaRF library.
 *
 *   The megaRF library is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   The megaRF library is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with the megaRF library.  If not, see <http://www.gnu.org/licenses/>.
 *
 *
 *
 *	This library uses the RFM70 library by Odo Maletzki (odo@maletzki.net)
 *
 ***************************************************/
 
 /*
                        ▗▄▄▄▄▄▄▄▟▄▄▄▄▄▄
                     ▐██████████████████▙▄▄
                     ███████████████████████▄
                    ▐████████████████████████▌
                    ██████████████████████████
                   ▐██████████████████████████▌
                   ███████████████████████████▌
                  ▗███████████████████████████▀
                  ▟███████████████████████████████████▄▄▖
                 ▐████████████████████████████████████████▄
    ▄            ▟██████████████████████████████████████████▖
    ▝▙▄▄        ▄▟▄▄▟███████████▀▀▀▀▀▀▀▀▀▜███████████████████
      ▀███████████████▖▀▛▘▀▀▀▐███         ▝▙▄▟███████████████
        ▝▀▜███████████▙     ▀███▛▀        ▗▛████████████████▛
           ▝▀▜████ ▟█▜▜▄    ██▀▀▀▀▚       ▐███████▙▄▝▀██████
               ▝▀█▖▘    ▘▛   ▝            ▐█▀▜██████▌  ▝███▘
                  ▘                        ▌   ▝██████▄▄▞▀
                       ▌▝▀                 ▘▌▝ ▟███████▛▀
                   ▘   ▗     ▄▘▘▖          ▝  ▗███▀▜███▛▘
                      ▗▄▟██████▄▟▄         ▖ ▄████▙
                   ▗▗███████▀▀▀▀▘▝▌        ▝▜████▖▀▚▖
                   ▝███▙▖▄▄▄▖  ▝▘▄█         ▐███▀▀▘
                    ▜██▖          █         ▟███▖
                     ▜██████▄▄▄▄▄▖▀         ▟█▛▜▙
                      ▜████████▙           ▗███▙▄
                       ▜█████▖▄    ▄▀▄  ▗▄▟███████▖
                   ▗▟█▘█▄▛▜▜██▙███▙▟███████████████▙
                 ▗███▛ ▜███▙▄▄ ▐████████████████████▙▄▄▄
  ▗▄▄▄▄▄▄▄███▛▀▗▄████▌▗▙█████████████████████████████████▄
  ▐█▀  ▄▄██▛▀▗▟██████▙████████████████████████████████████▙▖
   ▐ ▐██▌     ██████████████████████████████████████████████
   ▝ ▝▀▀▘     ▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀
                This code was approved by:
   ________               __      _   __                _
  / ____/ /_  __  _______/ /__   / | / /___  __________(_)____
 / /   / __ \/ / / / ___/ //_/  /  |/ / __ \/ ___/ ___/ / ___/
/ /___/ / / / /_/ / /__/ ,<    / /|  / /_/ / /  / /  / (__  )
\____/_/ /_/\__,_/\___/_/|_|  /_/ |_/\____/_/  /_/  /_/____/

*/

#ifndef _RFM70_H_
#define _RFM70_H_

#include <Arduino.h>
#include <SPI.h>

//this is new
#define RCV_BUFFER_SIZE 32
#define SEND_DELAY 255 // delay in µs to wait after sending
//************************RFM function parameter constants********************************//
#define WITH_ACK     0x01 // parameter for sendPayload(..): send with ack expectation
#define NO_ACK       0x00 // parameter for sendPayload(..): send without ack expectation
#define MODE_PTX     0x00 // parameter for setMode(mode): set to transmitter
#define MODE_PRX     0x01 // parameter for setMode(mode): set to receiver
#define EN_AA        0x01 // parameter for configRxPipe(..): enable pipe auto ack
#define NO_AA        0x00 // parameter for configRxPipe(..): disable pipe auto ack
#define TX_DPL       0x01 // parameter for configTxPipe(..): enable dynamic payload for PTX
#define TX_SPL       0x00 // parameter for configTxPipe(..): enable static payload for PTX
#define CRC0         0x00 // parameter for configCRC(crc): disable CRC
#define CRC1         0x01 // parameter for configCRC(crc): 1 byte CRC
#define CRC2         0x02 // parameter for configCRC(crc): 2 byte CRC
#define MBPS1        0x01 // parameter for configSpeed(speed): 1Mbps
#define MBPS2        0x02 // parameter for configSpeed(speed): 2Mbps
#define DBMM10       0x00 // parameter for confRfPwr(pwr): -10 dBm
#define DBMM5        0x01 // parameter for confRfPwr(pwr): -5 dBm
#define DBM0         0x02 // parameter for confRfPwr(pwr): 0 dBm
#define DBM5         0x03 // parameter for confRfPwr(pwr): +5 dBm
#define ADR_WIDTH3   0x03 // parameter for confAdrWidth(width): 3 byte
#define ADR_WIDTH4   0x03 // parameter for confAdrWidth(width): 4 byte
#define ADR_WIDTH5   0x03 // parameter for confAdrWidth(width): 5 byte
#define PWR_OFF      0x00 // parameter for setPower(pwr): off
#define PWR_ON       0x01 // parameter for setPower(pwr): on

//************************RFM Definitions************************************************//
#define RFM70_MAX_PACKET_LEN 32// max value is 32
#define RFM70_BEGIN_INIT_WAIT_MS 3000 // pause before Init Registers
#define RFM70_END_INIT_WAIT_MS 100 // pause after init registers
#define RFM70_CS_DELAY 0 // wait ms after CS pin state change

//************************RFM COMMAND and REGISTER****************************************//
// SPI(RFM70) commands
#define RFM70_CMD_READ_REG 0x00 // Define read command to register
#define RFM70_CMD_WRITE_REG 0x20 // Define write command to register
#define RFM70_CMD_RD_RX_PLOAD 0x61 // Define RX payload command
#define RFM70_CMD_WR_TX_PLOAD 0xA0 // Define TX payload command
#define RFM70_CMD_FLUSH_TX 0xE1 // Define flush TX register command
#define RFM70_CMD_FLUSH_RX 0xE2 // Define flush RX register command
#define RFM70_CMD_REUSE_TX_PL 0xE3 // Define reuse TX payload register command
#define RFM70_CMD_W_TX_PAYLOAD_NOACK 0xb0 // Define TX payload NOACK command
#define RFM70_CMD_W_ACK_PAYLOAD 0xa8 // Define Write ack command
#define RFM70_CMD_ACTIVATE 0x50 // Define feature activation command
#define RFM70_CMD_RX_PL_WID 0x60 // Define received payload width command
#define RFM70_CMD_NOP_NOP 0xFF // Define No Operation, might be used to read status register
// SPI(RFM70) registers(addresses)
#define RFM70_REG_CONFIG 0x00 // 'Config' register address
#define RFM70_REG_EN_AA 0x01 // 'Enable Auto Acknowledgment' register address
#define RFM70_REG_EN_RXADDR 0x02 // 'Enabled RX addresses' register address
#define RFM70_REG_SETUP_AW 0x03 // 'Setup address width' register address
#define RFM70_REG_SETUP_RETR 0x04 // 'Setup Auto. Retrans' register address
#define RFM70_REG_RF_CH 0x05 // 'RF channel' register address
#define RFM70_REG_RF_SETUP 0x06 // 'RF setup' register address
#define RFM70_REG_STATUS 0x07 // 'Status' register address
#define RFM70_REG_OBSERVE_TX 0x08 // 'Observe TX' register address
#define RFM70_REG_CD 0x09 // 'Carrier Detect' register address
#define RFM70_REG_RX_ADDR_P0 0x0A // 'RX address pipe0' register address
#define RFM70_REG_RX_ADDR_P1 0x0B // 'RX address pipe1' register address
#define RFM70_REG_RX_ADDR_P2 0x0C // 'RX address pipe2' register address
#define RFM70_REG_RX_ADDR_P3 0x0D // 'RX address pipe3' register address
#define RFM70_REG_RX_ADDR_P4 0x0E // 'RX address pipe4' register address
#define RFM70_REG_RX_ADDR_P5 0x0F // 'RX address pipe5' register address
#define RFM70_REG_TX_ADDR 0x10 // 'TX address' register address
#define RFM70_REG_RX_PW_P0 0x11 // 'RX payload width, pipe0' register address
#define RFM70_REG_RX_PW_P1 0x12 // 'RX payload width, pipe1' register address
#define RFM70_REG_RX_PW_P2 0x13 // 'RX payload width, pipe2' register address
#define RFM70_REG_RX_PW_P3 0x14 // 'RX payload width, pipe3' register address
#define RFM70_REG_RX_PW_P4 0x15 // 'RX payload width, pipe4' register address
#define RFM70_REG_RX_PW_P5 0x16 // 'RX payload width, pipe5' register address
#define RFM70_REG_FIFO_STATUS 0x17 // 'FIFO Status Register' register address
#define RFM70_REG_DYNPD 0x1c // 'Enable dynamic payload length' register address
#define RFM70_REG_FEATURE 0x1d // 'Feature' register address
//interrupt status
#define RFM70_IRQ_STATUS_RX_DR 0x40 // Status bit RX_DR IRQ
#define RFM70_IRQ_STATUS_TX_DS 0x20 // Status bit TX_DS IRQ
#define RFM70_IRQ_STATUS_MAX_RT 0x10 // Status bit MAX_RT IRQ
#define RFM70_IRQ_STATUS_TX_FULL 0x01 

//FIFO_STATUS
#define RFM70_FIFO_STATUS_TX_REUSE 0x40
#define RFM70_FIFO_STATUS_TX_FULL 0x20
#define RFM70_FIFO_STATUS_TX_EMPTY 0x10

#define RFM70_FIFO_STATUS_RX_FULL 0x02
#define RFM70_FIFO_STATUS_RX_EMPTY 0x01

//Register bit masks
//#define RFM70_CONFIG_MASK_RX_DR 0x01 // Mask interrupt caused by RX_DR 1 not reflect, 0 reflect on IRQ pin
//#define RFM70_CONFIG_MASK_TX_DS 0x02 // Mask interrupt caused by RX_DS 1 not reflect, 0 reflect on IRQ pin
//#define RFM70_CONFIG_MASK_MAX_RT 0x04 // Mask interrupt caused by MAX_RT 1 not reflect, 0 reflect on IRQ pin
//#define RFM70_CONFIG_EN_CRC 0x08 // Mask interrupt caused by MAX_RT 1 not reflect, 0 reflect on IRQ pin
// ...
#define RFM70_CD_CD 0x01 // 1 = Carrier detect
#define RFM70_PIN_PRIM_RX 0x01
#define RFM70_PIN_POWER 0x02

//************************RFM SPI Constants****************************************//
#define RFM77_SPI_CLOCK_DIV4 0x00
#define RFM77_SPI_CLOCK_DIV16 0x01
#define RFM77_SPI_CLOCK_DIV64 0x02
#define RFM77_SPI_CLOCK_DIV128 0x03
#define RFM77_SPI_CLOCK_DIV2 0x04
#define RFM77_SPI_CLOCK_DIV8 0x05
#define RFM77_SPI_CLOCK_DIV32 0x06
#define RFM77_SPI_CLOCK_DIV64 0x07
#define SPI_CLOCK_MASK 0x03 // SPR1 = bit 1, SPR0 = bit 0 on SPCR
#define SPI_2XCLOCK_MASK 0x01 // SPI2X = bit 0 on SPSR
#define RFM77_DEFAULT_SPI_CLOCK_DIV RFM77_SPI_CLOCK_DIV2

//************************RFM Debug Tokens******************************************//
#define RFM70_DEBUG_WRONG_CHIP_ID 0x01
#define RFM70_DEBUG_FIFO_FULL 0x02

//************************RFM class declaraions*************************************//

class RFM70 {
public:
	RFM70(uint8_t cs, uint8_t ce, uint8_t irq, uint8_t clkdiv);
	void begin(void);

	void send(uint8_t*, uint8_t len);
	void tick(void);
	void onReceive(void (*function)(void));
	uint8_t* getRcvBuffer(void);
	uint8_t getRcvByte(uint8_t byte);
	uint8_t getPacketLength(void);

	// Beginner
	void setMode(uint8_t mode); // 0=MODE_PTX, 1=MODE_PRX
	void setChannel(uint8_t cnum);
	uint8_t getChannel(void);
	uint8_t sendPayload(uint8_t * payload, uint8_t len); // No ACK expected
	uint8_t sendPayload(uint8_t * payload, uint8_t len, uint8_t toAck); // choose 0=nAck, 1=AckRequest
	uint8_t sendAckPayload(uint8_t * payload, uint8_t len); // prx can put payload in txFIFO to be sent with ACK
	uint8_t receivePayload(uint8_t *payload);
	void flushTxFIFO();
	void flushRxFIFO();

	// Advanced
	static inline void spiSetClockDivider(uint8_t rate);
	uint8_t getMode(void); // 0=MODE_PTX, 1=MODE_PRX
	uint8_t getCarrierDetect(void);
	uint8_t getPLC(void);
	uint8_t getARC(void);
	void setPower(uint8_t pwr); // PWR_ON | PWR_OFF = 1|0
	uint8_t rxDataReceived(); // >0 = recieved on pie 1..6
	uint8_t txDataSent(); // 1 = sent a NO_ACK pkg or received an ACK pkg
	uint8_t txTimeout(); // 1 = timeout
	uint8_t txFIFOFull(); // 1 = full 0 = available
	uint8_t txFIFOEmpty(); // 1 = empty 0 = available
	uint8_t rxFIFOFull(); // 1 = full 0 = available
	uint8_t rxFIFOEmpty(); // 1 = empty 0 = available
	void confIRQ(uint8_t irq_pin, uint8_t reflectTX_DS, uint8_t reflectRX_DR,
			uint8_t reflectMAX_RT);
	void cliAll();
	void cliRxDr();
	void cliTxDs();
	void cliTimeout();

	// Expert
	uint8_t configRxPipe(uint8_t nr, uint8_t * adr, uint8_t plLen,
			uint8_t ena_aa); // pipe=1..6 plLen=0=>DPL, plLen>0=>SPL(plLen). ena_aa = EN_AA | NO_AA
	void enableRxPipe(uint8_t nr); // pipe=1..6
	void disableRxPipe(uint8_t nr); // pipe=1..6
	void configTxPipe(uint8_t * adr, uint8_t pltype); // plType = TX_SPL || TX_DPL = 0|1
	void configCRC(uint8_t crc); // crc= CRC0 | CRC1 | CRC2 = 0..2
	void configARD(uint8_t ard); // 0..15 * 250us
	void configARC(uint8_t arc); // 0..15 times
	void configSpeed(uint8_t speed); // MPPS1 | MBPS2 = 1 | 2
	void configRfPower(uint8_t pwr); // 0=-10dBm, 1=-5dBm, 2=0dBM, 3=+5dBm
	void configLnaGain(uint8_t gain); // 0 = low , 1 = high
	void confAddrWidth(uint8_t width); // ADDR_WIDTH3..5 = 3..5
	void debug(uint8_t token);

	// Expert: Hardware access
	uint8_t readRegVal(uint8_t cmd);
	void readRegBuf(uint8_t reg, uint8_t * buf, uint8_t len);
	uint8_t writeRegVal(uint8_t cmd, uint8_t val);
	void selectBank(uint8_t bank);

private:
	uint8_t _cs; //Chip seclect pin for PN532 is optional.
	uint8_t _ce;
	uint8_t _irq;
	uint8_t _clkdiv;
	SPIClass rfm70_SPI;

	void setPinState(uint8_t pin, uint8_t state);
	void setPinMode(uint8_t pin, uint8_t mode);
	void delayMs(uint8_t ms);

	//void initSPI(uint8_t cs, uint8_t clk_div);
	void initHardware();
	void initRegisters(void);

	uint8_t writeRegPgmBuf(uint8_t * cmdbuf, uint8_t len);
	uint8_t writeRegCmdBuf(uint8_t cmd, uint8_t * buf, uint8_t len);

	void setModeRX(void);
	void setModeTX(void);

	// this is new
	static void (*user_onReceive)(void);
	uint8_t rcvBuffer[RCV_BUFFER_SIZE];
	uint8_t packetLength;

};

#endif

//TODOs
// debug(); // needs an output stream and a token. Documentation for tokens.
// streaming interface and examples
// check star configuration and write example for that
// multiple RFM at one avr
// error handler for FIFO full on Tx
// transmission quality indicator
// CSMA/AD example
// Carrier detect signal does not work
// ard does not work
// check adrlen3 with setrxadr

