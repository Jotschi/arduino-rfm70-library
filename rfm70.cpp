/***************************************************
 *	rfm70.cpp
 *	Original author: Heye Everts
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
 *	This library uses the RFM70 library by Odo Maletzki (odo@maletzki.net)
 *	The original library was updated to use the SPI.h methods instead of
 *	handling the SPI init code inside of here.
 *
 ***************************************************/
#include <Arduino.h>
#include <SPI.h>
#include "rfm70.h"
#include <util/delay.h>

void (*RFM70::user_onReceive)(void);

///////////////////////////////////////////////////////////////////////////////
// Register initialization values and command macros //
///////////////////////////////////////////////////////////////////////////////

//************ Address definition commands
const uint8_t PROGMEM RFM70_cmd_adrRX0[] = { (0x20|0x0A), 0x34, 0x43, 0x10, 0x10, 0x01 };
const uint8_t PROGMEM RFM70_cmd_adrTX[] =  { (0x20|0x10), 0x34, 0x43, 0x10, 0x10, 0x01 };
const uint8_t PROGMEM RFM70_cmd_adrRX1[] = { (0x20|0x0B), 0x35, 0x43, 0x10, 0x10, 0x02 };

//************ Bank0 register initialization commands
const uint8_t PROGMEM RFM70_bank0Init[][2] = {
	// address data
	{ (0x20|0x00), 0x0F }, //Disable CRC ,CRC=1byte, POWER UP, TX
	{ (0x20|0x01), 0x3F }, //Enable auto acknowledgement data pipe0-5
	{ (0x20|0x02), 0x3F }, //Enable RX Addresses pipe0-5
	{ (0x20|0x03), 0x03 }, //RX/TX address field width 5byte
	{ (0x20|0x04), 0x08 }, //x = 250 ms = 4000ms, y = 15 tries
	{ (0x20|0x05), 0x17 }, //channel = 0x17
	{ (0x20|0x06), 0x3F }, //air data rate-2M,out power 5dbm,setup LNA gain high (0dBM)
	{ (0x20|0x07), 0x07 }, //
	{ (0x20|0x08), 0x00 }, //
	{ (0x20|0x09), 0x00 }, //
	{ (0x20|0x0C), 0xc3 }, //LSB Addr pipe 2
	{ (0x20|0x0D), 0xc4 }, //LSB Addr pipe 3
	{ (0x20|0x0E), 0xc5 }, //LSB Addr pipe 4
	{ (0x20|0x0F), 0xc6 }, //LSB Addr pipe 5
	{ (0x20|0x11), 0x20 }, //Payload len pipe0
	{ (0x20|0x12), 0x20 }, //Payload len pipe0
	{ (0x20|0x13), 0x20 }, //Payload len pipe0
	{ (0x20|0x14), 0x20 }, //Payload len pipe0
	{ (0x20|0x15), 0x20 }, //Payload len pipe0
	{ (0x20|0x16), 0x20 }, //Payload len pipe0
	{ (0x20|0x17), 0x20 }, //Payload len pipe0
	{ (0x20|0x1C), 0x3F }, //Enable dynamic payload legth data pipe0-5
	{ (0x20|0x1D), 0x07 } //Enables Dynamic Payload Length,Enables Payload with ACK
};

//************ Bank1 register initialization commands
const uint8_t PROGMEM RFM70_bank1Init[][5] = {
	// address data
	{ (0x20|0x00), 0x40, 0x4B, 0x01, 0xE2 },
	{ (0x20|0x01), 0xC0, 0x4B, 0x00, 0x00 },
	{ (0x20|0x02), 0xD0, 0xFC, 0x8C, 0x02 },
	{ (0x20|0x03), 0x99, 0x00, 0x39, 0x41 },
	{ (0x20|0x04), 0xb9, 0x9E, 0x86, 0x0B }, // b9? f9?
	{ (0x20|0x05), 0x24, 0x06, 0x7F, 0xA6 },
	{ (0x20|0x06), 0x00, 0x00, 0x00, 0x00 },
	{ (0x20|0x07), 0x00, 0x00, 0x00, 0x00 },
	{ (0x20|0x08), 0x00, 0x00, 0x00, 0x00 },
	{ (0x20|0x09), 0x00, 0x00, 0x00, 0x00 },
	{ (0x20|0x0a), 0x00, 0x00, 0x00, 0x00 },
	{ (0x20|0x0b), 0x00, 0x00, 0x00, 0x00 },
	{ (0x20|0x0C), 0x00, 0x12, 0x73, 0x00 },
	{ (0x20|0x0D), 0x36, 0xb4, 0x80, 0x00 }
};

//************ Bank1 register 14 initialization commands
const uint8_t PROGMEM RFM70_bank1R0EInit[] = {
	(0x20|0x0E), 0x41, 0x20, 0x08, 0x04, 0x81, 0x20, 0xCF, 0xF7, 0xFE, 0xFF, 0xFF
};

//************ other commands: { <command>, <data>, ... }
// switch Register Bank
const uint8_t PROGMEM RFM70_cmd_switch_cfg[] = { 0x50, 0x53 };
// flush RX FIFO
const uint8_t PROGMEM RFM70_cmd_flush_rx[] = { 0xe2, 0x00 };
// flush TX FIFO
const uint8_t PROGMEM RFM70_cmd_flush_tx[] = { 0xe1, 0x00 };
// Activation command
const uint8_t PROGMEM RFM70_cmd_activate[] = { 0x50, 0x73 };
//assosciated with set1[4]!
const uint8_t PROGMEM RFM70_cmd_tog1[] = { (0x20|0x04), 0xd9 | 0x06, 0x9e, 0x86, 0x0b };
//assosciated with set1[4]!
const uint8_t PROGMEM RFM70_cmd_tog2[] = { (0x20|0x04), 0xd9 & ~0x06, 0x9e, 0x86, 0x0b };

/**
 * Construct PN532 object and construct PN532's own SPI interface object
 * at the same time.
 **/
RFM70::RFM70(uint8_t cs, uint8_t ce, uint8_t irq, uint8_t clkdiv) :
		rfm70_SPI() {
	_cs = cs;
	_ce = ce;
	_irq = irq;
	_clkdiv = clkdiv;
}

void RFM70::begin() {

	initHardware();

	rfm70_SPI.begin();
	rfm70_SPI.setDataMode(SPI_MODE0);
	rfm70_SPI.setBitOrder(MSBFIRST);

	// Set the SPI frequency to be one sixteenth of the frequency of the system clock
	rfm70_SPI.setClockDivider(RFM77_DEFAULT_SPI_CLOCK_DIV);
	digitalWrite(_cs, LOW);
	delay(1000);

	delayMs(RFM70_BEGIN_INIT_WAIT_MS);
	initRegisters();

	confAddrWidth(3);
	//max power
	configRfPower(3);
	//gain
	configLnaGain(1);
	//default mode is rx mode
	setMode(MODE_PRX);

}

void RFM70::initHardware() {

	pinMode(_cs, OUTPUT);
	digitalWrite(_cs, HIGH);

	// Setup RFM Port directions
	pinMode(_ce, OUTPUT);
	digitalWrite(_ce, LOW);

	if (_irq != -1) {
		pinMode(_irq, INPUT);
	}

}

void RFM70::initRegisters() {

	// Init bank 0 registers
	selectBank(0);

	// The last two regs in the bank0Init list will be handled later
	for (int i = 0; i < 20; i++) {
		writeRegVal(pgm_read_byte(&RFM70_bank0Init[i][0]),
				pgm_read_byte(&RFM70_bank0Init[i][1]));
	}

	// Init address registers in bank 0
	writeRegPgmBuf((uint8_t *) RFM70_cmd_adrRX0, sizeof(RFM70_cmd_adrRX0));
	writeRegPgmBuf((uint8_t *) RFM70_cmd_adrRX1, sizeof(RFM70_cmd_adrRX1));
	writeRegPgmBuf((uint8_t *) RFM70_cmd_adrTX, sizeof(RFM70_cmd_adrTX));

	// Activate Feature register
	if (!readRegVal(RFM70_REG_FEATURE)) {
		writeRegPgmBuf((uint8_t *) RFM70_cmd_activate, sizeof(RFM70_cmd_activate));
	}

	// Now set Registers 1D and 1C
	writeRegVal(pgm_read_byte(&RFM70_bank0Init[22][0]), pgm_read_byte(&RFM70_bank0Init[22][1]));
	writeRegVal(pgm_read_byte(&RFM70_bank0Init[21][0]), pgm_read_byte(&RFM70_bank0Init[21][1]));

	// Init bank 1 registers
	selectBank(1);

	for (int i = 0; i < 14; i++) {
		writeRegPgmBuf((uint8_t *) RFM70_bank1Init[i], sizeof(RFM70_bank1Init[i]));
	}

	// set ramp curve
	writeRegPgmBuf((uint8_t *) RFM70_bank1R0EInit, sizeof(RFM70_bank1R0EInit));

	// do we have to toggle some bits here like in the example code?
	writeRegPgmBuf((uint8_t *) RFM70_cmd_tog1, sizeof(RFM70_cmd_tog1));
	writeRegPgmBuf((uint8_t *) RFM70_cmd_tog2, sizeof(RFM70_cmd_tog2));

	delayMs(RFM70_END_INIT_WAIT_MS);

	//Check the ChipID
	if (readRegVal(0x08) != 0x63) {
		debug(RFM70_DEBUG_WRONG_CHIP_ID);
	}

	selectBank(0);
	setModeRX();
}

///////////////////////////////////////////////////////////////////////////////
// RFM70 Control //
///////////////////////////////////////////////////////////////////////////////

void RFM70::selectBank(uint8_t bank) {
	uint8_t tmp = readRegVal(0x07) & 0x80;
	if (bank) {
		if (!tmp) {
			writeRegPgmBuf((uint8_t *) RFM70_cmd_switch_cfg,
					sizeof(RFM70_cmd_switch_cfg));
		}
	} else {
		if (tmp) {
			writeRegPgmBuf((uint8_t *) RFM70_cmd_switch_cfg,
					sizeof(RFM70_cmd_switch_cfg));
		}
	}
}

void RFM70::setMode(uint8_t mode) {
	if (mode) {
		setModeRX();
	} else {
		setModeTX();
	}
}

void RFM70::setModeRX(void) {
	uint8_t val;
	// Flush RX FIFO
	writeRegPgmBuf((uint8_t *) RFM70_cmd_flush_rx, sizeof(RFM70_cmd_flush_rx));
	// Read Status
	val = readRegVal(RFM70_REG_STATUS);
	// Reset IRQ bits
	writeRegVal(RFM70_CMD_WRITE_REG | RFM70_REG_STATUS, val);
	// RFM chip disable
	digitalWrite(_ce, LOW);
	// set PRIM_RX bit to 1
	val = readRegVal(RFM70_REG_CONFIG);
	val |= RFM70_PIN_PRIM_RX;
	writeRegVal(RFM70_CMD_WRITE_REG | RFM70_REG_CONFIG, val);
	// RFM chip enable
	digitalWrite(_ce, HIGH);
}

void RFM70::setModeTX(void) {
	uint8_t val;

	// Flush TX FIFO
	writeRegPgmBuf((uint8_t *) RFM70_cmd_flush_tx, sizeof(RFM70_cmd_flush_tx));
	// RFM chip disable
	digitalWrite(_ce, LOW);
	// set PRIM_RX bit to 0
	val = readRegVal(RFM70_REG_CONFIG);
	val &= ~RFM70_PIN_PRIM_RX;
	writeRegVal(RFM70_CMD_WRITE_REG | RFM70_REG_CONFIG, val);
	// RFM chip enable
	digitalWrite(_ce, HIGH);
}

uint8_t RFM70::configRxPipe(uint8_t pipe_nr, uint8_t * adr, uint8_t plLen,
		uint8_t en_aa) {

	uint8_t tmp;
	uint8_t nr = pipe_nr - 1;

	if (plLen > 32 || nr > 5 || en_aa > 1) {
		return 0;
	}

	// write address
	// full length for rx pipe 0 an 1
	if (nr < 2) {
		writeRegCmdBuf(RFM70_CMD_WRITE_REG | (RFM70_REG_RX_ADDR_P0 + nr), adr,
				sizeof(adr));
	} else {
		// only LSB for pipes 2..5
		writeRegVal(RFM70_CMD_WRITE_REG | (RFM70_REG_RX_ADDR_P0 + nr), adr[0]); //ODO:check this
	}

	// static
	if (plLen) {
		// set payload len
		writeRegVal(RFM70_CMD_WRITE_REG | (RFM70_REG_RX_PW_P0 + nr), plLen);
		// set EN_AA bit
		tmp = readRegVal(RFM70_REG_EN_AA);
		if (en_aa) {
			tmp |= 1 << nr;
		} else {
			tmp &= ~(1 << nr);
		}
		writeRegVal(RFM70_CMD_WRITE_REG | RFM70_REG_EN_AA, tmp);
		// clear DPL bit
		tmp = readRegVal(RFM70_REG_DYNPD);
		tmp &= ~(1 << nr);
		writeRegVal(RFM70_CMD_WRITE_REG | RFM70_REG_DYNPD, tmp);
		// set Enable pipe bit
		enableRxPipe(nr);
	} else {
		// set payload len to default
		writeRegVal(RFM70_CMD_WRITE_REG | (RFM70_REG_RX_PW_P0 + nr), 0x20);
		// set EN_AA bit
		tmp = readRegVal(RFM70_REG_EN_AA);
		tmp |= 1 << nr;
		writeRegVal(RFM70_CMD_WRITE_REG | RFM70_REG_EN_AA, tmp);
		// set DPL bit
		tmp = readRegVal(RFM70_REG_DYNPD);
		tmp |= 1 << nr;
		writeRegVal(RFM70_CMD_WRITE_REG | RFM70_REG_DYNPD, tmp);
		// set Enable pipe bit
		enableRxPipe(nr);
	}
	return 1;
}

void RFM70::enableRxPipe(uint8_t pipe_nr) {
	uint8_t nr = pipe_nr - 1;
	if (nr > 5) {
		return;
	}
	uint8_t tmp;
	// set Enable pipe bit
	tmp = readRegVal(RFM70_REG_EN_RXADDR);
	tmp |= 1 << nr;
	writeRegVal(RFM70_CMD_WRITE_REG | RFM70_REG_EN_RXADDR, tmp);
}

void RFM70::disableRxPipe(uint8_t pipe_nr) {
	uint8_t nr = pipe_nr - 1;
	if (nr > 5) {
		return;
	}
	uint8_t tmp;
	// set Enable pipe bit
	tmp = readRegVal(RFM70_REG_EN_RXADDR);
	tmp &= ~(1 << nr);
	writeRegVal(RFM70_CMD_WRITE_REG | RFM70_REG_EN_RXADDR, tmp);

}

void RFM70::configTxPipe(uint8_t * adr, uint8_t pltype) {
	// write TX address
	writeRegCmdBuf(RFM70_CMD_WRITE_REG | RFM70_REG_TX_ADDR, adr, sizeof(adr));
	// write RX0 address
	writeRegCmdBuf(RFM70_CMD_WRITE_REG | RFM70_REG_RX_ADDR_P0, adr,
			sizeof(adr));
	// set static or dynamic payload
	uint8_t tmp;
	tmp = readRegVal(RFM70_REG_DYNPD);
	if (pltype == TX_DPL) {
		// dynamic
		tmp |= 1;
	} else {
		tmp &= ~(1 << 0);
	}
	writeRegVal(RFM70_CMD_WRITE_REG | RFM70_REG_DYNPD, tmp);
}

void RFM70::configCRC(uint8_t crc) {
	uint8_t tmp = readRegVal(RFM70_REG_CONFIG);
	//reset crc state
	tmp &= 0xF3;
	if (crc == CRC1) {
		tmp |= 0x08;
	} else if (crc == CRC2) {
		tmp |= 0x0C;
	}
	writeRegVal(RFM70_CMD_WRITE_REG | RFM70_REG_CONFIG, tmp);
}

void RFM70::configARD(uint8_t ard) {
	if (ard > 0x0f) {
		return;
	}
	uint8_t tmp = readRegVal(RFM70_REG_SETUP_RETR);
	tmp &= ((ard << 4) | 0x0F);
	writeRegVal(RFM70_CMD_WRITE_REG | RFM70_REG_SETUP_RETR, tmp);
}

void RFM70::configARC(uint8_t arc) {
	if (arc > 0x0f) {
		return;
	}
	uint8_t tmp = readRegVal(RFM70_REG_SETUP_RETR);
	tmp &= (arc | 0xF0);
	writeRegVal(RFM70_CMD_WRITE_REG | RFM70_REG_SETUP_RETR, tmp);
}

void RFM70::configSpeed(uint8_t speed) {
	if (speed > 2 || speed < 1) {
		return;
	}
	uint8_t tmp = readRegVal(RFM70_REG_RF_SETUP);
	tmp &= 0xF7;
	tmp |= speed << 3;
	writeRegVal(RFM70_CMD_WRITE_REG | RFM70_REG_RF_SETUP, tmp);
}

void RFM70::configLnaGain(uint8_t gain) {
	if (gain > 1) {
		return;
	}
	uint8_t tmp = readRegVal(RFM70_REG_RF_SETUP);
	tmp &= 0xFE;
	tmp |= gain;
	writeRegVal(RFM70_CMD_WRITE_REG | RFM70_REG_RF_SETUP, tmp);
}

void RFM70::configRfPower(uint8_t pwr) {
	if (pwr > 3) {
		return;
	}
	uint8_t tmp = readRegVal(RFM70_REG_RF_SETUP);
	tmp &= 0xF9;
	tmp |= pwr << 1;
	writeRegVal(RFM70_CMD_WRITE_REG | RFM70_REG_RF_SETUP, tmp);
}

void RFM70::confAddrWidth(uint8_t width) {
	if (width < 3 || width > 5) {
		return;
	}
	uint8_t tmp = readRegVal(RFM70_REG_SETUP_AW);
	tmp &= (0xF0 | (width - 2));
	writeRegVal(RFM70_CMD_WRITE_REG | RFM70_REG_SETUP_AW, tmp);
}

void RFM70::setPower(uint8_t pwr) {
	if (pwr > 1) {
		return;
	}
	uint8_t tmp = readRegVal(RFM70_REG_CONFIG);
	tmp &= (0xFD | (pwr << 1));
	writeRegVal(RFM70_CMD_WRITE_REG | RFM70_REG_CONFIG, tmp);
}

void RFM70::confIRQ(uint8_t irq_pin, uint8_t reflectTX_DS, uint8_t reflectRX_DR,
		uint8_t reflectMAX_RT) {
	if (irq_pin != -1) {
		pinMode(irq_pin, INPUT);
	}
	uint8_t tmp = readRegVal(RFM70_REG_CONFIG) & 0x8F;
	tmp |= ((reflectTX_DS & 0x01) ^ 0x01) << 6;
	tmp |= ((reflectRX_DR & 0x01) ^ 0x01) << 5;
	tmp |= ((reflectMAX_RT & 0x01) ^ 0x01) << 4;
	writeRegVal(RFM70_CMD_WRITE_REG | RFM70_REG_CONFIG, tmp);
}

void RFM70::cliAll() {
	uint8_t tmp = readRegVal(RFM70_REG_STATUS) | 0x70;
	writeRegVal(RFM70_CMD_WRITE_REG | RFM70_REG_STATUS, tmp);
}

void RFM70::cliRxDr() {
	uint8_t tmp = readRegVal(RFM70_REG_STATUS) | 0x40 & 0xCF;
	writeRegVal(RFM70_CMD_WRITE_REG | RFM70_REG_STATUS, tmp);
}

void RFM70::cliTxDs() {
	uint8_t tmp = readRegVal(RFM70_REG_STATUS) | 0x20 & 0xAF;
	writeRegVal(RFM70_CMD_WRITE_REG | RFM70_REG_STATUS, tmp);
}

void RFM70::cliTimeout() {
	uint8_t tmp = readRegVal(RFM70_REG_STATUS) | 0x10 & 0x9F;
	writeRegVal(RFM70_CMD_WRITE_REG | RFM70_REG_STATUS, tmp);
}

void RFM70::flushTxFIFO() {
	// Flush TX FIFO
	writeRegPgmBuf((uint8_t *) RFM70_cmd_flush_tx, sizeof(RFM70_cmd_flush_tx));
}

void RFM70::flushRxFIFO() {
	// Flush RX FIFO
	writeRegPgmBuf((uint8_t *) RFM70_cmd_flush_rx, sizeof(RFM70_cmd_flush_rx));
}

///////////////////////////////////////////////////////////////////////////////
// RFM70 getter //
///////////////////////////////////////////////////////////////////////////////

uint8_t RFM70::getMode(void) {
	return readRegVal(RFM70_REG_CONFIG) & RFM70_PIN_PRIM_RX;
}

uint8_t RFM70::getCarrierDetect(void) {
	return readRegVal(RFM70_REG_CD);
}

void RFM70::setChannel(uint8_t cnum) {
	writeRegVal(RFM70_CMD_WRITE_REG | RFM70_REG_RF_CH, cnum);
}

uint8_t RFM70::getChannel(void) {
	return readRegVal(RFM70_REG_RF_CH);
}

uint8_t RFM70::getPLC(void) {
	return readRegVal(RFM70_REG_OBSERVE_TX) >> 4 & 0x0F;
}

uint8_t RFM70::getARC(void) {
	return readRegVal(RFM70_REG_OBSERVE_TX) & 0x0F;
}

uint8_t RFM70::rxDataReceived() {
	uint8_t status = readRegVal(RFM70_REG_STATUS);
	if (status & RFM70_IRQ_STATUS_RX_DR) {
		return ((status & 0x0E) >> 1) + 1;
	}
}

uint8_t RFM70::txDataSent() {
	return readRegVal(RFM70_REG_STATUS) & RFM70_IRQ_STATUS_TX_DS;
}

uint8_t RFM70::txTimeout() {
	return readRegVal(RFM70_REG_STATUS) & RFM70_IRQ_STATUS_MAX_RT;
}

uint8_t RFM70::txFIFOFull() {
	return readRegVal(RFM70_REG_FIFO_STATUS) & RFM70_FIFO_STATUS_TX_FULL;
}

uint8_t RFM70::txFIFOEmpty() {
	return readRegVal(RFM70_REG_FIFO_STATUS) & RFM70_FIFO_STATUS_TX_EMPTY;
}

uint8_t RFM70::rxFIFOFull() {
	return readRegVal(RFM70_REG_FIFO_STATUS) & RFM70_FIFO_STATUS_RX_FULL;
}

uint8_t RFM70::rxFIFOEmpty() {
	return readRegVal(RFM70_REG_FIFO_STATUS) & RFM70_FIFO_STATUS_RX_EMPTY;
}

///////////////////////////////////////////////////////////////////////////////
// RFM70 Communication //
///////////////////////////////////////////////////////////////////////////////

uint8_t RFM70::readRegVal(uint8_t cmd) {
	uint8_t res;
	digitalWrite(_cs, LOW);
	delayMs(RFM70_CS_DELAY);
	rfm70_SPI.transfer(cmd);
	res = rfm70_SPI.transfer(0);
	digitalWrite(_cs, HIGH);
	delayMs(RFM70_CS_DELAY);
	return res;
}

void RFM70::readRegBuf(uint8_t reg, uint8_t * buf, uint8_t len) {
	uint8_t status, byte_ctr;
	digitalWrite(_cs, LOW);
	delayMs(RFM70_CS_DELAY);
	// Select register to write, and read status UINT8
	status = rfm70_SPI.transfer(reg);
	for (byte_ctr = 0; byte_ctr < len; byte_ctr++) {
		// Perform SPI_RW to read UINT8 from RFM70
		buf[byte_ctr] = rfm70_SPI.transfer(0);
	}
	digitalWrite(_cs, HIGH);
	delayMs(RFM70_CS_DELAY);
}

uint8_t RFM70::writeRegVal(uint8_t cmd, uint8_t val) {
	digitalWrite(_cs, LOW);
	delayMs(RFM70_CS_DELAY);
	rfm70_SPI.transfer(cmd);
	rfm70_SPI.transfer(val);
	digitalWrite(_cs, HIGH);
	delayMs(RFM70_CS_DELAY);
}

uint8_t RFM70::writeRegPgmBuf(uint8_t * cmdbuf, uint8_t len) {
	digitalWrite(_cs, LOW);
	delayMs(RFM70_CS_DELAY);
	while (len--) {
		rfm70_SPI.transfer(pgm_read_byte(cmdbuf++));
	}
	digitalWrite(_cs, HIGH);
	delayMs(RFM70_CS_DELAY);
}

uint8_t RFM70::writeRegCmdBuf(uint8_t cmd, uint8_t * buf, uint8_t len) {
	digitalWrite(_cs, LOW);
	delayMs(RFM70_CS_DELAY);
	rfm70_SPI.transfer(cmd);
	while (len--) {
		rfm70_SPI.transfer(*(buf++));
	}
	digitalWrite(_cs, HIGH);
	delayMs(RFM70_CS_DELAY);
}

uint8_t RFM70::sendAckPayload(uint8_t * payload, uint8_t len) {
	sendPayload(payload, len, -1);
}

uint8_t RFM70::sendPayload(uint8_t * payload, uint8_t len) {
	sendPayload(payload, len, NO_ACK);
}

uint8_t RFM70::sendPayload(uint8_t * payload, uint8_t len, uint8_t toAck) {
	// check TX_FIFO
	uint8_t status;
	status = readRegVal(RFM70_REG_FIFO_STATUS);
	if (status & RFM70_FIFO_STATUS_TX_FULL) {
		debug(RFM70_DEBUG_FIFO_FULL);
		return 0;
	}
	// send payload
	digitalWrite(_cs, LOW);
	delayMs(RFM70_CS_DELAY);
	if (toAck == -1) {
		rfm70_SPI.transfer(RFM70_CMD_W_ACK_PAYLOAD);
	} else if (toAck == 0) {
		rfm70_SPI.transfer(RFM70_CMD_W_TX_PAYLOAD_NOACK);
	} else {
		rfm70_SPI.transfer(RFM70_CMD_WR_TX_PLOAD);
	}
	while (len--) {
		rfm70_SPI.transfer(*(payload++));
	}
	digitalWrite(_cs, HIGH);
	delayMs(RFM70_CS_DELAY);

	return 1;
}

uint8_t RFM70::receivePayload(uint8_t *payload) {
	uint8_t len;
	// check RX_FIFO
	uint8_t status;
	status = readRegVal(RFM70_REG_STATUS);
	// RX_DR
	if (status & RFM70_IRQ_STATUS_RX_DR) {
		uint8_t fifo_sta;
		// Payload width
		len = readRegVal(RFM70_CMD_RX_PL_WID);
		readRegBuf(RFM70_CMD_RD_RX_PLOAD, payload, len);
		fifo_sta = readRegVal(RFM70_REG_FIFO_STATUS);
		if (fifo_sta & RFM70_FIFO_STATUS_RX_EMPTY) {
			// clear status bit rx_dr
			status |= 0x40 & 0xCF;
			writeRegVal(RFM70_CMD_WRITE_REG | RFM70_REG_STATUS, status);
		}
		return len;
	} else {
		return 0;
	}
}

///////////////////////////////////////////////////////////////////////////////
// RFM70 debug //
///////////////////////////////////////////////////////////////////////////////

void RFM70::debug(uint8_t token) {
	// to be done
}

void RFM70::delayMs(uint8_t ms) {
	if (ms) {
		delay(ms);
	}
}

/**
 *	void tick()
 *	checks if there is a complete transmission and calls the users callback function
 **/
void RFM70::tick() {
	byte len = receivePayload(rcvBuffer);
	if (len < 32) {
		rcvBuffer[len] = '\0';
	}
	//new packet if len > 0
	if (len != 0) {
		packetLength = (uint8_t) len;

		// don't bother if user hasn't registered a callback
		if (user_onReceive) {
			user_onReceive();
		}
	}
	return;
}

/**
 * Sets function called when receive transmission complete
 **/
void RFM70::onReceive(void (*function)(void)) {
	user_onReceive = function;
}

/**
 *	uint8_t* getBuffer(void)
 *	return pointer to the first element of the buffer
 **/
uint8_t* RFM70::getRcvBuffer(void) {
	return rcvBuffer;
}

/**
 *	uint8_t getByte(uint8_t)
 *	returns the specified byte from the package buffer
 **/
uint8_t RFM70::getRcvByte(uint8_t byte) {
	return rcvBuffer[byte];
}

uint8_t RFM70::getPacketLength(void) {
	return packetLength;
}

/**
 *	boolean send(char*) : function to enqueue a packet for transmission
 *	returns: true, false
 **/
void RFM70::send(uint8_t* s, uint8_t len) {

	//switch to tx mode
	cliTxDs();
	setMode(MODE_PTX);

	if (txTimeout()) {
		flushTxFIFO();
		cliTimeout();
	}

	sendPayload(s, len);

	//wait for packet to be sent
	_delay_us(SEND_DELAY);

	//switch to rx mode
	cliRxDr();
	setMode(MODE_PRX);
}
