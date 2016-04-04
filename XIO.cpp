/*************************************************************************
Title:    Iowa Scaled Engineering I2C-XIO (PCA9698) Driver Library
Authors:  Nathan D. Holmes <maverick@drgw.net>
File:     $Id: $
License:  GNU General Public License v3

LICENSE:
	Copyright (C) 2014 Nathan D. Holmes & Michael D. Petersen

	This program is free software; you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation; either version 3 of the License, or
	any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	For more information about the Iowa Scaled Engineering I2C-XIO, see:
	http://www.iascaled.com/store/I2C-XIO

*************************************************************************/

#include <stdlib.h>
#include <string.h>
#include "Arduino.h"
#include "Wire.h"
#include "XIO.h"

#ifndef _BV
#define _BV(a) (1<<(a))
#endif

XIO::XIO()
{
	this->addr = 0;
	this->dioReset = -1; // Unknown digital I/O for the reset line
	this->dioOutputEnable = -1; // Unknown digital I/O for the reset line
}

void XIO::begin(boolean a0, boolean a1, boolean a2, char dioResetPin, char dioOutputEnablePin)
{
	uint8_t addrBitmap = (a0 ? 0x01 : 0x00) | (a1 ? 0x02 : 0x00) | (a2 ? 0x04 : 0x00);
	this->addr = 0x20 | (addrBitmap);

	this->dioReset = dioResetPin;
	this->dioOutputEnable = dioOutputEnablePin;

	// If there's a DIO pin assigned to reset, use it to do a hardware reset on initialization
	if (-1 != this->dioOutputEnable)
		pinMode(this->dioOutputEnable, OUTPUT);

	if (-1 != this->dioReset)
		pinMode(this->dioReset, OUTPUT);

	if (-1 != this->dioReset)
	{
		digitalWrite(this->dioReset, LOW);
		delayMicroseconds(100);
		digitalWrite(this->dioReset, HIGH);
	}

	if (-1 != this->dioOutputEnable)
		digitalWrite(this->dioOutputEnable, LOW);

	memset(this->pinDirections, 0xFF, sizeof(this->pinDirections)); //start inicialization all IO -Input
	memset(this->pinOutputStates, 0x00, sizeof(this->pinOutputStates)); //start inicialization all IO- OUT -LOW
	memset(this->pinOutputStates, 0x00, sizeof(this->pinInputStates)); //start inicialization all IO -Input -total-pole
	memset(this->maskInterruptState, 0xFF, sizeof(this->maskInterruptState));//start inicialization all IO masked - interrupt-off
	memset(this->polarityInversion, 0x00, sizeof(this->polarityInversion));//start inicialization all IO polarity inversion off
}

void XIO::xioPinModeCached(byte pin, byte mode)
{
	uint8_t bit = pin & 0x0F;
	uint8_t bank = (pin >> 4) & 0x0F;
	// If things are out of range, do nothing
	if ((bit > 7) || (bank > 4))
		return;

	switch (mode)
	{
	case OUTPUT:
		pinDirections[bank] &= ~(_BV(bit));
		break;

	case INPUT:
		pinDirections[bank] |= _BV(bit);
		break;

	default:
		return; // Don't understand this pin mode, so do nothing
	}
}

void XIO::xioPinMode(byte pin, byte mode)
{
	uint8_t bit = pin & 0x0F;
	uint8_t bank = (pin >> 4) & 0x0F;
	// If things are out of range, do nothing
	if ((bit > 7) || (bank > 4))
		return;

	xioPinModeCached(pin, mode);

	sendOutputConfiguration(bank);
}

void XIO::xioDigitalWriteCached(byte pin, boolean value)
{
	uint8_t bit = pin & 0x0F;
	uint8_t bank = (pin >> 4) & 0x0F;
	// If things are out of range, do nothing
	if ((bit > 7) || (bank > 4))
		return;

	if (value)
		pinOutputStates[bank] |= _BV(bit);
	else
		pinOutputStates[bank] &= ~_BV(bit);
}

void XIO::xioMaskSetCached(byte pin, boolean value)
{
	uint8_t bit = pin & 0x0F;
	uint8_t bank = (pin >> 4) & 0x0F;
	// If things are out of range, do nothing
	if ((bit > 7) || (bank > 4))
		return;

	if (value)
		maskInterruptState[bank] |= _BV(bit);
	else
		maskInterruptState[bank] &= ~_BV(bit);
}

void XIO::xioPolaritySetCached(byte pin, boolean value)
{
	uint8_t bit = pin & 0x0F;
	uint8_t bank = (pin >> 4) & 0x0F;
	// If things are out of range, do nothing
	if ((bit > 7) || (bank > 4))
		return;

	if (value)
		polarityInversion[bank] |= _BV(bit);
	else
		polarityInversion[bank] &= ~_BV(bit);
}

void XIO::xioDigitalWrite(byte pin, boolean value)
{
	uint8_t bit = pin & 0x0F;
	uint8_t bank = (pin >> 4) & 0x0F;
	// If things are out of range, do nothing
	if ((bit > 7) || (bank > 4))
		return;

	xioDigitalWriteCached(pin, value);

	sendOutput(bank);
}

void XIO::xioMaskSet(byte pin, boolean value)
{
	uint8_t bit = pin & 0x0F;
	uint8_t bank = (pin >> 4) & 0x0F;
	// If things are out of range, do nothing
	if ((bit > 7) || (bank > 4))
		return;
	xioMaskSetCached(pin, value);
	sendMaskInterrupt(bank);
}

void XIO::xioPolaritySet(byte pin, boolean value)
{
	uint8_t bit = pin & 0x0F;
	uint8_t bank = (pin >> 4) & 0x0F;
	// If things are out of range, do nothing
	if ((bit > 7) || (bank > 4))
		return;
	xioPolaritySetCached(pin, value);
	sendPolarityInversion(bank);
}

boolean XIO::xioDigitalReadCached(byte pin)
{
	uint8_t bit = pin & 0x0F;
	uint8_t bank = (pin >> 4) & 0x0F;
	// If things are out of range, do nothing
	if ((bit > 7) || (bank > 4))
		return false;

	return((this->pinInputStates[bank] & _BV(bit)) ? HIGH : LOW);
}

boolean XIO::xioDigitalRead(byte pin)
{
	uint8_t bit = pin & 0x0F;
	uint8_t bank = (pin >> 4) & 0x0F;
	// If things are out of range, do nothing
	if ((bit > 7) || (bank > 4))
		return false;

	this->pinInputStates[bank] = getInput(bank);

	return xioDigitalReadCached(pin);
}

void XIO::sendOutputConfiguration(byte bank)
{
	uint8_t numWrites = 1, i;
	if (0xFF == bank)
	{
		bank = 0;
		numWrites = 5;
	}

	Wire.beginTransmission(this->addr);
	Wire.write((uint8_t)(0x80 | (0x18 + bank))); // 0x18 is the direction register base, 0x80 is auto-increment
	for (i = 0; i < numWrites; i++)
		Wire.write(this->pinDirections[bank + i]);
	Wire.endTransmission();
}

void XIO::xioMODE(boolean smba, boolean ioac, boolean och, boolean oepol)
{
	Wire.beginTransmission(this->addr);
	Wire.write(0x2A); // 0x2A is the MODE registers
	Wire.write((uint8_t)((smba ? 0x10 : 0x00) | (ioac ? 0x08 : 0x00) | (och ? 0x02 : 0x00) | (oepol ? 0x01 : 0x00)));
	Wire.endTransmission();
}

void XIO::xioOUTCONF(boolean out4, boolean out3, boolean out2, boolean out1,
	boolean out067, boolean out045, boolean out023, boolean out001)
{
	Wire.beginTransmission(this->addr);
	Wire.write(0x28); // 0x28 is the OUTCONF registers
	Wire.write((uint8_t)((out4 ? 0x80 : 0x00) | (out3 ? 0x40 : 0x00) | (out2 ? 0x20 : 0x00) | (out1 ? 0x10 : 0x00)
		| (out067 ? 0x08 : 0x00) | (out045 ? 0x04 : 0x00) | (out023 ? 0x02 : 0x00) | (out001 ? 0x01 : 0x00)));
	Wire.endTransmission();
}

void XIO::xioALLBNK(boolean bsel, boolean b4, boolean b3, boolean b2, boolean b1, boolean b0)
{
	Wire.beginTransmission(this->addr);
	Wire.write(0x29); // 0x29 is the ALLBNK registers
	Wire.write((uint8_t)((bsel ? 0x80 : 0x00) | (b4 ? 0x10 : 0x00) | (b3 ? 0x08 : 0x00) | (b2 ? 0x04 : 0x00) | (b1 ? 0x02 : 0x00) | (b0 ? 0x01 : 0x00)));
	Wire.endTransmission();
}

void XIO::sendOutput(byte bank)
{
	uint8_t numWrites = 1, i;
	if (0xFF == bank)
	{
		bank = 0;
		numWrites = 5;
	}

	Wire.beginTransmission(this->addr);
	Wire.write(0x80 | (0x08 + bank)); // 0x08 is the output register base, 0x80 is auto-increment
	for (i = 0; i < numWrites; i++)
		Wire.write(this->pinOutputStates[bank + i]);
	Wire.endTransmission();
}

void XIO::sendMaskInterrupt(byte bank)
{
	uint8_t numWrites = 1, i;
	if (0xFF == bank)
	{
		bank = 0;
		numWrites = 5;
	}

	Wire.beginTransmission(this->addr);
	Wire.write(0x80 | (0x20 + bank)); // 0x20-0x24 is the mask interrupt registers, 0x80 is auto-increment
	for (i = 0; i < numWrites; i++)
		Wire.write(this->maskInterruptState[bank + i]);
	Wire.endTransmission();
}

void XIO::sendPolarityInversion(byte bank)
{
	uint8_t numWrites = 1, i;
	if (0xFF == bank)
	{
		bank = 0;
		numWrites = 5;
	}

	Wire.beginTransmission(this->addr);
	Wire.write(0x80 | (0x10 + bank)); // 0x10-0x14 is the polarity inversion registers, 0x80 is auto-increment
	for (i = 0; i < numWrites; i++)
		Wire.write(this->polarityInversion[bank + i]);
	Wire.endTransmission();
}

byte XIO::getInput(byte bank)
{
	uint8_t numReads = 1, i;
	if (0xFF == bank)
	{
		bank = 0;
		numReads = 5;
	}

	Wire.beginTransmission(this->addr);
	Wire.write(0x80 | (0x00 + bank)); // 0x00 is the input register base, 0x80 is auto-increment
	i = Wire.endTransmission(false);

	if (0 != i)
	{
		Wire.endTransmission(true); // Make sure we send a stop bit
		return 0; // I2C error, don't continue
	}

	Wire.requestFrom(this->addr, numReads);
	if (Wire.available() < numReads)
	{
		Wire.endTransmission(true); // Make sure we send a stop bit
		return 0; // I2C error, don't continue
	}

	for (i = 0; i < numReads; i++)
	{
		this->pinInputStates[bank + i] = Wire.read();
		currentInputStates[bank + i] = pinInputStates[bank + i];
	}
	return(this->pinInputStates[bank]);
}

byte XIO::getSMBA() //ARA request
{
	Wire.requestFrom(0x0C, 1);
	if (!Wire.available())
		return 0;
	while (Wire.available())
	{
		this->smbaAddr = Wire.read();
		return(this->smbaAddr);
	}
}

void XIO::refreshPinModes()
{
	sendOutputConfiguration(0xFF);
}

void XIO::refreshIO()
{
	sendOutput(0xFF);
	getInput(0xFF);
}

void XIO::refreshMaskInterrupt()
{
	sendMaskInterrupt(0xFF);
}

void XIO::refreshPolarityInversion()
{
	sendPolarityInversion(0xFF);
}