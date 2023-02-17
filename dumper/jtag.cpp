/*
   https://github.com/gashtaan/sinowealth-8051-dumper

   Copyright (C) 2023, Michal Kovacik

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License version 3, as
   published by the Free Software Foundation.
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <avr/io.h>
#include <util/delay.h>

#include "config.h"
#include "jtag.h"
#include "icp_opcodes.h"

ICP::ICP()
{
	DDRD &= ~_BV(TDO);
	DDRD |= _BV(TDI);
	DDRD |= _BV(TMS);
	DDRD |= _BV(TCK);

	clrBit(TDO);
	setBit(TCK);
	setBit(TDI);
	setBit(TMS);

	_delay_us(500);

	clrBit(TCK);
	_delay_us(1);
	setBit(TCK);
	_delay_us(50);

	for (uint8_t n = 0; n < 165; ++n)
	{
		clrBit(TMS);
		_delay_us(2);
		setBit(TMS);
		_delay_us(2);
	}

	for (uint8_t n = 0; n < 105; ++n)
	{
		clrBit(TDI);
		_delay_us(2);
		setBit(TDI);
		_delay_us(2);
	}

	for (uint8_t n = 0; n < 90; ++n)
	{
		clrBit(TCK);
		_delay_us(2);
		setBit(TCK);
		_delay_us(2);
	}

	for (uint16_t n = 0; n < 25600; ++n)
	{
		clrBit(TMS);
		_delay_us(2);
		setBit(TMS);
		_delay_us(2);
	}

	_delay_us(8);

	clrBit(TMS);
	clrBit(TCK);
	_delay_us(2);

	sendMode(0x96);

	setBit(TCK);
	_delay_us(2);

	for (uint16_t n = 0; n < 25600; ++n)
	{
		clrBit(TCK);
		_delay_us(2);
		setBit(TCK);
		_delay_us(2);
	}

	setBit(TMS);
	_delay_us(5);
	clrBit(TMS);
	_delay_us(5);

	m_mode = 1;
}

void ICP::reset()
{
	if (m_mode == 0)
		return;

	if (m_mode == 0xA5)
	{
		setBit(TMS);

		for (uint8_t n = 0; n < 35; ++n)
		{
			setBit(TCK);
			_delay_us(2);
			clrBit(TCK);
			_delay_us(2);
		}
		setBit(TCK);

		clrBit(TMS);
	}
	else
	{
		setBit(TCK);

		setBit(TMS);
		_delay_us(2);
		clrBit(TMS);
		_delay_us(2);
	}

	m_mode = 1;
}

void ICP::switchMode(uint8_t mode)
{
	if (m_mode == mode)
		return;

	if (m_mode != 1)
		reset();

	m_mode = mode;

	clrBit(TCK);
	_delay_us(2);

	sendMode(m_mode);

	if (m_mode == 0x96)
	{
		_delay_us(800);
		setBit(TCK);
		_delay_us(2);

		ping();
	}
	else if (m_mode == 0xA5)
	{
		setBit(TMS);
		pulseClocks(6);
		clrBit(TMS);
		pulseClocks(2);

		// TODO...
	}
}

bool ICP::check() const
{
	clrBit(TCK);

	sendData8(0x40); // Another variant of mode switch related bytes
	sendData8(0x69);
	sendData8(0x41);
	sendData8(0xFF); // This 0xFFFF will end mode switch
	sendData8(0xFF);

	sendData8(0x43); // TODO: another ICP_ opcode
	auto b = receiveData8();
	receiveData8();

	return (b == 0x69);
}

void ICP::ping() const
{
	sendData8(0x49);  // TODO: another ICP_ opcode
	sendData8(0xFF);
}

void ICP::readFlash(uint8_t* buffer, uint32_t address, bool customBlock)
{
	reset();
	switchMode(0x96);

#if CHIP_TYPE != 1
	// This probably still belongs to switchMode(), seems to take two arguments?
	// May be not specific to reads
	sendData8(0x46);
	sendData8(0xFE);
	sendData8(0xFF);
#endif

	sendData8(ICP_ADDRESS_7B0);
	sendData8((address & ICP_ADDRESS_7B0_MASK) >> ICP_ADDRESS_7B0_SHIFT);
	sendData8(ICP_ADDRESS_15B8);
	sendData8((address & ICP_ADDRESS_15B8_MASK) >> ICP_ADDRESS_15B8_SHIFT);
#if CHIP_TYPE == 4 || CHIP_TYPE == 7
	// Chips having more than 15 address lines for flash
	sendData8(ICP_ADDRESS_23B16);
	sendData8((address & ICP_ADDRESS_23B16_MASK) >> ICP_ADDRESS_23B16_SHIFT);
#endif

	sendData8(customBlock ? ICP_READ_CUSTOM_BLOCK : ICP_READ);

	for (uint8_t n = 0; n < 16; ++n)
		buffer[n] = receiveData8();
}

void ICP::sendMode(uint8_t value) const
{
	for (uint8_t m = 0x80; m; m >>= 1)
	{
		if (value & m)
			setBit(TDI);
		else
			clrBit(TDI);

		setBit(TCK);
		_delay_us(2);
		clrBit(TCK);
		_delay_us(2);
	}

	setBit(TCK);
	_delay_us(2);
	clrBit(TCK);
	_delay_us(2);

	setBit(TCK);
	_delay_us(2);
	clrBit(TCK);
	_delay_us(2);
}

void ICP::sendData8(uint8_t value) const
{
	for (uint8_t m = 0x80; m; m >>= 1)
	{
		if (value & m)
			setBit(TDI);
		else
			clrBit(TDI);

		pulseClock();
	}

	clrBit(TDI);

	pulseClock();
}

uint8_t ICP::receiveData8() const
{
	uint8_t value = 0;
	clrBit(TDI);
	for (uint8_t m = 1; m; m <<= 1)
	{
		pulseClock();

		if (getBit(TDO))
			value |= m;
	}

	pulseClock();

	return value;
}

void ICP::pulseClock() const
{
	_delay_us(1);
	setBit(TCK);
	_delay_us(1);
	clrBit(TCK);
}

void ICP::pulseClocks(uint8_t count) const
{
	while (count-- > 0)
		pulseClock();
}
