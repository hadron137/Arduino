/*-------------------------------------------------------------------------
  Arduino library to control TLV493D magnetic sensor
  These sensors use I2C to communicate, 2 pins are required to interface.

  Written by Eric Kuntzelman

  -------------------------------------------------------------------------
    This file is part of the TLV493D library.

  the TLV493D library is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  the TLV493D library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with the TLV493D library.  If not, see <http://www.gnu.org/licenses/>.
  -------------------------------------------------------------------------*/
#include "Arduino.h"
#include <Wire.h>
#include <SPI.h>
#include "TLV493D.h"


/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/

const uint8_t TLV493D::addressMatrix[2][4] =
{
    {0x3F,0x37,0x1F,0x17}, // ADDR pin low during startup
    {0xBD,0xB5,0x9D,0x95}  // ADDR pin high during startup
};

TLV493D::TLV493D() 
{
    
  // set default values, to prevent undefined error
   _ignoreTemp = false;
   _addressCode = 0x00;
   _mode = masterControlled;
}

void TLV493D::begin(bool pinHi) {
  // pinHi is the state of the SDA pin when TLV493D is powered up
  _addressPinInitState = pinHi;
  
  //address = pinHi ? (uint8_t)TLV493_ADDRESS_1 : (uint8_t)TLV493_ADDRESS_0;
  address = addressMatrix[_addressPinInitState ? 1 : 0][0];
  address = address >> 1;
  
  Wire.begin();
  delayMicroseconds(1000); // needs 200 microseconds to stabalize after power up.
  
  x=0;
  y=0;
  z=0;
  
  frameCount = 4;
  
  measure();
}

void TLV493D::generalReset(void)
{
 // general reset by calling address 00
 // may be needed if power doesn't come on quickly
 Wire.requestFrom(0x00,1);
 delayMicroseconds(1000);
 Wire.write((byte)0x00);
 Wire.endTransmission();
 delayMicroseconds(1000);
}

void TLV493D::enableTemp(bool t)
{	
	configure(_mode,_addressCode, !t);
}

void TLV493D::configure(accessMode mode, uint8_t addressCode, bool ignoreTemp )
{
	//	addressCode 	00,01,10, or 11 to set I2C address
	// 	ignoreTemp		If 0, temperature measurement enabled (default)
	//	useInterrupt	After a completed conversion, an interrupt pulse will be generated (not recommended for bus communication)

	
	// some bytes from read register need to be sent to write registers
	readRegisters();
	
	_ignoreTemp = ignoreTemp;
	_addressCode = addressCode;
	_mode = mode;
		
	// prepare bytes before sending to TLV493D
	uint8_t writeValue0 = 0x00;
	uint8_t writeValue1;
	uint8_t writeValue2 = readRegister[8];
	uint8_t writeValue3 = 0x00;
	
	writeValue1 = addressCode << 5;
	writeValue1 += readRegister[7] & 0x18; //Bits 4:3 must correspond to bits 4:3 from read register 7H.
	//writeValue1 += useInterrupt ? TLV493_FLAG_INT 	: 0 ;
	//writeValue1 += fastMode 	? TLV493_FLAG_FAST 	: 0 ;
	//writeValue1 += lowPower 	? TLV493_FLAG_LOW 	: 0 ;
	
	writeValue3 += ignoreTemp 	? TLV493_FLAG_TEMP 	: 0;
	//writeValue3 += insomnia 	? TLV493_LPP 		: 0;
	writeValue3 &= ~(0x20); // disable parity test
	writeValue3 += readRegister[9] & 0x1F;
	
	switch (mode)
	{
		/*
		fastMode 		,	// fastmode=1, lp_mode=0, int_out=1   (byte settings [hex]: 00, x6, xx, xx, keep certain bits)
		lowPower 		,	// fastmode=0, lp_mode=1, int_out=1   (byte settings [hex]: 00, x5, xx, 4x, keep certain bits)
		ultraLow 		,	// fastmode=0, lp_mode=1, int_out=1   (byte settings [hex]: 00, x5, xx, 0x, keep certain bits)
		masterControlled,	// fastmode=1, lp_mode=1, int_out=0   (byte settings [hex]: 00, x3, xx, xx, keep certain bits)
		powerDown 			// fastmode=0, lp_mode=0              (byte settings [hex]: 00, x0, xx, xx, keep certain bits)
		*/
		case fastMode :
			writeValue1 += 0x06;
			break;	
		case lowPower :
			writeValue1 += 0x05;
			writeValue3 += 0x40;
			break;
		case ultraLow :
			writeValue1 += 0x05;
			writeValue3 &= 0x0F;
			break;
		case masterControlled : 
			writeValue1 += 0x03;
			break;
		case powerDown :
			writeValue1 += 0x00;
			break;
		default:
			break;
	}

	Wire.beginTransmission(address);
	Wire.write(writeValue0);
	Wire.write(writeValue1);
	Wire.write(writeValue2);
	Wire.write(writeValue3);
	Wire.endTransmission();

        // look up new I2C address from table
	address = addressMatrix[_addressPinInitState ? 1 : 0][addressCode & 0x03];
	
	// The Wire library uses 7 bit addresses. Shift the datasheet address one bit to the right.
	address = address >> 1;

}

bool  TLV493D::measure()
{
  
  //delayMicroseconds(1000); // 3.3kHz = 300 microseconds
  if (readRegisters())
  {  
    x = combineBits(readRegister[0], readRegister[4] & 0xF0) >> 4;
    y = combineBits(readRegister[1], readRegister[4] << 4) >> 4;
    z = combineBits(readRegister[2], readRegister[5] << 4) >> 4;
    
    // convert from unsigned to signed. bit12 represents signed bit
    if (x >> 11) x -= (2048 * 2);
    if (y >> 11) y -= (2048 * 2);
    if (z >> 11) z -= (2048 * 2);
     	
    tempC = (float)(combineBits(readRegister[3] >> 4, readRegister[6]) - 340) * 1.1;
  }
  else
  {
    x = y = z = tempC = 0;
  }
  
  uint8_t oldCount = frameCount;
  frameCount = (readRegister[3] >> 2) & 0x03;
  return (frameCount != oldCount);
	
}

/***************************************************************************
 PRIVATE FUNCTIONS
 ***************************************************************************/
 

/**************************************************************************/
/*!
    @brief  Reads all registers
*/
/**************************************************************************/
bool TLV493D::readRegisters(void)
{
  //Serial.print("address = ");
  //Serial.print(1+(address << 1),HEX);
  //Serial.print(" =  ");
  Wire.requestFrom( address ,(byte)10);
  	
  int count = Wire.available();

  for (int i=0; i < count; ++i)
  {
    readRegister[i] = Wire.read();

    //Serial.print(readRegister[i],HEX);
    //Serial.print("\t");
  }
  
  return (count == 10);
  //Serial.println();
}

 /**************************************************************************/
/*!
    @brief  combine hi and low bits into int16
*/
/**************************************************************************/
uint16_t TLV493D::combineBits(uint8_t HI,uint8_t LO)
{
  uint16_t result;
  result = ((uint16_t)HI) << 8;
  result += LO;

  return result;
}
