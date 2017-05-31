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
 
#if (ARDUINO >= 100)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#ifdef __AVR_ATtiny85__
 #include "TinyWireM.h"
 #define Wire TinyWireM
#else
 #include <Wire.h>
#endif

/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
  #define TLV493_ADDRESS_0                (0x3F)
  #define TLV493_ADDRESS_1                (0xBD)
/*=========================================================================*/

/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
  enum
  {
    TLV493_R_REGISTER_X_HI                = 0X00,
    TLV493_R_REGISTER_Y_HI                = 0X01,
    TLV493_R_REGISTER_Z_HI                = 0X02,
    TLV493_R_REGISTER_TEMP_HI             = 0X03,
    TLV493_R_REGISTER_XY_LO               = 0X04,
    TLV493_R_REGISTER_Z_LO                = 0X05,
    TLV493_R_REGISTER_TEMP_LO             = 0X06,
    TLV493_W_REGISTER_MODE1               = 0X01,
    TLV493_W_REGISTER_MODE2               = 0X03,
  	  
    TLV493_FLAG_INT			= 0X04,
    TLV493_FLAG_FAST			= 0X02,
    TLV493_FLAG_LOW			= 0X01,
    TLV493_FLAG_TEMP			= 0X80,
    TLV493_LPP				= 0X40
	
  };

class TLV493D
{
  public:
    TLV493D(void);
    //TLV493D(int8_t sdaPin,int8_t sclPin); // TODO: write implementation for pin select option
    int16_t  x;
    int16_t  y;
    int16_t  z;
    float    tempC;
    uint8_t   address;
	  
    void  begin(bool pinHi = true);
	
    enum accessMode
    {
      fastMode 	,	
      lowPower 	,	
      ultraLow 	,
      masterControlled,
      powerDown 			
    };

	
    void configure
    (
      accessMode mode,
      uint8_t addressCode = 0x00,
      bool ignoreTemp = false
      
      //bool useInterrupt = false, 
    );
	
    void  enableTemp(bool t);
    bool  measure();
    void  generalReset(void);

  private:
    	
    static const uint8_t addressMatrix[2][4];

    uint16_t combineBits(uint8_t HI,uint8_t LO);
    bool 	readRegisters(void);
    void  	write8(byte reg, byte value);
    uint8_t	read8(byte reg);

    bool 	_addressPinInitState;
    bool  	_ignoreTemp;
    accessMode  _mode;
    uint8_t	_addressCode;
    uint8_t	readRegister[10];
    uint8_t     frameCount;
   
};

