/*************************************************** 
  This is a library for the MLX90614 Temp Sensor

  Designed specifically to work with the MLX90614 sensors in the
  adafruit shop
  ----> https://www.adafruit.com/products/1748
  ----> https://www.adafruit.com/products/1749

  These sensors use I2C to communicate, 2 pins are required to  
  interface
  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include "Adafruit_MLX90614.h"

Adafruit_MLX90614::Adafruit_MLX90614(uint8_t i2caddr) {
  _addr = i2caddr;
}


boolean Adafruit_MLX90614::begin(void) {
  Wire.begin();

  /*
  for (uint8_t i=0; i<0x20; i++) {
    Serial.print(i); Serial.print(" = ");
    Serial.println(read16(i), HEX);
  }
  */
  return true;
}

//////////////////////////////////////////////////////


double Adafruit_MLX90614::readObjectTempF(void) {
  return (readTemp(MLX90614_TOBJ1) * 9 / 5) + 32;
}


double Adafruit_MLX90614::readAmbientTempF(void) {
  return (readTemp(MLX90614_TA) * 9 / 5) + 32;
}

double Adafruit_MLX90614::readObjectTempC(void) {
  return readTemp(MLX90614_TOBJ1);
}


double Adafruit_MLX90614::readAmbientTempC(void) {
  return readTemp(MLX90614_TA);
}

float Adafruit_MLX90614::readTemp(uint8_t reg) {
  float temp;
  
  temp = read16(reg);
  temp *= .02;
  temp  -= 273.15;
  return temp;
}

/*********************************************************************/

uint16_t Adafruit_MLX90614::read16(uint8_t a) {
  uint16_t ret;

  Wire.beginTransmission(_addr); // start transmission to device 
  Wire.write(a); // sends register address to read from
  Wire.endTransmission(false); // end transmission
  
  Wire.requestFrom(_addr, (uint8_t)3);// send data n-bytes read
  ret = Wire.read(); // receive DATA
  ret |= Wire.read() << 8; // receive DATA

  uint8_t pec = Wire.read();

  return ret;
}

uint8_t crc8Msb(uint8_t poly, uint8_t* data, int size);

void Adafruit_MLX90614::write16(uint8_t addr, uint16_t data) {
  uint8_t buff[5];
  buff[0] = _addr;
  buff[1] = addr;
  buff[2] = data & 0x00FF;
  buff[3] = data >> 8;
  buff[4] = crc8Msb(0x07, buff, 4);

  Wire.beginTransmission(_addr); // start transmission to device
  Wire.write(&buff[1],4); // sends register address to read from
  Wire.endTransmission();
}

/****************************************************************
 * Function Name: crc8_msb
 * Description:  CRC8 check to compare PEC data
 * Parameters: poly - x8+x2+x1+1, data - array to check, array size
 * Return: 0 – data right; 1 – data Error
****************************************************************/
uint8_t crc8Msb(uint8_t poly, uint8_t* data, int size) {
  uint8_t crc = 0x00;
  int bit;

  while (size--)
  {
    crc ^= *data++;
    for (bit = 0; bit < 8; bit++)
    {
      if (crc & 0x80) {
        crc = (crc << 1) ^ poly;
      } else {
        crc <<= 1;
      }
    }
  }

  return crc;
}
