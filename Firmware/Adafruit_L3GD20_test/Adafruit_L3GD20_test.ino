/*************************************************** 
  This is an example for the Adafruit Triple-Axis Gyro sensor

  Designed specifically to work with the Adafruit L3GD20 Breakout 
  ----> https://www.adafruit.com/products/1032

  These sensors use I2C or SPI to communicate, 2 pins (I2C) 
  or 4 pins (SPI) are required to interface.

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Kevin "KTOWN" Townsend for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#define LSM330DLGYRO
#ifdef LSM330DLGYRO
#include <Wire.h> 
#include <Adafruit_L3GD20.h>

// Comment this next line to use SPI
#define USE_I2C

#ifdef USE_I2C
  // The default constructor uses I2C
  Adafruit_L3GD20 gyro;
#else
  // To use SPI, you have to define the pins
  #define GYRO_CS 4 // labeled CS
  #define GYRO_DO 5 // labeled SA0
  #define GYRO_DI 6  // labeled SDA
  #define GYRO_CLK 7 // labeled SCL
  Adafruit_L3GD20 gyro(GYRO_CS, GYRO_DO, GYRO_DI, GYRO_CLK);
#endif //USE_I2C
#endif //LSM330DLGYRO


void setup() 
{
  Serial.begin(57600);


  #ifdef LSM330DLGYRO
  // Try to initialise and warn if we couldn't detect the chip
  // if (!gyro.begin(gyro.L3DS20_RANGE_250DPS))
  //if (!gyro.begin(gyro.L3DS20_RANGE_500DPS))
  //if (!gyro.begin(gyro.L3DS20_RANGE_2000DPS))
  // if (!gyro.begin(gyro.L3DS20_RANGE_250DPS, LSM330DL_ADDRESS))
  //if (!gyro.begin(gyro.L3DS20_RANGE_500DPS, LSM330DL_ADDRESS))
  if (!gyro.begin(gyro.L3DS20_RANGE_2000DPS, LSM330DL_ADDRESS))
  {
    Serial.println("Oops ... unable to initialize the LSM330DLGYRO. Check your wiring!");
  }
 #endif //LSM330DLGYRO

        Serial.println("start");         // print the character

}

void loop() 
{
  
     /* gyro section */
#ifdef LSM330DLGYRO
  gyro.read();
   Serial.print("X: "); Serial.print((int)gyro.data.x);   Serial.print(" ");
  Serial.print("Y: "); Serial.print((int)gyro.data.y);   Serial.print(" ");
  Serial.print("Z: "); Serial.println((int)gyro.data.z); Serial.print(" -> ");

  
  //12.5 mV/dps, Clockwise rotation is positive output 
  #define dps2mV 0.0125
  int analoggyro = (int)((2.5+(gyro.data.z-337)*dps2mV)/5*1024);
  Serial.println(analoggyro); Serial.print(" ");

#endif //LSM330DLGYRO 

  delay(100);
}
