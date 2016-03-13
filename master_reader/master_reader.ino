// Wire Master Reader
// by Nicholas Zambetti <http://www.zambetti.com>

// Demonstrates use of the Wire library
// Reads data from an I2C/TWI slave device
// Refer to the "Wire Slave Sender" example for use with this

// Created 29 March 2006

// This example code is in the public domain.
#include <Wire.h>
#define LSM330DL_ADDRESS              (0b1101000)   //

#define LSM330DL_ADDRESS 	    0b1101000 //(0b0011000) //
#define L3GD20_REGISTER_WHO_AM_I            (0x0F)   // 11010011   r D3
#define L3GD20_REGISTER_CTRL_REG1           (0x20)
#define L3GD20_REGISTER_CTRL_REG4           (0x23)
#define L3GD20_REGISTER_OUT_X_L           (0x28)


void setup()
{
  Wire.begin();        // join i2c bus (address optional for master)
  Serial.begin(57600);  // start serial for output
      Serial.println("start");         // print the character
         Serial.println(0x6B, BIN);         // print the character

    Wire.beginTransmission(LSM330DL_ADDRESS );
    Wire.write((byte)L3GD20_REGISTER_WHO_AM_I);
    Wire.endTransmission();
    Wire.requestFrom((byte)LSM330DL_ADDRESS , (byte)1);
    byte value = Wire.read();
    Wire.endTransmission();
    Serial.println(value, HEX);         // print the character
    Wire.beginTransmission(LSM330DL_ADDRESS );
    Wire.write((byte)L3GD20_REGISTER_CTRL_REG1);
    Wire.write(0x0F);
    Wire.endTransmission();
//write8(L3GD20_REGISTER_CTRL_REG4, 0x00);
    Wire.beginTransmission(LSM330DL_ADDRESS );
    Wire.write((byte)L3GD20_REGISTER_CTRL_REG4);
    Wire.write(0x20);
    Wire.endTransmission();


}
#define HMC5883L_Address 0x1E
#define ConfigurationRegisterA 0x00
#define ConfigurationRegisterB 0x01
#define ModeRegister 0x02
#define DataRegisterBegin 0x03

//#define LSM330DL_ADDRESS 		(0b1101000)
void loop()
{
  
  Wire.beginTransmission(LSM330DL_ADDRESS );
  Wire.write(0x20);
  Wire.endTransmission(); 
  Wire.beginTransmission(LSM330DL_ADDRESS );
  Wire.requestFrom(LSM330DL_ADDRESS , 1);
  while(Wire.available())    // slave may send less than requested
  { 
    byte c = Wire.read(); // receive a byte as character
    Serial.print(c, BIN);         // print the character
     Serial.print(" ");         // print the character
 }
  Wire.endTransmission();
  Serial.println(" ");         // print the character
 
   uint8_t xhi, xlo, ylo, yhi, zlo, zhi;

    Wire.beginTransmission(LSM330DL_ADDRESS );
    // Make sure to set address auto-increment bit
    Wire.write(L3GD20_REGISTER_OUT_X_L | 0x80);
    Wire.endTransmission();
    Wire.requestFrom((byte)LSM330DL_ADDRESS , (byte)6);
    
    // Wait around until enough data is available
    while (Wire.available() < 6);
    
    xlo = Wire.read();
    xhi = Wire.read();
    ylo = Wire.read();
    yhi = Wire.read();
    zlo = Wire.read();
    zhi = Wire.read();

  // Shift values to create properly formed integer (low byte first)
  Serial.print(xlo | (xhi << 8));
  Serial.print(" "); 
  Serial.print(ylo | (yhi << 8));
  Serial.print(" "); 
  Serial.println(zlo | (zhi << 8));

 
  delay(500);
}
