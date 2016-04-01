/* *************************************************************
   Encoder definitions
   
   Add a "#if defined" block to this file to include support for
   a particular encoder board or library. Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.
   
   ************************************************************ */
#include "common.h"
   
#ifdef USE_BASE

#if defined ROBOGAIA
  /* The Robogaia Mega Encoder shield */
  #include "MegaEncoderCounter.h"

  /* Create the encoder shield object */
  MegaEncoderCounter encoders = MegaEncoderCounter(4); // Initializes the Mega Encoder Counter in the 4X Count mode
  
  /* Wrap the encoder reading function */
  long readEncoder(int i) {
    if (i == LEFT) return encoders.YAxisGetCount();
    else return encoders.XAxisGetCount();
  }

  /* Wrap the encoder reset function */
  void resetEncoder(int i) {
    if (i == LEFT) return encoders.YAxisReset();
    else return encoders.XAxisReset();
  }
#elif defined TINYQED
  /* Tiny QED Encoder Counter - https://github.com/KristofRobot/TQED */
  #include <Wire.h>
  #include <TQED.h>
  
  /* Create the encoder counter objects */
  TQED qedLeft(0x38);     // Change the I2C address as needed
  TQED qedRight(0x36);    // Change the I2C address as needed

  /* Wrap the encoder reading function */
  long readEncoder(int i) {
    if (i == LEFT) return qedLeft.getCount();
    else return qedRight.getCount();
  }

  /* Wrap the encoder reset function */
  void resetEncoder(int i) {
    if (i == LEFT){
      qedLeft.resetCount();
      return;
    } else { 
      qedRight.resetCount();
      return;
    }
  }
#elif defined ARDUINO_ENC_COUNTER
  volatile long left_enc_pos = 0L;
  volatile long right_enc_pos = 0L;
  static const int8_t ENC_STATES [] = {0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0};  //encoder lookup table
    
  /* Interrupt routine for LEFT encoder, taking care of actual counting */
  ISR (PCINT2_vect){
  	static uint8_t enc_last=0;
        
    	enc_last <<=2; //shift previous state two places
  	enc_last |= (PIND & (3 << 2)) >> 2; //read the current state into lowest 2 bits
  
  	left_enc_pos += ENC_STATES[(enc_last & 0x0f)];
  }
  
  /* Interrupt routine for RIGHT encoder, taking care of actual counting */
  ISR (PCINT1_vect){
        static uint8_t enc_last=0;
          	
        enc_last <<=2; //shift previous state two places
  	enc_last |= (PINC & (3 << 4)) >> 4; //read the current state into lowest 2 bits
  
  	right_enc_pos += ENC_STATES[(enc_last & 0x0f)];
  }
  
  /* Wrap the encoder reading function */
 
  long readEncoder(int i) {
    if (i == LEFT) return left_enc_pos;
    else return right_enc_pos;
  }
  
  /* Wrap the encoder reset function */
  void resetEncoder(int i) {
    if (i == LEFT){
      left_enc_pos=0L;
      return;
    } else { 
      right_enc_pos=0L;
      return;
    }
  }
#elif defined ARDUINO_NON_QUADRO_ENC_COUNTER
  volatile unsigned int left_enc_pos = 0x7FFF;
  volatile unsigned int right_enc_pos = 0x7FFF;
  

  void leftEncoderTick(){
    left_enc_pos += getMotorDirection(LEFT);  
  }
  
  void rightEncoderTick(){
    right_enc_pos += getMotorDirection(RIGHT);
  }  


  /* Wrap the encoder reading function */
 
  long readEncoder(int i) {
    if (i == LEFT) return left_enc_pos;
    else return right_enc_pos;
  }
  
  /* Wrap the encoder reset function */
  void resetEncoder(int i) {
    if (i == LEFT){
      left_enc_pos=0x7FFF;
      return;
    } else { 
      right_enc_pos=0x7FFF;
      return;
    }
  }
#else
  #error A encoder driver must be selected!
#endif

/* Wrap the encoder reset function */
void resetEncoders() {
  resetEncoder(LEFT);
  resetEncoder(RIGHT);
}

#endif

