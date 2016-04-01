/* *************************************************************
   Encoder driver function definitions - by James Nugen
   ************************************************************ */

#ifndef ENCODER_DRIVER_H
#define ENCODER_DRIVER_H

#include "common.h"
   
   
#ifdef ARDUINO_ENC_COUNTER
  //below can be changed, but should be PORTD pins; 
  //otherwise additional changes in the code are required
  #define LEFT_ENC_PIN_A PD2  //pin 2
  #define LEFT_ENC_PIN_B PD3  //pin 3
  
  //below can be changed, but should be PORTC pins
  #define RIGHT_ENC_PIN_A PC4  //pin A4
  #define RIGHT_ENC_PIN_B PC5   //pin A5
#elif defined ARDUINO_NON_QUADRO_ENC_COUNTER
  void rightEncoderTick();
  void leftEncoderTick();
#endif
   
long readEncoder(int i);
void resetEncoder(int i);
void resetEncoders();

void initEncoders(){
  #ifdef TINYQED
    //set fast I2C bus speed for TQED encoder counters
    TWBR = ((16000000L / 400000L) - 16) / 2;
  #endif //TINYQED
  
  #ifdef ARDUINO_ENC_COUNTER
    //set as inputs
    DDRD &= ~(1<<LEFT_ENC_PIN_A);
    DDRD &= ~(1<<LEFT_ENC_PIN_B);
    DDRC &= ~(1<<RIGHT_ENC_PIN_A);
    DDRC &= ~(1<<RIGHT_ENC_PIN_B);
    
    //enable pull up resistors
    PORTD |= (1<<LEFT_ENC_PIN_A);
    PORTD |= (1<<LEFT_ENC_PIN_B);
    PORTC |= (1<<RIGHT_ENC_PIN_A);
    PORTC |= (1<<RIGHT_ENC_PIN_B);
    
    PCMSK2 |= (1 << LEFT_ENC_PIN_A)|(1 << LEFT_ENC_PIN_B); // tell pin change mask to listen to left encoder pins
    PCMSK1 |= (1 << RIGHT_ENC_PIN_A)|(1 << RIGHT_ENC_PIN_B); // tell pin change mask to listen to right encoder pins
    
    PCICR |= (1 << PCIE1) | (1 << PCIE2);   // enable PCINT1 and PCINT2 interrupt in the general interrupt mask
  #endif //ARDUINO_ENC_COUNTER

  #ifdef ARDUINO_NON_QUADRO_ENC_COUNTER
    attachInterrupt(0, rightEncoderTick, RISING); //pin 2
    attachInterrupt(1, leftEncoderTick, RISING); //pin 3
  #endif //ARDUINO_NON_QUADRO_ENC_COUNTER 
}

#endif //ENCODER_DRIVER_H
