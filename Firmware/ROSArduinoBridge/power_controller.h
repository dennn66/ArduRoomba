
#ifndef POWER_CONTROLLER_H
#define POWER_CONTROLLER_H

#include "common.h"


#define BATVOLTAGE            A4       //
#define BATCURRENT            A2       //
#define LEFTMOTORCURRENT     A0
#define RIGHTMOTORCURRENT    A1
#define BATCHARGECURRENT    A3
#define BATCURRENTMID 698

int batVoltageRAW=0, batCurrentRAW=0,  batChargeCurrentRAW=0, leftMotorCurrentRAW=0, rightMotorCurrentRAW=0;
long batConsumption = 0, batChargeCurrentI = 0, batCurrentI = 0;

long lastcurrenttime=0;
// Convert the rate into an interval 
const int VOLTAGE_INTERVAL = 200;
//Track the next time we get voltage
unsigned long nextVoltageTimer = VOLTAGE_INTERVAL;
#define ADC2VOLTAGE            26.72 //488* x = 13.04 

const int CURRENT_INTERVAL = 500;
//Track the next time we get current
unsigned long nextCurrentTimer = CURRENT_INTERVAL;
#define ADC2CURRENT            42 // 2290 = x*54
#define ADC2MOTORCURRENT            4 // 900 = x*230

#define ADC2CHRGE 72 // -5*x = 0.36
/*
    REFS1 REFS0 ADLAR MUX4 MUX3 MUX2 MUX1 MUX0
     Bit 5  ADLAR: ADC Left Adjust Result
    //   (admux & 11100000) | (channel & 00000111)
     Bit 7:6  REFS1:0: Reference Selection Bits
 
    REFS1   REFS0   Voltage Reference Selection
        0       0   AREF, Internal VREF turned off
        0       1   AVCC with external capacitor at AREF pin
        1       0   Internal 1.1V Voltage Reference with external capacitor at AREF pin
        1       1   Internal 2.56V Voltage Reference with external capacitor at AREF pin */
#define ADMUXREF 0b01000000

void pc_init(){
    DIDR0 = 0xFF; // digital inputs disabled       
    ADCSRA = 0b10000111; // AD-converter on, interrupt disabled, prescaler = 128
    ADCSRB = 0b00000000; // AD channels MUX on
    lastcurrenttime = 0;
}


 int myanalogRead(uint8_t pin)
{
	uint8_t low, high;
	// the MUX5 bit of ADCSRB selects whether we're reading from channels
	// 0 to 7 (MUX5 low) or 8 to 15 (MUX5 high).
//	ADCSRB = (ADCSRB & ~(1 << MUX5)) | (((pin >> 3) & 0x01) << MUX5);

	// set the analog reference (high two bits of ADMUX) and select the
	// channel (low 4 bits).  this also sets ADLAR (left-adjust result)
	// to 0 (the default).
//	ADMUX = (myanalog_reference << 6) | (pin & 0x07);
	ADMUX = (ADMUXREF) | (pin & 0x07);
	// start the conversion
        bitWrite(ADCSRA, ADSC, 1); // Start the conversion by setting bit 6 (=ADSC) in ADCSRA

	// ADSC is cleared when the conversion finishes
	while (bit_is_set(ADCSRA, ADSC));

	// we have to read ADCL first; doing so locks both ADCL
	// and ADCH until ADCH is read.  reading ADCL second would
	// cause the results of each conversion to be discarded,
	// as ADCL and ADCH would be locked when it completed.
	low  = ADCL;
	high = ADCH;
	// combine the two bytes
	return (high << 8) | low;
}
void pc_spin(int *batVoltage, int *batCurrent, int *batRemain){
  if (millis() > nextVoltageTimer) {
    nextVoltageTimer = millis() + VOLTAGE_INTERVAL;
    batVoltageRAW = myanalogRead(BATVOLTAGE);
    batChargeCurrentRAW = myanalogRead(BATCHARGECURRENT);
    leftMotorCurrentRAW = myanalogRead(LEFTMOTORCURRENT);
    rightMotorCurrentRAW = myanalogRead(RIGHTMOTORCURRENT);
    *batVoltage = (int)((float)batVoltageRAW*ADC2VOLTAGE);
    long currenttime = millis();
    nextCurrentTimer = currenttime + CURRENT_INTERVAL;
    //batCurrentRAW = (lastcurrenttime==0)?(myanalogRead(0)-BATCURRENTMID):((7*batCurrentRAW+myanalogRead(0)-BATCURRENTMID)/8);
    batCurrentRAW = myanalogRead(BATCURRENT); //-BATCURRENTMID;
    if(lastcurrenttime!=0) batConsumption += (currenttime-lastcurrenttime)*batCurrentRAW/1000;
    lastcurrenttime = currenttime;
    batChargeCurrentI = (batChargeCurrentI*31+((511-batChargeCurrentRAW)*ADC2CHRGE))/32;
    batCurrentI = (batCurrentI*7+(batCurrentRAW-511))/8;
    *batRemain -= (batCurrentRAW)*ADC2CHRGE;
    *batCurrent=rightMotorCurrentRAW;//(int)((float)(batCurrentRAW));//*ADC2CURRENT);
    DEBUG_UART.print(nextVoltageTimer);DEBUG_UART.print(" ");
    DEBUG_UART.print("BATVOLTAGE ");DEBUG_UART.print(*batVoltage);DEBUG_UART.print(" ");
    DEBUG_UART.print("BATCHARGECURRENT ");DEBUG_UART.print(batChargeCurrentI);DEBUG_UART.print(" ");
    DEBUG_UART.print("LEFTMOTORCURRENT ");DEBUG_UART.print(leftMotorCurrentRAW*4);DEBUG_UART.print(" ");
    DEBUG_UART.print("RIGHTMOTORCURRENT ");DEBUG_UART.print(rightMotorCurrentRAW*4);DEBUG_UART.print(" ");
    DEBUG_UART.print("BATCURRENT ");DEBUG_UART.println(batCurrentI*ADC2CURRENT);
  }
}
#endif //POWER_CONTROLLER_H
