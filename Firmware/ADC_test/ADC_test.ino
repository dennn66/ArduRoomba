
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif



int batVoltage=6000, batCurrent=-250, batRemain=4500, batCharge=4500;
int batVoltageRAW=0, batCurrentRAW=0,  batChargeCurrentRAW=0, leftMotorCurrentRAW=0, rightMotorCurrentRAW=0;
int batConsumption = 0;
 
uint8_t batVoltageSem =1;
long lastcurrenttime=0;
long lastvoltagetime=0;
 
#define BATCURRENTADMUX           0b10000000
#define BATVOLTAGEADMUX           0b01000001
#define LEFTMOTORCURRENTADMUX     0b10000010
#define RIGHTMOTORCURRENTADMUX    0b10000011
#define BATCHARGECURRENTADMUX     0b10000100
#define BATCURRENTMID 698
 
 
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
#define ADMUXREF 0b11000000
// #define FLOW
void setup() {
     Serial.begin(57600);
    Serial.println("Start");

 
    //TIMSK0 = 0x00; // disable timer (causes anoying interrupts)
 #ifdef FLOW
    DIDR0 = 0xFF; // digital inputs disabled       
    ADMUX = 0b01000000; //0xC0; // measuring on ADC0, left adjust, internal AVCC ref
//ADEN ADSC ADATE ADIF ADIE ADPS2 ADPS1 ADPS0 
//  1  1    0      0    0    1    1      1 0xC7  - analogRead
//  1  0    1      0    1    1    1      1       - FLOW  
    ADCSRA = 0b10101111; //0xAC; // AD-converter on, interrupt enabled, prescaler = 128
//- ACME  –  –  MUX5  ADTS2  ADTS1  ADTS0 
    ADCSRB = 0x00; // AD channels MUX on, free running mode
    bitWrite(ADCSRA, 6, 1); // Start the conversion by setting bit 6 (=ADSC) in ADCSRA
    sei(); // Set global interrupt flag
#else
    DIDR0 = 0xFF; // digital inputs disabled       
    ADCSRA = 0b10000111; // AD-converter on, interrupt disabled, prescaler = 128
    ADCSRB = 0b00000000; // AD channels MUX on
#endif
   lastcurrenttime = 0;
   lastvoltagetime = millis();

}
 
 uint8_t myanalog_reference = DEFAULT;

void myanalogReference(uint8_t mode)
{
	// can't actually set the register here because the default setting
	// will connect AVCC and the AREF pin, which would cause a short if
	// there's something connected to AREF.
	myanalog_reference = mode;
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

 
 
void loop() {
    if (millis()-lastvoltagetime > 1000) {
        batVoltageSem =1;
        lastvoltagetime = millis();
#ifndef FLOW
 /*   Bit 7:6  REFS1:0: Reference Selection Bits
 
    REFS1   REFS0   Voltage Reference Selection
        0       0   AREF, Internal VREF turned off
        0       1   AVCC with external capacitor at AREF pin
        1       0   Internal 1.1V Voltage Reference with external capacitor at AREF pin
        1       1   Internal 2.56V Voltage Reference with external capacitor at AREF pin */

         batVoltageRAW = myanalogRead(1);
       long currenttime = millis();
        //batCurrentRAW = (lastcurrenttime==0)?(myanalogRead(0)-BATCURRENTMID):((7*batCurrentRAW+myanalogRead(0)-BATCURRENTMID)/8);
        batCurrentRAW = myanalogRead(0)-BATCURRENTMID;
        if(lastcurrenttime!=0) batConsumption += (currenttime-lastcurrenttime)*batCurrentRAW/1000;
        lastcurrenttime = currenttime;
         batChargeCurrentRAW = myanalogRead(4);
         leftMotorCurrentRAW = myanalogRead(2);
        rightMotorCurrentRAW = myanalogRead(3);
#endif
        Serial.print("\tV: ");Serial.print(batVoltageRAW); 
        Serial.print("\tI: ");Serial.print(batCurrentRAW); 
        Serial.print("\tCh:");Serial.print(batChargeCurrentRAW);
        Serial.print("\tlI:");Serial.print(leftMotorCurrentRAW); 
        Serial.print("\trI:");Serial.print(rightMotorCurrentRAW); 
        Serial.print("\tCons:");Serial.println(batConsumption);
    }
    delay(500);
}
 
 
/*** Interrupt routine ADC ready ***/
ISR(ADC_vect) {
   long currenttime ;
   uint8_t low, high;
   uint8_t mux;
   mux = ADMUX;
   low  = ADCL;
   high = ADCH;
	// combine the two bytes
   int aval=(high << 8) | low;
    switch(mux){
        case BATCURRENTADMUX:
            //currenttime = millis();
            //batConsumption += ((currenttime-lastcurrenttime)*(batCurrentRAW+aval-BATCURRENTMID))/2;
            batCurrentRAW=(9*batCurrentRAW+aval)/10; //-BATCURRENTMID;
            //lastcurrenttime = currenttime;
            break;
        case BATVOLTAGEADMUX:
            batVoltageRAW = (9*batVoltageRAW+aval)/10;
            batVoltageSem =0;
            break;
        case BATCHARGECURRENTADMUX:
            batChargeCurrentRAW=(9*batChargeCurrentRAW+aval)/10;//-BATCURRENTMID;
            break;
        case LEFTMOTORCURRENTADMUX:
            leftMotorCurrentRAW=(9*leftMotorCurrentRAW+aval)/10;
            break;
        case RIGHTMOTORCURRENTADMUX:
            rightMotorCurrentRAW=(9*rightMotorCurrentRAW+aval)/10;
            break;
        default:
            break;
    }
    if(batVoltageSem ==1) {
        ADMUX = BATVOLTAGEADMUX;
    } else {
        switch(mux){
        case BATCURRENTADMUX:
            ADMUX = BATCHARGECURRENTADMUX;
            break;
        case BATVOLTAGEADMUX:
            ADMUX = BATCURRENTADMUX;
            break;
        case BATCHARGECURRENTADMUX:
            ADMUX = LEFTMOTORCURRENTADMUX;
            break;
        case LEFTMOTORCURRENTADMUX:
            ADMUX = RIGHTMOTORCURRENTADMUX;
            break;
        case RIGHTMOTORCURRENTADMUX:
            ADMUX = BATCURRENTADMUX;
            break;
        default:
            break;
        }
    }
}
