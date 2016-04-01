

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif



#define DEBUG_UART Serial

/* Serial port baud rate */
#define BAUDRATE     57600


#define SONAR_LEFT 45  //#sonar_left:   pin: 45
//#define SONAR_FRONT_LEFT 47  //#sonar_front_left:   pin: 47
#define SONAR_FRONT_CENTER 49  //#sonar_front_center:   pin: 49
//#define SONAR_FRONT_RIGHT 51  //#sonar_front_right:   pin: 51
#define SONAR_RIGHT 53  //#sonar_right:   pin: 53
#define NSONARS 3
#define PINGINTERVAL 40 //ms
long lastpingtime=0;
uint8_t lastsonar=0;
uint8_t sonars[NSONARS]={SONAR_FRONT_CENTER,SONAR_LEFT, SONAR_RIGHT};
 
 
 
float microsecondsToCm(long microseconds)
{
  // The speed of sound is 340 m/s or 29 microseconds per cm.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}

long Ping(int pin) {
  
   long duration, range;

  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  delayMicroseconds(2);
  digitalWrite(pin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pin, LOW);

  // The same pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(pin, INPUT);
  duration = pulseIn(pin, HIGH);

  // convert the time into meters
  range = microsecondsToCm(duration);
  
  return(range);
}
 
void setup() {
  // put your setup code here, to run once:
  DEBUG_UART.begin(BAUDRATE);
  DEBUG_UART.println("Starting up S...");

}

void loop() {
  // put your main code here, to run repeatedly:
  DEBUG_UART.println("checkpoint spin.oi");

    int sonar_cm=0;
    uint8_t sonar_state=0;
    lastsonar++;
    if(lastsonar == NSONARS) lastsonar=0;
   //sonar_cm = 60;
   sonar_cm = Ping(sonars[lastsonar]);
    sonar_state = (sonar_cm < 6)?1:0;
    DEBUG_UART.println("checkpoint2 spin.oi");
    DEBUG_UART.print(lastsonar);
    DEBUG_UART.print(" ");
    DEBUG_UART.print(sonar_cm);
    DEBUG_UART.print(" ");
    DEBUG_UART.println(sonar_state);
  
    DEBUG_UART.println("checkpoint2 spin.oi");
    delay(1000);
}
