
#define PIEZOBUZZER

#define LEFT              0
#define RIGHT             1


#ifdef PIEZOBUZZER
#define BUZZER                  44     //
#define BUZZER_GND              42     //

void alert(int _times, int _runTime, int _stopTime)
{
    for(int _ct=0; _ct < _times; _ct++)
    {
      delay(_stopTime);
      analogWrite(BUZZER, 20);      // Almost any value can be used except 0 and 255
      delay(_runTime);
      analogWrite(BUZZER, 0);       // 0 turns it off
    }
}
#endif



//1 - left, 2 - right
#define __M1DIR 29 
#define __M1PWM 9
#define __M1FB A3
#define __M2DIR 27
#define __M2PWM 10
#define __M2FB A2
#define __nD1 25
#define __nD2 23
#define __nSF 31

  /* Include the Pololu library */
  #include "DualMC33926MotorShield.h"

  /* Create the motor driver object */
 // M1DIR, M1PWM, M1FB, M2DIR, M2PWM, M2FB, nD2, nSF
  DualMC33926MotorShield drive(__M1DIR, __M1PWM, __M1FB, __M2DIR, __M2PWM, __M2FB, __nD1,__nD2, __nSF);
  
  /* Wrap the motor driver initialization */
  void initMotorController() {
    drive.init();
  }


  /* Wrap the drive motor set speed function */
  void setMotorSpeed(int i, int spd) {
    if (i == LEFT) drive.setM1Speed(spd);
    else drive.setM2Speed(spd);
  }

  // A convenience function for setting both motor speeds
  void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    setMotorSpeed(LEFT, leftSpeed);
    setMotorSpeed(RIGHT, rightSpeed);
  }
  
  void setMotorEnableFlag(boolean isEnabled){
   // drive.setMotorEnableFlag(isEnabled); 
  }
  
  boolean isMotorFault(){
    return false; //drive.getFault();
  }

void setup() {
  Serial.begin(57600);
  Serial.println("Starting up...");

  initMotorController();
  
}

void loop() {
  
  /*
 int reverse = 1;
 int speed = 400;
 
 if (reverse){
    digitalWrite(__M1DIR,HIGH);
    analogWrite(__M1PWM, 255 - speed * 51 / 80); // default to using analogWrite, mapping 400 to 255
  }else{
    digitalWrite(__M1DIR,LOW);
    analogWrite(__M1PWM,speed * 51 / 80); // default to using analogWrite, mapping 400 to 255
  }
 if (reverse){
    digitalWrite(__M2DIR,HIGH);
    analogWrite(__M2PWM, 255 - speed * 51 / 80); // default to using analogWrite, mapping 400 to 255
  }else{
    digitalWrite(__M2DIR,LOW);
    analogWrite(__M2PWM,speed * 51 / 80); // default to using analogWrite, mapping 400 to 255
  }
*/

  for(int i = -400; i<=400; i+=10){
    Serial.println(i);
    setMotorSpeeds(i, i);
    delay(1000);
  }
  

  
}

