/***************************************************************
   Motor driver definitions
   
   Add a "#elif defined" block to this file to include support
   for a particular motor driver.  Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.
   
   *************************************************************/

#ifdef USE_BASE
  #if defined ARDUINO_NON_QUADRO_ENC_COUNTER
    int left_direction=0;
    int right_direction=0;
  #endif
   
#if defined POLOLU_VNH5019
  /* Include the Pololu library */
  #include "DualVNH5019MotorShield.h"

  /* Create the motor driver object */
//  DualVNH5019MotorShield drive;
  
   DualVNH5019MotorShield drive(__M1DIR, __M1PWM, __M1FB,
                                                __M2DIR, __M2PWM,  __M2FB,
                                                __nD1, __nD2, __nSF);
  
  /* Wrap the motor driver initialization */
  void initMotorController() {
    drive.init();
  }
  #if defined ARDUINO_NON_QUADRO_ENC_COUNTER
  int sign(int value){
    if(value>0) return 1;
    if(value<0) return -1;
    return 0;
  }
 int getMotorDirection(int i){
       if (i == LEFT) return left_direction;
    else return right_direction;
 };  
 #endif
  
  /* Wrap the drive motor set speed function */
  void setMotorSpeed(int i, int spd) {
  #if defined ARDUINO_NON_QUADRO_ENC_COUNTER
    if (i == LEFT) left_direction=sign(spd);
    else right_direction=sign(spd);
 #endif
    if (i == LEFT) drive.setM1Speed(spd);
    else drive.setM2Speed(spd);
  }

  // A convenience function for setting both motor speeds
  void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    setMotorSpeed(LEFT, leftSpeed);
    setMotorSpeed(RIGHT, rightSpeed);
  }
  
    void setMotorEnableFlag(boolean isEnabled){
 
  }
  
  boolean isMotorFault(){
    return false;
  }

#elif defined POLOLU_MC33926
  /* Include the Pololu library */
  #include "DualMC33926MotorShield.h"

  /* Create the motor driver object */
 // M1DIR, M1PWM, M1FB, M2DIR, M2PWM, M2FB, nD2, nSF
  DualMC33926MotorShield drive(__M1DIR, __M1PWM, __M1FB, __M2DIR, __M2PWM, __M2FB, __nD1,__nD2, __nSF);
  
  /* Wrap the motor driver initialization */
  void initMotorController() {
    drive.init();
  }
  
 #if defined ARDUINO_NON_QUADRO_ENC_COUNTER
  int sign(int value){
    return (value < 0) ? -1 : (value > 0);
  }
 int getMotorDirection(int i){
      return (i == LEFT) ? left_direction : right_direction;
 };  
 #endif

  /* Wrap the drive motor set speed function */
  void setMotorSpeed(int i, int spd) {
   #if defined ARDUINO_NON_QUADRO_ENC_COUNTER
    if (i == LEFT) left_direction=sign(spd);
    else right_direction=sign(spd);
   #endif
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
#else
  #error A motor driver must be selected!
#endif

#endif
