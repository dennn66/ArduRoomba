/***************************************************************
   Motor driver function definitions - by James Nugen
   *************************************************************/
#define __INA1 6
#define __INB1 8
#define __EN1DIAG1 7
#define __CS1 A0
#define __INA2 12
#define __INB2 4
#define __EN2DIAG2 7
#define __CS2 A1

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


void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int leftSpeed, int rightSpeed);
void setMotorEnableFlag(boolean isEnabled);
boolean isMotorFault();

#if defined ARDUINO_NON_QUADRO_ENC_COUNTER
  int getMotorDirection(int i);
#endif
