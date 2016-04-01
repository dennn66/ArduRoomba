
#ifndef OI_CONTROLLER_H
#define OI_CONTROLLER_H

#include "common.h"

#include  "encoder_driver.h"
// Create an instance of the Open Interface library
// Pass in the serial port to communicate with the
// controller. Mega's could use Serial1/2/3 etc Serial for Arduinos
OpenInterface oi(&OI_UART);

//variables for the battery data
int batVoltage=6000, batCurrent=-250, batRemain=4500, batCharge=4500;
uint8_t batTemp=24;
int irVelocity=200;
/*
REMOTE_OPCODES = {
    # Remote control.
    129: 'left',
    130: 'forward',
    131: 'right',
    132: 'spot',
    133: 'max',
    134: 'small',
    135: 'medium',
    136: 'large',
    136: 'clean',
    137: 'pause',
    138: 'power',
    139: 'arc-left',
    140: 'arc-right',
    141: 'drive-stop',
    # Scheduling remote.
    142: 'send-all',
    143: 'seek-dock',
    # Home base.
    240: 'reserved',
    242: 'force-field',
    244: 'green-buoy',
    246: 'green-buoy-and-force-field',
    248: 'red-buoy',
    250: 'red-buoy-and-force-field',
    252: 'red-buoy-and-green-buoy',
    254: 'red-buoy-and-green-buoy-and-force-field',
    255: 'none',
    }
*/    
#undef IRREMOTE
#ifdef IRREMOTE

#define RECV_PIN 32
#include <IRremote.h>
IRrecv irrecv(RECV_PIN);
decode_results results;
uint8_t lastopcode=0;
long lastoptime=0;

#define IRPRESSED 0x4AB0F7B6
#define IRSTOP 0xB4B41AE5
#define IRFW 0xB4B4E21D
#define IRLT 0xB4B49A65
#define IRRT 0xB4B45AA5
#define IRBW 0xB4B412ED

#endif //IRREMOTE

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
uint8_t cliff[NSONARS]={OI_WALL, OI_CLIFF_LEFT, OI_CLIFF_RIGHT};
uint8_t cliff_signals[NSONARS]={OI_WALL_SIGNAL, OI_CLIFF_LEFT_SIGNAL, OI_CLIFF_RIGHT_SIGNAL};
 
/**
 * Receive one byte of data containing a bitmap of Digital Output states
 */
void controlDigitalOutput(uint8_t ioBitmap)
{
}

/**
 * Receive one byte of data containing a bitmap of PWM Output states, i.e on/off
 */
void controlPwmOutput(uint8_t pwmBitmap)
{
}

/**
 * Receive three bytes of data containing a PWM Output levels, range 0-128
 */
void controlPwmLevels(uint8_t one, uint8_t two, uint8_t three)
{
}

/**
 * Receive three bytes of data containing LED bit, Power LED colour, Power LED intensity
 * ledsBitmap bits 1,3 for Play and Advance LEDs respectively
 * powerColour 0-255, 0=green, 255=red. Sets Power Led colour to any level between.
 * powerBrightness 0-255, 0=off, 255=full. Sets Power LED brightness
 */
void controlLeds(uint8_t ledsBitmap, uint8_t powerColour, uint8_t powerBrightness)
{
}



/**
 * Callback function to handle songs
 * Notes are passed as a array consisting of pairs; pitch & duration.
 * See Create Open Interface V2 documentation (op code
 */
void songPlay(uint8_t notes[])
{
}

/**
 * Callback function to handle drivePWM commands
 */
void drivePWM(int leftPWM, int rightPWM)
{
      lastMotorCommand =  millis();

      
      if (isMotorDisabled) {
        setMotorEnableFlag(true);
        isMotorDisabled=false;
      }
     
      int leftSpeed = leftPWM;
      int rightSpeed = rightPWM;
   
      //ensure speeds are below MAX_PWM speed
      if (leftSpeed > MAX_PWM) leftSpeed = MAX_PWM;
      else if (leftSpeed < -MAX_PWM) leftSpeed = -MAX_PWM;
      
      if (rightSpeed > MAX_PWM) rightSpeed = MAX_PWM;
      else if (rightSpeed < -MAX_PWM) rightSpeed = -MAX_PWM;
       uint8_t pwmdebug;
      pwmdebug = (uint8_t)(leftSpeed/10+40);
      oi.setSensorValue(OI_DIRT_DETECTOR_LEFT, pwmdebug);    
      pwmdebug = (uint8_t)(rightSpeed/10+40);
      oi.setSensorValue(OI_DIRT_DETECTOR_RIGHT, pwmdebug);    
 //     setMotorSpeeds(MAX_PWM, MAX_PWM);
      setMotorSpeeds(leftSpeed, rightSpeed);
   
      moving = 0; //no need to do PID}
      
}

#define VELOCITY2PWMA 1.6 //8.7
#define VELOCITY2PWMB 10.5 //56.4/5.361
#define VELOCITY_MAX 240  // mm/s
#define VELOCITY_MIN 20  // mm/s
#define WHEELS_SEPARATION 235.0  // mm
#define VELOCITY2TICKS 0.1552  // 1 mm/s = 0.1552 tics/PID_INTERVAL => (4656 encoder tics/m)/(1000 mm/m)/ (30 Hz PID_RATE)


/**
 * Callback function to handle driveDirect commands
 */
void driveDirect(int leftVel, int rightVel)
{
   int velocity_max = VELOCITY_MAX;
   if(oi.getSensorValue(OI_SENSOR_OI_MODE) == OI_MODE_SAFE ||
        oi.getSensorValue(OI_SENSOR_OI_MODE) == OI_MODE_PASSIVE) {
        int obsticle_cm = min(oi.getSensorValue(OI_WALL_SIGNAL), min(oi.getSensorValue(OI_CLIFF_LEFT_SIGNAL), oi.getSensorValue(OI_CLIFF_RIGHT_SIGNAL)));
        velocity_max = (leftVel+rightVel<10)?min(VELOCITY_MAX, obsticle_cm*10):min(VELOCITY_MAX, obsticle_cm*10/5);
        velocity_max = max(velocity_max, VELOCITY_MIN);
   }
   oi.setSensorValue(OI_MOTOR_OVERCURRENTS, (uint8_t) velocity_max);
   if(leftVel == 0&& rightVel==0) {
      /* Reset the auto stop timer */
      lastMotorCommand = millis();
      if (isMotorDisabled) {
          setMotorEnableFlag(true);
          isMotorDisabled=false;
      }
      setMotorSpeeds(0, 0);
      resetPID();

      moving = 0;
      return;
  }
  int max_abs_velocity = max(abs(leftVel), abs(rightVel));
  int left_velocity = leftVel;
  int right_velocity = rightVel;

  if(max_abs_velocity>velocity_max){
      float correction = (float)velocity_max/(float)max_abs_velocity;
      left_velocity = (int)((float)left_velocity*correction);
      right_velocity = (int)((float)right_velocity*correction);
  }
#ifdef PID_REGULATOR
    moving = 1;
    leftPID.targetTicksPerFrame = (int)((float)left_velocity*VELOCITY2TICKS);
    rightPID.targetTicksPerFrame = (int)((float)right_velocity*VELOCITY2TICKS);
#else

    drivePWM((int) (VELOCITY2PWMA*(float)left_velocity+VELOCITY2PWMB), (int) (VELOCITY2PWMA*(float)right_velocity+VELOCITY2PWMB));
#endif //PID_REGULATOR
}


/**
 * Callback function to handle drive commands
 */
void driveOI(int velocity, int radius)
{
  if(velocity == 0) {
      /* Reset the auto stop timer */
      lastMotorCommand = millis();
      if (isMotorDisabled) {
          setMotorEnableFlag(true);
          isMotorDisabled=false;
      }
      
      setMotorSpeeds(0, 0);
      resetPID();

      moving = 0;
      return;
  }
  else if(velocity > VELOCITY_MAX) {velocity = VELOCITY_MAX;}
  else if(velocity < -VELOCITY_MAX) {velocity = -VELOCITY_MAX;}

  switch(radius){
    case 32767: 
    case 32768:
       driveDirect( velocity,  velocity);
       return;
    case 1:
       driveDirect( -velocity,  velocity);
       return;
    case -1:
       driveDirect( velocity,  -velocity);
       return;
    default:
    
       break;
  }    
  int max_velocity = (int)((float)VELOCITY_MAX/(1.0+(float)abs(radius)/WHEELS_SEPARATION/2));
  int corrected_velocity = (max_velocity<abs(velocity))?(velocity/abs(velocity))*max_velocity:velocity;
  int delta_velocity = corrected_velocity*WHEELS_SEPARATION/radius/2;
  int left_velocity = corrected_velocity-delta_velocity;
  int right_velocity = corrected_velocity+delta_velocity;
  driveDirect(left_velocity, right_velocity);
}

/**
 * Bootup commands
 */
void setupOI()
{
  // Init interface at 57600 Baud
  oi.init(BAUDRATE);

  // All oi.registerXXXXXX functions register a function that the controller
  // will
  // Register callback to drivePWM() defined in this sketch
  oi.registerDrivePWM(drivePWM);

  // Register callback to driveDirect() defined in this sketch
  oi.registerDriveDirect(driveDirect);

  // Register callback to drive() defined in this sketch
  oi.registerDrive(driveOI);

  // Register callback to play songs, gets passed an array of notes, timings
  oi.registerSong(songPlay);

  // Register callback to handle LED Control commands, gets passed an 8bit bitmap of the led states
  oi.registerLedControl(controlLeds);

  // Register callback to handle PWM output commands, gets passed 3 bytes with 0-128 PWM levels for each output
  oi.registerLsdPwmControl(controlPwmLevels);

  // Register callback to handle PWM output commands, gets passed 1 bitmap byte with on/off values for each output
  oi.registerLsdOutputCallback(controlPwmOutput);

  // Register callback to handle Digital output commands, gets passed 1 bitmap byte with on/off values for each output
  oi.registerDigitalOutputCallback(controlDigitalOutput);

 // Setup battery information
  // Voltage (mV), Current (mA)(Neg = discharging), Charge (mAh), Charge Remaining (mAh), Temp (C)
  oi.setBatteryInfo(batVoltage, batCurrent, batRemain, batCharge, batTemp);

  // Setup counters 
  oi.updateEncoderCounts(0,0);
  uint8_t oi_mode = OI_MODE_SAFE;
  oi.setSensorValue(OI_SENSOR_OI_MODE, oi_mode);

  
  
}
void spinOI()
{
  /**
   * Handle requests from the OI controller
   */
  oi.handle();
//DEBUG_UART.println("Spin oi ...");
  /**
   * Read data from inputs to update sensor values if they differ from previous values.
   */
  oi.updateBatteryVoltageCurrent(batVoltage, batCurrent);
  oi.updateBatteryTemperature(batTemp);
  oi.updateBatteryChargeEstimate(batCharge);
  oi.updateEncoderCounts(readEncoder(LEFT),readEncoder(RIGHT));
  oi.updateAnalogInput(500); // Gyro
  
  
   /* gyro section */
#ifdef LSM330DLGYRO
#define ZOFFSET 337
  gyro.read();
  //12.5 mV/dps, Clockwise rotation is positive output 
  #define dps2mV 0.0125
  int analoggyro = (int)((2.5+(gyro.data.z-ZOFFSET)*dps2mV)/5*1024);
  oi.updateAnalogInput(analoggyro);
#else
  oi.updateAnalogInput(512);
#endif //LSM330DLGYRO 

#ifdef IRREMOTE

  if (irrecv.decode(&results)) {
    uint8_t opcode;
    switch(results.value){
      case IRSTOP: 
      case IRBW: 
        opcode = OP_STOP;
        break;
      case IRFW: 
        opcode = OP_FORWARD;
        break;
      case IRLT: 
        opcode = OP_LEFT;
        break;
      case IRRT: 
        opcode = OP_RIGHT;
        break;
      case IRPRESSED: 
        opcode = lastopcode;
        break;
      default:
        opcode = 255;
        break;
    }
    oi.setSensorValue(OI_SENSOR_IR, opcode);
    lastopcode=opcode;
    irrecv.resume(); // Receive the next value
    lastoptime=millis();
  } else if(millis()-lastoptime>200){    
      lastopcode=0;
      oi.setSensorValue(OI_SENSOR_IR, lastopcode);
      //alert(2, 95, 10);
      lastoptime=millis();
  }
  
#endif //IRREMOTE


    int zero=0;
    int tmprad;
    switch(oi.getSensorValue(OI_SENSOR_IR)){
      case OP_STOP: 
        oi.setSensorValue(OI_SENSOR_REQ_LEFT_VEL, zero);    
        oi.setSensorValue(OI_SENSOR_REQ_RIGHT_VEL, zero);    
        oi.setSensorValue(OI_SENSOR_REQ_VEL, zero);    
        oi.setSensorValue(OI_SENSOR_REQ_RADIUS, zero);    
        driveDirect(0, 0);
        break;
      case OP_FORWARD: 
        oi.setSensorValue(OI_SENSOR_REQ_LEFT_VEL, irVelocity);    
        oi.setSensorValue(OI_SENSOR_REQ_RIGHT_VEL, irVelocity);    
        oi.setSensorValue(OI_SENSOR_REQ_VEL, zero);    
        oi.setSensorValue(OI_SENSOR_REQ_RADIUS, zero);    
       driveDirect(irVelocity, irVelocity);
        break;
      case OP_LEFT: 
        oi.setSensorValue(OI_SENSOR_REQ_LEFT_VEL, zero);    
        oi.setSensorValue(OI_SENSOR_REQ_RIGHT_VEL, zero);    
        oi.setSensorValue(OI_SENSOR_REQ_VEL, irVelocity); 
        tmprad = 1;   
        oi.setSensorValue(OI_SENSOR_REQ_RADIUS, tmprad);    
        driveOI(irVelocity, 1);
//       driveDirect(-irVelocity, irVelocity);
        break;
      case OP_RIGHT: 
        oi.setSensorValue(OI_SENSOR_REQ_LEFT_VEL, zero);    
        oi.setSensorValue(OI_SENSOR_REQ_RIGHT_VEL, zero);    
        oi.setSensorValue(OI_SENSOR_REQ_VEL, irVelocity);  
        tmprad = -1;  
        oi.setSensorValue(OI_SENSOR_REQ_RADIUS, tmprad);    
        driveOI(irVelocity, -1);
        break;
      default:
        break;
    }
  //DEBUG_UART.println("checkpoint spin.oi");

  if(millis()-lastpingtime>PINGINTERVAL){
    int sonar_cm=0;
    uint8_t sonar_state=0;
    lastsonar++;
    if(lastsonar == NSONARS) lastsonar=0;
   //sonar_cm = 60;
    sonar_cm = Ping(sonars[lastsonar]);
    oi.setSensorValue(cliff_signals[lastsonar], sonar_cm);
    sonar_state = (sonar_cm < 6)?1:0;
    oi.setSensorValue(cliff[lastsonar], sonar_state);
    //DEBUG_UART.print("lastsonar: ");DEBUG_UART.print(lastsonar); DEBUG_UART.print(" ");
    //DEBUG_UART.print("sonar_cm: ");  DEBUG_UART.print(sonar_cm);    DEBUG_UART.print(" ");
    //DEBUG_UART.print("sonar_state"); DEBUG_UART.println(sonar_state);
    lastpingtime = millis();
  }
   // DEBUG_UART.println("checkpoint2 spin.oi");

#define RIGHT_BUMP_BIT 0
#define LEFT_BUMP_BIT  1
  uint8_t bumps_state=(uint8_t)oi.getSensorValue(OI_SENSOR_WHEELDROPS_BUMPS);
  if(oi.getSensorValue(OI_WALL)==1|oi.getSensorValue(OI_CLIFF_LEFT)==1) {
    bumps_state |= (1<<LEFT_BUMP_BIT);
  } else {
    bumps_state &= ~(1<<LEFT_BUMP_BIT);
  }  
  if(oi.getSensorValue(OI_WALL)==1|oi.getSensorValue(OI_CLIFF_RIGHT)==1) {
    bumps_state |= (1<<RIGHT_BUMP_BIT);
  } else {
    bumps_state &= ~(1<<RIGHT_BUMP_BIT);
  }  
  oi.setSensorValue(OI_SENSOR_WHEELDROPS_BUMPS, bumps_state);

}

#endif //OI_CONTROLLER_H

