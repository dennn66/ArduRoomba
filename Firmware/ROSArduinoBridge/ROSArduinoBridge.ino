/*********************************************************************
 *  01.04.2016
 *  ROSArduinoBridge
 
    A set of simple serial commands to control a differential drive
    robot and receive back sensor and odometry data. Default 
    configuration assumes use of an Arduino Mega + Pololu motor
    controller shield + Robogaia Mega Encoder shield.  Edit the
    readEncoder() and setMotorSpeed() wrapper functions if using 
    different motor controller or encoder method.

    Created for the Pi Robot Project: http://www.pirobot.org
    and the Home Brew Robotics Club (HBRC): http://hbrobotics.org
    
    Authors: Patrick Goebel, James Nugen
    
    Inspired and modeled after the ArbotiX driver by Michael Ferguson
    
    Extended by Kristof Robot with:
    - DEBUG routines (incl. free ram detection, and logic analyzer debug pins)
    - WATCHDOG timer
    - Motorfault detection and motor coasting stop
    - Additional wheel encoder counter support:
        - Onboard wheel encoder counters
        - TinyQed wheel encoder counters
    - Two types of PID controllers (position and velocity)
    - Collision avoidance routine (to be run without ROS, and three sonars)
    
    Software License Agreement (BSD License)

    Copyright (c) 2012, Patrick Goebel.
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

     * Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.
     * Redistributions in binary form must reproduce the above
       copyright notice, this list of conditions and the following
       disclaimer in the documentation and/or other materials provided
       with the distribution.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include "common.h"

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define LEFT              0
#define RIGHT             1


//#define DEBUG
#undef DEBUG

#ifdef DEBUG
int freeRam ();
#endif //DEBUG

//Roomba Open Interface communication protocol
#define OPENINTERFACE
//#undef OPENINTERFACE

//#undef LSM330DLGYRO
#define LSM330DLGYRO
#ifdef LSM330DLGYRO
#include <Wire.h> 
#include <Adafruit_L3GD20.h>

// Comment this next line to use SPI
#define USE_I2C

#ifdef USE_I2C
  // The default constructor uses I2C
  Adafruit_L3GD20 gyro;
#else
  // To use SPI, you have to define the pins
  #define GYRO_CS 4 // labeled CS
  #define GYRO_DO 5 // labeled SA0
  #define GYRO_DI 6  // labeled SDA
  #define GYRO_CLK 7 // labeled SCL
  Adafruit_L3GD20 gyro(GYRO_CS, GYRO_DO, GYRO_DI, GYRO_CLK);
#endif //USE_I2C
#endif //LSM330DLGYRO


//#define COLLISION_AVOIDANCE
#undef COLLISION_AVOIDANCE

//#define WATCHDOG
#undef WATCHDOG

#ifdef WATCHDOG
  #include <avr/wdt.h>
#endif

#define PIEZOBUZZER

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

//#undef CHECKPOWER 
#define CHECKPOWER 
#ifdef CHECKPOWER
#include "power_controller.h"
#endif


#define USE_BASE      // Enable the base controller code
//#undef USE_BASE     // Disable the base controller code

/* Define the motor controller and encoder library you are using */
#ifdef USE_BASE
   /* The Pololu VNH5019 dual motor driver shield */
//#define POLOLU_VNH5019

   /* The Pololu MC33926 dual motor driver shield */
   #define POLOLU_MC33926

   /* The RoboGaia encoder shield */
   //#define ROBOGAIA

   /* TinyQED encoder counters */
   //#define TINYQED
   
   /* Encoders directly attached to Arduino board */
//   #define ARDUINO_ENC_COUNTER

   /* non-quadro Encoders directly attached to Arduino board */
   #define ARDUINO_NON_QUADRO_ENC_COUNTER
   
   //#define POSITION_PID
   #define VELOCITY_PID
   #define PID_REGULATOR
   
   #undef MAGNETO_HMC5883L

#endif

//#define USE_SERVOS  // Enable use of PWM servos as defined in servos.h
#undef USE_SERVOS     // Disable use of PWM servos


/* Maximum PWM signal */
#define MAX_PWM        400
#define MIN_PWM        220    //lowest PWM before motors start moving reliably






/* Sensor functions */
#include "sensors.h"

/* Include servo support if required */
#ifdef USE_SERVOS
   #include <Servo.h>
   #include "servos.h"

#endif

#ifdef USE_BASE
  /* Motor driver function definitions */
  #include "motor_driver.h"

  /* Encoder driver function definitions */
  #include "encoder_driver.h"

  /* PID parameters and functions */
  #include "diff_controller.h"
  
  /* Initial PID Parameters */
  const int INIT_KP = 50;    
  const int INIT_KD = 1;
  const int INIT_KI = 800;      
  const int INIT_KO = 50;  

  /* Run the PID loop at 30 times per second */
  #define PID_RATE           30     // Hz

  /* Convert the rate into an interval */
  const int PID_INTERVAL = 1000 / PID_RATE;
  
  /* Track the next time we make a PID calculation */
  unsigned long nextPID = PID_INTERVAL;
  
  /* Stop the robot if it hasn't received a movement command
   in this number of milliseconds */
  #define AUTO_STOP_INTERVAL 2000
  long lastMotorCommand = AUTO_STOP_INTERVAL;
  
  boolean isMotorDisabled=false;
  
#ifdef OPENINTERFACE

#include <OpenInterface.h>
#include  "oi_controller.h"

#endif //OPENINTERFACE

#endif //USE_BASE


/* Setup function--runs once at startup. */
void setup() {
#ifdef OPENINTERFACE
  setupOI();
#endif //OPENINTERFACE
  DEBUG_UART.begin(BAUDRATE);
  DEBUG_UART.println("Starting up S...");
 OI_UART.begin(BAUDRATE);
 OI_UART.println("Starting up S1...");
  IR_UART.begin(BAUDRATE);
 IR_UART.println("Starting up S2...");

#ifdef PIEZOBUZZER
  pinMode(BUZZER,   OUTPUT); digitalWrite(BUZZER,   LOW);
  pinMode(BUZZER_GND,   OUTPUT); digitalWrite(BUZZER_GND,   LOW);
  alert(1, 200, 0);
#endif //PIEZOBUZZER
#ifdef IRREMOTE
    irrecv.enableIRIn(); // Start the receiver
#endif //IRREMOTE
#ifdef DEBUG
  DEBUG_UART.println("Starting up...");
  pinMode(40, OUTPUT); //cpu measurement pin for logic analyzer
  pinMode(41, OUTPUT); //PID frequency measurement pin for logic analyzer
  pinMode(42, OUTPUT); //left encoder
  pinMode(43, OUTPUT); //right encoder

  DEBUG_UART.println("CPU and PID frequency measurement PINs active.");
#endif //DEBUG

// Initialize the motor controller if used */
#ifdef USE_BASE
  initEncoders();
  
  #ifdef LSM330DLGYRO
  if (!gyro.begin(gyro.L3DS20_RANGE_2000DPS, LSM330DL_ADDRESS))
  {
    DEBUG_UART.println("Oops ... unable to initialize the LSM330DLGYRO. Check your wiring!");
  }
 #endif //LSM330DLGYRO
 
  #ifdef MAGNETO_HMC5883L
  initMC5883L();
  #endif //MAGNETO_HMC5883L
  
  initMotorController();
  //init PID
  #ifdef PID_REGULATOR
  setPIDParams(INIT_KP, INIT_KD, INIT_KI, INIT_KO, PID_RATE);
  resetPID();
  #endif //PID_REGULATOR
  resetEncoders();
  #ifdef COLLISION_AVOIDANCE
    ca_init();
  #endif //COLLISION_AVOIDANCE
  
#endif //USE_BASE

/* Attach servos if used */
#ifdef USE_SERVOS
  for (int i = 0; i < N_SERVOS; i++) servoAttach(i,1);  
  for (int i = 0; i < N_SERVOS; i++) setPositionDegree(i, 90);
  delay(1500);
  setPositionDegree(0, 170);
  setPositionDegree(1, 45);
  delay(1500);  
  for (int i = 0; i < N_SERVOS; i++) servoAttach(i,0);
#endif //USE_SERVOS

#ifdef WATCHDOG
  wdt_reset();
  wdt_enable(WDTO_1S);  //reset after 1s of inactivity
#endif //WATCHDOG

#ifdef DEBUG
    DEBUG_UART.print("Free Mem:");
    DEBUG_UART.println(freeRam());
#endif //DEBUG

#ifdef CHECKPOWER
   pc_init();
#endif //CHECKPOWER

}


/* Enter the main loop.  Read and parse input from the serial port
   and run any valid commands. Run a PID calculation at the target
   interval and check for auto-stop conditions.
*/

#undef SLINTERFACE
#ifdef SLINTERFACE
/* Include definition of single-letter serial commands */
#include "commands.h"

#endif //SLINTERFACE
void loop() {
  //DEBUG_UART.println("Start loop");
  byte opcode=0;
    while (IR_UART.available() > 0) {
    
    // Read the next character
        opcode = IR_UART.read();
        oi.setSensorValue(OI_SENSOR_IR, opcode);
        DEBUG_UART.println(opcode);
    //    DEBUG_UART.println(opcode);
    }

    
while(0)  {
   for(int i = -MAX_PWM; i<=-MIN_PWM; i+=1){
    DEBUG_UART.print("1 ");
    DEBUG_UART.println(i);
    setMotorSpeeds(0, i); 
    delay(100);
  }
  for(int j = MIN_PWM; j<=MAX_PWM; j+=1){
    DEBUG_UART.print("2 ");
    DEBUG_UART.println(j);
    setMotorSpeeds(0, j); 
    delay(100);
  }
   for(int i = MAX_PWM; i>=MIN_PWM; i-=1){
    DEBUG_UART.print("1 ");
    DEBUG_UART.println(i);
    setMotorSpeeds(0, i); 
    delay(100);
  }
  for(int j = -MIN_PWM; j>=-MAX_PWM; j-=1){
    DEBUG_UART.print("2 ");
    DEBUG_UART.println(j);
    setMotorSpeeds(0, j); 
    delay(100);
  }

}
#ifdef WATCHDOG
    //watchdog protection; if we dont reset this every X ms
    //arduino will reset
    wdt_reset();
#endif //WATCHDOG

#ifndef COLLISION_AVOIDANCE 
#ifndef OPENINTERFACE
  getCommand(&SL_UART);
#endif //OPENINTERFACE
#ifdef OPENINTERFACE
    /**
   * Handle requests from the OI controller
   */
  spinOI();
  //oi.handle();
  //DEBUG_UART.println("End spin.oi");

#endif //OPENINTERFACE

#endif //COLLISION_AVOIDANCE
  
// If we are using base control, run a PID calculation at the appropriate intervals
#ifdef USE_BASE
#ifdef PID_REGULATOR
  if (moving==1 & millis() > nextPID) {
    nextPID = millis() + PID_INTERVAL;
  #ifdef DEBUG
     digitalWrite(41, HIGH - digitalRead(41));
//    PINC = (1<<PC3);   //toggle for PID interval measurement with logic analyzer, pin A3
  #endif //DEBUG
    updatePID();
  }
#endif //PID_REGULATOR

#ifdef MAGNETO_HMC5883L
  tickMC5883L();
#endif  //MAGNETO_HMC5883L



  #ifndef COLLISION_AVOIDANCE
  // Check to see if we have exceeded the auto-stop interval
  if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) {;
    //setMotorSpeeds(0, 0);
    //coast to a stop
    //setMotorEnableFlag(false);
    //isMotorDisabled=true;
    //moving = 0;
  }
  #endif //COLLISION_AVOIDANCE
  
#ifdef CHECKPOWER
  pc_spin(&batVoltage, &batCurrent, &batRemain);
#endif //CHECKPOWER
  
  //detect motor faults
  //if motor is disabled, motor fault is active; so excluding that case
  if (!isMotorDisabled && isMotorFault())
  {
    DEBUG_UART.println("FATAL ERROR: Motor fault - stopping");
    setMotorEnableFlag(false);
    isMotorDisabled=true;
    moving = 0;
 #ifdef WATCHDOG
    wdt_disable();
 #endif //WATCHDOG
    while(1){
      DEBUG_UART.println("FATAL ERROR: Motor fault - stopped");
      delay(1000);
    }
  }
  
  #ifdef COLLISION_AVOIDANCE
    ca_spin()     
  #endif
  
  #ifdef DEBUG
    //toggle cpu measurement for logic analyzer on pin 5
    //PINC = (1<<PC2); //pin A2
    digitalWrite(40, HIGH - digitalRead(40));
  #endif //DEBUG

#endif //USE_BASE
}

#ifdef DEBUG
/* Report free ram */
/* From http://jeelabs.org/2011/05/22/atmega-memory-use/ */
int freeRam () {

  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 

}
#endif //DEBUG



