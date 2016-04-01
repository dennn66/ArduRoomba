/*
 Copyright (c) 2011 Andy "Bob" Brockhurst

 Permission is hereby granted, free of charge, to any person obtaining
 a copy of this software and associated documentation files (the
 "Software"), to deal in the Software without restriction, including
 without limitation the rights to use, copy, modify, merge, publish,
 distribute, sublicense, and/or sell copies of the Software, and to
 permit persons to whom the Software is furnished to do so, subject to
 the following conditions:

 The above copyright notice and this permission notice shall be
 included in all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
#ifndef OpenInterface_h
#define OpenInterface_h

//OP Codes
#define OC_START                0x80 //128
#define OC_BAUD                 0x81 //129
#define OC_CONTROL              0x82 //130
#define OC_SAFE                 0x83 //131
#define OC_FULL                 0x84 //132
#define OC_POWER                0x85 //133
#define OC_SPOT                 0x86 //134
#define OC_COVER                0x87 //135
#define OC_DEMO                 0x88 //136
#define OC_DRIVE                0x89 //137
#define OC_LOW_SIDE_DRIVERS     0x8a //138
#define OC_LEDS                 0x8b //139
#define OC_SONG                 0x8c //140
#define OC_PLAY_SONG            0x8d //141
#define OC_SENSORS              0x8e //142
#define OC_COVER_AND_DOCK       0x8f //143
#define OC_PWM_LOW_SIDE_DRIVERS 0x90 //144
#define OC_DIRECT_DRIVE         0x91 //145
#define OC_DRIVE_PWM            0x92 //146
#define OC_DIGITAL_OUTPUTS      0x93 //147
#define OC_STREAM               0x94 //148
#define OC_QUERY_LIST           0x95 //149
#define OC_PAUSE_RESUME_STREAM  0x96 //150
#define OC_SEND_IR              0x97 //151
#define OC_SCRIPT               0x98 //152
#define OC_PLAY_SCRIPT          0x99 //153
#define OC_SHOW_SCRIPT          0x9a //154
#define OC_WAIT_TIME            0x9b //155
#define OC_WAIT_DISTANCE        0x9c //156
#define OC_WAIT_ANGLE           0x9d //157
#define OC_WAIT_EVENT           0x9e //158


// Sensor Packet Groups
#define PACKET_GRP_7_26   0
#define PACKET_GRP_7_16   1
#define PACKET_GRP_17_20  2
#define PACKET_GRP_21_26  3
#define PACKET_GRP_27_34  4
#define PACKET_GRP_35_42  5
#define PACKET_GRP_7_42   6
//Roomba 500 extentions
#define PACKET_GRP_7_58   100
#define PACKET_GRP_43_58   101
#define PACKET_GRP_46_51   106
#define PACKET_GRP_54_58   107

// OI Modes
#define OI_MODE_OFF     0
#define OI_MODE_PASSIVE 1
#define OI_MODE_SAFE    2
#define OI_MODE_FULL    3

// Sensors
#define OI_SENSOR_WHEELDROPS_BUMPS  7   //7 _x.bumps_wheeldrops
#define OI_WALL                     8   //8 _x.wall
#define OI_CLIFF_LEFT               9   //9 _x.cliff_left,
#define OI_CLIFF_FRONT_LEFT         10  //10 _x.cliff_front_left,
#define OI_CLIFF_FRONT_RIGHT        11  //11 _x.cliff_front_right,
#define OI_CLIFF_RIGHT              12  //12 _x.cliff_right,
//13 _x.virtual_wall, 
#define OI_MOTOR_OVERCURRENTS       14  //14 _x.motor_overcurrents,
#define OI_DIRT_DETECTOR_LEFT       15  //15 _x.dirt_detector_left,
#define OI_DIRT_DETECTOR_RIGHT      16  //16 _x.dirt_detector_right,
#define OI_SENSOR_IR                17  //17 _x.remote_opcode,
//18 _x.buttons, 
#define OI_DISTANCE                 19  //19 _x.distance,
#define OI_ANGLE                    20  //20 _x.angle,
//21 _x.charging_state,
#define OI_SENSOR_BAT_VOLTAGE       22  // 22 _x.voltage
#define OI_SENSOR_BAT_CURRENT       23  //23 _x.current
#define OI_SENSOR_BAT_TEMPERATURE   24  //24 _x.temperature
#define OI_SENSOR_BAT_CHARGE        25  //25 _x.charge
#define OI_SENSOR_BAT_CHARGE_REMAIN 26  //26 _x.capacity
#define OI_WALL_SIGNAL              27  //27 _x.wall_signal,
#define OI_CLIFF_LEFT_SIGNAL        28  //28 _x.cliff_left_signal,
#define OI_CLIFF_FRONT_LEFT_SIGNAL  29  //29_x.cliff_front_left_signal,
#define OI_CLIFF_FRONT_RIGHT_SIGNAL 30  //30 _x.cliff_front_right_signal,
#define OI_CLIFF_RIGHT_SIGNAL       31  //31 _x.cliff_right_signal,
#define OI_USER_DIGITAL_INPUTS      32  //32 _x.user_digital_inputs,
#define OI_USER_ANALOG_INPUT        33  //33 _x.user_analog_input, 
//34 _x.charging_sources_available
#define OI_SENSOR_OI_MODE           35  // 35 _x.oi_mode
//36 _x.song_number, 
//37 _x.song_playing, 
//38 _x.number_of_stream_packets
#define OI_SENSOR_REQ_VEL           39	//39 _x.requested_velocity
#define OI_SENSOR_REQ_RADIUS        40  //40 _x.requested_radius
#define OI_SENSOR_REQ_RIGHT_VEL     41  //41 _x.requested_right_velocity
#define OI_SENSOR_REQ_LEFT_VEL      42  //42 _x.requested_left_velocity

/* Roomba 500 extentions */
#define OI_ENCODER_COUNTS_LEFT      43  //43 self.encoder_counts_left, 
#define OI_ENCODER_COUNTS_RIGHT     44  //44 self.encoder_counts_right,
//45 self.light_bumper,
//46 self.light_bump_left, 
//47 self.light_bump_front_left, 
//48 self.light_bump_center_left,
//49 self.light_bump_center_right, 
//50 self.light_bump_front_right, 
//51 self.light_bump_right,
//52 self.ir_opcode_left, 
//53 self.ir_opcode_right,
//54 self.left_motor_current, 
//55 self.right_motor_current,
//56 self.main_brish_current, 
//57 self.side_brush_current,
//58 self.statis


/// Masks wheedrops and bumps OI_SENSOR_REQ_WHEELDROPS_BUMPS (packet id 7)
#define OI_MASK_BUMP_RIGHT  1
#define OI_MASK_BUMP_LEFT   2
#define OI_MASK_DROP_RIGHT  4
#define OI_MASK_DROP_LEFT   8
#define OI_MASK_DROP_CASTER 16

// REMOTE_OPCODES Remote control.
#define OP_LEFT    129 //: 'left',
#define OP_FORWARD    130 //: 'forward',
#define OP_RIGHT    131 //: 'right',
//#define OP_    132 //: 'spot',
//#define OP_    133 //: 'max',
//#define OP_    134 //: 'small',
//#define OP_    135 //: 'medium',
//#define OP_    136 //: 'large',
//#define OP_    136 //: 'clean',
//#define OP_    137 //: 'pause',
//#define OP_    138 //: 'power',
//#define OP_    139 //: 'arc-left',
//#define OP_   140 //: 'arc-right',
#define OP_STOP    141 //: 'drive-stop',
//# Scheduling remote.
//#define OP_    142 //: 'send-all',
//#define OP_    143 //: 'seek-dock',
//    # Home base.
//#define OP_    240 //: 'reserved',
//#define OP_    242 //: 'force-field',
//#define OP_    244 //: 'green-buoy',
//#define OP_    246 //: 'green-buoy-and-force-field',
//#define OP_    248 //: 'red-buoy',
//#define OP_    250 //: 'red-buoy-and-force-field',
//#define OP_    252 //: 'red-buoy-and-green-buoy',
//#define OP_    254 //: 'red-buoy-and-green-buoy-and-force-field',
//#define OP_    255 //: 'none',



// Other stuff
#define READ_TIMEOUT 1000

#include <inttypes.h>
  #if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
  #else
  #include "WProgram.h"
  #endif

/**
 * Callback functions for user implemented features
 */
typedef void (*callbackWithOneInt)(int);
typedef void (*callbackWithTwoInts)(int, int);
typedef void (*callbackWithByteArr)(uint8_t[]);
typedef void (*callbackWithOneByte)(uint8_t);
typedef void (*callbackWithThreeBytes)(uint8_t, uint8_t, uint8_t);

class OpenInterface
{
private:
  /**
   * Sensors
   */
  int     sensorInt[58];
  uint8_t sensor[58];
  uint8_t songs[16][32];
  uint8_t script[101]; //max 100 command length + 1 for storing the length

  HardwareSerial* serial;

  /**
   * Callbacks
   */
  callbackWithTwoInts    drivePWMCallback; //OC_DRIVE_PWM
  callbackWithTwoInts    driveDirectCallback;
  callbackWithTwoInts    driveCallback;
  callbackWithByteArr    songCallback;
  callbackWithOneInt     waitDistanceCallback;
  callbackWithOneInt     waitAngleCallback;
  callbackWithOneByte    waitEventCallback;
  callbackWithOneByte    controlLsdOutputCallback;
  callbackWithOneByte    controlDigitalOutputCallback;
  callbackWithOneByte    sendIrCallback;
  callbackWithThreeBytes controlLedsCallback;
  callbackWithThreeBytes controlLsdPwmCallback;

  void handleOpCode(byte);

  void getSensors();

  void queryList();

  void stream();

  void showScript();

  void scriptSet();

  void scriptPlay();

  void song();

  void songPlay();

  void drive();

  void driveDirect();

  void drivePWM();

  void waitAction(callbackWithOneInt);

  void waitTime();

  bool readBytes(uint8_t*, uint8_t);

  bool readByte(uint8_t*);

  bool isDoublePacket(uint8_t packet);

  uint8_t opCodeDataLen(uint8_t opCode);

  void callCallbackWithThreeBytes(callbackWithThreeBytes);

  void callCallbackWithOneByte(callbackWithOneByte);

public:
  /**
   * Constructor
   */
  OpenInterface(HardwareSerial* serialPort = &Serial);

  /**
   * Initialise interface
   */
  void init(long);

  /**
   * Main loop
   */
  void handle();

  /**
   * Register callbacks for user implemented features
   */
  void registerDrivePWM(callbackWithTwoInts f) {drivePWMCallback = f;}
  void registerDriveDirect(callbackWithTwoInts f) {driveDirectCallback = f;}
  void registerDrive(callbackWithTwoInts f) {driveCallback = f;}
  void registerSong(callbackWithByteArr f) {songCallback = f;}
  void registerWaitDistance(callbackWithOneInt f) {waitDistanceCallback = f;}
  void registerWaitAngle(callbackWithOneInt f) {waitAngleCallback = f;}
  void registerWaitEvent(callbackWithOneByte f) {waitEventCallback = f;}
  void registerLedControl(callbackWithThreeBytes f) {controlLedsCallback = f;}
  void registerLsdPwmControl(callbackWithThreeBytes f) {controlLsdPwmCallback = f;}
  void registerLsdOutputCallback(callbackWithOneByte f) {controlLsdOutputCallback = f;}
  void registerDigitalOutputCallback(callbackWithOneByte f) {controlDigitalOutputCallback = f;}
  void registerSendIrCallback(callbackWithOneByte f) {sendIrCallback = f;}
  /**
   * Sensor information
   */
  void setSensorValue(uint8_t, uint8_t);
  void setSensorValue(uint8_t, int);
  int  getSensorValue(uint8_t);

  /**
   * Battery sensor information
   */
  void setBatteryInfo(int, int, int, int, uint8_t);
  void updateBatteryVoltageCurrent(int, int);
  void updateEncoderCounts(int, int);
  void updateBatteryCurrent(int);
  void updateBatteryVoltage(int);
  void updateBatteryTemperature(uint8_t);
  void updateBatteryChargeEstimate(int);
  void updateAnalogInput(int);
};

#endif
