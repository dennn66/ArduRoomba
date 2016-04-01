#undef WATCHDOG

#ifdef WATCHDOG
  #include <avr/wdt.h>
#endif

#undef PIEZOBUZZER

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
#define BAUDRATE     57600

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif


#define RECV_PIN 11
#include <IRremote.h>
IRrecv irrecv(RECV_PIN);
decode_results results;
uint8_t opcode=0;
uint8_t lastopcode=0;
long lastoptime=0;

#define IRPRESSED 0x4AB0F7B6
#define IRSTOP 0xB4B41AE5
#define IRFW 0xB4B4E21D
#define IRLT 0xB4B49A65
#define IRRT 0xB4B45AA5
#define IRBW 0xB4B412ED

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

void getOIcode(){
  if (irrecv.decode(&results)) {

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
    lastopcode=opcode;
    irrecv.resume(); // Receive the next value
    lastoptime=millis();
  } else if(millis()-lastoptime>200){    
      lastopcode=0;
      opcode = 0;
      //alert(2, 95, 10);
      lastoptime=millis();
  }
}


void setup() {
  Serial.begin(BAUDRATE);
  Serial.println("Starting up...");
    irrecv.enableIRIn(); // Start the receiver
}


void loop() {
#ifdef WATCHDOG
    //watchdog protection; if we dont reset this every X ms
    //arduino will reset
    wdt_reset();
#endif
getOIcode();
  Serial.write(opcode);
//  Serial.println(opcode);
  
  delay(100);
}
