/*
 * IRremote: IRrecvDemo - demonstrates receiving IR codes with IRrecv
 * An IR detector/demodulator must be connected to the input RECV_PIN.
 * Version 0.1 July, 2009
 * Copyright 2009 Ken Shirriff
 * http://arcfn.com
 */

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <IRremote.h>
#define BAUDRATE     115200

int RECV_PIN = 32;
int VCC_PIN = 34;
int GND_PIN = 36;

IRrecv irrecv(RECV_PIN);

decode_results results;

void setup()
{
  Serial.begin(BAUDRATE);
  pinMode(VCC_PIN, OUTPUT);
  pinMode(GND_PIN, OUTPUT);
  digitalWrite(GND_PIN, LOW);
  digitalWrite(VCC_PIN, HIGH);
  irrecv.enableIRIn(); // Start the receiver
}

void loop() {
  if (irrecv.decode(&results)) {
    Serial.println(millis());
    Serial.println(results.value, HEX);
    irrecv.resume(); // Receive the next value
  }
  //delay(100);
}
