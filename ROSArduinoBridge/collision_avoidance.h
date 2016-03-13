/*
* Collision avoidance routines
* For use without ROS
*/


#ifdef COLLISION_AVOIDANCE
//  #include <NewPing.h> //IMPORTANT: assuming adapted NewPing library, which returns MAX DISTANCE when no distance detected, rather than 0!!
  #include "FastRunningMedian.h"
  #include "collision_avoidance.h"

/*############################ Variable Declarations ###########################################*/
const boolean ENABLE_DEBUG=true;

/******* SONAR variables *****************/
const byte MAX_DISTANCE=75; // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm. No reason to wait longer for ping to return than this.
const byte CHANGE_DIST=50;  //distance to use for changing direction without stopping
const byte MIN_FRONT_DIST=30;  //distance to stop immediately, and seek new direction
const byte DEFAULT_DIST=75; //distance to return when sonar disabled

NewPing sonarFront[3] = { // Sensor object array.
  NewPing(11, 11, MAX_DISTANCE), //left
  NewPing(6, 6, MAX_DISTANCE),   //middle
  NewPing(5, 5, MAX_DISTANCE)   //right
};

const int pingSpeed = 100; // How frequently are we going to send out a ping (in milliseconds). 50ms would be 20 times a second; retriggering of same sonar is best kept above 30ms
unsigned long pingTimer;     // Holds the next ping time.
byte pingPointer=0;            //holds position of next sonar to use; will overflow by design
const byte FRONT_PING_NUM=3;
const byte PING_MEDIAN_NUM=3;  //holds number of pings for median filter

//int frontDistance[FRONT_PING_NUM][PING_MEDIAN_NUM]; //init distance on 0, left, center, right
FastRunningMedian<unsigned int,5, 0> frontDistance[FRONT_PING_NUM];
//int backDistance[3]= {0, 0, 0}; //init distance on 0, left, center, right

const unsigned int MAX_ECHO_TIME = min(MAX_DISTANCE, MAX_SENSOR_DISTANCE) * US_ROUNDTRIP_CM + (US_ROUNDTRIP_CM / 2); // Calculate the maximum distance in uS.

/******** Move variables ***************/
const int TURN_SPEED=50;  //in ticks per frame
const int FORWARD_SPEED=50;

const int FORWARD_DELAY=100; //time to move forward without doing any check
const int STOP_DELAY=1000;
const int DELAY_TURN=1000;  //time to turn - best to make this high enough to do at least a quarter turn 
const int BACKWARD_DELAY=2000;

unsigned long moveTimer=0; //keeps track of whether we need another move action

/* variables to detect whether we are stuck */
byte turnCounterBucket=0;  //leaky bucket to keep track of turns
unsigned long bucketTimer=0;
const int BUCKET_LEAK_SPEED=1000 + 2*STOP_DELAY; //leak one value every x s
const byte MAX_TURN_BUCKET_COUNT = 5;

unsigned long stuckTimer=0;
byte stuckCounterBucket=0;
const byte MAX_STUCK_BUCKET_COUNT=3;   //number of stuck triggers before we react with escape routine
const byte STOP_MAX_STUCK_BUCKET_COUNT = 10;  //number of stuck triggers before we give up entirely
const int STUCK_CHECK_DELAY=500; // how long to wait between "stuck" checks
const byte STUCK_PWM=100;

byte escapeMode=0;

/*######################################################################################################*/

/* 
* Get next distance by pinging using the next sonar in row
* and adding the value to its running median
*/
void getNextDistance(){
  byte sonarPos = pingPointer%FRONT_PING_NUM;
  
  frontDistance[sonarPos].addValue(sonarFront[sonarPos].ping_cm());
  
  pingPointer++; //increment, and let it overflow when it reaches max (255)
}

/* Return minimal front distance detected */
int getMinFrontDistance(){
   return min(frontDistance[2].getMedian(),min(frontDistance[0].getMedian(),frontDistance[1].getMedian()));
}


/*########################### Routines ############################*/
void printDistances(){
  String space=" ";
  Serial.print(frontDistance[0].getMedian());
  Serial.print(space);
  Serial.print(frontDistance[1].getMedian());
  Serial.print(space);
  Serial.println(frontDistance[2].getMedian());
}

void motorCommand(int motor1, int motor2){
  if (motor1 == 0 && motor2 == 0) {
    setMotorSpeeds(0, 0);
    moving = 0;
  } else moving = 1;
    leftPID.targetTicksPerFrame = motor1;
    rightPID.targetTicksPerFrame = motor2;
}

/**************** MOVE procedures ***********************/

void stop(){
  if (ENABLE_DEBUG) Serial.println("S");
  motorCommand(0,0);
  moveTimer = millis() + STOP_DELAY;
}

void turn(boolean isLeft){
  if (isLeft){
      //turn left
      if (ENABLE_DEBUG) Serial.println("L");
      motorCommand(-TURN_SPEED, TURN_SPEED);
  }else{
      //turn right
      if (ENABLE_DEBUG) Serial.println("R");
      motorCommand(TURN_SPEED, -TURN_SPEED);
  }
  
  turnCounterBucket++;
  moveTimer = millis() + DELAY_TURN;
 
}

void turnRight(){
  turn(false);
}

void turnLeft(){
  turn(true);
}

void moveForward(){
  if (ENABLE_DEBUG) Serial.println("F");
  motorCommand(FORWARD_SPEED, FORWARD_SPEED);
  
  //leak bucket
  if (turnCounterBucket > 0 && millis() >= bucketTimer){
    bucketTimer = millis() + BUCKET_LEAK_SPEED;
    turnCounterBucket--;
  }
  
  moveTimer = millis()+FORWARD_DELAY;

}

void moveBackward(){
  if (ENABLE_DEBUG) Serial.println("B");
  motorCommand(-FORWARD_SPEED, -FORWARD_SPEED); //re-using defined forward speeds here
  
  moveTimer = millis()+BACKWARD_DELAY;
}

void escapeTurn(){
    //do about 180 degree turn
    motorCommand(TURN_SPEED, -TURN_SPEED);
    moveTimer = millis()+DELAY_TURN*4;
    turnCounterBucket=0;
}

void turnAfterCheck(boolean doStop){
  //if moving and stop requested, stop and return
  if (moving && doStop) {
    stop();
    return;
  }
  
  if (turnCounterBucket < MAX_TURN_BUCKET_COUNT){
    //if right < left
    if (frontDistance[2].getMedian() < frontDistance[0].getMedian()) 
      turnLeft();
    else 
      turnRight();
  } else {
    escapeTurn();
  }
    
}

//Check whether we got stuck - is the robot actually moving?
boolean isStuck(){
  
  if (!moving) return false; //nothing to do 
  
  //if we've been stuck for too long time, just give up
  if (stuckCounterBucket > STOP_MAX_STUCK_BUCKET_COUNT){
    escapeMode=10; //give up
    if (ENABLE_DEBUG) Serial.println("ES");
    return true;
  }
  
  /*stuck detection routine based on encoder differences; didnt work very well */
  /*if (abs(readEncoder(0) - leftPID.prevEnc) < MIN_TICKS || abs(readEncoder(1) - rightPID.prevEnc) < MIN_TICKS){
    stuckCounterBucket++;
    if (ENABLE_DEBUG) Serial.println("EI");
  } else if (stuckCounterBucket > 0) {
    stuckCounterBucket--; //decrement bucket
    if (ENABLE_DEBUG) Serial.println("ED");
  }*/
  
  /*stuck detection routine based on PWM */
  if ((leftPID.output > STUCK_PWM || rightPID.output > STUCK_PWM)){
    stuckCounterBucket+=2;
    if (ENABLE_DEBUG) Serial.println("EI");
  } else if (stuckCounterBucket > 0) {
    stuckCounterBucket--; //decrement bucket
    if (ENABLE_DEBUG) Serial.println("ED");
  }
  
  if (ENABLE_DEBUG) {
    Serial.print("EC ");
    Serial.println(stuckCounterBucket);
  }
  
  if (stuckCounterBucket > MAX_STUCK_BUCKET_COUNT) {
    //reset bucket
    return true;  
  } else 
     //stuck!
    return false;
}

/* Escape routine */
void escape(){
  if (ENABLE_DEBUG) Serial.println("E");
  
  if (escapeMode != 10) escapeMode++;  //increment escapemode, protecting special give up mode
  
  switch(escapeMode){
    case 1:
        stop();
        break;
    case 2:
        moveBackward();
        break;
    case 3:
        stop();
        break;
    case 4:
        escapeTurn();
        break;
    case 5:
        stop();
        break;
    case 10:  //give up mode; just stop
        stop();
        break;
    default:
        escapeMode=0; //reset
  }
}

/* Move routine */
void move(){
 if (escapeMode>0){
       //continue escape routines
       escape();
 } else if (getMinFrontDistance() < MIN_FRONT_DIST) 
     turnAfterCheck(true);  //stop and turn
 else {     
     moveForward();  //> CHANGE_DIST; move forward
 }
}

void ca_init(){
    //get first distances
    for (int i=0; i<FRONT_PING_NUM*PING_MEDIAN_NUM; i++){
      getNextDistance();
      delay(10); //need to wait at least 30ms between pings; with three sonars; 10ms
    }
    
    if (ENABLE_DEBUG) printDistances();
    
    bucketTimer=millis();
    
    if (ENABLE_DEBUG) Serial1.println("Collision Setup Ready");
}

void ca_spin(){
      //if pingSpeed time has passed, trigger new sonar ping
      if (millis() >= pingTimer) {   // pingSpeed milliseconds since last ping, do another ping.
        //recalibrate pingTimer - to avoid problems
        pingTimer = millis()+pingSpeed;
        getNextDistance();
      }
     
     //check whether we are stuck
     if (millis() >= stuckTimer){
         stuckTimer += STUCK_CHECK_DELAY;
         if (isStuck()) escape(); //if we are supposed to be moving, but not moving, initiate escape procedures
     }
    
    if (millis() >= moveTimer){
        //time for new movement
        if (ENABLE_DEBUG) printDistances();
        move();
    }
}
#endif
