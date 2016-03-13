/* 
*  Functions and type-defs for PID control.
*
*  Based on the Beginner PID's series by Brett Beauregard - http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
*  Adapted to use ideal velocity form or position form.
*
*  Originally adapted from Mike Ferguson's ArbotiX code which lives at:
*   
*  http://vanadium-ros-pkg.googlecode.com/svn/trunk/arbotix/
*/
#ifdef VELOCITY_PID
  /* PID setpoint info for a Motor */
  typedef struct {
    int targetTicksPerFrame;    // target speed in ticks per frame
    long encoder;                  // encoder count
    long prevEnc;                  // last encoder count
    int prevInput;                 // last input
    int prevPrevInput;             // input before last input
    int prevError;                 // last error
    int output;                   // last motor setting
  }
  SetPointInfo;
#elif defined POSITION_PID
  /* PID setpoint info For a Motor */
  typedef struct {
    int targetTicksPerFrame;    // target speed in ticks per frame
    long encoder;                  // encoder count
    long prevEnc;                  // last encoder count
  
    /*
    * Using previous input (PrevInput) instead of PrevError to avoid derivative kick,
    * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
    */
    int prevInput;                // last input
    //int prevErr;                   // last error
  
    /*
    * Using integrated term (ITerm) instead of integrated error (Ierror),
    * to allow tuning changes,
    * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
    */
    long iTerm;                    //integrated term
    int output;                    // last motor setting
  }
  SetPointInfo;
#endif

SetPointInfo leftPID, rightPID;

/* PID Parameters 
* Do not SET these directly here, unless you know what you are doing 
* Use setPIDParameters() instead
*/
int Kp = 0;    
int Kd = 0;
int Ki = 0;      
int Ko = 1; 

unsigned char moving = 0; // is the base in motion?

/*
* Initialize PID variables to zero to prevent startup spikes
* when turning PID on to start moving
* In particular, assign both encoder and prevEnc the current encoder value
* See http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
* Note that the assumption here is that PID is only turned on
* when going from stop to moving, that's why we can init everything on zero.
*/
void resetPID(){
#ifdef VELOCITY_PID
   leftPID.targetTicksPerFrame = 0;
   leftPID.encoder = readEncoder(0);
   leftPID.prevEnc = leftPID.encoder;
   leftPID.output = 0;
   leftPID.prevInput = 0;
   leftPID.prevPrevInput = 0;
   leftPID.prevError = 0;

   rightPID.targetTicksPerFrame = 0;
   rightPID.encoder = readEncoder(1);
   rightPID.prevEnc = rightPID.encoder;
   rightPID.output = 0;
   rightPID.prevInput = 0;
   rightPID.prevPrevInput = 0;
   rightPID.prevError = 0;
#elif defined POSITION_PID
   leftPID.targetTicksPerFrame = 0;
   leftPID.encoder = readEncoder(0);
   leftPID.prevEnc = leftPID.encoder;
   leftPID.output = 0;
   leftPID.prevInput = 0;
   leftPID.iTerm = 0;

   rightPID.targetTicksPerFrame = 0;
   rightPID.encoder = readEncoder(1);
   rightPID.prevEnc = rightPID.encoder;
   rightPID.output = 0;
   rightPID.prevInput = 0;
   rightPID.iTerm = 0;
#endif
}

#ifdef VELOCITY_PID
/* PID routine to compute the next motor commands */
void doPID(SetPointInfo * p) {
  int error;
  int output;
  int input;

  input = p->encoder - p->prevEnc;
  error = p->targetTicksPerFrame - input;
  
  /*
  * Use velocity form rather than position form.
  *
  * Avoid derivative kick (derivative on measurement):
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
  *
  * see https://groups.google.com/forum/#!topic/diy-pid-control/1Tkwp9e_8co for full derivation 
  */
  output = p->output + (Kp * (error - p->prevError) + Ki * error - Kd * (input - 2*p->prevInput + p->prevPrevInput)) / Ko;
  
  /*version robust against too aggressive setpoint tracking (with proportional on measurement)*/
  //output = p->output + (Kp * (p->prevInput - input) + Ki * error - Kd * (input - 2*p->prevInput + p->prevPrevInput)) / Ko;
  
  /*
  * Limit output.
  * 
  * Avoid motor moving back when requesting forward movement, and vice versa (avoid oscillating around 0)
  * Also avoid sending output of 0 (stopping motors)
  *
  * Stop accumulating integral error when output is limited.
  */
  if (p->targetTicksPerFrame > 0 && output < MIN_PWM) 
    output = MIN_PWM;
  else if (p->targetTicksPerFrame < 0 && output > -MIN_PWM) 
    output = -MIN_PWM;
  else if (output > MAX_PWM)
    output = MAX_PWM;
  else if (output < -MAX_PWM)
    output = -MAX_PWM;
    
  p->output = output;
  p->prevEnc = p->encoder;
  p->prevPrevInput = p->prevInput;
  p->prevInput = input;
  p->prevError = error;
  
  /*Serial.print(output);
  Serial.print(" ");
  Serial.print(error);
  Serial.print(" ");
  Serial.println(input);*/
}
#elif defined POSITION_PID
/* PID routine to compute the next motor commands */
void doPID(SetPointInfo * p) {
  int error;
  int output;
  int input;

  input = p->encoder - p->prevEnc;
  error = p->targetTicksPerFrame - input;
  
  p->iTerm += (Ki * error) / Ko;
  if (p->iTerm > (MAX_PWM-MIN_PWM)) p->iTerm = MAX_PWM;
  else if (p->iTerm < (-MAX_PWM+MIN_PWM)) p->iTerm = -MAX_PWM;

  /*
  * Avoid derivative kick and allow tuning changes,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
  output = (((long)Kp) * error - Kd * (input - p->prevInput))/ Ko + p->iTerm;
  p->prevEnc = p->encoder;

  /*
  * Accumulate Integral error *or* Limit output.
  * 
  * Avoid motor moving back when requesting forward movement, and vice versa (avoid oscillating around 0)
  * Also avoid sending output of 0 (stopping motors)
  *
  * Stop accumulating integral error when output is limited.
  */
  if (p->targetTicksPerFrame > 0){
    output += MIN_PWM;
    if (output < MIN_PWM) output = MIN_PWM;
  } else if (p->targetTicksPerFrame < 0){
    output += -MIN_PWM;
    if (output > -MIN_PWM) output = -MIN_PWM;
  } 
  
  if (output > MAX_PWM)
    output = MAX_PWM;
  else if (output < -MAX_PWM)
    output = -MAX_PWM;

  p->output = output;
  p->prevInput = input;
}
#endif

/* Read the encoder values and call the PID routine */
void updatePID() {
  /* Read the encoders */
  leftPID.encoder = readEncoder(0);
  rightPID.encoder = readEncoder(1);
    
  /* If we're not moving there is nothing more to do */
  if (!moving){
    /*
    * Reset PIDs, to prevent startup spikes,
    * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
    * Most importantly, keep Encoder and PrevEnc synced; use that as criteria whether we need reset
    */
    if (leftPID.prevEnc != leftPID.encoder || rightPID.prevEnc != rightPID.encoder) resetPID();
    return;
  }
  
  /* Compute PID update for each motor */
  doPID(&rightPID);
  doPID(&leftPID);

  /* Set the motor speeds accordingly */
  setMotorSpeeds(leftPID.output, rightPID.output);
}


/* Set PID parameters */
//Assuming pid_rate and Kx parameters all given in s
//Doing some effort to keep things in integer math, through use of Ko
void setPIDParams(int newKp, int newKd, int newKi, int newKo, int pidRate){
    Kp = newKp * pidRate;
    Ki = newKi;
    Kd = newKd * pidRate * pidRate;  
    Ko = newKo * pidRate;
}




