/* Define single-letter commands that will be sent by the PC over the
   serial link.
*/

#ifndef COMMANDS_H
#define COMMANDS_H

#define ANALOG_READ      'a'
#define GET_BAUDRATE     'b'
#define PIN_MODE         'c'
#define DIGITAL_READ     'd'
#define READ_ENCODERS    'e'
#define ALL_SERVO_READ   'f'
#define MAGNETO_READ     'g'
#define SERVO_ATTACH     'h'
#define ALL_SERVO_ATTACH 'i'
#define ALL_SERVO_STATE  'j'
#define ALL_SERVO_WRITE  'k'
#define MOTOR_SPEEDS     'm'
#define SEND_PWM         'n'
#define PING             'p'
#define RESET_ENCODERS   'r'
#define SERVO_WRITE      's'
#define SERVO_READ       't'
#define UPDATE_PID       'u'
#define DIGITAL_WRITE    'w'
#define ANALOG_WRITE     'x'

/* Variable initialization */
// A pair of varibles to help parse serial commands (thanks Fergs)
int arg = 0;
int index = 0;

// Variable to hold an input character
char chr;

// Variable to hold the current single-character command
char cmd;

// Character arrays to hold the first and second arguments
char argv1[16];
char argv2[16];

// The arguments converted to integers
long arg1;
long arg2;

/* Clear the current command parameters */
void resetCommand() {
  cmd = NULL;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg = 0;
  index = 0;
}

/* Run a command.  Commands are defined in commands.h */
int runCommand() {
  int i = 0;
  char *p = argv1;
  char *str;
  int pid_args[4];
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);
  
  switch(cmd) {
  case GET_BAUDRATE:
    Serial.println(BAUDRATE);
    break;
  case ANALOG_READ:
    Serial.println(analogRead(arg1));
    break;
  case DIGITAL_READ:
    Serial.println(digitalRead(arg1));
    break;
  case ANALOG_WRITE:
    analogWrite(arg1, arg2);
    Serial.println("OK"); 
    break;
  case DIGITAL_WRITE:
    if (arg2 == 0) digitalWrite(arg1, LOW);
    else if (arg2 == 1) digitalWrite(arg1, HIGH);
    Serial.println("OK"); 
    break;
  case PIN_MODE:
    if (arg2 == 0) pinMode(arg1, INPUT);
    else if (arg2 == 1) pinMode(arg1, OUTPUT);
    Serial.println("OK");
    break;
  case PING:
    Serial.println(Ping(arg1));
    break;
#ifdef USE_SERVOS
  case SERVO_WRITE:
    setPositionDegree(arg1, arg2);
    Serial.println("OK");
    break;
  case SERVO_READ:
    Serial.println(servos[arg1].read());
    break;
  case ALL_SERVO_READ:
    for (i = 0; i < N_SERVOS; i++) {
       Serial.print(servos[i].read());
       Serial.print(" ");
    }
    Serial.println("");
    break;
  case ALL_SERVO_STATE:
    for (i = 0; i < N_SERVOS; i++) {
       Serial.print(servos[i].attached());
       Serial.print(" ");
    }
    Serial.println("");
    break;
  case ALL_SERVO_WRITE:
    while ((str = strtok_r(p, ":", &p)) != '\0') {
       if(strlen(str) >0) setPositionDegree(i, atoi(str));
       i++;
    }
    Serial.println("OK");
    break;

    for (i = 0; i < N_SERVOS; i++) {
       Serial.print(servos[i].attached());
       Serial.print(" ");
    }
    Serial.println("");
    break;
  case SERVO_ATTACH:
    servoAttach(arg1, arg2);
    Serial.println("OK");
    break;
  case ALL_SERVO_ATTACH:
    //int i;
    for (i = 0; i < N_SERVOS; i++) {
       servoAttach(i, arg1);
    }
    Serial.println("OK");
    break;

#endif //USE_SERVOS
    
#ifdef USE_BASE

#ifdef MAGNETO_HMC5883L
  case MAGNETO_READ:
    Serial.print(readMilliGauss_OnThe_XAxis());
    Serial.print(" ");
    Serial.print(readMilliGauss_OnThe_YAxis());
    Serial.print(" ");
    Serial.println(readMilliGauss_OnThe_ZAxis());
    break;
#endif //MAGNETO_HMC5883L
  case READ_ENCODERS:
    Serial.print(readEncoder(LEFT));
    Serial.print(" ");
    Serial.println(readEncoder(RIGHT));
    break;
   case RESET_ENCODERS:
    resetEncoders();
    resetPID(); //also need to reset PID
    Serial.println("OK");
    break;
  case MOTOR_SPEEDS:
    /* Reset the auto stop timer */
    lastMotorCommand = millis();
    if (isMotorDisabled) {
        setMotorEnableFlag(true);
        isMotorDisabled=false;
    }
    
    if (arg1 == 0 && arg2 == 0) {
      setMotorSpeeds(0, 0);
      moving = 0;
    }
    else moving = 1;
    leftPID.targetTicksPerFrame = arg1;
    rightPID.targetTicksPerFrame = arg2;
    Serial.println("OK"); 
    break;
  case UPDATE_PID:
    while ((str = strtok_r(p, ":", &p)) != '\0') {
       pid_args[i] = atoi(str);
       i++;
    }
    setPIDParams(pid_args[0], pid_args[1], pid_args[2], pid_args[3], PID_RATE);
    Serial.println("OK");
    break;
  case SEND_PWM:
    { //need brackets to restrict scope of newly created variables to this case statement
      lastMotorCommand =  millis();
      
      if (isMotorDisabled) {
        setMotorEnableFlag(true);
        isMotorDisabled=false;
      }
     
      int leftSpeed = arg1;
      int rightSpeed = arg2;
     
      //ensure speeds are below MAX_PWM speed
      if (leftSpeed > MAX_PWM) leftSpeed = MAX_PWM;
      else if (leftSpeed < -MAX_PWM) leftSpeed = -MAX_PWM;
      if (rightSpeed > MAX_PWM) rightSpeed = MAX_PWM;
      else if (rightSpeed < -MAX_PWM) rightSpeed = -MAX_PWM;
      
      setMotorSpeeds(leftSpeed, rightSpeed);
      moving = 0; //no need to do PID
      Serial.println("OK");
      break;
    }
#endif //USE_BASE
  default:
    Serial.println("Invalid Command");
    break;
  }
}
void getCommand(HardwareSerial* serialPort){
    while (serialPort->available() > 0) {
    
    // Read the next character
    chr = serialPort->read();

    // Terminate a command with a CR
    if (chr == 13) {
      if (arg == 1) argv1[index] = NULL;
      else if (arg == 2) argv2[index] = NULL;
      runCommand();
      resetCommand();
    }
    // Use spaces to delimit parts of the command
    else if (chr == ' ') {
      // Step through the arguments
      if (arg == 0) arg = 1;
      else if (arg == 1)  {
        argv1[index] = NULL;
        arg = 2;
        index = 0;
      }
      continue;
    }
    else {
      if (arg == 0) {
        // The first arg is the single-letter command
        cmd = chr;
      }
      else if (arg == 1) {
        // Subsequent arguments can be more than one character
        argv1[index] = chr;
        index++;
      }
      else if (arg == 2) {
        argv2[index] = chr;
        index++;
      }
    }
  }
}

#endif


