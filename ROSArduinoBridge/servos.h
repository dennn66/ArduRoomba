

/* Define the attachment of any servos here.
   The example shows two servos attached on pins 3 and 5.
*/

#define N_SERVOS 8

Servo servos [N_SERVOS];
byte servoPins [N_SERVOS] = {4,5,6, 7, 8, 11,12, 44};


#define SERVOL_MIN 951 
#define SERVOL_MAX 2415 
#define SERVOR_MIN 700 
#define SERVOR_MAX 1800 
#define SERVOROT_MIN 600 
#define SERVOROT_MAX 2320 
#define SERVOHAND_OPEN_MIN 890 
#define SERVOHAND_CLOSE_MAX 1304 

#define D150A_SERVO_MIN_PUL     535
#define D150A_SERVO_MAX_PUL     2415
#define D009A_SERVO_MIN_PUL     700
#define D009A_SERVO_MAX_PUL     2650

#define SERVO_L             0    //
#define SERVO_R             1    //
#define SERVO_ROT           2    //
#define SERVO_HAND_ROT      3    //
#define SERVO_HAND          4     //
#define SERVO_CAMERA_ROT    5    //
#define SERVO_CAMERA_TILT   6     //

void setPositionMS(int _servoNum, int _position){
  int positionMS;
     if(servos[_servoNum].attached()){  
	switch(_servoNum)
	{
		case SERVO_L:
			_position = constrain(_position, SERVOL_MIN,   SERVOL_MAX);
			break;
		case SERVO_R:
			_position = constrain(_position, SERVOR_MIN,   SERVOR_MAX);
			break;
		case SERVO_ROT:
			_position = constrain(_position, SERVOROT_MIN,   SERVOROT_MAX);
			break;
		case SERVO_HAND_ROT:
			_position = constrain(_position, D009A_SERVO_MIN_PUL,   D009A_SERVO_MAX_PUL);
			break;
		case SERVO_HAND:
			_position = constrain(_position, SERVOHAND_OPEN_MIN,   SERVOHAND_CLOSE_MAX);
			break;
		case SERVO_CAMERA_ROT:
			_position = constrain(_position, D009A_SERVO_MIN_PUL,   D009A_SERVO_MAX_PUL);
			break;
		case SERVO_CAMERA_TILT:
			_position = constrain(_position, D009A_SERVO_MIN_PUL,   D009A_SERVO_MAX_PUL);
			break;
		default: 
			_position = constrain(_position, D009A_SERVO_MIN_PUL,   D009A_SERVO_MAX_PUL);
			break;
	}
	servos[_servoNum].writeMicroseconds(_position);
     }
}    // 

void setPositionDegree(int _servoNum, int _position){
  int positionMS = 1500;
	switch(_servoNum)
	{
		case SERVO_L:
		case SERVO_R:
		case SERVO_ROT:
			positionMS = map(_position, 0, 180, D150A_SERVO_MIN_PUL, D150A_SERVO_MAX_PUL);
			break;
		case SERVO_HAND_ROT:
		case SERVO_HAND:
		case SERVO_CAMERA_ROT:
		case SERVO_CAMERA_TILT:
                        positionMS = map(_position, 0, 180, D009A_SERVO_MIN_PUL, D009A_SERVO_MAX_PUL); 
			break;
		default:  
			break;
	}
	setPositionMS( _servoNum, positionMS);
}    // 

void servoAttach(int _servoNum, int state){
  if(state == 1){
	switch(_servoNum)
	{
		case SERVO_L:
		case SERVO_R:
		case SERVO_ROT:
		        servos[_servoNum].attach(servoPins[_servoNum], D150A_SERVO_MIN_PUL, D150A_SERVO_MAX_PUL);
			break;
		case SERVO_HAND_ROT:
		case SERVO_HAND:
		case SERVO_CAMERA_ROT:
		case SERVO_CAMERA_TILT:
			servos[_servoNum].attach(servoPins[_servoNum], D009A_SERVO_MIN_PUL,   D009A_SERVO_MAX_PUL);
			break;
		default:  
			servos[_servoNum].attach(servoPins[_servoNum], D009A_SERVO_MIN_PUL,   D009A_SERVO_MAX_PUL);
			break;
	}
  } else if(state == 0) {
        servos[_servoNum].detach();    
  }
}    //  

float getPositionRAD(int _servoNum){
  int positionMS;
        positionMS = servos[_servoNum].readMicroseconds();
	switch(_servoNum)
	{
		case SERVO_L:
		case SERVO_R:
		case SERVO_ROT:
			return map(servos[_servoNum].readMicroseconds(),D150A_SERVO_MIN_PUL,  D150A_SERVO_MAX_PUL, -PI/2, PI/2);
			break;
		case SERVO_HAND_ROT:
		case SERVO_HAND:
		case SERVO_CAMERA_ROT:
		case SERVO_CAMERA_TILT:
			return map(servos[_servoNum].readMicroseconds(), D009A_SERVO_MIN_PUL,   D009A_SERVO_MAX_PUL, -PI/2, PI/2);
			break;
		default:  
			return 0;
                        break;
	}
}

