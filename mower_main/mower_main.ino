#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <MeAuriga.h>

typedef enum {
    FORWARD,
    BACKWARDS,
    LEFT,
    RIGHT,
    STOP
} moveDirection;

typedef enum {
    MOWER_IDLE = 0,
    MOWER_MAN_FORWARD,
    MOWER_MAN_BACKWARDS,
    MOWER_MAN_LEFT,
    MOWER_MAN_RIGHT,
    MOWER_CHANGEMODE
} mowerState_t;

// functions for moving the robot
void moveForward();
void moveBackward();
void turnLeft();
void turnRight();
void stopMotors();


MeLineFollower linefollower_9(9);
MeUltrasonicSensor ultrasonic_10(10);
MeEncoderOnBoard motorLeft(SLOT1);
MeEncoderOnBoard motorRight(SLOT2);
MeLightSensor lightsensor_12(12);
MeGyro gyro_0(0, 0x69);

int distanceToObstacle = 10;
int state = 0;
int mode = 2;

void move(moveDirection direction, int speed)
{
  int leftSpeed = 0;
  int rightSpeed = 0;
  if(direction == FORWARD){
    leftSpeed = -speed;
    rightSpeed = speed;
  }else if(direction == BACKWARDS){
    leftSpeed = speed;
    rightSpeed = -speed;
  }else if(direction == LEFT){
    leftSpeed = -speed;
    rightSpeed = -speed;
  }else if(direction == RIGHT){
    leftSpeed = speed;
    rightSpeed = speed;
  }else if(direction == STOP){
    leftSpeed = 0;
    rightSpeed = 0;
  }
  motorLeft.setTarPWM(leftSpeed);
  motorRight.setTarPWM(rightSpeed);
}

void moveForward() {
    move(FORWARD, 40 / 100.0 * 255);
}

void moveBackward() {
    move(BACKWARDS, 50 / 100.0 * 255);
}

void turnLeft() {
    move(LEFT, 40 / 100.0 * 255);
}

void turnRight() {
    move(RIGHT, 40 / 100.0 * 255);
}

void collision() {
    moveBackward();
    delay(500);
    turnRight();
    delay(500);
}

void isr_process_motorLeft(void)
{
  if(digitalRead(motorLeft.getPortB()) == 0){
    motorLeft.pulsePosMinus();
  }else{
    motorLeft.pulsePosPlus();
  }
}
void isr_process_motorRight(void)
{
  if(digitalRead(motorRight.getPortB()) == 0){
    motorRight.pulsePosMinus();
  }else{
    motorRight.pulsePosPlus();
  }
}

int checkSensors(){
  if(ultrasonic_10.distanceCm() <= distanceToObstacle){
    return 2;
  }else if(linefollower_9.readSensors()!=3){
    return 3;
  }else{
    return 1;
  }
}

int autonomousDriving(int currentState){
  int nextState = 0;
  switch(currentState){
    case 0:
    //StartMotors
    break;

    case 1:
    //Check sensors while driving forward
    nextState = checkSensors(); 
    break;

    case 2:
    //Found obstacle, handle it
    break;

    case 3:
    //Out of bounds, handle it
    break;

    case 4:
    //Turn, handle orientation
    break;

    case 5:
    //Stop the robot and change mode to bt
    break;        
  }
  return nextState;
}

void bluetoothDriving(int nextState){
  switch(nextState){
    case MOWER_IDLE:
    //Stop
    stopMotors();

    break;

    case MOWER_MAN_FORWARD:
    //Drive forward
    moveForward();
    break;

    case MOWER_MAN_BACKWARDS:
    //Drive backward
    moveBackward();
    break;

    case MOWER_MAN_LEFT:
    //Turn left
    turnLeft();
    break;

    case MOWER_MAN_RIGHT:
    //Turn right
    turnRight();
    break;
    
    case MOWER_CHANGEMODE:
    //Stop the robot and change mode to auto
      break;
      
    default:
        break;
  }
}

void autoRandomTurn() {
  int turnLeft = random(1);
  float timeToTurn = random(800, 1800);

  if (turnLeft) {
    delay(500);
    move(LEFT, 40 / 100.0 * 255);
    delay(timeToTurn);
    move(STOP, 0);
  }else if(!turnLeft){
    delay(500);
    move(RIGHT, 40 / 100.0 * 255);
    delay(timeToTurn);
    move(STOP, 0);
  }
}

void setup() {
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);
  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);
  attachInterrupt(motorLeft.getIntNum(), isr_process_motorLeft, RISING);
  attachInterrupt(motorRight.getIntNum(), isr_process_motorRight, RISING);
  gyro_0.begin();
  Serial.begin(115200);
  randomSeed((unsigned long)(lightsensor_12.read() * 123456)); 
  delay(3000);
}

void _loop() {
  motorLeft.loop();
  motorRight.loop();
}

void loop() {
  switch(mode){
    case 0:
    state = autonomousDriving(state);
    break;

    case 1:
    bluetoothDriving(state);
    break;

    case 2:
    //Standby
    break;

    default:
    //Dont get here
    break;
  }
  _loop();
}
