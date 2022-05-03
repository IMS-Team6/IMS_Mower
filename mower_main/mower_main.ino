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
MeRGBLed rgbLED(0, 12);

int distanceToObstacle = 10;
int autoTurnDirection = 0;
int state = 0;
int autoState = 0;
int mode = 0;

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

void _delay(float milliSeconds) {
  if(milliSeconds < 0.0){
    milliSeconds = 0.0;
  }
  long endTime = millis() + milliSeconds;
  while(millis() < endTime) _loop();
}

void moveForward() {
  move(FORWARD, 40 / 100.0 * 255);
}

void moveBackward() {
  move(BACKWARDS, 40 / 100.0 * 255);
}

void turnLeft() {
  move(LEFT, 40 / 100.0 * 255);
}

void turnRight() {
  move(RIGHT, 40 / 100.0 * 255);
}

void stopMotors() {
  move(STOP, 0);
}

void isr_process_motorLeft(void){
  if(digitalRead(motorLeft.getPortB()) == 0){
    motorLeft.pulsePosMinus();
  }else{
    motorLeft.pulsePosPlus();
  }
}

void isr_process_motorRight(void){
  if(digitalRead(motorRight.getPortB()) == 0){
    motorRight.pulsePosMinus();
  }else{
    motorRight.pulsePosPlus();
  }
}

int checkSensors(){
  if(ultrasonic_10.distanceCm() <= distanceToObstacle){
    return 2;
  }else if(linefollower_9.readSensors() != 3){
    return 3;
  }else{
    return 1;
  }
}

void autoRandomTurn() {
  int left = random(1);
  float turnDuration = random(800, 1800);

  if (left) {
    delay(500);
    turnLeft();
    delay(turnDuration);
    stopMotors();
  }else if(!left){
    delay(500);
    turnRight();
    delay(turnDuration);
    stopMotors();
  }
}

int autonomousDriving(int currentState){
  int nextState = 0;

  rgbLED.setColor(0,0,0,100);
  rgbLED.show();
  
  switch(autoState){
    case 0:
    //StartMotors
    moveForward();
    autoState = 1;
    break;

    case 1:
    //Check sensors while driving forward
    autoState = checkSensors(); 
    break;

    case 2:
    //Found obstacle, handle it
    stopMotors();
    autoState = 4;
    break;

    case 3:
    //Out of bounds, handle it
    stopMotors();
    autoState = 4;
    break;

    case 4:
    //Turn, handle orientation
    autoRandomTurn();
    autoState = 0;
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

void setup() {
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);
  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);
  attachInterrupt(motorLeft.getIntNum(), isr_process_motorLeft, RISING);
  attachInterrupt(motorRight.getIntNum(), isr_process_motorRight, RISING);
  gyro_0.begin();
  rgbLED.setpin(44);
  rgbLED.fillPixelsBak(0, 2, 1);
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
