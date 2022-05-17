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
  MOWER_IDLE = '0',
  MOWER_MAN_FORWARD = '1',
  MOWER_MAN_LEFT = '2',
  MOWER_MAN_RIGHT = '3',
  MOWER_MAN_BACKWARDS = '4',
  MOWER_CHANGEMODE = '5'
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
int autoState = 0;
char bluetoothState;
int mode = 1;
int turnFlag = 0;
moveDirection autoTurnDirection = STOP;

void _delay(float milliSeconds) {
  if (milliSeconds < 0.0) {
    milliSeconds = 0.0;
  }
  long endTime = millis() + milliSeconds;
  while (millis() < endTime) {
    _loop();
    gyro_0.update();
  }
}

void move(moveDirection direction, int speed)
{
  int leftSpeed = 0;
  int rightSpeed = 0;
  if (direction == FORWARD) {
    leftSpeed = -speed;
    rightSpeed = speed;
  } else if (direction == BACKWARDS) {
    leftSpeed = speed;
    rightSpeed = -speed;
  } else if (direction == LEFT) {
    leftSpeed = -speed;
    rightSpeed = -speed;
  } else if (direction == RIGHT) {
    leftSpeed = speed;
    rightSpeed = speed;
  } else if (direction == STOP) {
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

void collision() {
  moveBackward();
  delay(500);
  turnRight();
  delay(500);
}

void autoTurn() {
  float timeToTurn = random(800, 1800);

  move(BACKWARDS, 40 / 100.0 * 255);
  _delay(200);

  if (autoTurnDirection == LEFT) {
    move(LEFT, 40 / 100.0 * 255);
    _delay(timeToTurn);
    move(STOP, 0);
  } else if (autoTurnDirection == RIGHT) {
    move(RIGHT, 40 / 100.0 * 255);
    _delay(timeToTurn);
    move(STOP, 0);
  }
  autoTurnDirection = STOP;
}

void isr_process_motorLeft(void)
{
  if (digitalRead(motorLeft.getPortB()) == 0) {
    motorLeft.pulsePosMinus();
  } else {
    motorLeft.pulsePosPlus();
  }
}
void isr_process_motorRight(void)
{
  if (digitalRead(motorRight.getPortB()) == 0) {
    motorRight.pulsePosMinus();
  } else {
    motorRight.pulsePosPlus();
  }
}

int checkSensors() {
  if (ultrasonic_10.distanceCm() <= distanceToObstacle) {
    if (random(0,1)){
      autoTurnDirection = RIGHT;
    }else{
      autoTurnDirection = LEFT;
    }
    return 2;
  } else if (linefollower_9.readSensors() == 1) {
    autoTurnDirection = RIGHT;
    return 3;
  } else if ((linefollower_9.readSensors() == 2) || (linefollower_9.readSensors() == 0)) {
    autoTurnDirection = LEFT;
    return 3;
  } else {
    return 1;
  }
}

String getOrientation() {
  gyro_0.update();
  int angle = (int) gyro_0.getAngleZ();
  if (angle < 0) {
    angle += 360;
  }
  return String(angle);
}

void receiveAck() {
  while (1) {
    if (Serial.available() > 0) {
      char data = Serial.read();
      if (data == 'A') {
        break;
      /*}else if (data == 'M'){
        mode = 1;
        //Might need to do some other shit here aswell.....*/
      }
    }
  }
}

int autonomousDriving(int currentState) {
  int nextState = 0;
  switch (currentState) {
    case 0:
      //StartMotors
      Serial.println('S');
      moveForward();
      //receiveAck();
      nextState = 1;
      break;

    case 1:
      //Check sensors while driving forward
      moveForward();
      nextState = checkSensors();
      break;

    case 2:
      //Found obstacle, handle it
      stopMotors();
      Serial.println('O');
      _delay(3000);
      //receiveAck();
      nextState = 4;
      break;

    case 3:
      //Out of bounds, handle it
      stopMotors();
      Serial.println('B');
      //receiveAck();
      nextState = 4;
      break;

    case 4:
      //Turn, handle orientation
      Serial.println("T");
      //receiveAck();
      autoTurn();
      Serial.println(getOrientation()); //Add the rounded value of new direction
      //receiveAck();
      nextState = 0;
      break;

      /*case 5:
        //Stop the robot and change mode to manual
        move(STOP, 0);
        //Serial.println('B');
        receiveAck();
        mode = 1;   //Change to manual
        //state = 0;  //Initial value for next mode
        break;*/
  }
  return nextState;
}

void bluetoothDriving(char nextState) {
  switch (nextState) {
    case MOWER_IDLE:
      //Stop
      stopMotors();
      if (turnFlag == 1) {
        //Read gyro and send data to rpi
        Serial.println(getOrientation());
        //receiveAck();
        turnFlag = 0;

      }
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
      turnFlag = 1;
      break;

    case MOWER_MAN_RIGHT:
      //Turn right
      turnRight();
      turnFlag = 1;
      break;

    case MOWER_CHANGEMODE:
      //Stop the robot and change mode to auto
      mode = 0;
      autoState = 0;
      _delay(1);
      break;

    default:
      //Weird data
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
  _delay(1000);
}

void _loop() {
  motorLeft.loop();
  motorRight.loop();
}

void loop() {
  gyro_0.update();

  switch (mode) {
    case 0:
      //Autonomous
      if (Serial.available() > 0) {
        //_loop();
        move(STOP, 0);
        //_loop();
        char data = Serial.read();
        if (data == 'M') {
          mode = 1;
        }
        //autonomousDriving(5);
      } else {
        rgbLED.setColor(0,0,100,0);
        rgbLED.show();
        
        autoState = autonomousDriving(autoState);
      }
      //autoState = autonomousDriving(autoState);
      break;

    case 1:
      //Manual
      if (Serial.available() > 0) {
        rgbLED.setColor(0,0,0,100);
        rgbLED.show();
        
        char bluetoothState = Serial.read();
        bluetoothDriving(bluetoothState);
      }
      break;

    default:
      //Dont get here
      break;
  }
  _loop();
}
