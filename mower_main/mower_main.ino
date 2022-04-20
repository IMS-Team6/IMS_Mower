#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <MeAuriga.h>


MeLineFollower linefollower_9(9);
MeUltrasonicSensor ultrasonic_10(10);
MeEncoderOnBoard motorLeft(SLOT1);
MeEncoderOnBoard MotorRight(SLOT2);
MeLightSensor lightsensor_12(12);
MeGyro gyro_0(0, 0x69);

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
    case 0:
    //Stop
    break;

    case 1:
    //Drive forward
    break;

    case 2:
    //Drive backward
    break;

    case 3:
    //Turn left
    break;

    case 4:
    //Turn right
    break;
    
    case 5:
    //Stop the robot and change mode to auto
    break;
  }
}


void setup() {
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);
  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);
  attachInterrupt(motorLeft.getIntNum(), isr_process_encoder1, RISING);
  attachInterrupt(motorRight.getIntNum(), isr_process_encoder2, RISING);
  gyro_0.begin();
  Serial.begin(115200);
  randomSeed((unsigned long)(lightsensor_12.read() * 123456)); 
  int state = 0;
  int mode = 2;
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
