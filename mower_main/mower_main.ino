#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <MeAuriga.h>


MeLineFollower linefollower_9(9);
MeUltrasonicSensor ultrasonic_10(10);
MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT2);
MeLightSensor lightsensor_12(12);
MeGyro gyro_0(0, 0x69);

int state = 0;
int mode = 2;

void isr_process_encoder1(void)
{
  if(digitalRead(Encoder_1.getPortB()) == 0){
    Encoder_1.pulsePosMinus();
  }else{
    Encoder_1.pulsePosPlus();
  }
}
void isr_process_encoder2(void)
{
  if(digitalRead(Encoder_2.getPortB()) == 0){
    Encoder_2.pulsePosMinus();
  }else{
    Encoder_2.pulsePosPlus();
  }
}

int autonomousDriving(int currentState){
  switch(currentState){
    case 0:
    //StartMotors
    break;

    case 1:
    //Check sensors while driving forward
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
  return currentState;
}

int bluetoothDriving(int currentState){
  switch(currentState){
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
  return currentState;
}


void setup() {
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);
  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);
  attachInterrupt(Encoder_1.getIntNum(), isr_process_encoder1, RISING);
  attachInterrupt(Encoder_2.getIntNum(), isr_process_encoder2, RISING);
  gyro_0.begin();
  Serial.begin(115200);
  randomSeed((unsigned long)(lightsensor_12.read() * 123456));
  delay(3000);
}

void _loop() {
  Encoder_1.loop();
  Encoder_2.loop();
}

void loop() {
  switch(mode){
    case 0:
    state = autonomousDriving(state);
    break;

    case 1:
    state = bluetoothDriving(state);
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
