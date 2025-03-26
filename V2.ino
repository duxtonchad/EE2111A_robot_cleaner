// things to add
// 1. obstacle clearance (ultrasonic)
// 2. return to charging station (IR)
#include "MeMCore.h"
void setup() {
  Serial.begin(9600);
  // ports
  MeDCMotor leftMotor(M1);
  MeDCMotor rightMotor(M2);
  MeLineFollower lineFinder(PORT_3);

  // pins
  const int FSR_PIN = A0;     
  const int BUZZER_PIN = 3;

  // initialization
  uint8_t motorSpeed = 100;
  int weightThreshold = 1000;  
  int buzzerFrequency = 2000; 
  int buzzerDuration = 500;

}

void turnRight() {
  leftMotor.run(motorSpeed);
  rightMotor.run(-motorSpeed);
  delay(600);
  leftMotor.stop();
  rightMotor.stop();
}
void circumvent () {}


void loop() {
  int sensorState = lineFinder.readSensors();
  if (sensorState == S1_OUT_S2_OUT) {//Sensor 1 and 2 are outside of black line
    leftMotor.run(motorSpeed);
    rightMotor.run(motorSpeed);
  }else {
    leftMotor.stop();
    rightMotor.stop();
    turnRight();
  }
  delay(200);
}
