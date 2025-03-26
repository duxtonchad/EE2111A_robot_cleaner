// things to add
// 1. obstacle clearance (ultrasonic)
// 2. return to charging station (IR)
#include "MeMCore.h"
// initialization
uint8_t motorSpeed = 100;// -255 to 255
int weightThreshold = 1000;  
int buzzerFrequency = 2000; 
int buzzerDuration = 500;
// pins
const int FSR_PIN = A0;// analog input
const int IR_PIN = A1;// analog input 
const int BUZZER_PIN = 9;// analog output

void setup() {
  MeDCMotor leftMotor(M1);
  MeDCMotor rightMotor(M2);
  MeLineFollower lineFinder(PORT_3);
  MeUltrasonicSensor ultraSensor(PORT_4);
  
  pinMode(FSR_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);

}

// movement functions
void foward() {
  leftMotor.run(motorSpeed);
  rightMotor.run(motorSpeed);
}

void backword() {
  leftMotor.run(-motorSpeed);
  rightMotor.run(-motorSpeed);
}

void stop() {
  leftMotor.stop();
  rightMotor.stop();
}

void turnLeft(int time) {
  leftMotor.run(-motorSpeed);
  rightMotor.run(motorSpeed);
  delay(time);
  stop();
}

void turnRight(int time) {
  leftMotor.run(motorSpeed);
  rightMotor.run(-motorSpeed);
  delay(time);
  stop();
}

// sensor functions
int calcHeadDistance() {// ultrasonic sensor
  int distance = ultraSensor.distanceCm();
  return distance;
}

int calcIrDistance() { // IR sensor
  int sensorValue = analogRead(IR_PIN);
  return sensorValue;
}

int testBoundary() {// line follower
  int sensorState = lineFinder.readSensors();
  return sensorState;
}

//main loop
void loop() {
  int boundaryState = testBoundary();
  if (boundaryState == S1_OUT_S2_OUT) {// within boundary
    foward();
  }else {
    stop();
    turnRight();
  }
  delay(200);
}
