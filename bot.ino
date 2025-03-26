// things to add
// 2. return to charging station (IR)
#include "MeMCore.h"
// initialization
uint8_t motorSpeed = 100;// -255 to 255
uint8_t minObstacleProx = 10;//cm
int weightThreshold = 1000;  
int buzzerFrequency = 2000; 
int buzzerDuration = 500;
int leftTurn = 0;
int time4turn90 = 1000;
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
void forward(int time = -1) {
  leftMotor.run(motorSpeed);
  rightMotor.run(motorSpeed);
  if (time >= 0) {
    delay(time);
    stop();
  }
}

void backword(int time = -1) {
  leftMotor.run(-motorSpeed);
  rightMotor.run(-motorSpeed);
  if (time >= 0) {
    delay(time);
    stop();
  }
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

void sTurn() { // switches from left to right turn
  if (leftTurn == 0) {
    turnLeft(time4turn90);
    safeForward(1000); // Use safeForward instead of direct forward
    turnLeft(time4turn90);
  } else {
    turnRight(time4turn90);
    safeForward(1000); // Use safeForward instead of direct forward
    turnRight(time4turn90);
  }
  leftTurn = 1 - leftTurn;
}

void safeForward(int time) {
  int elapsedTime = 0;
  int stepTime = 100; // Check every 100 ms
  while (elapsedTime < time) {
    int headDistance = calcHeadDistance();
    int boundaryState = testBoundary();

    // Stop and handle if an obstacle or boundary is detected
    if (headDistance <= minObstacleProx || boundaryState == S1_IN_S2_IN) {
      stop();
      return;
    }

    // Move forward for the step time
    forward(stepTime);
    elapsedTime += stepTime;
  }
}

// sensor functions
int calcHeadDistance() {// ultrasonic sensor
  int distance = ultraSensor.distanceCm();
  return distance;
}

int calcIrDistance() { // IR sensor
  int sensorValue = analogRead(IR_PIN);
  // Example conversion (adjust based on your IR sensor's datasheet)
  int distance = map(sensorValue, 0, 1023, 0, 100); // Map to 0-100 cm
  return distance;
}

int testBoundary() {// line follower
  int sensorState = lineFinder.readSensors();
  return sensorState;
}

// maneuver functions
void boundaryManeuver() {
  int boundaryState = testBoundary();
  if (boundaryState == S1_IN_S2_IN) {
    stop();
    sTurn();
  }else if (boundaryState == S1_IN_S2_OUT) {
    turnRight(600);
  }else if (boundaryState == S1_OUT_S2_IN) {
    turnLeft(600);
  }else {
    forward();
  }
  delay(100);
}

void obstacleManeuver() {
  int headDistance = calcHeadDistance();
  if (headDistance <= minObstacleProx) {
    stop();
    sTurn();
  }else {
    forward();
  }
  delay(100);
}

void maxWeightExceededAlarm() {
  int fsrValue = analogRead(FSR_PIN);
  if (fsrValue > weightThreshold) {
    tone(BUZZER_PIN, buzzerFrequency, buzzerDuration);
    delay(buzzerDuration);
    noTone(BUZZER_PIN);
  }
}

//main loop
void loop() {
  boundaryManeuver();
  obstacleManeuver();
  maxWeightExceededAlarm();
}