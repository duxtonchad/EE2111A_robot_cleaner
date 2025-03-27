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

void goForward(int speed=100);
void turnRight(int speed=100);
void turnLeft(int speed=100);
void stopMotors();
bool checkIR();
void approachStation();
void searchForStation();

#include <MeMCore.h>
#include <SoftwareSerial.h>

MeInfraredReceiver infraredReceiverDecode(PORT_6);


// Motors
MeDCMotor leftMotor(M1);  // Left motor
MeDCMotor rightMotor(M2); // Right motor

// IR sensor
MeIR ir;

// Replace with your charging station's IR code
const unsigned long CHARGING_STATION_CODE = 0xFFA25D;

// Movement speeds

const int Motorspeed = 100;
const int TURN_TIME = 300; // ms to turn

void setup() {
  Serial.begin(9600);
  ir.begin();
  Serial.println("mBot Charging Station Finder Ready!");
}

void loop() {
  if (checkIR()) { // If charging station is detected
    approachStation();
  } else {
    searchForStation();
  }
}

bool checkIR() {
   if (ir.keyPressed(IR_BUTTON_A)) {  // Example button check
    // For custom codes, we'd normally use:
    // unsigned long code = ir.getCode();
    // return (code == CHARGING_STATION_CODE);
    return true;
  }
  return false;
}

void approachStation() {
  Serial.println("Charging station detected! Moving forward...");
  goForward();
  delay(1000); // Move forward for 1 second
  stopMotors();
  Serial.println("Reached charging station!");
  delay(5000); // Simulate charging
}

void searchForStation() {
  Serial.println("Searching...");
  
  // Alternate between left and right turns
  static bool turnDirection = false; // false = left, true = right
  
  if (turnDirection) {
    Serial.println("Turning right");
    turnRight();
  } else {
    Serial.println("Turning left");
    turnLeft();
  }
  
  delay(TURN_TIME);
  stopMotors();
  delay(200); // Brief pause
  turnDirection = !turnDirection; // Switch direction next time
}

// Movement functions
void goForward(int speed=100) {
  leftMotor.run(100);
  rightMotor.run(100); // Negative because motors are mounted opposite
}

void turnRight(int speed=100) {
  leftMotor.run(100);
  rightMotor.run(-100); // Both same direction = turn
}

void turnLeft(int speed=100) {
  leftMotor.run(-100);
  rightMotor.run(100); // Both same opposite direction = turn
}

void stopMotors() {
  leftMotor.stop();
  rightMotor.stop();
}
