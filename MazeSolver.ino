// #include <Arduino.h>
#include "FloodFill.h"

// Power for US sensor
int trigFront=A1;
int echoFront=A0;
int trigLeft=A2;
int echoLeft=A3;
int trigRight=A4;
int echoRight=A5;

float leftduration, leftdistance;
float frontduration, frontdistance; 
// Motor FL connections
int enA = 3;
int in1 = 12;
int in2 = 13;
// Motor FR connections
int enB = 11;
int in3 = 9;
int in4 = 10;

// Motor RL connections
int enC = 5;
int in5 = 2;
int in6 = 4;

// Motor Top right connections
int enD = 6;
int in7 = 7;
int in8 = 8;



class Ultrasonic{
public:
  int distance;
  int trigPin;
  int echoPin;

  void init(int trigPin1, int echoPin1){
    echoPin = echoPin1;
    trigPin = trigPin1;
    pinMode(trigPin,OUTPUT);
    pinMode(echoPin,INPUT);
    distance = getDistance();
  }

  int getDistance(){
    digitalWrite(trigPin, LOW);  
    delayMicroseconds(2);  
    digitalWrite(trigPin, HIGH);  
    delayMicroseconds(10);  
    digitalWrite(trigPin, LOW);  
    int duration = pulseIn(echoPin, HIGH); 
    distance = (duration*.0343)/2;
    return distance;
  }

};


class Motor {
public:
  int en;
  int in1;
  int in2;

  void init(int enPin,int in1Pin, int in2Pin) {
    en = enPin;
    in1 = in1Pin;
    in2 = in2Pin;
    pinMode(en, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    digitalWrite(en, LOW);
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    
  }

  void setSpeed(int speed, int direction){
    analogWrite(en, speed);
    if (direction == 1){
      digitalWrite(in1,HIGH);
      digitalWrite(in2,LOW);
    } else if (direction == -1) {
      digitalWrite(in1,LOW);
      digitalWrite(in2,HIGH);
    } else{
      digitalWrite(in1,LOW);
      digitalWrite(in2,LOW);
    }
  }

  void stop() {
    analogWrite(en, 0);
  }
};

// Create FloodFill object
FloodFill mazeSolver;

Ultrasonic front_US;
Ultrasonic left_US;
Ultrasonic right_US;

Motor FL_MOTOR;
Motor FR_MOTOR;
Motor RL_MOTOR;
Motor RR_MOTOR;

void setup() {
  front_US.init(trigFront, echoFront);
  left_US.init(trigLeft, echoLeft);
  right_US.init(trigRight, echoRight);

  FL_MOTOR.init(enA, in1, in2);
  FR_MOTOR.init(enB, in3, in4);
  RL_MOTOR.init(enC, in5, in6);
  RR_MOTOR.init(enD, in7, in8);

  Serial.begin(9600);
  
  // Initialize floodfill algorithm
  mazeSolver.initialize();
  mazeSolver.setWallThreshold(15); // Set wall detection threshold to 15cm
  
  Serial.println("Maze Solver with Floodfill Algorithm Started");
  Serial.print("Target: (");
  Serial.print(mazeSolver.getTargetX());
  Serial.print(", ");
  Serial.print(mazeSolver.getTargetY());
  Serial.println(")");
}

void loop() {
  // Print current position and sensor readings
  Serial.print("Position: (");
  Serial.print(mazeSolver.getRobotX());
  Serial.print(", ");
  Serial.print(mazeSolver.getRobotY());
  Serial.print(") Direction: ");
  Serial.println(mazeSolver.getRobotDirection());
  
  Serial.print("Front: ");
  Serial.println(front_US.getDistance());
  Serial.print("Left: ");
  Serial.println(left_US.getDistance());
  Serial.print("Right: ");
  Serial.println(right_US.getDistance());
  
  // Check if we've reached the target
  if (mazeSolver.hasReachedTarget()) {
    Serial.println("TARGET REACHED!");
    stopALL();
    while(1) {
      // Stay here - maze solved!
      delay(1000);
    }
  }
  
  // Update wall information based on sensor readings
  mazeSolver.updateWalls(front_US.getDistance(), left_US.getDistance(), right_US.getDistance());
  
  // Recalculate flood values
  mazeSolver.calculateFloodValues();
  
  // Get the optimal direction to move
  int optimalDir = mazeSolver.getOptimalDirection();
  
  if (optimalDir == -1) {
    // No accessible direction - this shouldn't happen in a solvable maze
    Serial.println("ERROR: No accessible direction!");
    stopALL();
    delay(1000);
    return;
  }
  
  // Turn to the optimal direction
  turnToDirection(optimalDir);
  
  // Move forward one cell
  moveForwardOneCell(optimalDir);
  
  Serial.print("Moved to direction: ");
  Serial.println(optimalDir);
  Serial.print("Flood value: ");
  Serial.println(mazeSolver.getFloodValue(mazeSolver.getRobotX(), mazeSolver.getRobotY()));
  Serial.println("---");
  
  delay(500); // Small delay between moves
}

void moveForward() {
  FL_MOTOR.setSpeed(255, -1);
  FR_MOTOR.setSpeed(85,  -1);
  RL_MOTOR.setSpeed(255, -1);
  RR_MOTOR.setSpeed(85, -1);
}

void moveLeft() {
  FL_MOTOR.setSpeed(255,   1);
  FR_MOTOR.setSpeed(255, -1);
  RL_MOTOR.setSpeed(255,   1);
  RR_MOTOR.setSpeed(255, -1);
}

void moveRight() {
  FL_MOTOR.setSpeed(255, -1);
  FR_MOTOR.setSpeed(255, 1);
  RL_MOTOR.setSpeed(255, -1);
  RR_MOTOR.setSpeed(255, 1);
}

void stopALL() {
  FL_MOTOR.stop();
  FR_MOTOR.stop();
  RL_MOTOR.stop();
  RR_MOTOR.stop();
}

void turnToDirection(int targetDirection) {
  while (mazeSolver.getRobotDirection() != targetDirection) {
    int diff = (targetDirection - mazeSolver.getRobotDirection() + 4) % 4;
    if (diff == 1) {
      // Turn right
      turnRight90();
    } else {
      // Turn left (for diff == 3, 2)
      turnLeft90();
    }
    delay(500); // Give time for turn to complete
  }
}

void turnRight90() {
  unsigned long startTime = millis();
  while (millis() - startTime < 500) { // Adjust timing as needed
    moveRight();
  }
  stopALL();
}

void turnLeft90() {
  unsigned long startTime = millis();
  while (millis() - startTime < 500) { // Adjust timing as needed
    moveLeft();
  }
  stopALL();
}

void moveForwardOneCell(int direction) {
  // Move forward one cell and update robot position
  unsigned long startTime = millis();
  while (millis() - startTime < 1000) { // Adjust timing based on cell size
    moveForward();
  }
  stopALL();
  
  // Update robot position in the maze solver
  mazeSolver.updateRobotPosition(direction);
}

