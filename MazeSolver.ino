
// Power for US sensor
int trigFront=A1;
int echoFront=A0;
int trigLeft=A2;
int echoLeft=A3;
int trigRight=A4;
int echoRight=A5;

//power pins


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
}

void loop() {
  Serial.print("Front: ");
  Serial.println(front_US.getDistance());
  Serial.print("Left: ");
  Serial.println(left_US.getDistance());
  Serial.print("Right: ");
  Serial.println(right_US.getDistance());
  



  if (front_US.getDistance() < 20){
    if (left_US.getDistance() > 20){
      // moveLeft();
      // moveLeft();
      // moveLeft();
      // moveLeft();
      // moveLeft();
      // moveLeft();

      for (int i =0; i<15;i++) moveLeft();
    }
    else if (left_US.getDistance() < 20 && right_US.getDistance() < 20){
      Uturn(5);
    }

    
  } else {
    moveForward();
  }
  
}

void moveForward() {
  FL_MOTOR.setSpeed(255, -1);
  FR_MOTOR.setSpeed(85,  -1);
  RL_MOTOR.setSpeed(255, -1);
  RR_MOTOR.setSpeed(85, -1);
  Serial.println("forward");
}

void moveLeft() {
  FL_MOTOR.setSpeed(255,   1);
  FR_MOTOR.setSpeed(255, -1);
  RL_MOTOR.setSpeed(255,   1);
  RR_MOTOR.setSpeed(255, -1);
}

void moveRight() {
  FL_MOTOR.setSpeed(85, 1);
  FR_MOTOR.setSpeed(255, -1);
  RL_MOTOR.setSpeed(85, 1);
  RR_MOTOR.setSpeed(255, -1);
}

void Uturn(int Utime) {
  float start=millis();
  while ((millis() - start) < Utime*1000){
    moveRight();
  }
}

void stopALL() {
  FL_MOTOR.stop();
  FR_MOTOR.stop();
  RL_MOTOR.stop();
  RR_MOTOR.stop();
}
void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
