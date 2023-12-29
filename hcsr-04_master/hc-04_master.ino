// Master code
#include <Wire.h>

// 1 component (Motor)
// Motor A connections
int enA = 11;
int in1 = A0;
int in2 = A1;
// Motor B connections
int enB = 10;
int in3 = A2;
int in4 = A3;

// distance variable
int distance;

void setup() {
  Wire.begin(9); // Set the slave address to 9

  // Set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
    
  // Turn off motors - Initial state
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  
  Wire.onReceive(receive_Obstacle_Distance);
  
  delay(5000);
}



void moveForward(){
  analogWrite(enA,90);
  analogWrite(enB,90);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void stopMoving(){
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void receive_Obstacle_Distance(){
  byte highByteDist = Wire.read();  // Read the high byte
  byte lowByteDist = Wire.read();   // Read the low byte
  distance = (highByteDist << 8) | lowByteDist;  // Combine the two bytes into a 16-bit integer
}


void loop() {
  if ((distance>0) && (distance<=30)) stopMoving();
  else moveForward();
}


