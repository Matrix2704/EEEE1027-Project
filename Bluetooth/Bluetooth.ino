#include <SoftwareSerial.h>

SoftwareSerial bluetoothSerial(0, 1); // RX, TX pins for HC-05

int enA = 11;
int in1 = A0;
int in2 = A1;
// Motor B connections
int enB = 10;
int in3 = A2;
int in4 = A3;

void setup() {
  Serial.begin(9600);

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

  bluetoothSerial.begin(9600); // HC-05 baud rate
}

void loop() {
    if (bluetoothSerial.available() > 0) {
      char data = bluetoothSerial.read();
      Serial.print("Received: ");
      Serial.println(data);

        // Interpret Bluetooth commands and control motors accordingly
        switch (data) {
            case 'F':
                moveForward();
                break;
            case 'B':
                moveBackward();
                break;
            case 'L':
                turnLeft();
                break;
            case 'R':
                turnRight();
                break;
            case 'S':
                stopMotion();
                break;
            default:
                // If an unrecognized command is received
                break;
        }
    }
}

// Function to move forward
void moveForward() {
  analogWrite(enA, 105);
  analogWrite(enB, 100);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

// Function to move backward
void moveBackward() {
  analogWrite(enA, 105);
  analogWrite(enB, 100);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

// Function to turn right
void turnRight() {
  analogWrite(enA, 105);
  analogWrite(enB, 100);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

// Function to turn left
void turnLeft() {
  analogWrite(enA, 105);
  analogWrite(enB, 100);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

// Function to stop motion
void stopMotion() {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
}
