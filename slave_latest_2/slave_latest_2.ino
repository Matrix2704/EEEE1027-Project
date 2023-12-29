// Slave code
#include <LiquidCrystal.h>
#include <Wire.h>
// 5 components (Motor, IR sensor, LCD, encoder sensor, mpu-6050)
// 2 components in master board (LCD and encoder sensor)
// 3 components at slave board (IR sensor, motor, MPU-6050)

// LCD
const int rs = 8, en = 9, d4 = 4, d5 = 5, d6 = 6, d7 = 7;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// Rotary encoder for measuring distance
const int DO_PIN_L = 2;  // Encoder 1 pin
const int DO_PIN_R = 3;  // Encoder 2 pin

// Constants for distance calculation
const float distancePerPulse = 0.275;  // Distance traveled per pulse

volatile unsigned long pulseCount_L = 0;  // Pulse count for encoder 1
volatile unsigned long pulseCount_R = 0;  // Pulse count for encoder 2

unsigned long totalDistance_L = 0;  // Total distance for encoder 1
unsigned long totalDistance_R = 0;  // Total distance for encoder 2
unsigned long averageDistance = 0;  // Average distance

// time variable
unsigned long previousMillis;
unsigned long startTimer;
const long interval = 1000;  // Interval in milliseconds (1 second in this case)

// angle variable
int angle;
int MAX_ANGLE;

// Flag to indicate when to start calculate distance
bool startCalculation = false;

// Use switch case for different stage of task
// 1 - go up the ramp and display the angle
// 2 - stop on top of the ramp and spin for 1 round
// 3 - go down the ramp
// 4 - stop 
// 5 - continue the track for 70cm (Group number is 7) then stop for 2 seconds
// 6 - continue the path until the end and display distance convered and time

int state = 1;

// Time variable
unsigned long elapsedSeconds;

int distance;

void setup() {
  // Initialize serial monitor 
  Serial.begin(9600);

  // Initialize LCD
  lcd.begin(16, 2);

  // Read initial state of rotation
  pinMode(DO_PIN_L, INPUT_PULLUP);
  pinMode(DO_PIN_R, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(DO_PIN_L), pulseISR_L, RISING);
  attachInterrupt(digitalPinToInterrupt(DO_PIN_R), pulseISR_R, RISING);

  // setup for slave arduino board
  Wire.begin();
  // Print a message to the LCD to prepare the task
  lcd.setCursor(0, 0);
  lcd.print("Prepare to begin");
  delay(1000);
  lcd.clear();
  lcd.print("Moving in");
  lcd.setCursor(0,1);
  lcd.print("3");
  delay(1000);
  lcd.setCursor(1,1);
  lcd.print("2");
  delay(1000);
  lcd.setCursor(2,1);
  lcd.print("1");
  delay(1000);
  lcd.clear();
  lcd.print("Start!");
  delay(1000);
  lcd.clear();
}



// Read pulse
void pulseISR_L() {
  if (startCalculation){
    pulseCount_L++;
  }
}

void pulseISR_R() {
  if (startCalculation){
    pulseCount_R++;
  }
}

void calculateDistance(){
  // Calculate distance for encoder 1
  totalDistance_L = pulseCount_L * distancePerPulse;

  // Calculate distance for encoder 2
  totalDistance_R = pulseCount_R * distancePerPulse;

  // Calculate the average distance
  averageDistance = (totalDistance_L + totalDistance_R) / 2;

  distance = int(averageDistance);

}

void timer(){
  // Display time using millis
  unsigned long currentMillis = millis();

  if ((currentMillis - previousMillis) >= interval) {
    previousMillis = currentMillis;  // Reset the timer
    elapsedSeconds = (currentMillis - startTimer) / 1000;
    // Perform an action after the specified interval
    // For example, display something on the LCD
    // lcd.print("Timer activated");
  }
}

void receiveAngle(){
  Wire.requestFrom(9, 2); // Request 6 bytes from the slave with address 9
  if (Wire.available() >= 2) {
    delay(10);
    byte highByteX = Wire.read();  // Read the high byte
    byte lowByteX = Wire.read();   // Read the low byte
    angle = (highByteX << 8) | lowByteX;  // Combine the two bytes into a 16-bit integer
    // Check if angle is detected through serial monitor
  }
  if ((angle>0) && (angle>MAX_ANGLE) && (angle<90)) MAX_ANGLE = angle;
}

void spin_around(){
  receiveAngle();
  while (angle<=360){
    readAngle();
  }
  if (angle>=360){ 
    delay(2000);
  }
}

void loop(){
  switch (state){
    case 1:
    // Read angle
    Serial.println("Successfully enter stage 1 (Moving from the ground)");

    receiveAngle();

    if (MAX_ANGLE<11 || angle>340){
      // Display inclination angle
      lcd.setCursor(0,0);
      lcd.print("Tilt angle: ");
      lcd.setCursor(0,1);
      lcd.print(MAX_ANGLE);
      lcd.setCursor(4,1);
      lcd.print("degree");
    }

    else{
      state = 2;
      break;
    }
    break;

    case 2:
    // 2 - stop on top of the ramp and spin for 1 round
    // once angle go back to less than 2 degree, car stops
    // then spin 1 round
    // stop again for a second
    // continue to go down the ramp
    // Enter stage 3
    // Without loop
    Serial.println("Sucessfully enter stage 2 (Going up ramp)");

    receiveAngle();

    if (angle>=2){
      lcd.setCursor(0,0);
      lcd.print("Tilt angle: ");
      lcd.setCursor(0,1);
      lcd.print(MAX_ANGLE);
      lcd.setCursor(4,1);
      lcd.print("degree");
    } else {
      state = 21;
      break;
    }

    break;

    case 21:
    // 21 - double check
    Serial.println("Sucessfully enter stage 21 (Double check)");

    receiveAngle();

    if ((angle>=2) && (angle<=50)){
      state = 2;
    } else {
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Tilt angle: ");
        lcd.setCursor(0,1);
        lcd.print(MAX_ANGLE);
        lcd.setCursor(4,1);
        lcd.print("degree");
        delay(4400);
        lcd.setCursor(0,0);
        lcd.print("Task 1");
        lcd.setCursor(0,1);
        lcd.print("Turn 360 degree");
        spinAround();

        state = 3;
        break;
    }

    break;

    case 3:
    // 3 - continue to follow track on top of the ramp
    // enter stage 4 when going down the ramp
    
    Serial.println("Successfully enter stage 3 (On the ramp)");

    receiveAngle();

    Serial.print("Angle: ");
    Serial.println(angle);

    if ((angle<=50) || (angle>330)) {
      
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Car in motion");
      lcd.setCursor(0,1);
      lcd.print("Angle:");
      lcd.setCursor(6,1);
      lcd.print(angle);
      lcd.setCursor(9,1);
      lcd.print("deg");

    } else {
      state = 4;
      break;
    }
    break;

    case 4:
    // 4 - going down the ramp by following the track
    // enter stage 5 when on ground

    Serial.println("Successfully enter stage 4 (Going down the ramp & stop after ramp)");

    receiveAngle();
    
    Serial.print("Angle: ");
    Serial.println(angle);

    if (angle>=300){
      
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Car in motion");
      lcd.setCursor(0,1);
      lcd.print("Angle:");
      lcd.setCursor(6,1);
      lcd.print(angle);
      lcd.setCursor(9,1);
      lcd.print("deg");
      /*
      lcd.setCursor(0,0);
      lcd.print("Stage 4: ");
      lcd.setCursor(0,1);
      lcd.print(angle);
      lcd.setCursor(4,1);
      lcd.print("degree");
      */
    } else {
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Vehicle stop");
      lcd.setCursor(0,1);
      lcd.print("After the ramp");
      delay(2000); // Stop for 2 seconds after going down the ramp
      state = 5;
      startTimer = millis();
      previousMillis = millis();
      break;
    }
    break;

    case 5:
    // 5 - follow line for 70 cm

    startCalculation = true;

    Serial.println("Successfully enter stage 5 (Follow the track for 70cm)");

    calculateDistance();
    Wire.beginTransmission(9);
    Wire.write(averageDistance);
    Wire.endTransmission();
    timer();

    Serial.print("Average Distance: ");
    Serial.println(averageDistance);

    if (averageDistance <=70){
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Distance =");
      lcd.setCursor(11,0);
      lcd.print(averageDistance);
      lcd.setCursor(14,0);
      lcd.print("cm");
      lcd.setCursor(0,1);
      lcd.print("Timer(s): ");
      lcd.setCursor(11,1);
      lcd.print(elapsedSeconds);
      // Perform an action after the specified interval
      // For example, display something on the LCD
      // lcd.print("Timer activated");
      } else{
        timer();
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Distance =");
        lcd.setCursor(11,0);
        lcd.print(averageDistance);
        lcd.setCursor(14,0);
        lcd.print("cm");
        lcd.setCursor(0,1);
        lcd.print("Timer(s): ");
        lcd.setCursor(11,1);
        lcd.print(elapsedSeconds);
        delay(2000); // stop for 2 seconds after moving for 70 cm
        state = 6;
        break;
    }

    break;

    case 6:

    Serial.println("Successfully enter stage 6 (Continue to follow the track)");   

    calculateDistance();

    timer();
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Distance =");
    lcd.setCursor(11,0);
    lcd.print(averageDistance);
    lcd.setCursor(14,0);
    lcd.print("cm");
    lcd.setCursor(0,1);
    lcd.print("Timer(s): ");
    lcd.setCursor(11,1);
    lcd.print(elapsedSeconds);
    break;
    }
}



