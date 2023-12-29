#include <LiquidCrystal.h>
#include <Wire.h>

#define LCD_BACKLIGHT_PIN 10
#define LCD_CONTRAST_PIN 13


// 2 components (LCD, HC-04)
// Ultrasonic Sensor
const int trig = 12;
const int echo = 11;

// LCD
const int rs = 8, en = 9, d4 = 4, d5 = 5, d6 = 6, d7 = 7;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// Variable to calculate distance
float distance;
float duration;
int Dist;

long unsigned initial;
long unsigned pass;

void setup() {
  
  analogWrite(LCD_BACKLIGHT_PIN,220);
  analogWrite(LCD_CONTRAST_PIN,150);
  
  lcd.begin(16,2);
  Wire.begin(8);

  // Ultrasonic sensor pinout
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);

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

  pass = millis();
  initial = millis();
}

void loop() {
  digitalWrite(trig,LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig,LOW);

  duration = pulseIn(echo,HIGH);
  distance = 0.0343 * (duration/2);
  Dist = int(distance);

  Wire.beginTransmission(9);
  Wire.write((byte)(Dist >> 8));
  Wire.write((byte)(Dist & 0xFF));
  Wire.endTransmission();

  initial = millis();

  if ((initial - pass) >= 1000){
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Distance(cm):");
    lcd.setCursor(0,1);
    lcd.print(distance);
    pass = initial;
  }
}

