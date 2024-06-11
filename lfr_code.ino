#include <SD.h>
#include <NewPing.h>
#include <Wire.h>
#include "U8glib.h"

#define IR_SENSOR_RIGHT A0
#define IR_SENSOR_LEFT A1
#define ULTRASONIC_TRIGGER_PIN 3
#define ULTRASONIC_ECHO_PIN 2
#define MOTOR_SPEED 180


// Reset pin not used
U8GLIB_SSD1306_128X64 u8g(10, 9);

File logfile;
NewPing sonar(ULTRASONIC_TRIGGER_PIN, ULTRASONIC_ECHO_PIN);

//Right motor
int enableRightMotor=6;
int rightMotorPin1=7;
int rightMotorPin2=8;

//Left motor
int enableLeftMotor=5;
int leftMotorPin1=9;
int leftMotorPin2=10;
// Voltage sensor pin
int voltageSensorPin = A2;

// Current sensor pin
int currentSensorPin = A3;

void setup()
{
  
  //This sets frequecny as 7812.5 hz.
  TCCR0B = TCCR0B & B11111000 | B00000010 ;
  
  // put your setup code here, to run once:
  pinMode(enableRightMotor, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);
  
  pinMode(enableLeftMotor, OUTPUT);
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);

  pinMode(IR_SENSOR_RIGHT, INPUT);
  pinMode(IR_SENSOR_LEFT, INPUT);
   pinMode(ULTRASONIC_TRIGGER_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
    
Serial.begin(9600);
  // Initialize SD card
  if (!SD.begin(10))
  {
    Serial.println("SD Card initialization failed!");
    return;
  }

  // Open or create the log file
  logfile = SD.open("log.txt", FILE_WRITE);
  if (!logfile)
  {
    Serial.println("Error opening log file!");
    return;
  }
u8g.begin();

  rotateMotor(0,0); 
  
}


void loop()
{
  // Read voltage sensor value
  float voltage = analogRead(voltageSensorPin) * (5.0 / 1023.0);

  // Read current sensor value
  float current = analogRead(currentSensorPin) * (5.0 / 1023.0);

  // Display voltage and current on OLED
  u8g.firstPage();
  do {
    // Draw voltage and current values
    u8g.setFont(u8g_font_6x10);
    u8g.drawStr(0, 10, "Voltage: ");
    char voltageStr[10];
    dtostrf(voltage, 4, 2, voltageStr); // Convert float to string
    u8g.drawStr(70, 10, voltageStr);

    u8g.drawStr(0, 20, "Current: ");
    char currentStr[10];
    dtostrf(current, 4, 2, currentStr); // Convert float to string
    u8g.drawStr(70, 20, currentStr);
  } while(u8g.nextPage());
  int rightIRSensorValue = digitalRead(IR_SENSOR_RIGHT);
  int leftIRSensorValue = digitalRead(IR_SENSOR_LEFT);

   // 20 is distance from obstacle
  if (sonar.ping_cm() < 20)
  {
    rotateMotor(-MOTOR_SPEED, MOTOR_SPEED); // Rotate right
    delay(1000); // Rotate for 1 second
  }
  else
  {
    // If none of the sensors detect a line, then go straight
    if (rightIRSensorValue == LOW && leftIRSensorValue == LOW)
    {
      rotateMotor(MOTOR_SPEED, MOTOR_SPEED);
  
    }
    // If right sensor detects a line, then turn right
    else if (rightIRSensorValue == HIGH && leftIRSensorValue == LOW)
    {
      rotateMotor(-MOTOR_SPEED, MOTOR_SPEED);
  
    }
    // If left sensor detects a line, then turn left
    else if (rightIRSensorValue == LOW && leftIRSensorValue == HIGH)
    {
      rotateMotor(MOTOR_SPEED, -MOTOR_SPEED);
      
    }
    // If both sensors detect a line, then stop
    else
    {
      rotateMotor(0, 0);
      
    }
  }
}


void rotateMotor(int rightMotorSpeed, int leftMotorSpeed)
{
  
  if (rightMotorSpeed < 0)
  {
    digitalWrite(rightMotorPin1,LOW);
    digitalWrite(rightMotorPin2,HIGH);    
  }
  else if (rightMotorSpeed > 0)
  {
    digitalWrite(rightMotorPin1,HIGH);
    digitalWrite(rightMotorPin2,LOW);      
  }
  else
  {
    digitalWrite(rightMotorPin1,LOW);
    digitalWrite(rightMotorPin2,LOW);      
  }

  if (leftMotorSpeed < 0)
  {
    digitalWrite(leftMotorPin1,LOW);
    digitalWrite(leftMotorPin2,HIGH);    
  }
  else if (leftMotorSpeed > 0)
  {
    digitalWrite(leftMotorPin1,HIGH);
    digitalWrite(leftMotorPin2,LOW);      
  }
  else 
  {
    digitalWrite(leftMotorPin1,LOW);
    digitalWrite(leftMotorPin2,LOW);      
  }
  analogWrite(enableRightMotor, abs(rightMotorSpeed));
  analogWrite(enableLeftMotor, abs(leftMotorSpeed));    
}