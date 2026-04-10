/* 
Code for Boomba, a heavily modified Roomba. Has a collision avoidance system and light control. 
*/

#include "pwm.h"

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MCP23X17.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_MCP23X17 mcp;

PwmOut light1Pwm(D3);
PwmOut light2Pwm(D5);

// Pin Assignments
const int throttlePin = 2;
const int steeringPin = 4;

const int lightSwitchPin = 7;
const int avoidanceTogglePin = 8;

const int trigPinFront = A3;
const int echoPinFront = A2;

const int trigPinLeft = 12;
const int echoPinLeft = 13;

const int trigPinRight = 10;
const int echoPinRight = 11;

const int vrPin = A0;

const int voltSensorPin = A1;

// Left motor
const int ENA = 6;
const int IN1 = 0; //MCP
const int IN2 = 1; //MCP

// Right motor
const int ENB = 9;
const int IN3 = 2; //MCP
const int IN4 = 3; //MCP

const unsigned long NEUTRAL = 1507;
const unsigned long MAX_FORWARD = 976;
const unsigned long MAX_REVERSE = 2040;
const unsigned long DEADBAND = 30;

String avoidanceToggle = "off";
int lightBrightness = 0;

unsigned long lastBatteryCheck = 0;
const unsigned long batteryInterval = 2000;
const float ratio = 4.57;
const float vRef = 5.0;
float voltage = 0;
int batPercentage = 0;

unsigned long throttlePwm;
unsigned long steeringPwm;
unsigned long switch1Pwm;
unsigned long togglePwm;
unsigned long vrPinPwm;

float dutyCycle1;
float dutyCycle2;

void setup() {

  Serial.begin(9600);

  // Initiate display
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
  } else {
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.display();
  }

  // Initiate MCP23017
  if (!mcp.begin_I2C(0x27))  {   
    Serial.println(F("MCP23017 not found!"));
    while (1);
  }

  // Set IN1-IN4 as outputs on MCP
  mcp.pinMode(IN1, OUTPUT);
  mcp.pinMode(IN2, OUTPUT);
  mcp.pinMode(IN3, OUTPUT);
  mcp.pinMode(IN4, OUTPUT);

  light1Pwm.begin(490.0f, 0.0f);
  light2Pwm.begin(490.0f, 0.0f);

  pinMode(throttlePin, INPUT);
  pinMode(steeringPin, INPUT);
  pinMode(lightSwitchPin, INPUT);
  pinMode(avoidanceTogglePin, INPUT);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(trigPinFront, OUTPUT);
  pinMode(echoPinFront, INPUT);

  pinMode(trigPinLeft, OUTPUT);
  pinMode(echoPinLeft, INPUT);

  pinMode(trigPinRight, OUTPUT);
  pinMode(echoPinRight, INPUT);

  analogReadResolution(14);
}

void loop() {
  
  // Get PWM values
  throttlePwm = pulseIn(throttlePin, HIGH, 25000);
  steeringPwm = pulseIn(steeringPin, HIGH, 25000);
  switch1Pwm = pulseIn(lightSwitchPin, HIGH, 25000);
  togglePwm = pulseIn(avoidanceTogglePin, HIGH, 25000);
  vrPinPwm = pulseIn(vrPin, HIGH, 25000);

  int throttle = 0;
  int steering = 0;

  // Collision detection switch
  if (togglePwm < 1300) {
    avoidanceToggle = "off";
  }
  else if (togglePwm > 1800) {
    avoidanceToggle = "aggressive";
  }
  else {
    avoidanceToggle = "neutral";
  }

  // Ultrasonic sensor logic
  int fDist = getDistance(trigPinFront, echoPinFront);
  int lDist = getDistance(trigPinLeft,  echoPinLeft);
  int rDist = getDistance(trigPinRight, echoPinRight);

  // Find the smallest value of all sensors
  int distance = min(fDist, min(lDist, rDist));

    if (throttlePwm > 500 && throttlePwm <= 1400) {
      throttle = map(throttlePwm, 1400, 976, 0, 255);
    } else if (throttlePwm >= 1534 && throttlePwm < 2500) {
      throttle = map(throttlePwm, 1534, 2040, 0, -255);
    }

  if (avoidanceToggle == "neutral") {
    throttle = collisionSystem(throttle, distance, 25, 100);
  } else if (avoidanceToggle == "aggressive") {
    throttle = collisionSystem(throttle, distance, 75, 50);
  } else {

  }

  // Map steering
  if (steeringPwm > 500 && steeringPwm <= 1400) {
    steering = map(steeringPwm, 1400, 976, 0, 255);
  } else if (steeringPwm >= 1534 && steeringPwm < 2500) {
    steering = map(steeringPwm, 1534, 2040, 0, -255);
  }

  // Drive logic for throttle and steering conjunctikn
  int leftMotorSpeed = steering + throttle;
  int rightMotorSpeed = steering - throttle;

  leftMotorSpeed = constrain(leftMotorSpeed, -255, 255);
  rightMotorSpeed = constrain(rightMotorSpeed, -255, 255);

  // Send commands to function to control motor
  driveMotor(leftMotorSpeed, ENA, IN1, IN2);
  driveMotor(rightMotorSpeed, ENB, IN3, IN4);

  // Light logic
  int lightBrightness = 0;

  if (vrPinPwm > 1500) {
    lightBrightness = map(vrPinPwm, 1500, 2040, 0, 100);
  } else {
    lightBrightness = map(vrPinPwm, 1500, 976, 0, 100);
  }
  
  lightBrightness = constrain(lightBrightness, 0, 100);


  if (switch1Pwm < 1300) {
    dutyCycle1 = 0;
    dutyCycle2 = 0;
  } else if (switch1Pwm > 1800) {
    if (vrPinPwm > 1500) {
      dutyCycle1 = lightBrightness;
      dutyCycle2 = 0;
    } else {
      dutyCycle2 = lightBrightness;
      dutyCycle1 = 0;
    }
  }
  else {
    dutyCycle1 = lightBrightness;
    dutyCycle2 = lightBrightness;
  }

  // Battery Reading
  if (millis() - lastBatteryCheck >= batteryInterval) {
    int rawValue = analogRead(voltSensorPin);
    float vOut = (rawValue * vRef) / 16383.0;
    voltage = vOut * ratio;
    batPercentage = calculatePercentage(voltage);
    
    lastBatteryCheck = millis();
  }

  // Display
  display.clearDisplay();

  display.setTextSize(1);

  display.setCursor(0, 0);
  display.print(lDist);
  display.print(F(" - ")); 
  display.print(fDist);
  display.print(F(" - ")); 
  display.println(rDist);
  
  display.setTextSize(1);
  display.setCursor(0, 15);
  display.print(F("Avoidance: ")); 
  if (avoidanceToggle == "off") {
    display.println("off");
  } else if (avoidanceToggle == "neutral") {
    display.println("neutral");
  } else {
    display.println("aggressive");
  }
  
  display.setCursor(0, 30);
  display.print(F("Lights ")); 
  if (dutyCycle1 > 0 || dutyCycle2 > 0) {
    display.print("(on): ");
  } else {
    display.print("(off): ");
  }
  display.print(lightBrightness);
  display.println(F("%")); 

  display.setCursor(0,45);
  display.print(voltage);
  display.print(F("v"));
  display.print(F(" - "));
  display.print(batPercentage);
  display.print(F("%"));
  
  display.display();


  light1Pwm.pulse_perc(dutyCycle1);
  light2Pwm.pulse_perc(dutyCycle2);
}

// Function to execute PWM output to h-bridge
void driveMotor(int speed, int enPin, int in1Pin, int in2Pin) {
  if (speed > 0) {
    mcp.digitalWrite(in1Pin, HIGH);
    mcp.digitalWrite(in2Pin, LOW);
    analogWrite(enPin, speed);
  } else if (speed < 0) {
    mcp.digitalWrite(in1Pin, LOW);
    mcp.digitalWrite(in2Pin, HIGH);
    analogWrite(enPin, abs(speed));
  } else {
    mcp.digitalWrite(in1Pin, LOW);
    mcp.digitalWrite(in2Pin, LOW);
    analogWrite(enPin, 0);
  }
}

// Collision system
int collisionSystem(int throttle, int distance, int activationDistance, int maxPower) {
  if (throttle > 0 && distance <= activationDistance && distance > 0) {
    
    float multiplier = map(distance, 0, activationDistance, 15, maxPower) / 100.0;
    
    multiplier = constrain(multiplier, 0.15, 1.0); 
    
    throttle = throttle * multiplier;

    return(throttle);
  }
  else {
    return(throttle);
  }
}

int getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  long duration = pulseIn(echoPin, HIGH, 30000);
  
  int cm = duration * 0.034 / 2;

  if (cm <= 0) return 999; 
  
  return cm;
}

int calculatePercentage(float voltage) {
  int pc;
  if (voltage >= 16.8) pc = 100;
  else if (voltage >= 16.0) pc = mapFloat(voltage, 16.0, 16.8, 80, 100);
  else if (voltage >= 15.2) pc = mapFloat(voltage, 15.2, 16.0, 50, 80);
  else if (voltage >= 14.0) pc = mapFloat(voltage, 14.0, 15.2, 15, 50);
  else if (voltage >= 12.0) pc = mapFloat(voltage, 12.0, 14.0, 0, 15);
  else pc = 0;
  
  return pc;
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}



















