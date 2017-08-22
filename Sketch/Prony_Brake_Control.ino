 /*  
  *  Prony Brake Control Software by Daniel Meyer
  *  Project started April 2017
  *  
  *  Optimized for Teensy 3.5
  *  
  */

#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <FreqMeasure.h>
#include <ResponsiveAnalogRead.h>
#include <BME280I2C.h> //Press, Temp, Humid Sensor, BME280 by Tyler Glenn
#include <ArduinoJson.h>
#include <PID_v1.h>

BME280I2C bme;

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE); // 128x64 OLED


//RFM69 Setup
/*#include <SPI.h>
#include <RH_RF69.h>
#include <RHReliableDatagram.h>
#define CLIENT_ADDRESS 1
#define SERVER_ADDRESS 2
RH_RF69 driver;
RHReliableDatagram manager(driver, SERVER_ADDRESS);
uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];*/

/*
#define RF69_FREQ 433.0
#define RFM69_RST     14   // "A"
#define RFM69_CS      10   // "B"
#define RFM69_INT     digitalPinToInterrupt(20)
RH_RF69 rf69(RFM69_CS, RFM69_INT);*/


// Sensor Inputs
//const int rpmPin = 3; // 3 cannot be changed for FreqMeasure
//const int forcePin = A20;
const byte limitSwitch = 32;
const byte servoError = 30;

ResponsiveAnalogRead analogOne(A1, true, 0.99);
ResponsiveAnalogRead analogTwo(A20, true, 0.99);

// Buttons
const byte startPin = 1;
const byte stopPin = 6;
const byte upPin = 17;
const byte downPin = 16;
const byte selectPin = 8;
const byte releasePin = 4; // 4 cannot be used for PWM when FreqMeasure is running
const byte displayPin = 15;

// LEDs
const byte statusLED = 13; // Onboard Teensy

const byte runLED = 2;
const byte releaseLED = 5;
const byte stopLED = 7;
const byte selectLED = 9;

// Outputs
const byte dirPin = 23;
const byte stepPin = 22;
const byte disableStepper = 21;
const byte waterPin = 31;

// Constants
const int fastDelay = 100000; // microseconds
const int medDelay = 200000;
const int slowDelay = 500000;
const byte minRPM = 100;
const float armLength = 5.25; // ft

// Variables
bool statusBool = 0;
unsigned long statusOn = 0;
unsigned long statusOff = 0;

unsigned long buttonTime = 0;
unsigned long loadTime = 0;
float rpm = 0;
int force = 0;
double hp = 0;
byte waterStatus = 0;
volatile unsigned long waterTime = 0;
volatile unsigned long selectTime = 0;
volatile bool selectBit = 0;

volatile int commandedRPM = 0;
volatile int setRPM = 0;
volatile byte displaySelect = 4;
volatile byte previousDisplay = 0;

float temp = 0;
float pres = 0;
float hum = 0;

// RPM Vars
volatile double sum = 0;//freqmeasure
volatile int count = 0;//freqmeasure
//volatile int time_last = 0;
//volatile int recTime = 0;
//volatile double RPMarray[8];
//volatile int RPMsize = 0;
const byte resolution = 17; // pulses per revolution: 17 for PB, else for test
unsigned long freqTime = 0;

// PID Setup

double setpoint, PIDinput, PIDoutput;
double Kp = 2, Ki = 5, Kd = 1;
PID myPID(&setpoint, &PIDoutput, &PIDinput, Kp, Ki, Kd, P_ON_M, DIRECT);

volatile byte runMode = 2;


// Function Declarations

void startRun();
void stopRun();
void coastRun();
void buildDisplay();
void water();
void load();
void unload();
void getBME();
void initializeRPM();
void displayButton();
void selectButton();
void upArrow();
void downArrow();
void transmitData();
void runModes();
void getRPM();



void setup() {
  
  Serial.begin(9600);
  Serial.print(F("Starting Prony Brake."));
  FreqMeasure.begin(); // Must use pin 3
  bme.begin();
  myPID.SetMode(AUTOMATIC);
  u8g2.begin();
  u8g2.setDisplayRotation(U8G2_R1);
  waterTime = millis() + 10000L;

  pinMode(statusLED, OUTPUT);

  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(disableStepper, OUTPUT);
  pinMode(runLED, OUTPUT);
  pinMode(releaseLED, OUTPUT);
  pinMode(stopLED, OUTPUT);
  pinMode(selectLED, OUTPUT);
  pinMode(waterPin, OUTPUT);
  //pinMode(RFM69_RST, OUTPUT);

  pinMode(limitSwitch, INPUT);
  pinMode(startPin, INPUT);
  pinMode(stopPin, INPUT);
  pinMode(upPin, INPUT);
  pinMode(downPin, INPUT);
  pinMode(selectPin, INPUT);
  pinMode(displayPin, INPUT);
  pinMode(servoError, INPUT);

  attachInterrupt(digitalPinToInterrupt(startPin), startRun, RISING);
  attachInterrupt(digitalPinToInterrupt(stopPin), stopRun, RISING);
  attachInterrupt(digitalPinToInterrupt(upPin), upArrow, RISING);
  attachInterrupt(digitalPinToInterrupt(downPin), downArrow, RISING);
  attachInterrupt(digitalPinToInterrupt(selectPin), select, RISING);
  attachInterrupt(digitalPinToInterrupt(releasePin), coastRun, RISING);
  attachInterrupt(digitalPinToInterrupt(displayPin), displayButton, RISING);
  //attachInterrupt(digitalPinToInterrupt(rpmPin), getRPM, RISING);

  u8g2.clearBuffer();
  u8g2.drawBox(0,0,64,128);
  u8g2.sendBuffer();

  delay(150);

  digitalWrite(runLED, HIGH);
  digitalWrite(stopLED, HIGH);
  digitalWrite(releaseLED, HIGH);
  digitalWrite(selectLED, HIGH);

  delay(1000);

  digitalWrite(runLED, LOW);
  digitalWrite(stopLED, LOW);
  digitalWrite(releaseLED, LOW);
  digitalWrite(selectLED, LOW);
  //digitalWrite(RFM69_RST, LOW);

  //driver.setTxPower(20); // HANGS HERE???
  // range from 14-20 for power

  coastRun();
}

void loop() {
  
  status();
  getRPM();
  water();
  horsepower();
  buildDisplay();
  myPID.Compute();
  runModes();
  //transmitData();


  if(digitalRead(upPin) == HIGH){
    if(millis() - selectTime > 500){
      setRPM += 5;
      delay(80);
    }
  }
  
  else if(digitalRead(downPin) == HIGH){
    if(millis() - selectTime > 500){
      setRPM -= 5;
      delay(80);
    }
  }
}



// Interrupts

void stopRun(){
  runMode = 0;
  noTone(stepPin);
  digitalWrite(stopLED, HIGH);
  digitalWrite(runLED, LOW);
  digitalWrite(releaseLED, LOW);
  digitalWrite(selectLED, LOW);
  digitalWrite(disableStepper, LOW);
  delay(200);
}

void startRun(){
  noTone(stepPin);
  delay(200);
  runMode = 1;
  initializeRPM();
  digitalWrite(stopLED, LOW);
  digitalWrite(runLED, HIGH);
  digitalWrite(releaseLED, LOW);
  digitalWrite(selectLED, LOW);
  digitalWrite(disableStepper, LOW);
}

void coastRun(){
  noTone(stepPin);
  runMode = 2;
  digitalWrite(stopLED, LOW);
  digitalWrite(runLED, LOW);
  digitalWrite(releaseLED, HIGH);
  digitalWrite(selectLED, LOW);
}
