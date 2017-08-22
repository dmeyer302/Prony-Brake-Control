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
#include <BME280I2C.h> //Press, Temp, Humid Sensor
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


 if (runMode == 0){ // Reversed/Reversing

      if(digitalRead(limitSwitch) != LOW){
        digitalWrite(dirPin, LOW);
        tone(stepPin,15000);
      }
  
      else if(digitalRead(limitSwitch) == LOW){
        noTone(stepPin);
        releaseStepper();
      }
    }
  
    else if(runMode == 1){ // Active mode
  
        if (rpm < 100){ //100 for pb, else for test
          stopRun();
        }
   
        else if (rpm > commandedRPM && rpm > minRPM){
  
          // Pauses stepper when near target to limit overshooting the target
          if(abs(rpm-commandedRPM) < 19){
            
              if(millis() - loadTime > 3000){
                loadTime = millis();
                noTone(stepPin);
              }
              
              else if(millis() - loadTime > 2000){
                load();
              }
             }
          
          else{ load(); }
        }
      
        else if(rpm < commandedRPM && digitalRead(limitSwitch) != LOW){
          if(abs(rpm-commandedRPM) < 19){
            
            if(abs(millis() - loadTime) > 3000){
              loadTime = millis();
              noTone(stepPin);
            }
            
            else if(abs(millis() - loadTime) > 2000){
              unload();
            }
            
          }
          
          else{ unload(); }
        }
  
        else if(digitalRead(limitSwitch) == LOW){ //reversed all the way to limit switch
          noTone(stepPin);
        }
  
        else{ // when rpm = commandedRPM
          noTone(stepPin);
        }
  
  
        // Illuminates read (blue) indicator when RPM is within 2 from commanded
        if(abs(rpm-commandedRPM) < 1){
          digitalWrite(selectLED, HIGH);
        }
        
        else{digitalWrite(selectLED, LOW);}
        
    }

    else if (runMode == 2){ // Coasting, does not reverse
        noTone(stepPin);
        releaseStepper();
    }
}



// Functions
/*
void sendData(){

byte sendPacket[8] = [rpm, force, hp, waterStatus, pres, temp, hum, runMode];
byte packetLength = sizeof(sendPacket);

rf69.send(sendPacket, packetLength);

}*/

void getRPM(){
    if (FreqMeasure.available()) {
  
      freqTime = millis();
      sum = sum + FreqMeasure.read();
      count = count + 1;
      
      if (count >= resolution) {
        float frequency = FreqMeasure.countToFrequency(sum / count);
        sum = 0;
        count = 0;
        rpm = (((float)frequency)*60)/resolution; // freq in hz, 60 sec/min, cycles per revolution
      }
    }
  
    // freqTime comparison sets RPM to 0 if a pulse is not received in a certain amount of time.
    // FreqMeasure is not able to measure 0 directly, so this code effectively times itself out and assumes brake is at 0.
    if(millis()-freqTime > 1000){
      rpm = 0;
    }
}

// Flashes onboard status LED to indicate program has not frozen
void status(){
    if(statusBool && millis() - statusOn > 200){
      statusOff = millis();
      statusBool = 0;
    }
    else if(!statusBool && millis() - statusOff > 2000){
      statusOn = millis();
      statusBool = 1;
    }
    digitalWrite(statusLED, statusBool);
}


void displayButton(){
  if(millis() - buttonTime > 200){
    buttonTime = millis();
    displaySelect++;

    // Loop display back to the beginning
    if(displaySelect == 6){
      displaySelect = 1;
      }
    }}


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

void upArrow(){
  if(millis()-selectTime > 100){
    setRPM += 5;
    selectTime = millis();
  //selectBit = true;
  
  if (displaySelect != 3){selectBit = true;}
  
  previousDisplay = displaySelect;
  }
}

void downArrow(){
  if(millis()-selectTime > 100){
  if(setRPM > 5){
    setRPM = setRPM - 5;
  }
  selectTime = millis();
  
  if (displaySelect != 3){selectBit = true;
  }
  previousDisplay = displaySelect;
  }
}

void select(){
  if(millis() - buttonTime > 0){
  buttonTime = millis();
  if(displaySelect == 3){
    commandedRPM = setRPM;
  }
  //selectTime += 2000;
}}

void releaseStepper(){
  digitalWrite(disableStepper, HIGH);
}

void horsepower(){
  //analogOne.update();
  analogTwo.update();
  //rpm = analogOne.getValue();
  force = analogTwo.getValue();
  //rpm = analogRead(A1);
  hp = (force * armLength * rpm) / 5252;
}

void water(){
  // Delays water on to "debounce" RPM threshold during hookup
  if(millis() - waterTime > 5000){
    if(rpm > 99){
        waterStatus = 1;
        waterTime = millis();
      }
      else {
        waterStatus = 0;
      }

      // Water turns off in reversing mode; stays on in active mode
      if(runMode == 0){ waterStatus = 0; }
      if(runMode == 1){ waterStatus = 1; }
      
      digitalWrite(waterPin, waterStatus);
  }}


void load(){
    digitalWrite(dirPin, HIGH);
    
    // Increase multiplier for greater speed range; increase constant for higher min speed
    int diff = 2000 + abs(rpm - commandedRPM)*100;

    // Sets maximum speed
    if(diff > 20000){diff = 20000;}

    // Sets minimum speed
    else if(diff <= 10){diff = 10;}
    tone(stepPin,diff);
}

void unload(){
    digitalWrite(dirPin, LOW);    
    //Serial.println("unload");

    int diff = 2000 + abs(rpm - commandedRPM)*100;
    if(diff > 20000){diff = 20000;}
    else if(diff <= 0){diff = 10;}
    tone(stepPin,diff);
    
}

void initializeRPM(){
  //getRPM();
  setRPM = rpm;
  commandedRPM = rpm;
  while (setRPM % 5 != 0){
    setRPM++;
  }
  commandedRPM = setRPM;
}


void buildDisplay(){

      int line1 = 0;
      int line2 = 0;
      
      u8g2.clearBuffer();
    
    // Small text
      
      u8g2.setFont(u8g2_font_helvB08_tr);

      if (millis() - selectTime <= 3000 && selectBit == true){
        
        displaySelect = 3;
      }
      else if (millis() - selectTime > 5000 && selectBit == true){
        selectBit = false;
        displaySelect = previousDisplay;
      }
      //else{displaySelect = previousDisplay;}
      
    //Serial.println(selectBit);
     /* if(digitalRead(servoError) == HIGH){
        u8g2.drawStr(0,10,"SERVO");
        u8g2.drawStr(0,30,"ERROR");

        u8g2.drawStr(0,80,"RESTART");
        u8g2.drawStr(0,100,"SYSTEM");
        line1 = "";
        line2 = "";
      }*/
    
      if(displaySelect == 1){
        u8g2.drawStr(0,10,"RPM:");
        u8g2.drawStr(0,65,"HP:");
        line1 = rpm;
        line2 = hp;
      }
    
      else if(displaySelect == 2){
        u8g2.drawStr(0,10,"RPM:");
        u8g2.drawStr(0,65,"FORCE:");
        line1 = rpm;
        line2 = force;
      }
    
      else if(displaySelect == 3){
        u8g2.drawStr(0,10,"RPM SET TO:");
        u8g2.drawStr(0,65,"NEXT RPM:");
        line1 = commandedRPM;
        line2 = setRPM;
      }

      else if(displaySelect == 4){
        getBME();
        u8g2.drawStr(0,10,"Temperature");
        u8g2.drawStr(0,65,"Pressure");
        line1 = temp;
        line2 = pres;
      }

      else if(displaySelect == 5){
        u8g2.drawStr(0,10,"Kp");
        u8g2.drawStr(0,40,"Ki");
        u8g2.drawStr(0,70,"Kd");
      }
    
    
    // Icons
    
      char buf1[2];
      sprintf(buf1, "%d", runMode);
      u8g2.drawStr(0,122,buf1);
    
      if(waterStatus == 1){
        u8g2.drawStr(13,122,"W");
        }

      if(digitalRead(limitSwitch) == LOW){
        u8g2.drawStr(30,122,"LIM");
        }
    
    // Large text
    
      u8g2.setFont(u8g2_font_fub20_tn);
    
      char buf2[4];
      char buf3[4];
      char buf4[4];
      
      if(displaySelect == 4){
        sprintf(buf2, "%d", line1);
        sprintf(buf3, "%d", line2); // uses float for pressure
        u8g2.drawStr(0,40,buf2);
        u8g2.drawStr(0,95,buf3);
      }

      else if(displaySelect == 5){
        sprintf(buf2,"%lf",Kp);
        sprintf(buf3,"%lf",Ki);
        sprintf(buf4,"%lf",Kd);

        u8g2.drawStr(0,15,buf2);
        //buf3
        //buf4
      }
      
      else{
      sprintf(buf2, "%d", line1);
      sprintf(buf3, "%d", line2);
      u8g2.drawStr(0,40,buf2);
      u8g2.drawStr(0,95,buf3);
      }
      
      
      
      u8g2.sendBuffer();  
  
}

void getBME(){
  //float temp(NAN), hum(NAN), pres(NAN);
  uint8_t pressureUnit(2);   // unit: B000 = Pa, B001 = hPa, B010 = Hg, B011 = atm, B100 = bar, B101 = torr, B110 = N/m^2, B111 = psi
  bme.read(pres, temp, hum, false, pressureUnit); // Parameters: (float& pressure, float& temp, float& humidity, bool hPa = true, bool celsius = false)
  /* Alternatives to ReadData():
    float ReadTemperature(bool celsius = false);
    float ReadPressure(uint8_t unit = 0);
    float ReadHumidity();

    Keep in mind the temperature is used for humidity and
    pressure calculations. So it is more effcient to read
    temperature, humidity and pressure all together.
   */
  //float dewPoint = bme.dew(temp, hum, metric);
}



void transmitData(){

//ArduinoJSON
  char data1[250];
  StaticJsonBuffer<250> jsonBuffer;
  JsonObject& pb = jsonBuffer.createObject();

  pb["r"] = rpm;
  pb["f"] = force;
  pb["h"] = hp;
  pb["cRPM"] = commandedRPM;
  pb["t"] = temp;
  pb["p"] = pres;
  pb["rM"] = runMode;
  pb["runL"] = digitalRead(runLED);
  pb["relL"] = digitalRead(releaseLED);
  pb["stL"] = digitalRead(stopLED);
  pb["selL"] = digitalRead(selectLED);
  pb["w"] = digitalRead(waterPin);

  pb.printTo(data1, sizeof(data1));  //Use data1 as the string to send
  Serial.println(data1);


// RFM ** SWITCH TO SIMPLE RFM TEST, NOT RHRELIABLE
  /*if (manager.available())
    {
      Serial.println("Manager available");
      // Wait for a message addressed to us from the client
      byte len = sizeof(buf);
      byte from;
      if (manager.recvfromAck(buf, &len, &from))
      {Serial.println("recvfromAck");
        Serial.print("got request from : 0x");
        Serial.print(from, HEX);
        Serial.print(": ");
        Serial.println((char*)buf);
  
        // Send a reply back to the originator client
        if (!manager.sendtoWait((unsigned char*)data1, sizeof(data1), from))
          Serial.println("sendtoWait failed");
      }
    }*/
}

