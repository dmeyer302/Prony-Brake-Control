void runModes(){
   if (runMode == 0){ // Reversed/Reversing
    //Serial.print("limitSwitch is ");
    //Serial.println(digitalRead(limitSwitch));
    

      if(digitalRead(limitSwitch) != 0){
        digitalWrite(dirPin, LOW);
        tone(stepPin,15000);
        
      }
  
      else if(digitalRead(limitSwitch) == 0){
        noTone(stepPin);
        releaseStepper();
        coastRun();
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

    else if (runMode == 3){
        
        digitalWrite(dirPin, HIGH);
        tone(stepPin, loadSpeed);
        if(rpm <= minAutoRPM){
          stopRun();
        }

        if(millis() - prevRPMTime > 1000){
          
          if( abs((prevRPM - rpm)) > 40){
            loadSpeed = loadSpeed - 500;
          }
          else if( abs((prevRPM - rpm)) <= 2){
            loadSpeed = loadSpeed + 500;
          }

          if(loadSpeed > maxLoadSpeed){
            loadSpeed = maxLoadSpeed;
          }
          if(loadSpeed < 500){
            loadSpeed = 500;
          }
          
          prevRPM = rpm;
          prevRPMTime = millis();
          
        }
        
      
    }
}


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


void getRPM(){
    if (FreqMeasure.available()) {
  
      freqTime = millis();
      sum = sum + FreqMeasure.read();
      count = count + 1;
      
      if (count >= resolution) {
        //Serial.println("Available");
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


void horsepower(){
  //analogOne.update();
  //analogTwo.update();
  //rpm = analogOne.getValue();
  //force = analogTwo.getValue();
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


void releaseStepper(){
  digitalWrite(disableStepper, HIGH);
}
