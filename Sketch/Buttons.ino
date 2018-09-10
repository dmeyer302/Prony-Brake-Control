void displayButton(){
  if(millis() - buttonTime > 200){
    buttonTime = millis();
    displaySelect++;

    if(displaySelect == 3){
      displaySelect = 4;
    }

    // Loop display back to the beginning
    if(displaySelect == 6){
      displaySelect = 1;
      }
    }}

void upArrow(){
  if(millis()-selectTime > 100){
    /*
    if(runMode == 3){
      loadSpeed += 1000;
    }
    
    setRPM += 5;*/
    selectTime = millis();
  //selectBit = true;
  
  /*if (displaySelect != 3){selectBit = true;}
  
  previousDisplay = displaySelect;*/
  if (displaySelect == 5){
    calibration_factor += 10;
    scale.set_scale(calibration_factor);
    Serial.println(calibration_factor);
  }
}}

void downArrow(){
  if(millis()-selectTime > 100){
    /*if(setRPM > 5){
      setRPM = setRPM - 5;
    }
  
      if(runMode == 3){
        loadSpeed -= 1000;
      }
      */
    selectTime = millis();
    
    /*if (displaySelect != 3){selectBit = true;
    }
    previousDisplay = displaySelect;
    }*/
    if (displaySelect == 5){
      calibration_factor -= 10;
      scale.set_scale(calibration_factor);
      Serial.println(calibration_factor);
    }
  }
}

void selectFn(){
  if(millis() - buttonTime > 0){
  buttonTime = millis();
  if(displaySelect == 3){
    commandedRPM = setRPM;
  }
  else if(displaySelect == 5){
    scale.tare();
  }
  //selectTime += 2000;
}
}
