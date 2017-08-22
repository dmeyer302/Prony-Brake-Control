void displayButton(){
  if(millis() - buttonTime > 200){
    buttonTime = millis();
    displaySelect++;

    // Loop display back to the beginning
    if(displaySelect == 6){
      displaySelect = 1;
      }
    }}

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
