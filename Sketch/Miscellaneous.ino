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

void getBME(){
  bme.read(pres, temp, hum,BME280::TempUnit_Fahrenheit, BME280::PresUnit_atm); // Parameters: (float& pressure, float& temp, float& humidity, bool hPa = true, bool celsius = false)
}


// Outputs tach signal
void mapTach(int tempRPM){
  int tempTone = map(tempRPM,200,400,260,520);
  tone(tachPin,tempTone);
}

void readScale(){
  if(millis() - scaleTime > 200){
  
  //scaleAverage.reading(scale.get_units());
  //force = scaleAverage.getAvg();
  
  force = scale.get_units();
  Serial.println(force);
  /*
  if(scale.is_ready()){
      force = scale.get_units(5);
      Serial.print("Force: ");
      Serial.print(force);
      Serial.println(" lbs");
    }
  else { force = 0; }
  */
  }
}

