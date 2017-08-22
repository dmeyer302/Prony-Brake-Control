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
