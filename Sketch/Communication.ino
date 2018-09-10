void sendData(){

  int tempRPM = round(rpm);
  int tempForce = round(force);
  
    if (millis() - sendTime > 500){
    /*rpm = random(50,300);
    //force = random(50,150);
    force = 300-rpm;
  */
    sprintf(sendString,"%d,%d",tempRPM,tempForce);
    Serial.println(sendString);
    sendTime = millis();
  
  }
}

