/*
void sendData(){

byte sendPacket[8] = [rpm, force, hp, waterStatus, pres, temp, hum, runMode];
byte packetLength = sizeof(sendPacket);

rf69.send(sendPacket, packetLength);

}*/

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

