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
        //getBME();
        u8g2.drawStr(0,10,"Temperature");
        u8g2.drawStr(0,65,"Pressure");
        line1 = temp;
        line2 = pres;
      }

      else if(displaySelect == 5){ // Tare scale
        u8g2.drawStr(6,10,"i  Press SEL");
        u8g2.drawStr(0,25,"to tare scale");
        u8g2.drawStr(0,60,"FORCE:");
        line2 = force;
        u8g2.drawLine(0,38,64,38);
        u8g2.drawCircle(6,6,6,U8G2_DRAW_ALL);
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
      //char buf4[4];
      
      if(displaySelect == 4){
        sprintf(buf2, "%d", line1);
        sprintf(buf3, "%d", line2); // uses float for pressure
        u8g2.drawStr(0,40,buf2);
        u8g2.drawStr(0,95,buf3);
      }

      else if(displaySelect == 5){
        sprintf(buf2, "%d", line2);
        u8g2.drawStr(0,90,buf2);
      }
      
      else{
      sprintf(buf2, "%d", line1);
      sprintf(buf3, "%d", line2);
      u8g2.drawStr(0,40,buf2);
      u8g2.drawStr(0,95,buf3);
      }
      
      
      
      u8g2.sendBuffer();  
  
}
