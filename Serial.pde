static uint8_t point;
static uint8_t s[128];
void serialize16(int16_t a) {s[point++]  = a; s[point++]  = a>>8&0xff;}
void serialize8(uint8_t a)  {s[point++]  = a;}

// ***********************************
// Interrupt driven UART transmitter for MIS_OSD
// ***********************************
static uint8_t tx_ptr;

ISR_UART {
  UDR0 = s[tx_ptr++];           /* Transmit next byte */
  if ( tx_ptr == point )        /* Check if all data is transmitted */
    UCSR0B &= ~(1<<UDRIE0);     /* Disable transmitter UDRE interrupt */
}

void UartSendData() {          // start of the data block transmission
  tx_ptr = 0;
  UCSR0A |= (1<<UDRE0);        /* Clear UDRE interrupt flag */
  UCSR0B |= (1<<UDRIE0);       /* Enable transmitter UDRE interrupt */
  UDR0 = s[tx_ptr++];          /* Start transmission */
}

void serialCom() {
  int16_t a;
  uint8_t i;
  if (Serial.available()) {
    switch (Serial.read()) {
    case 'A': //arduino to GUI all data
      point=0;
      serialize8('A');
      serialize8(VERSION);  // MultiWii Firmware version
      for(i=0;i<3;i++) serialize16(accSmooth[i]);
      for(i=0;i<3;i++) serialize16(gyroData[i]); //13
      for(i=0;i<3;i++) serialize16(magADC[i]/3); //19
      serialize16(altitudeSmooth);
      serialize16(heading); // compass
      for(i=0;i<4;i++) serialize16(servo[i]); //31
      for(i=0;i<6;i++) serialize16(motor[i]); //43
      for(i=0;i<8;i++) serialize16(rcData[i]); //49
      serialize8(nunchuk|ACC<<1|BARO<<2|MAG<<3);
      serialize8(accMode|baroMode<<1|magMode<<2);
      serialize16(cycleTime);
      for(i=0;i<2;i++) serialize16(angle[i]/10); //67
    #if defined(TRI)
      serialize8(1);
    #elif defined(QUADP)
      serialize8(2);
    #elif defined(QUADX)
      serialize8(3);
    #elif defined(BI)
      serialize8(4);
    #elif defined(GIMBAL)
      serialize8(5);
    #elif defined(Y6)
      serialize8(6);
    #elif defined(HEX6)
      serialize8(7);
    #elif defined(FLYING_WING)
      serialize8(8);
    #elif defined(Y4)
      serialize8(9);
    #elif defined(HEX6X)
      serialize8(10);
    #elif defined(OCTOX8)
      serialize8(11);
    #elif defined(OCTOFLATP)
      serialize8(11);
    #elif defined(OCTOFLATX)
      serialize8(11);
    #endif
      for(i=0;i<4;i++) {serialize8(P8[i]);serialize8(I8[i]);serialize8(D8[i]);}
      serialize8(P8[PIDLEVEL]);serialize8(I8[PIDLEVEL]);
      serialize8(P8[PIDMAG]);
      serialize8(rcRate8); serialize8(rcExpo8);
      serialize8(rollPitchRate); serialize8(yawRate);
      serialize8(dynThrPID);
      for(i=0;i<6;i++) serialize8(activate[i]);
      serialize8('A');
      UartSendData(); // Serial.write(s,point);
      break;
    case 'O':  // arduino to OSD data - contribution from MIS
      point=0;
      serialize8('O');
      for(i=0;i<3;i++) serialize16(accSmooth[i]);
      for(i=0;i<3;i++) serialize16(gyroData[i]);
      serialize16(altitudeSmooth);
      serialize16(heading); // compass - 16 bytes
      for(i=0;i<2;i++) serialize16(angle[i]); //20
      for(i=0;i<6;i++) serialize16(motor[i]); //32
      for(i=0;i<6;i++) {serialize16(rcData[i]);} //44
      serialize8(nunchuk|ACC<<1|BARO<<2|MAG<<3);
      serialize8(accMode|baroMode<<1|magMode<<2);
      serialize8(vbat);     // Vbatt 47
      serialize8(VERSION);  // MultiWii Firmware version
      serialize8('O'); //49
      UartSendData();
      break;
    case 'C': //GUI to arduino param
      while (Serial.available()<27) {}
      for(i=0;i<4;i++) {P8[i]= Serial.read(); I8[i]= Serial.read(); D8[i]= Serial.read();} //9
      P8[PIDLEVEL] = Serial.read(); I8[PIDLEVEL] = Serial.read(); //11
      P8[PIDMAG] = Serial.read();
      rcRate8 = Serial.read(); rcExpo8 = Serial.read();
      rollPitchRate = Serial.read(); yawRate = Serial.read(); //16
      dynThrPID = Serial.read();
      for(i=0;i<6;i++) activate[i] = Serial.read(); //22
      writeParams();
      break;
    case 'D': //GUI to arduino ACC calibration request
      calibratingA=400;
      break;
    case 'E': //GUI to arduino MAG calibration request
      calibratingM=1;
      break;
//added JDH
    case 'P':
      Serial.println(cycleTime);
      Serial.println(BATTERY_MONITOR_SCALE_FACTOR);
      Serial.println(batVoltage);
      Serial.println(batRaw);
      Serial.println(int (armed));
      Serial.println(rf);
      break;
    case 'o': //print angles
      Serial.print(angle[PITCH]);
      Serial.print(" ");
      Serial.print(rcCommand[YAW]);
      Serial.print(" ");
      Serial.print(angle[ROLL]);
      Serial.print(" ");
      Serial.print(accZero[ROLL]);
      Serial.print(" ");
      Serial.print(softTrimROLL);
      Serial.print(" ");
      Serial.print(accZero[PITCH]);
      Serial.print(" ");
      Serial.println(softTrimPITCH);
      break;
//added JDH
    }
  }
}
