
static uint8_t checkNewConf = 139;



void readEEPROM() {
  uint8_t i,p=1;
  for(i=1;i<21;i++) *param[i].var = EEPROM.read(p++);
  for(i=0;i<6;i++) activate[i] = EEPROM.read(p++); //22
  for(i=0;i<3;i++) accZero[i] = (EEPROM.read(p++)&0xff) + (EEPROM.read(p++)<<8); // 28
  for(i=0;i<3;i++) magZero[i] = (EEPROM.read(p++)&0xff) + (EEPROM.read(p++)<<8); // 34
// changed JDH
  softTrimROLL = EEPROM.read(p++);
  softTrimPITCH = EEPROM.read(p++);
  accZero[ROLL] = accZero[ROLL] + softTrimROLL;
  accZero[PITCH] = accZero[PITCH] + softTrimPITCH;
// changed JDH  
  //note on the following lines: we do this calcul here because it's a static and redundant result and we don't want to load the critical loop whith it
  rcFactor1 = rcRate8/50.0*rcExpo8/100.0/250000.0;
  rcFactor2 = (100-rcExpo8)*rcRate8/5000.0;
}

void writeParams() {
  uint8_t i,p=1;
  EEPROM.write(0, checkNewConf);
  for(i=1;i<21;i++) EEPROM.write(p++,*param[i].var);
  for(i=0;i<6;i++) EEPROM.write(p++,activate[i]); //22
  for(i=0;i<3;i++) {EEPROM.write(p++,accZero[i]);EEPROM.write(p++,accZero[i]>>8&0xff);} // 28
  for(i=0;i<3;i++) {EEPROM.write(p++,magZero[i]);EEPROM.write(p++,magZero[i]>>8&0xff);} // 34
// changed JDH  
  EEPROM.write(p++,softTrimROLL);EEPROM.write(p++,softTrimPITCH);
// changed JDH  
  readEEPROM();
  blinkLED(15,20,1);
}

void checkFirstTime() {
  if ( EEPROM.read(0) == checkNewConf ) return;
  P8[ROLL] = 40; I8[ROLL] = 30; D8[ROLL] = 15;
  P8[PITCH] = 40; I8[PITCH] = 30; D8[PITCH] = 15;
  P8[YAW]  = 80; I8[YAW]  = 0;  D8[YAW]  = 0;
  P8[PIDALT]  = 20; I8[PIDALT]  = 0;  D8[PIDALT]  = 0;
  P8[PIDLEVEL] = 120; I8[PIDLEVEL] = 45;
  P8[PIDMAG] = 40;
  rcRate8 = 45;
  rcExpo8 = 65;
  rollPitchRate = 0;
  yawRate = 0;
  dynThrPID = 0;
  for(uint8_t i=0;i<6;i++) activate[i] = 0;
  writeParams();
}

