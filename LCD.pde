// *****************************
// LCD & display & monitoring
// *****************************

// 1000000 / 9600  = 104 microseconds at 9600 baud.
// we set it below to take some margin with the running interrupts
#define BITDELAY 102
void LCDprint(uint8_t i) {
  #if defined(LCD_TEXTSTAR)
    Serial.print( i , BYTE);
  #else
    LCDPIN_OFF
    delayMicroseconds(BITDELAY);
    for (uint8_t mask = 0x01; mask; mask <<= 1) {
      if (i & mask) LCDPIN_ON else LCDPIN_OFF // choose bit
      delayMicroseconds(BITDELAY);
    }
    LCDPIN_ON //switch ON digital PIN 0
    delayMicroseconds(BITDELAY);
  #endif
}

void LCDprintChar(const char *s) {
  while (*s) LCDprint(*s++);
}

void initLCD() {
  blinkLED(20,30,1);
  #if defined(LCD_TEXTSTAR)
    // Cat's Whisker Technologies 'TextStar' Module CW-LCD-02
    // http://cats-whisker.com/resources/documents/cw-lcd-02_datasheet.pdf
    // Modified by Luca Brizzi aka gtrick90 @ RCG
    LCDprint(0xFE);LCDprint(0x43);LCDprint(0x02); //cursor blink mode
    LCDprint(0x0c); //clear screen
    LCDprintChar("MultiWii Config");
    LCDprint(0x0d); // carriage return
    LCDprintChar("for all params");
    delay(2500);
    LCDprint(0x0c); //clear screen
  #else
    Serial.end();
    //init LCD
    PINMODE_LCD //TX PIN for LCD = Arduino RX PIN (more convenient to connect a servo plug on arduino pro mini)
  #endif
}

void configurationLoop() {
  uint8_t chan,i;
  uint8_t p,paramActive;
  uint8_t val,valActive;
  static char line1[17],line2[17];
  uint8_t LCD=1;
  uint8_t refreshLCD = 1;

  initLCD();
  p = 0;
  while (LCD == 1) {
    if (refreshLCD == 1) {
      strcpy(line2,"                ");
      strcpy(line1,"                ");
      i=0; char* point = param[p].paramText; while (*point) line1[i++] = *point++;
      uint16_t unit = *param[p].var;
      if (p == 12) {unit *=2;} // RC RATE can go up to 500
      char c1 = '0'+unit/100; char c2 = '0'+unit/10-(unit/100)*10; char c3 = '0'+unit-(unit/10)*10;
      if (param[p].decimal == 0) {line2[6] = c1;  line2[7] = c2;   line2[8] = c3;}
      if (param[p].decimal == 1) {line2[5] = c1;  line2[6] = c2;   line2[7] = '.'; line2[8] = c3;}
      if (param[p].decimal == 2) {line2[5] = c1;  line2[6] = '.';  line2[7] = c2;  line2[8] = c3;}
      if (param[p].decimal == 3) {line2[4] = '0'; line2[5] = '.';  line2[6] = c1;  line2[7] = c2; line2[8] = c3;}

      #if defined(LCD_TEXTSTAR)
        LCDprint(0xFE);LCDprint('L');LCDprint(1);LCDprintChar(line1); //refresh line 1 of LCD
        LCDprint(0xFE);LCDprint('L');LCDprint(2);LCDprintChar(line2); //refresh line 2 of LCD
      #else
        LCDprint(0xFE);LCDprint(128);LCDprintChar(line1);
        LCDprint(0xFE);LCDprint(192);LCDprintChar(line2);
      #endif
      refreshLCD=0;
    }
    for (chan = ROLL; chan < 4; chan++) rcData[chan] = readRawRC(chan);
    //switch config param with pitch
    if (rcData[PITCH] < MINCHECK && paramActive == 0 && p<16) {
      paramActive = 1;refreshLCD=1;blinkLED(10,20,1);
      p++;
    }
    if (rcData[PITCH] > MAXCHECK && paramActive == 0 && p>0) {
      paramActive = 1;refreshLCD=1;blinkLED(10,20,1);
      p--; 
    }
    if (rcData[PITCH] < MAXCHECK && rcData[PITCH] > MINCHECK)  paramActive = 0;
    //+ or - param with low and high roll
    if (rcData[ROLL] < MINCHECK && valActive == 0 && *param[p].var>param[p].increment-1) {
      valActive = 1;refreshLCD=1;blinkLED(10,20,1);
      *param[p].var -= param[p].increment;  //set val -
      if (p == 0) *param[4].var = *param[0].var; //PITCH P
    }
    if (rcData[ROLL] > MAXCHECK && valActive == 0) {
      valActive = 1;refreshLCD=1;blinkLED(10,20,1);
      *param[p].var += param[p].increment;       //set val +
      if (p == 0) *param[4].var = *param[0].var; //PITCH P
    }
    if (rcData[ROLL] < MAXCHECK && rcData[ROLL]  > MINCHECK) valActive = 0;
    if (rcData[YAW]  < MINCHECK && rcData[PITCH] > MAXCHECK) LCD = 0; // save and exit
    if (rcData[YAW]  > MAXCHECK && rcData[PITCH] > MAXCHECK) LCD = 2; // exit without save: eeprom has only 100.000 write cycles
  }
  #if defined(LCD_TEXTSTAR)
    blinkLED(20,30,1);
    LCDprint(0x0c); //clear screen
    if ( LCD == 0) LCDprintChar("Saving Settings.."); else LCDprintChar("skipping Save.");
  #endif
  if ( LCD == 0) writeParams();
  #if defined(LCD_TEXTSTAR)
    LCDprintChar("..done! Exit.");
  #else
    Serial.begin(SERIAL_COM_SPEED);
  #endif
}


