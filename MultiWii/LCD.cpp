#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "MultiWii.h"
#include "Alarms.h"
#include "EEPROM.h"
#include "Output.h"
#include "RX.h"
#include "Serial.h"
#include "Sensors.h"
#include "LCD.h"

void __u8Inc(void * var, int16_t inc);
void __s8Inc(void * var, int16_t inc);
void __u16Inc(void * var, int16_t inc);
void __s16Inc(void * var, int16_t inc);
void __nullInc(void * var, int16_t inc);
void __u8Fmt(void * var, uint8_t mul, uint8_t dec);
void __u16Fmt(void * var, uint8_t mul, uint8_t dec);
void __s8BitsFmt(void * var, uint8_t mul, uint8_t dec);
void __s16Fmt(void * var, uint8_t mul, uint8_t dec);
void __uAuxFmt(void * var, uint8_t mul, uint8_t dec, uint8_t aux);
void __uAuxFmt1(void * var, uint8_t mul, uint8_t dec);
void __uAuxFmt2(void * var, uint8_t mul, uint8_t dec);
void __uAuxFmt3(void * var, uint8_t mul, uint8_t dec);
void __uAuxFmt4(void * var, uint8_t mul, uint8_t dec);
void __upMFmt(void * var, uint8_t mul, uint8_t dec);

void serviceCheckPLog(void);
void i2c_clear_OLED(void);
void LCDnextline(void);
void i2c_OLED_DIGOLE_send_string(const char *string);

// ************************************************************************************************************
// LCD & display & monitoring
// ************************************************************************************************************
// in any of the following cases an LCD is required and
// the primitives for exactly one of the available types are setup
#if defined(LCD_CONF) || defined(LCD_TELEMETRY) || defined(HAS_LCD)
static char line1[17], line2[17];
static char template7[7] = " .... ";
static char template3[3] = ". ";



#if ( defined(LOG_PERMANENT) && defined(DISPLAY_MULTILINE) )
static uint8_t lnr = 0;
#endif

#define LCD_FLUSH {/*UartSendData();*/  delay(30);}

char digit10000(uint16_t v) {
  return '0' + v / 10000;
}
char digit1000(uint16_t v) {
  return '0' + v / 1000 - (v / 10000) * 10;
}
char digit100(uint16_t v) {
  return '0' + v / 100 - (v / 1000) * 10;
}
char digit10(uint16_t v) {
  return '0' + v / 10 - (v / 100) * 10;
}
char digit1(uint16_t v) {
  return '0' + v - (v / 10) * 10;
}



#if defined(ST7735) // ST7735
#include <SPI.h>
#include "Ucglib.h"

Ucglib_ST7735_18x128x160_HWSPI tft(/*cd=*/ 3, /*cs=*/ 53, /*reset=*/ 2);
// *********************
// ST7735 primitives
// *********************
void ST7735_init () {
  tft.begin(UCG_FONT_MODE_SOLID);
  //tft.begin(UCG_FONT_MODE_TRANSPARENT);
  tft.clearScreen();    // clear screen
  tft.setFont(ucg_font_8x13_mf);
}

void ST7735_send_string(const char *string) { // Sends a string of chars but not the null terminator
  tft.print(string);
}
void ST7735_printString(const char *string) { // prints a string of chars
  tft.print(string);
}
void ST7735_printChar(char c) { // prints a single char - should be printable
  tft.print(c);
}
void ST7735_set_line(byte row) {   // goto the beginning of a single row, compattible with LCD_CONFIG
  tft.setPrintPos(0, (row + 1) * 12);
}
void ST7735_clear(void) {
  tft.clearScreen();
}
#endif // ST7735
/*-----------------------------------------------------------------------------------------------------------------*/
void LCDprint(uint8_t i) {
  tft.print(i);
}

void LCDprintChar(const char *s) {
  tft.print(s);
}

void LCDcrlf() {
#if ( defined(OLED_I2C_128x64)|| defined(LCD_VT100) || defined(OLED_DIGOLE) )
  // do nothing - these displays use line positioning
#else
  LCDprintChar("\r\n");
#endif
}
void LCDclear() {
  tft.clearScreen();
}

void LCDsetLine(byte line) {
  ST7735_set_line(line - 1);
}
#if defined(LCD_VT100)
void LCDattributesBold() {
  LCDprint(0x1b);
  LCDprint(0x5b);
  LCDprintChar("1m");
}
void LCDattributesReverse() {
  LCDprint(0x1b);
  LCDprint(0x5b);
  LCDprintChar("7m");
}
void LCDattributesOff() {
  LCDprint(0x1b);
  LCDprint(0x5b);
  LCDprintChar("0m");
}
void LCDalarmAndReverse() {
  LCDattributesReverse();  // audio for errors only while armed
  if (f.ARMED) {
    LCDprint(0x07);
  };
}
#elif defined(OLED_I2C_128x64)
void LCDattributesBold() {
  /*CHAR_FORMAT = 0b01111111; */
}
void LCDattributesReverse() {
  CHAR_FORMAT = 0b01111111;
}
void LCDattributesOff() {
  CHAR_FORMAT = 0;
}
void LCDalarmAndReverse() {
  LCDattributesReverse();
}
#else
void LCDattributesBold() {}
void LCDattributesReverse() {}
void LCDattributesOff() {}
void LCDalarmAndReverse() {}
#endif

void LCDprintInt16(int16_t v, uint8_t x, uint8_t y) {
  uint16_t unit;
  char buff[7]; // = "      ";
  if (v < 0 ) {
    unit = -v;
    buff[0] = '-';
  } else {
    unit = v;
    buff[0] = ' ';
  }
  buff[1] = digit10000(unit);
  buff[2] = digit1000(unit);
  buff[3] = digit100(unit);
  buff[4] = digit10(unit);
  buff[5] = digit1(unit);
  buff[6] = 0;

  tft.print(buff);
}
void lcdprint_uint32(uint32_t v) {
  static char line[14] = "-.---.---.---";
  //                      0 2 4 6 8   12
  line[0]  = '0' + v  / 1000000000;
  line[2]  = '0' + v  / 100000000 - (v / 1000000000) * 10;
  line[3]  = '0' + v  / 10000000  - (v / 100000000)  * 10;
  line[4]  = '0' + v  / 1000000   - (v / 10000000)   * 10;
  line[6]  = '0' + v  / 100000    - (v / 1000000)    * 10;
  line[7]  = '0' + v  / 10000     - (v / 100000)     * 10;
  line[8]  = '0' + v  / 1000      - (v / 10000)      * 10;
  line[10]  = '0' + v  / 100       - (v / 1000)       * 10;
  line[11] = '0' + v  / 10        - (v / 100)        * 10;
  line[12] = '0' + v              - (v / 10)         * 10;
  LCDprintChar(line);
}

void initLCD() {
  char buff[16];
  blinkLED(20, 30, 1);
  SET_ALARM_BUZZER(ALRM_FAC_CONFIRM, ALRM_LVL_CONFIRM_1);
#if defined(ST7735)
  ST7735_init();
#endif
#ifndef OLED_I2C_128x64LOGO_PERMANENT
  LCDclear();
  strcpy_P(buff, PSTR( BOARD_NAME )); // user defined macro
  //                     0123456789.123456
  buff[12] = digit100(VERSION);
  buff[14] = digit10(VERSION);
  buff[15] = digit1(VERSION);
  LCDattributesBold();
  LCDsetLine(1); LCDprintChar(buff);
  strcpy_P(line2, PSTR("  Unknown Modell"));
#if defined(TRI)
  strcpy_P(line2, PSTR("  TRICopter"));
#elif defined(QUADP)
  strcpy_P(line2, PSTR("  QUAD-P"));
#elif defined(QUADX)
  strcpy_P(line2, PSTR("  QUAD-X"));
#elif defined(BI)
  strcpy_P(line2, PSTR("  BICopter"));
#elif defined(Y6)
  strcpy_P(line2, PSTR("  Y6"));
#elif defined(HEX6)
  strcpy_P(line2, PSTR("  HEX6"));
#elif defined(FLYING_WING)
  strcpy_P(line2, PSTR("  FLYING_WING"));
#elif defined(Y4)
  strcpy_P(line2, PSTR("  Y4"));
#elif defined(HEX6X)
  strcpy_P(line2, PSTR("  HEX6-X"));
#elif defined(HEX6H)
  strcpy_P(line2, PSTR("  HEX6-H"));
#elif defined(OCTOX8)
  strcpy_P(line2, PSTR("  OCTOX8"));
#elif defined(OCTOFLATP)
  strcpy_P(line2, PSTR("  OCTOFLAT-P"));
#elif defined(OCTOFLATX)
  strcpy_P(line2, PSTR("  OCTOFLAT-X"));
#elif defined (AIRPLANE)
  strcpy_P(line2, PSTR("  AIRPLANE"));
#elif defined (HELI_120_CCPM)
  strcpy_P(line2, PSTR("  HELI_120_CCPM"));
#elif defined (HELI_90_DEG)
  strcpy_P(line2, PSTR("  HELI_90_DEG"));
#elif defined(VTAIL4)
  strcpy_P(line2, PSTR("  VTAIL Quad"));
#endif
  //LCDattributesBold();
  tft.setPrintPos(5, 30);
  tft.print(line2);
  LCDattributesOff();
  delay(2500);
#endif // OLED_I2C_128x64LOGO_PERMANENT

#if defined(ST7735) && defined(NEW_OLED_FONT) && !(defined(LCD_TELEMETRY))
  // no need to diplay this, if LCD telemetry is enabled
  //   optional instruction on the display......
  LCDsetLine(4); LCDprintChar("To ENTER CONFIG      ");// 21 characters on each line
  LCDsetLine(5); LCDprintChar("YAW RIGHT & PITCH FWD");
  LCDsetLine(7); LCDprintChar("To SAVE CONFIG       ");
  LCDsetLine(8); LCDprintChar("YAW LEFT & PITCH FWD ");
#endif
  //  if (cycleTime == 0) { //Called from Setup()
  //    strcpy_P(line1,PSTR("Ready to Fly")); LCDsetLine(2); LCDprintChar(line1);
  //  } else {
  //    strcpy_P(line1,PSTR("Config All Parms")); LCDsetLine(2); LCDprintChar(line1);
  //  }
#ifdef LCD_TELEMETRY_STEP
  // load the first page of the step sequence
  LCDclear();
  telemetry = telemetryStepSequence[telemetryStepIndex]; //[++telemetryStepIndex % strlen(telemetryStepSequence)];
#endif
}
#endif //Support functions for LCD_CONF and LCD_TELEMETRY

// -------------------- configuration menu to LCD over serial/i2c ----------------------------------

#ifdef LCD_CONF
static uint8_t reset_to_defaults;

typedef void (*formatter_func_ptr)(void *, uint8_t, uint8_t);
typedef void (*inc_func_ptr)(void *, int16_t);

/*typedef*/struct lcd_type_desc_t {
  formatter_func_ptr fmt;
  inc_func_ptr inc;
};

static lcd_type_desc_t LTU8 = {&__u8Fmt, &__u8Inc};
static lcd_type_desc_t LTS8 = {&__s8BitsFmt, &__s8Inc};
static lcd_type_desc_t LTU16 = {&__u16Fmt, &__u16Inc};
static lcd_type_desc_t LTS16 = {&__s16Fmt, &__s16Inc};
static lcd_type_desc_t LPMM = {&__upMFmt, &__nullInc};
//static lcd_type_desc_t LPMS = {&__upSFmt, &__nullInc};
static lcd_type_desc_t LAUX1 = {&__uAuxFmt1, &__u16Inc};
static lcd_type_desc_t LAUX2 = {&__uAuxFmt2, &__u16Inc};
static lcd_type_desc_t LAUX3 = {&__uAuxFmt3, &__u16Inc};
static lcd_type_desc_t LAUX4 = {&__uAuxFmt4, &__u16Inc};

/*typedef*/struct lcd_param_def_t {
  lcd_type_desc_t * type;
  uint8_t decimal;
  uint8_t multiplier;
  uint16_t increment;
};

//typedef struct lcd_param_t{
//  char*  paramText;
//  void * var;
//  lcd_param_def_t * def;
//};

// ************************************************************************************************************
// LCD Layout definition
// ************************************************************************************************************
// Descriptors
static lcd_param_def_t __P = {&LTU8, 1, 1, 1};
static lcd_param_def_t __I = {&LTU8, 3, 1, 1};
static lcd_param_def_t __D = {&LTU8, 0, 1, 1};
static lcd_param_def_t __RC = {&LTU8, 2, 1, 1};
static lcd_param_def_t __PM = {&LPMM, 1, 1, 0};
//static lcd_param_def_t __PS = {&LPMS, 1, 1, 0};
static lcd_param_def_t __PT = {&LTU8, 0, 1, 1};
static lcd_param_def_t __VB = {&LTU8, 1, 1, 0};
static lcd_param_def_t __L = {&LTU8, 0, 1, 0};
static lcd_param_def_t __FS = {&LTU8, 1, 1, 0};
static lcd_param_def_t __SE = {&LTU16, 0, 1, 10};
static lcd_param_def_t __SE1 = {&LTU16, 0, 1, 1};
static lcd_param_def_t __ST = {&LTS16, 0, 1, 10};
static lcd_param_def_t __AUX1 = {&LAUX1, 0, 1, 1};
static lcd_param_def_t __AUX2 = {&LAUX2, 0, 1, 8};
static lcd_param_def_t __AUX3 = {&LAUX3, 0, 1, 64};
static lcd_param_def_t __AUX4 = {&LAUX4, 0, 1, 512};
static lcd_param_def_t __BITS = {&LTS8, 0, 1, 1};

// Program Space Strings - These sit in program flash, not SRAM.
//                                       0123456789
const char PROGMEM lcd_param_text01 [] = "Pit&Roll P";
const char PROGMEM lcd_param_text02 [] = "Roll     P";
const char PROGMEM lcd_param_text03 [] = "Roll     I";
const char PROGMEM lcd_param_text04 [] = "Roll     D";
const char PROGMEM lcd_param_text05 [] = "Pitch    P";
const char PROGMEM lcd_param_text06 [] = "Pitch    I";
const char PROGMEM lcd_param_text07 [] = "Pitch    D";
const char PROGMEM lcd_param_text08 [] = "Yaw      P";
const char PROGMEM lcd_param_text09 [] = "Yaw      I";
const char PROGMEM lcd_param_text10 [] = "Yaw      D";
#if  BARO && (!defined(SUPPRESS_BARO_ALTHOLD))
const char PROGMEM lcd_param_text11 [] = "Alt      P";
const char PROGMEM lcd_param_text12 [] = "Alt      I";
const char PROGMEM lcd_param_text13 [] = "Alt      D";
const char PROGMEM lcd_param_text14 [] = "Vel      P";
const char PROGMEM lcd_param_text15 [] = "Vel      I";
const char PROGMEM lcd_param_text16 [] = "Vel      D";
#endif
const char PROGMEM lcd_param_text17 [] = "Ang/Hor  P";
const char PROGMEM lcd_param_text18 [] = "Ang/Hor  I";
const char PROGMEM lcd_param_text188[] = "Ang/Hor  D";
const char PROGMEM lcd_param_text19 [] = "Mag      P";
const char PROGMEM lcd_param_text20 [] = "RC Rate   ";
const char PROGMEM lcd_param_text21 [] = "RC Expo   ";
const char PROGMEM lcd_param_text20t [] = "Thrott Mid";
const char PROGMEM lcd_param_text21t [] = "ThrottExpo";
const char PROGMEM lcd_param_text22 [] = "P&R Rate  ";
const char PROGMEM lcd_param_text23 [] = "Yaw Rate  ";
const char PROGMEM lcd_param_text24 [] = "Thrott PID";
#ifdef LOG_VALUES
#if (LOG_VALUES >= 3)
const char PROGMEM lcd_param_text25 [] = "pmeter m0";
const char PROGMEM lcd_param_text26 [] = "pmeter m1";
const char PROGMEM lcd_param_text27 [] = "pmeter m2";
const char PROGMEM lcd_param_text28 [] = "pmeter m3";
const char PROGMEM lcd_param_text29 [] = "pmeter m4";
const char PROGMEM lcd_param_text30 [] = "pmeter m5";
const char PROGMEM lcd_param_text31 [] = "pmeter m6";
const char PROGMEM lcd_param_text32 [] = "pmeter m7";
#endif //                                 0123456789
#endif
#ifdef FLYING_WING
const char PROGMEM lcd_param_text36 [] = "SERvTRIM1";
const char PROGMEM lcd_param_text37 [] = "SERvTRIM2";
#endif
#ifdef TRI //                             0123456789
const char PROGMEM lcd_param_text38 [] = "SERvTRIMy";
const char PROGMEM lcd_param_text39 [] = "SERvINVy";
const char PROGMEM lcd_param_text152 [] = "SERvMINy";
const char PROGMEM lcd_param_text153 [] = "SERvMAXy";
#endif
//#ifdef LOG_VALUES
//const char PROGMEM lcd_param_text39 [] = "failsafes ";
//const char PROGMEM lcd_param_text40 [] = "i2c errors";
//const char PROGMEM lcd_param_text41 [] = "an overrun";
//#endif
#if defined(LCD_CONF_AUX)
const char PROGMEM lcd_param_text41 [] = "AUX angle ";
const char PROGMEM lcd_param_text42 [] = "AUX horizn";
const char PROGMEM lcd_param_text43 [] = "AUX baro  ";
const char PROGMEM lcd_param_text44 [] = "AUX mag   ";
const char PROGMEM lcd_param_text45 [] = "AUX camstb";
const char PROGMEM lcd_param_text46 [] = "AUX camtrg";
const char PROGMEM lcd_param_text47 [] = "AUX arm   ";
const char PROGMEM lcd_param_text48 [] = "AUX gpshom";
const char PROGMEM lcd_param_text49 [] = "AUX gpshld";
const char PROGMEM lcd_param_text50 [] = "AUX passth";
const char PROGMEM lcd_param_text51 [] = "AUX headfr";
const char PROGMEM lcd_param_text52 [] = "AUX buzzer";
const char PROGMEM lcd_param_text53 [] = "AUX vario ";
const char PROGMEM lcd_param_text54 [] = "AUX calib ";
const char PROGMEM lcd_param_text55 [] = "AUX govern";
const char PROGMEM lcd_param_text56 [] = "AUX osd   ";
// 53 to 61 reserved
#endif
#ifdef HELI_120_CCPM //                  0123456789
const char PROGMEM lcd_param_text73 [] = "SERvTRIMn";
const char PROGMEM lcd_param_text74 [] = "SERvTRIMl";
const char PROGMEM lcd_param_text75 [] = "SERvTRIMy";
const char PROGMEM lcd_param_text76 [] = "SERvTRIMr";
const char PROGMEM lcd_param_text140 [] = "SERvINVn";
const char PROGMEM lcd_param_text141 [] = "SERvINVl";
const char PROGMEM lcd_param_text142 [] = "SERvINVy";
const char PROGMEM lcd_param_text143 [] = "SERvINVr";
#endif
#ifdef GYRO_SMOOTHING //                 0123456789
const char PROGMEM lcd_param_text80 [] = "GSMOOTH R ";
const char PROGMEM lcd_param_text81 [] = "GSMOOTH P ";
const char PROGMEM lcd_param_text82 [] = "GSMOOTH Y ";
#endif
#ifdef AIRPLANE //                       0123456789
const char PROGMEM lcd_param_text83 [] = "SERVoMID3";
const char PROGMEM lcd_param_text84 [] = "SERVoMID4";
const char PROGMEM lcd_param_text85 [] = "SERVoMID5";
const char PROGMEM lcd_param_text86 [] = "SERVoMID6";
const char PROGMEM lcd_param_text87 [] = "SERVoMID7";
#endif
#if GPS
const char PROGMEM lcd_param_text91 [] = "GPS Pos. P";
const char PROGMEM lcd_param_text92 [] = "GPS Pos. I";
const char PROGMEM lcd_param_text93 [] = "Pos Rate P";
const char PROGMEM lcd_param_text94 [] = "Pos Rate I";
const char PROGMEM lcd_param_text95 [] = "Pos Rate D";
const char PROGMEM lcd_param_text96 [] = "NAV Rate P";
const char PROGMEM lcd_param_text97 [] = "NAV Rate I";
const char PROGMEM lcd_param_text98 [] = "NAV Rate D";
#endif
#if defined (FAILSAFE)
const char PROGMEM lcd_param_text101 [] = "FailThrot";
#endif
#ifdef VBAT
const char PROGMEM lcd_param_text35 [] =  "batt volt ";
const char PROGMEM lcd_param_text102 [] = "VBAT SCALE";
const char PROGMEM lcd_param_text103 [] = "BattWarn 1";
const char PROGMEM lcd_param_text104 [] = "BattWarn 2";
const char PROGMEM lcd_param_text106 [] = "BattW Crit";
//const char PROGMEM lcd_param_text107 [] = "Batt NoBat";
#endif
#ifdef POWERMETER
const char PROGMEM lcd_param_text33 [] = "pmeterSum";
const char PROGMEM lcd_param_text34 [] = "pAlarm /50"; // change text to represent PLEVELSCALE value
const char PROGMEM lcd_param_text111 [] = "PMsENSOR0";
const char PROGMEM lcd_param_text114 [] = "PM INT2MA ";
//const char PROGMEM lcd_param_text112 [] = "PM DIVSOFT";
//const char PROGMEM lcd_param_text113 [] = "PM DIV    ";
#endif
#ifdef MMGYRO
const char PROGMEM lcd_param_text121 [] = "MMGYRO    ";
#endif
const char PROGMEM lcd_param_text131 [] = "MINTHROTL";
#if defined(ARMEDTIMEWARNING)
const char PROGMEM lcd_param_text132 [] = "ArmdTWarn";
#endif
#ifdef GOVERNOR_P
const char PROGMEM lcd_param_text133 [] = "Govern  P";
const char PROGMEM lcd_param_text134 [] = "Govern  D";
const char PROGMEM lcd_param_text135 [] = "GovernRpm";
#endif
#ifdef MULTIPLE_CONFIGURATION_PROFILES
const char PROGMEM lcd_param_text150 [] = "writeCset";
#endif
const char PROGMEM lcd_param_text151 [] = "Reset (7)";
#ifdef YAW_COLL_PRECOMP
const char PROGMEM lcd_param_text155 [] = "yawPrcomp";
const char PROGMEM lcd_param_text156 [] = "yawPrDead";
#endif
//                                         012345678

PROGMEM const void * const lcd_param_ptr_table [] = {
  &lcd_param_text01, &conf.pid[ROLL].P8, &__P,
  &lcd_param_text02, &conf.pid[ROLL].P8, &__P,
  &lcd_param_text03, &conf.pid[ROLL].I8, &__I,
  &lcd_param_text04, &conf.pid[ROLL].D8, &__D,
  &lcd_param_text05, &conf.pid[PITCH].P8, &__P,
  &lcd_param_text06, &conf.pid[PITCH].I8, &__I,
  &lcd_param_text07, &conf.pid[PITCH].D8, &__D,
  &lcd_param_text08, &conf.pid[YAW].P8, &__P,
  &lcd_param_text09, &conf.pid[YAW].I8, &__I,
#if (!(PID_CONTROLLER == 1)) || (!defined(COPTER_WITH_SERVO))
  &lcd_param_text10, &conf.pid[YAW].D8, &__D,
#endif
#if BARO && (!defined(SUPPRESS_BARO_ALTHOLD))
  &lcd_param_text11, &conf.pid[PIDALT].P8, &__P,
  &lcd_param_text12, &conf.pid[PIDALT].I8, &__I,
  &lcd_param_text13, &conf.pid[PIDALT].D8, &__D,
  &lcd_param_text14, &conf.pid[PIDVEL].P8, &__P,
  &lcd_param_text15, &conf.pid[PIDVEL].I8, &__I,
  &lcd_param_text16, &conf.pid[PIDVEL].D8, &__D,
#endif
  &lcd_param_text17, &conf.pid[PIDLEVEL].P8, &__P,
  &lcd_param_text18, &conf.pid[PIDLEVEL].I8, &__I,
  &lcd_param_text188, &conf.pid[PIDLEVEL].D8, &__D,
#if MAG
  &lcd_param_text19, &conf.pid[PIDMAG].P8, &__P,
#endif
  &lcd_param_text20t, &conf.thrMid8, &__RC,
  &lcd_param_text21t, &conf.thrExpo8, &__RC,
  &lcd_param_text20, &conf.rcRate8, &__RC,
  &lcd_param_text21, &conf.rcExpo8, &__RC,
  &lcd_param_text22, &conf.rollPitchRate, &__RC,
  &lcd_param_text23, &conf.yawRate, &__RC,
  &lcd_param_text24, &conf.dynThrPID, &__RC,
#if GPS
  &lcd_param_text91, &conf.pid[PIDPOS].P8 , &__RC,
  &lcd_param_text92, &conf.pid[PIDPOS].I8 , &__I,
  &lcd_param_text93, &conf.pid[PIDPOSR].P8, &__P,
  &lcd_param_text94, &conf.pid[PIDPOSR].I8, &__I,
  &lcd_param_text95, &conf.pid[PIDPOSR].D8, &__I,
  &lcd_param_text96, &conf.pid[PIDNAVR].P8, &__P,
  &lcd_param_text97, &conf.pid[PIDNAVR].I8, &__RC,
  &lcd_param_text98, &conf.pid[PIDNAVR].D8, &__I,
#endif
#ifdef LCD_CONF_AUX
#if ACC
  &lcd_param_text41, &conf.activate[BOXANGLE], &__AUX1,
#ifndef SUPPRESS_LCD_CONF_AUX2
  &lcd_param_text41, &conf.activate[BOXANGLE], &__AUX2,
#endif
  &lcd_param_text42, &conf.activate[BOXHORIZON], &__AUX1,
#ifndef SUPPRESS_LCD_CONF_AUX2
  &lcd_param_text42, &conf.activate[BOXHORIZON], &__AUX2,
#endif
#ifndef SUPPRESS_LCD_CONF_AUX34
  &lcd_param_text41, &conf.activate[BOXANGLE], &__AUX3,
  &lcd_param_text41, &conf.activate[BOXANGLE], &__AUX4,
  &lcd_param_text42, &conf.activate[BOXHORIZON], &__AUX3,
  &lcd_param_text42, &conf.activate[BOXHORIZON], &__AUX4,
#endif
#endif
#if BARO && (!defined(SUPPRESS_BARO_ALTHOLD))
  &lcd_param_text43, &conf.activate[BOXBARO], &__AUX1,
#ifndef SUPPRESS_LCD_CONF_AUX2
  &lcd_param_text43, &conf.activate[BOXBARO], &__AUX2,
#endif
#ifndef SUPPRESS_LCD_CONF_AUX34
  &lcd_param_text43, &conf.activate[BOXBARO], &__AUX3,
  &lcd_param_text43, &conf.activate[BOXBARO], &__AUX4,
#endif
#endif
  &lcd_param_text44, &conf.activate[BOXMAG], &__AUX1,
#ifndef SUPPRESS_LCD_CONF_AUX2
  &lcd_param_text44, &conf.activate[BOXMAG], &__AUX2,
#endif
#ifndef SUPPRESS_LCD_CONF_AUX34
  &lcd_param_text44, &conf.activate[BOXMAG], &__AUX3,
  &lcd_param_text44, &conf.activate[BOXMAG], &__AUX4,
#endif
#ifdef GIMBAL
  &lcd_param_text45, &conf.activate[BOXCAMSTAB], &__AUX1,
#ifndef SUPPRESS_LCD_CONF_AUX2
  &lcd_param_text45, &conf.activate[BOXCAMSTAB], &__AUX2,
#endif
#ifndef SUPPRESS_LCD_CONF_AUX34
  &lcd_param_text45, &conf.activate[BOXCAMSTAB], &__AUX3,
  &lcd_param_text45, &conf.activate[BOXCAMSTAB], &__AUX4,
#endif
  &lcd_param_text46, &conf.activate[BOXCAMTRIG], &__AUX1,
#ifndef SUPPRESS_LCD_CONF_AUX2
  &lcd_param_text46, &conf.activate[BOXCAMTRIG], &__AUX2,
#endif
#ifndef SUPPRESS_LCD_CONF_AUX34
  &lcd_param_text46, &conf.activate[BOXCAMTRIG], &__AUX3,
  &lcd_param_text46, &conf.activate[BOXCAMTRIG], &__AUX4,
#endif
#endif
  &lcd_param_text47, &conf.activate[BOXARM], &__AUX1,
#ifndef SUPPRESS_LCD_CONF_AUX2
  &lcd_param_text47, &conf.activate[BOXARM], &__AUX2,
#endif
#ifndef SUPPRESS_LCD_CONF_AUX34
  &lcd_param_text47, &conf.activate[BOXARM], &__AUX3,
  &lcd_param_text47, &conf.activate[BOXARM], &__AUX4,
#endif
#if GPS
  &lcd_param_text48, &conf.activate[BOXGPSHOME], &__AUX1,
#ifndef SUPPRESS_LCD_CONF_AUX2
  &lcd_param_text48, &conf.activate[BOXGPSHOME], &__AUX2,
#endif
#ifndef SUPPRESS_LCD_CONF_AUX34
  &lcd_param_text48, &conf.activate[BOXGPSHOME], &__AUX3,
  &lcd_param_text48, &conf.activate[BOXGPSHOME], &__AUX4,
#endif
  &lcd_param_text49, &conf.activate[BOXGPSHOLD], &__AUX1,
#ifndef SUPPRESS_LCD_CONF_AUX2
  &lcd_param_text49, &conf.activate[BOXGPSHOLD], &__AUX2,
#endif
#ifndef SUPPRESS_LCD_CONF_AUX34
  &lcd_param_text49, &conf.activate[BOXGPSHOLD], &__AUX3,
  &lcd_param_text49, &conf.activate[BOXGPSHOLD], &__AUX4,
#endif
#endif
#if defined(FIXEDWING) || defined(HELICOPTER)
  &lcd_param_text50, &conf.activate[BOXPASSTHRU], &__AUX1,
#ifndef SUPPRESS_LCD_CONF_AUX2
  &lcd_param_text50, &conf.activate[BOXPASSTHRU], &__AUX2,
#endif
#ifndef SUPPRESS_LCD_CONF_AUX34
  &lcd_param_text50, &conf.activate[BOXPASSTHRU], &__AUX3,
  &lcd_param_text50, &conf.activate[BOXPASSTHRU], &__AUX4,
#endif
#endif
#if defined(HEADFREE)
  &lcd_param_text51, &conf.activate[BOXHEADFREE], &__AUX1,
#ifndef SUPPRESS_LCD_CONF_AUX2
  &lcd_param_text51, &conf.activate[BOXHEADFREE], &__AUX2,
#endif
#ifndef SUPPRESS_LCD_CONF_AUX34
  &lcd_param_text51, &conf.activate[BOXHEADFREE], &__AUX3,
  &lcd_param_text51, &conf.activate[BOXHEADFREE], &__AUX4,
#endif
#endif
#if defined(BUZZER)
  &lcd_param_text52, &conf.activate[BOXBEEPERON], &__AUX1,
#ifndef SUPPRESS_LCD_CONF_AUX2
  &lcd_param_text52, &conf.activate[BOXBEEPERON], &__AUX2,
#endif
#ifndef SUPPRESS_LCD_CONF_AUX34
  &lcd_param_text52, &conf.activate[BOXBEEPERON], &__AUX3,
  &lcd_param_text52, &conf.activate[BOXBEEPERON], &__AUX4,
#endif
#endif
#ifdef VARIOMETER
  &lcd_param_text53, &conf.activate[BOXVARIO], &__AUX1,
#ifndef SUPPRESS_LCD_CONF_AUX2
  &lcd_param_text53, &conf.activate[BOXVARIO], &__AUX2,
#endif
#ifndef SUPPRESS_LCD_CONF_AUX34
  &lcd_param_text53, &conf.activate[BOXVARIO], &__AUX3,
  &lcd_param_text53, &conf.activate[BOXVARIO], &__AUX4,
#endif
#endif
#ifdef INFLIGHT_ACC_CALIBRATION
  &lcd_param_text54, &conf.activate[BOXCALIB], &__AUX1,
#ifndef SUPPRESS_LCD_CONF_AUX2
  &lcd_param_text54, &conf.activate[BOXCALIB], &__AUX2,
#endif
#ifndef SUPPRESS_LCD_CONF_AUX34
  &lcd_param_text54, &conf.activate[BOXCALIB], &__AUX3,
  &lcd_param_text54, &conf.activate[BOXCALIB], &__AUX4,
#endif
#endif
#ifdef GOVERNOR_P
  &lcd_param_text55, &conf.activate[BOXGOV], &__AUX1,
#ifndef SUPPRESS_LCD_CONF_AUX2
  &lcd_param_text55, &conf.activate[BOXGOV], &__AUX2,
#endif
#ifndef SUPPRESS_LCD_CONF_AUX34
  &lcd_param_text55, &conf.activate[BOXGOV], &__AUX3,
  &lcd_param_text55, &conf.activate[BOXGOV], &__AUX4,
#endif
#endif
#ifdef OSD_SWITCH
  &lcd_param_text56, &conf.activate[BOXOSD], &__AUX1,
#ifndef SUPPRESS_LCD_CONF_AUX2
  &lcd_param_text56, &conf.activate[BOXOSD], &__AUX2,
#endif
#ifndef SUPPRESS_LCD_CONF_AUX34
  &lcd_param_text56, &conf.activate[BOXOSD], &__AUX3,
  &lcd_param_text56, &conf.activate[BOXOSD], &__AUX4,
#endif
#endif
#endif //lcd.conf.aux

#ifdef LOG_VALUES
#if (LOG_VALUES >= 3)
#if (NUMBER_MOTOR > 0)
  &lcd_param_text25, &pMeter[0], &__PM,
#endif
#if (NUMBER_MOTOR > 1)
  &lcd_param_text26, &pMeter[1], &__PM,
#endif
#if (NUMBER_MOTOR > 2)
  &lcd_param_text27, &pMeter[2], &__PM,
#endif
#if (NUMBER_MOTOR > 3)
  &lcd_param_text28, &pMeter[3], &__PM,
#endif
#if (NUMBER_MOTOR > 4)
  &lcd_param_text29, &pMeter[4], &__PM,
#endif
#if (NUMBER_MOTOR > 5)
  &lcd_param_text30, &pMeter[5], &__PM,
#endif
#if (NUMBER_MOTOR > 6)
  &lcd_param_text31, &pMeter[6], &__PM,
#endif
#if (NUMBER_MOTOR > 7)
  &lcd_param_text32, &pMeter[7], &__PM,
#endif
#endif
#endif
  &lcd_param_text131, &conf.minthrottle, &__ST,
#if defined (FAILSAFE)
  &lcd_param_text101, &conf.failsafe_throttle, &__ST,
#endif
#ifdef FLYING_WING
  &lcd_param_text36, &conf.servoConf[3].middle, &__SE,
  &lcd_param_text37, &conf.servoConf[4].middle, &__SE,
#endif
#ifdef TRI
  &lcd_param_text152, &conf.servoConf[5].min, &__SE,
  &lcd_param_text153, &conf.servoConf[5].max, &__SE,
  &lcd_param_text38, &conf.servoConf[5].middle, &__SE,
  &lcd_param_text39, &conf.servoConf[5].rate, &__BITS,
#endif
#ifdef HELI_120_CCPM
  &lcd_param_text73, &conf.servoConf[3].middle, &__SE,
  &lcd_param_text74, &conf.servoConf[4].middle, &__SE,
  &lcd_param_text76, &conf.servoConf[6].middle, &__SE,
  &lcd_param_text75, &conf.servoConf[5].middle, &__SE,
  &lcd_param_text140, &conf.servoConf[3].rate, &__BITS,
  &lcd_param_text141, &conf.servoConf[4].rate, &__BITS,
  &lcd_param_text143, &conf.servoConf[6].rate, &__BITS,
  &lcd_param_text142, &conf.servoConf[5].rate, &__BITS,
#endif
#ifdef GOVERNOR_P
  &lcd_param_text133, &conf.governorP, &__D,
  &lcd_param_text134, &conf.governorD, &__D,
#endif
#ifdef YAW_COLL_PRECOMP
  &lcd_param_text155, &conf.yawCollPrecomp, &__PT,
  &lcd_param_text156, &conf.yawCollPrecompDeadband, &__SE,
#endif
#ifdef GYRO_SMOOTHING
  &lcd_param_text80, &conf.Smoothing[0], &__D,
  &lcd_param_text81, &conf.Smoothing[1], &__D,
  &lcd_param_text82, &conf.Smoothing[2], &__D,
#endif
#ifdef AIRPLANE
  &lcd_param_text83, &conf.servoConf[3].middle, &__SE,
  &lcd_param_text84, &conf.servoConf[4].middle, &__SE,
  &lcd_param_text85, &conf.servoConf[5].middle, &__SE,
  &lcd_param_text86, &conf.servoConf[6].middle, &__SE,
#endif
#ifdef MMGYRO
  &lcd_param_text121, &conf.mmgyro, &__D,
#endif
#ifdef VBAT
  //  &lcd_param_text35, &analog.vbat, &__VB,
  &lcd_param_text102, &conf.vbatscale, &__PT,
  &lcd_param_text103, &conf.vbatlevel_warn1, &__P,
  &lcd_param_text104, &conf.vbatlevel_warn2, &__P,
  &lcd_param_text106, &conf.vbatlevel_crit, &__P,
  //  &lcd_param_text107, &conf.no_vbat, &__P,
#endif
#ifdef POWERMETER
  //  &lcd_param_text33, &pMeter[PMOTOR_SUM], &__PM,
  &lcd_param_text114, &conf.pint2ma, &__PT,
#ifdef POWERMETER_HARD
  &lcd_param_text111, &conf.psensornull, &__SE1,
#endif
  &lcd_param_text34, &conf.powerTrigger1, &__PT,
#endif
#if defined(ARMEDTIMEWARNING)
  &lcd_param_text132, &conf.armedtimewarning, &__SE,
#endif
#ifdef MULTIPLE_CONFIGURATION_PROFILES
  &lcd_param_text150, &global_conf.currentSet, &__D,
#endif
  &lcd_param_text151, &reset_to_defaults, &__D,
  //#ifdef LOG_VALUES
  //  &lcd_param_text39, &failsafeEvents, &__L,
  //  &lcd_param_text40, &i2c_errors_count, &__L,
  //  &lcd_param_text41, &annex650_overrun_count, &__L
  //#endif
};
#define PARAMMAX (sizeof(lcd_param_ptr_table)/6 - 1)
// ************************************************************************************************************

void __u8Inc(void * var, int16_t inc) {
  *(uint8_t*)var += (uint8_t)inc;
};
void __s8Inc(void * var, int16_t inc) {
  *(int8_t*)var += (int8_t)inc;
};
void __u16Inc(void * var, int16_t inc) {
  *(uint16_t*)var += inc;
};
void __s16Inc(void * var, int16_t inc) {
  *(int16_t*)var += inc;
};
void __nullInc(void * var, int16_t inc) {};

void __u8Fmt(void * var, uint8_t mul, uint8_t dec) {
  uint16_t unit = *(uint8_t*)var;
  unit *= mul;
  char c1 = '0' + unit / 100; char c2 = '0' + unit / 10 - (unit / 100) * 10; char c3 = '0' + unit - (unit / 10) * 10;
  switch (dec) {
    case 0: line2[3] = c1; line2[4] = c2; line2[5] = c3; break;
    case 1: line2[2] = c1; line2[3] = c2; line2[4] = '.'; line2[5] = c3; break;
    case 2: line2[2] = c1; line2[3] = '.'; line2[4] = c2; line2[5] = c3; break;
    case 3: line2[1] = '0'; line2[2] = '.'; line2[3] = c1; line2[4] = c2; line2[5] = c3; break;
  }
}

void __u16Fmt(void * var, uint8_t mul, uint8_t dec) {
  uint16_t unit = *(uint16_t*)var;
  unit *= mul;
  line2[2] = digit10000(unit);
  line2[3] = digit1000(unit);
  line2[4] = digit100(unit);
  line2[5] = digit10(unit);
  line2[6] = digit1(unit);
}
void __s16Fmt(void * var, uint8_t mul, uint8_t dec) {
  int16_t unit = *(int16_t*)var;
  if (unit >= 0) {
    line2[1] = ' ';
  } else {
    line2[1] = '-';
    unit = -unit;
  }
  __u16Fmt(&unit, mul, dec);
}
void __uAuxFmt1(void * var, uint8_t mul, uint8_t dec) {
  __uAuxFmt(var, mul, dec, 1);
}
void __uAuxFmt2(void * var, uint8_t mul, uint8_t dec) {
  __uAuxFmt(var, mul, dec, 2);
}
void __uAuxFmt3(void * var, uint8_t mul, uint8_t dec) {
  __uAuxFmt(var, mul, dec, 3);
}
void __uAuxFmt4(void * var, uint8_t mul, uint8_t dec) {
  __uAuxFmt(var, mul, dec, 4);
}


void __uAuxFmt(void * var, uint8_t mul, uint8_t dec, uint8_t aux) {
  uint16_t unit = *(uint16_t*)var;
  line2[1] =  digit1(aux);
  line2[3] =  ( unit & 1 << (3 * aux - 3) ? 'L' : '.' );
  line2[4] =  ( unit & 1 << (3 * aux - 2) ? 'M' : '.' );
  line2[5] =  ( unit & 1 << (3 * aux - 1) ? 'H' : '.' );
}

void __s8BitsFmt(void * var, uint8_t mul, uint8_t dec) {
  int8_t unit = *(int8_t*)var;
  line2[1] =  ( unit & 1 << 2 ? 'C' : '.' );
  line2[2] =  ( unit & 1 << 1 ? 'N' : '.' );
  line2[3] =  ( unit & 1 << 0 ? 'R' : '.' );
}

#ifdef POWERMETER
void __upMFmt(void * var, uint8_t mul, uint8_t dec) {
  uint32_t unit = *(uint32_t*)var;
  unit /= PLEVELDIV;
  __u16Fmt(&unit, mul, dec);
}

//void __upSFmt(void * var, uint8_t mul, uint8_t dec) {
//  uint32_t unit = *(uint32_t*)var;
//#if defined(POWERMETER_SOFT)
//  unit = (unit * (uint32_t)conf.pint2ma) / 100000; // was = unit / conf.pleveldivsoft;
//#elif defined(POWERMETER_HARD)
//  unit = unit / PLEVELDIV;
//#endif
//  __u16Fmt(&unit, mul, dec);
//}
#endif

static uint8_t lcdStickState[4];
#define IsLow(x)  (lcdStickState[x] & 0x1)
#define IsHigh(x) (lcdStickState[x] & 0x2)
#define IsMid(x)  (!lcdStickState[x])


/* ------------ DISPLAY_2LINES ------------------------------------*/
#ifdef DISPLAY_2LINES
void ConfigRefresh(uint8_t p) {
  blinkLED(10, 20, 1);
  SET_ALARM_BUZZER(ALRM_FAC_TOGGLE, ALRM_LVL_TOGGLE_1);
  strcpy_P(line1, PSTR("                "));
  strcpy(line2, line1);
  strcpy_P(line1, (char*)pgm_read_word(&(lcd_param_ptr_table[p * 3])));
  lcd_param_def_t* deft = (lcd_param_def_t*)pgm_read_word(&(lcd_param_ptr_table[(p * 3) + 2]));
  deft->type->fmt((void*)pgm_read_word(&(lcd_param_ptr_table[(p * 3) + 1])), deft->multiplier, deft->decimal);
  LCDclear();
  LCDsetLine(1); LCDprintChar(line1); //refresh line 1 of LCD
  LCDsetLine(2); LCDprintChar(line2); //refresh line 2 of LCD
}
#endif // DISPLAY_2LINES
/* ------------ DISPLAY_MULTILINE ---------------------------------*/
#ifdef DISPLAY_MULTILINE
// display slice of config items prior and after current item (index p)
void ConfigRefresh(uint8_t p) {
  uint8_t j, l = 1;
  int8_t pp = (int8_t)p;
#ifndef OLED_I2C_128x64
  blinkLED(2, 4, 1);
  SET_ALARM_BUZZER(ALRM_FAC_TOGGLE, ALRM_LVL_TOGGLE_1);
  LCDclear();
#else
  delay(60);
#endif
  for (int8_t i = pp - MULTILINE_PRE; i < pp + MULTILINE_POST; i++) {
    //j = i % (1+PARAMMAX); // why does modulo not work here?
    j = (i < 0 ? i + 1 + PARAMMAX : i);
    if (j > PARAMMAX) j -= (1 + PARAMMAX);
    strcpy_P(line1, PSTR("          "));
    strcpy(line2, line1);
    strcpy_P(line1, (char*)pgm_read_word(&(lcd_param_ptr_table[j * 3])));
    lcd_param_def_t* deft = (lcd_param_def_t*)pgm_read_word(&(lcd_param_ptr_table[(j * 3) + 2]));
    deft->type->fmt((void*)pgm_read_word(&(lcd_param_ptr_table[(j * 3) + 1])), deft->multiplier, deft->decimal);

    LCDsetLine(l++);
    if (j == p) {
#if ( !defined(OLED_I2C_128x64) && !defined(OLED_DIGOLE) )
      LCDprint('>');
#endif
#if ( defined(OLED_DIGOLE) )
      line2[0] = '*';
#endif
      LCDattributesReverse();
    }
    LCDprintChar(line1); // the label
    if (j == p) {
      LCDattributesOff(); /*LCDattributesBold();*/
    }
    //LCDprint(' ');
    LCDprintChar(line2); // the value
#if ( !defined(OLED_I2C_128x64) && !defined(OLED_DIGOLE) )
    if (j == p) {
      LCDattributesOff();
      LCDprint('<');
    }
#endif
    LCD_FLUSH;
  }
  LCDcrlf();
}
#endif // DISPLAY_MULTILINE
void configurationLoop() {
  uint8_t i, p;
  uint8_t LCD = 1;
  uint8_t refreshLCD = 1;
  uint8_t key = 0;
  uint8_t allow_exit = 0;
#ifdef MULTIPLE_CONFIGURATION_PROFILES
  uint8_t currentSet = global_conf.currentSet; // keep a copy for abort.case
#endif
  initLCD();
#if defined OLED_I2C_128x64LOGO_PERMANENT
  LCDclear();
#endif
  reset_to_defaults = 0;
  p = 0;
  while (LCD == 1) {
    if (refreshLCD) {
      ConfigRefresh(p);
      refreshLCD = 0;
    }
#if defined(SERIAL_RX)
    delay(10); // may help with timing for some serial receivers -1/100 second seems non-critical here?
    if (spekFrameFlags == 0x01) readSerial_RX();
    delay(44); // For digital receivers , to ensure that an "old" frame does not cause immediate exit at startup.
#endif
#if defined(LCD_TEXTSTAR) || defined(LCD_VT100) || defined(LCD_TTY) // textstar, vt100 and tty can send keys
    key = ( SerialAvailable(LCD_SERIAL_PORT) ? SerialRead(LCD_SERIAL_PORT) : 0 );
#endif
#ifdef LCD_CONF_DEBUG
    delay(1000);
    if (key == LCD_MENU_NEXT) key = LCD_VALUE_UP; else key = LCD_MENU_NEXT;
#endif
    for (i = ROLL; i <= THROTTLE; i++) {
      uint16_t Tmp = readRawRC(i);
      lcdStickState[i] = (Tmp < MINCHECK) | ((Tmp > MAXCHECK) << 1);
    };
    if (IsMid(YAW) && IsMid(PITCH) && IsMid(ROLL)) allow_exit = 1;
    if (key == LCD_MENU_SAVE_EXIT || (IsLow(YAW) && IsHigh(PITCH) && allow_exit))   LCD = 0; // save and exit
    else if (key == LCD_MENU_ABORT || (IsHigh(YAW) && IsHigh(PITCH) && allow_exit)) LCD = 2;// exit without save: eeprom has only 100.000 write cycles
    else if (key == LCD_MENU_NEXT || (IsLow(PITCH) && IsMid(YAW))) { //switch config param with pitch
      refreshLCD = 1; p++; if (p > PARAMMAX) p = 0;
    } else if (key == LCD_MENU_PREV || (IsHigh(PITCH) && IsMid(YAW))) {
      refreshLCD = 1; p--; if (p == 0xFF) p = PARAMMAX;
    } else if (key == LCD_VALUE_DOWN || (IsLow(ROLL))) { //+ or - param with low and high roll
      refreshLCD = 1;
      lcd_param_def_t* deft = (lcd_param_def_t*)pgm_read_word(&(lcd_param_ptr_table[(p * 3) + 2]));
      deft->type->inc((void*)pgm_read_word(&(lcd_param_ptr_table[(p * 3) + 1])), -(IsHigh(THROTTLE) ? 10 : 1) * deft->increment);
      if (p == 0) conf.pid[PITCH].P8 = conf.pid[ROLL].P8;
    } else if (key == LCD_VALUE_UP || (IsHigh(ROLL))) {
      refreshLCD = 1;
      lcd_param_def_t* deft = (lcd_param_def_t*)pgm_read_word(&(lcd_param_ptr_table[(p * 3) + 2]));
      deft->type->inc((void*)pgm_read_word(&(lcd_param_ptr_table[(p * 3) + 1])), +(IsHigh(THROTTLE) ? 10 : 1) * deft->increment);
      if (p == 0) conf.pid[PITCH].P8 = conf.pid[ROLL].P8;
    }
#if defined(PRI_SERVO_TO)
#define MAX_SERV 7
#if(PRI_SERVO_TO < MAX_SERV)
#undef  MAX_SERV
#define MAX_SERV PRI_SERVO_TO
#endif
    for (i = PRI_SERVO_FROM - 1; i < MAX_SERV; i++) servo[i] = conf.servoConf[i].middle;
#if defined(HELICOPTER) && YAWMOTOR
    servo[5] =  MINCOMMAND;
#endif
#if defined(TRI) && defined(MEGA_HW_PWM_SERVOS) && defined(MEGA)
    servo[3] = servo[5];
#endif
    writeServos();
#endif
  } // while (LCD == 1)
  blinkLED(20, 30, 1);
  SET_ALARM_BUZZER(ALRM_FAC_CONFIRM, ALRM_LVL_CONFIRM_1);
  LCDclear();
  LCDsetLine(1);
  if (LCD == 0) { //     0123456789
    strcpy_P(line1, PSTR("Saving..."));
#ifdef MULTIPLE_CONFIGURATION_PROFILES
    line1[7] = digit1(global_conf.currentSet);
#endif
    LCDprintChar(line1);
#ifdef MULTIPLE_CONFIGURATION_PROFILES
    writeGlobalSet(0);
#endif
    if (reset_to_defaults == 7) {
      // magic number lucky 7 hit, then do the reset to defaults
      strcpy_P(line1, PSTR("RESET.."));
      //                   0123456789.12345
      LCDprintChar(line1);
      LoadDefaults(); // this invokes writeParams()
    } else {
      writeParams(1);
    }
  } else {
    strcpy_P(line1, PSTR("Aborting"));
    LCDprintChar(line1);
#ifdef MULTIPLE_CONFIGURATION_PROFILES
    global_conf.currentSet = currentSet; // restore
#endif
    readEEPROM(); // roll back all values to last saved state
  }
  LCDsetLine(2);
  strcpy_P(line1, PSTR("Exit"));
  LCDprintChar(line1);
#if defined(LCD_LCD03) || defined(LCD_LCD03S)
  delay(2000); // wait for two seconds then clear screen and show initial message
  initLCD();
#endif
#if defined(LCD_SERIAL3W)
  SerialOpen(0, 115200);
#endif
#if defined(LCD_TELEMETRY) || defined(OLED_I2C_128x64)
  delay(1500); // keep exit message visible for one and one half seconds even if (auto)telemetry continues writing in main loop
#endif
  cycleTime = 0;
#if defined(OLED_I2C_128x64)
#if defined(OLED_I2C_128x64LOGO_PERMANENT)
  i2c_OLED_Put_Logo();
#elif !defined(LOG_PERMANENT_SHOW_AFTER_CONFIG)
  LCDclear();
#endif
#endif
#ifdef LOG_PERMANENT_SHOW_AFTER_CONFIG
  if (!f.ARMED) dumpPLog(0);
#endif
}
#endif // LCD_CONF
// -------------------- telemetry output to LCD over serial/i2c ----------------------------------

#ifdef LCD_TELEMETRY

void bar(uint8_t x,uint8_t y,uint8_t s,float n, float v)
{
  tft.drawFrame(x,y,s,10);
  tft.drawBox(x,y,s*v/n,10);
}

void LCDbar(uint8_t n, uint8_t v) {
  
  if (v > 200) v = 0;
  else if (v > 100) v = 100;
#if defined(LCD_SERIAL3W)
  for (uint8_t i = 0; i < n; i++) LCDprint((i < n * v / 100 ? '=' : '.'));
#elif defined(LCD_TEXTSTAR)
  LCDprint(0xFE); LCDprint('b'); LCDprint(n); LCDprint(v);
#elif defined(LCD_VT100) || defined(LCD_TTY) // || defined(OLED_DIGOLE)
  uint8_t i, j = (n * v) / 100;
  for (i = 0; i < j; i++) LCDprint( '=' );
  for (i = j; i < n; i++) LCDprint( '.' );
#elif defined(LCD_ETPP)
  ETPP_barGraph(n, v);
#elif defined(LCD_LCD03) || defined(LCD_LCD03S)
  for (uint8_t i = 0; i < n; i++) LCDprint((i < n * v / 100 ? '=' : '.'));
#elif defined(OLED_I2C_128x64)
  uint8_t i, j = (n * v) / 100;
  for (i = 0; i < j; i++) LCDprint( 159 );    // full
  if (j < n) LCDprint(154 + (v * n * 5 / 100 - 5 * j)); // partial fill
  for (i = j + 1; i < n; i++) LCDprint( 154 ); // empty
#elif defined(OLED_DIGOLE)
  uint8_t i, j = (n * v) / 100;
  char l[n + 1];
  for (i = 0; i < j; i++) l[i] = '=';
  for (i = j; i < n; i++) l[i] = '.';
  l[n] = 0;
  LCDprintChar(l);
#elif defined(ST7735)
  tft.drawFrame(50,50,n,10);
  tft.drawBox(50,50,v/n,10);
#endif
}

void fill_line1_deg() {
  uint16_t unit;
  strcpy_P(line1, PSTR("Deg ---.-  ---.-"));
  // 0123456789.12345
  if (att.angle[0] < 0 ) {
    unit = -att.angle[0];
    line1[3] = '-';
  } else
    unit = att.angle[0];
  line1[4] = digit1000(unit);
  line1[5] = digit100(unit);
  line1[6] = digit10(unit);
  line1[8] = digit1(unit);
  if (att.angle[1] < 0 ) {
    unit = -att.angle[1];
    line1[10] = '-';
  } else
    unit = att.angle[1];
  line1[11] = digit1000(unit);
  line1[12] = digit100(unit);
  line1[13] = digit10(unit);
  line1[15] = digit1(unit);
}
void output_AmaxA() {
#ifdef POWERMETER_HARD
  //uint16_t unit;
  strcpy_P(line2, PSTR("---,-A max---,-A"));
  //unit = analog.amperage; //((uint32_t)powerValue * conf.pint2ma) / 100;
  line2[0] = digit1000(analog.amperage);
  line2[1] = digit100(analog.amperage);
  line2[2] = digit10(analog.amperage);
  line2[4] = digit1(analog.amperage);
  //if (analog.amperage > powerValueMaxMAH) powerValueMaxMAH = analog.amperage;
  line2[10] = digit1000(powerValueMaxMAH);
  line2[11] = digit100(powerValueMaxMAH);
  line2[12] = digit10(powerValueMaxMAH);
  line2[14] = digit1(powerValueMaxMAH);
  LCDprintChar(line2);
#endif
}
#ifdef WATTS
void output_WmaxW() {
  //                   0123456789.12345
  strcpy_P(line2, PSTR("----W   max----W"));
  line2[0] = digit1000(analog.watts);
  line2[1] = digit100(analog.watts);
  line2[2] = digit10(analog.watts);
  line2[3] = digit1(analog.watts);
  line2[11] = digit1000(wattsMax);
  line2[12] = digit100(wattsMax);
  line2[13] = digit10(wattsMax);
  line2[14] = digit1(wattsMax);
  LCDprintChar(line2);
}
#endif

void output_V() {
#ifdef VBAT
  strcpy_P(line1, PSTR(" --.-V"));
  //                   0123456789.12345
  line1[1] = digit100(analog.vbat);
  line1[2] = digit10(analog.vbat);
  line1[4] = digit1(analog.vbat);
#ifndef OLED_I2C_128x64
  if (analog.vbat < conf.vbatlevel_warn1) {
    LCDattributesReverse();
  }
#endif
  LCDbar(DISPLAY_COLUMNS - 9, (((analog.vbat - conf.vbatlevel_warn1) * 100) / (VBATNOMINAL - conf.vbatlevel_warn1)) );
  LCDattributesOff(); // turn Reverse off for rest of display
  LCDprintChar(line1);
#endif
}

void output_Vmin() {
#ifdef VBAT
  strcpy_P(line1, PSTR(" --.-Vmin"));
  //                   0123456789.12345
  line1[1] = digit100(vbatMin);
  line1[2] = digit10(vbatMin);
  line1[4] = digit1(vbatMin);
#ifndef OLED_I2C_128x64
  if (vbatMin < conf.vbatlevel_crit) {
    LCDattributesReverse();
  }
#endif
  LCDbar(DISPLAY_COLUMNS - 9, (vbatMin > conf.vbatlevel_crit ? (((vbatMin - conf.vbatlevel_crit) * 100) / (VBATNOMINAL - conf.vbatlevel_crit)) : 0 ));
  LCDattributesOff();
  LCDprintChar(line1);
#endif
}
void output_mAh() {
#ifdef POWERMETER
  uint16_t mah = analog.intPowerMeterSum; // fallback: display consumed mAh
  if (analog.intPowerMeterSum < (uint16_t)conf.powerTrigger1 * PLEVELSCALE)
    mah = conf.powerTrigger1 * PLEVELSCALE - analog.intPowerMeterSum; // display mah mAh
  strcpy_P(line1, PSTR(" -----mAh"));
  line1[1] = digit10000(mah);
  line1[2] = digit1000(mah);
  line1[3] = digit100(mah);
  line1[4] = digit10(mah);
  line1[5] = digit1(mah);
  if (conf.powerTrigger1) {
    int8_t v = 100 - ( analog.intPowerMeterSum / (uint16_t)conf.powerTrigger1) * 2; // bar graph powermeter (scale intPowerMeterSum/powerTrigger1 with *100/PLEVELSCALE)
#ifndef OLED_I2C_128x64
    if (v <= 0) {
      LCDattributesReverse();  // buzzer on? then add some blink for attention
    }
#endif
    LCDbar(DISPLAY_COLUMNS - 9, v);
    LCDattributesOff();
  }
  LCDprintChar(line1);
#endif
}
void output_errors_or_armedTime() {
  if (failsafeEvents || (i2c_errors_count >> 10)) { // errors
    // ignore i2c==1 because of bma020-init
    LCDalarmAndReverse();
    output_fails();
    LCDattributesOff();
  } else { // ... armed time
    uint16_t ats = armedTime / 1000000;
#ifdef ARMEDTIMEWARNING
#ifndef OLED_I2C_128x64
    if (ats > conf.armedtimewarning) {
      LCDattributesReverse();
    }
#endif
    LCDbar(DISPLAY_COLUMNS - 9, (ats < conf.armedtimewarning ? (((conf.armedtimewarning - ats + 1) * 25) / (conf.armedtimewarning + 1) * 4) : 0 ));
    LCDattributesOff();
#endif
    LCDprint(' ');
#ifdef ARMEDTIMEWARNING
    print_uptime( (conf.armedtimewarning > ats ? conf.armedtimewarning - ats : ats) );
#else
    print_uptime(ats);
#endif
  }
}
void output_altitude() {
#if BARO
  {
    int16_t h = alt.EstAlt / 100;
    LCDprint('A'); LCDprintInt16(h, 0, 0); LCDprint('m');
    h = BAROaltMax / 100;
    LCDprintChar(" ("); LCDprintInt16(h, 0, 0);
  }
#endif
}
void output_uptime_cset() {
  strcpy_P(line1, PSTR("Up ")); LCDprintChar(line1); print_uptime(millis() / 1000 );
#ifdef MULTIPLE_CONFIGURATION_PROFILES
  strcpy_P(line1, PSTR("  Cset -")); line1[7] = digit1(global_conf.currentSet); LCDprintChar(line1);
#endif
}
void output_cycle() {
  strcpy_P(line1, PSTR("Cycle    -----us")); //uin16_t cycleTime
  // 0123456789.12345*/
  //strcpy_P(line2,PSTR("(-----, -----)us")); //uin16_t cycleTimeMax
  line1[9] = digit10000(cycleTime);
  line1[10] = digit1000(cycleTime);
  line1[11] = digit100(cycleTime);
  line1[12] = digit10(cycleTime);
  line1[13] = digit1(cycleTime);
  LCDprintChar(line1);
}
void output_cycleMinMax() {
#if (LOG_VALUES >= 2)
  strcpy_P(line2, PSTR("(-----, -----)us")); //uin16_t cycleTimeMax
  line2[1] = digit10000(cycleTimeMin );
  line2[2] = digit1000(cycleTimeMin );
  line2[3] = digit100(cycleTimeMin );
  line2[4] = digit10(cycleTimeMin );
  line2[5] = digit1(cycleTimeMin );
  line2[8] = digit10000(cycleTimeMax);
  line2[9] = digit1000(cycleTimeMax);
  line2[10] = digit100(cycleTimeMax);
  line2[11] = digit10(cycleTimeMax);
  line2[12] = digit1(cycleTimeMax);
  LCDprintChar(line2);
#endif
}
void output_fails() {
  uint16_t unit;
  //                   0123456789012345
  strcpy_P(line2, PSTR("-- Fails  -- i2c"));
  unit = failsafeEvents;
  line2[0] = digit10(unit);
  line2[1] = digit1(unit);
  unit = i2c_errors_count;
  line2[10] = digit10(unit);
  line2[11] = digit1(unit);
  LCDprintChar(line2);
}
void output_annex() {
  //                   0123456789
  strcpy_P(line2, PSTR("annex --"));
  line2[6] = digit10(annex650_overrun_count);
  line2[7] = digit1(annex650_overrun_count);
  LCDprintChar(line2);
}
static char checkboxitemNames[][4] = {
  "Arm",
#if ACC
  "Ang", "Hor",
#endif
#if BARO && (!defined(SUPPRESS_BARO_ALTHOLD))
  "Bar",
#endif
#ifdef VARIOMETER
  "Var",
#endif
  "Mag",
#if defined(HEADFREE)
  "HFr",
  "HAd",
#endif
#if defined(SERVO_TILT) || defined(GIMBAL)|| defined(SERVO_MIX_TILT)
  "CSt",
#endif
#if defined(CAMTRIG)
  "CTr",
#endif
#if GPS
  "GHm",
  "GHd",
#endif
#if defined(FIXEDWING) || defined(HELICOPTER)
  "Pas",
#endif
#if defined(BUZZER)
  "Buz",
#endif
#if defined(LED_FLASHER)
  "LEM",
  "LEL",
#endif
#if defined(LANDING_LIGHTS_DDR)
  "LLs",
#endif
#ifdef INFLIGHT_ACC_CALIBRATION
  "Cal",
#endif
#ifdef GOVERNOR_P
  "Gov",
#endif
#ifdef OSD_SWITCH
  "OSD",
#endif
  ""
};
void output_checkboxitems() {
  for (uint8_t i = 0; i < CHECKBOXITEMS; i++ ) {
    if (rcOptions[i] || ((i == BOXARM) && (f.ARMED)) ) {
      LCDprintChar(checkboxitemNames[i]);
      LCDprint(' ');
    }
  }
}
void output_checkboxstatus() {
#if (defined(DISPLAY_COLUMNS))
  uint8_t cntmax = DISPLAY_COLUMNS / 4;
#else
  uint8_t cntmax = 4;
#endif
  uint8_t cnt = 0;
#ifdef BUZZER
  if (isBuzzerON()) {
    LCDalarmAndReverse();  // buzzer on? then add some blink for attention
  }
#endif
  for (uint8_t i = 0; (i < CHECKBOXITEMS) && (cnt < cntmax); i++ ) {
    if (rcOptions[i] || ((i == BOXARM) && (f.ARMED)) ) {
      LCDprintChar(checkboxitemNames[i]);
      LCDprint(' ');
      cnt++;
    }
  }
  for (uint8_t i = cnt; i < cntmax; i++) LCDprintChar(".   "); // padding to EOL
  LCDattributesOff();
}

#define GYROLIMIT 60 // threshold: for larger values replace bar with dots
#define ACCLIMIT 60 // threshold: for larger values replace bar with dots
void outputSensor(uint8_t num, int16_t data, int16_t limit) {
  if (data < -limit)     {
    LCDprintChar("<<<<");
  }
  else if (data > limit) {
    LCDprintChar(">>>>");
  }
  else LCDbar(num, limit + data * 50 / limit);
}
void print_uptime(uint16_t sec) {
  uint16_t m, s;
  static char line[6] = "--:--";
  m = sec / 60;
  s = sec - (60 * m);
  line[0] = digit10(m);
  line[1] = digit1(m);
  line[3] = digit10(s);
  line[4] = digit1(s);
  LCDprintChar(line);
}
#if GPS
void fill_line1_gps_lat(uint8_t sat) {
  int32_t aGPS_latitude = abs(GPS_coord[LAT]);
  strcpy_P(line1, PSTR(".---.------- #--"));
  //                   0123456789012345
  line1[0] = GPS_coord[LAT] < 0 ? 'S' : 'N';
  if (sat) {
    //line1[13] = '#';
    line1[14] = digit10(GPS_numSat);
    line1[15] = digit1(GPS_numSat);
  } //                                987654321
  line1[1]  = '0' + aGPS_latitude  / 1000000000;
  line1[2]  = '0' + aGPS_latitude  / 100000000 - (aGPS_latitude / 1000000000) * 10;
  line1[3]  = '0' + aGPS_latitude  / 10000000  - (aGPS_latitude / 100000000)  * 10;
  line1[5]  = '0' + aGPS_latitude  / 1000000   - (aGPS_latitude / 10000000)   * 10;
  line1[6]  = '0' + aGPS_latitude  / 100000    - (aGPS_latitude / 1000000)    * 10;
  line1[7]  = '0' + aGPS_latitude  / 10000     - (aGPS_latitude / 100000)     * 10;
  line1[8]  = '0' + aGPS_latitude  / 1000      - (aGPS_latitude / 10000)      * 10;
  line1[9]  = '0' + aGPS_latitude  / 100       - (aGPS_latitude / 1000)       * 10;
  line1[10] = '0' + aGPS_latitude  / 10        - (aGPS_latitude / 100)        * 10;
  line1[11] = '0' + aGPS_latitude              - (aGPS_latitude / 10)         * 10;
}
void fill_line2_gps_lon(uint8_t status) {
  int32_t aGPS_longitude = abs(GPS_coord[LON]);
  strcpy_P(line2, PSTR(".---.-------    "));
  //                   0123456789012345
  line2[0] = GPS_coord[LON] < 0 ? 'W' : 'E';
  if (status) {
    line2[13] = (GPS_update ? 'U' : '.');
    //line2[15] = (1 ? 'P' : '.');
  }
  line2[1]  = '0' + aGPS_longitude / 1000000000;
  line2[2]  = '0' + aGPS_longitude / 100000000 - (aGPS_longitude / 1000000000) * 10;
  line2[3]  = '0' + aGPS_longitude / 10000000  - (aGPS_longitude / 100000000)  * 10;
  line2[5]  = '0' + aGPS_longitude / 1000000   - (aGPS_longitude / 10000000)   * 10;
  line2[6]  = '0' + aGPS_longitude / 100000    - (aGPS_longitude / 1000000)    * 10;
  line2[7]  = '0' + aGPS_longitude / 10000     - (aGPS_longitude / 100000)     * 10;
  line2[8]  = '0' + aGPS_longitude / 1000      - (aGPS_longitude / 10000)      * 10;
  line2[9]  = '0' + aGPS_longitude / 100       - (aGPS_longitude / 1000)       * 10;
  line2[10] = '0' + aGPS_longitude / 10        - (aGPS_longitude / 100)        * 10;
  line2[11] = '0' + aGPS_longitude             - (aGPS_longitude / 10)         * 10;
}
#endif


//MATRIX FUNCTIONS----------------------------------------------------
void icon_CB(bool stat, uint8_t x, uint8_t y, uint8_t s);

void drawBitmap(int16_t x, int16_t y, const uint8_t bitmap[], int16_t w, int16_t h) {
  int16_t byteWidth = (w + 7) / 8; // Bitmap scanline pad = whole byte
  uint8_t byte = 0;
  for (int16_t j = 0; j < h; j++, y++) {
    for (int16_t i = 0; i < w; i++) {
      if (i & 7)
        byte <<= 1;
      else
        byte   = bitmap[j * byteWidth + i / 8];
      if (byte & 0x80)
        tft.drawPixel(x + i, y);
    }
  }
}

void screen_FRAME(char *label, uint8_t w, uint8_t h)
{
  const uint8_t PROGMEM copter_bits[] = {/*8x8*/
    0x63, 0x63, 0x14, 0x08, 0x14, 0x63, 0x63, 0x00
  };

  tft.setColor(255, 255, 0);
  tft.drawFrame(0, 0, w , h);
  tft.drawHLine(0, 14, w);
  tft.setFont(ucg_font_8x13_mf);
  tft.setPrintPos(2, 12);
  tft.print(label);
  tft.setColor(0, 255, 0);
  drawBitmap(w - 12, 3, copter_bits, 8, 8);
}

void screen_S(void)
{
  screen_FRAME("Sensors", 128, 96);
  tft.setFont(ucg_font_7x13_mf);
      static char sensorNames[6][3] = {"Gx", "Gy", "Gz", "Ax", "Ay", "Az"};
      for (uint8_t n = 0; n < 6; n++)
      {
        tft.setPrintPos(5, 27 + n * 13);
        tft.print(sensorNames[n]);
        tft.setPrintPos(30, 27 + n * 13);
        LCDprintInt16(imu.gyroData[n], 0, 0);
      }



  
}


void screen_CB(void)
{
  screen_FRAME("Checkboxitems", 128, 96);
  char buff[18];
  int32_t n;
  tft.setFont(ucg_font_7x13_mf);
  for (uint8_t n = 0; n < 6; n++)
  {
    tft.setColor(0, 255, 0);
    tft.setPrintPos(5, 27 + n * 13);
    tft.print(checkboxitemNames[n]);
    icon_CB(rcOptions[n], 114, 17 + n * 13, 12);
  }
}

void screen_RX(void)
{
  const uint8_t cnt = 8;
  screen_FRAME("Rx Inputs", 128, 31 + cnt * 13);
  static char channelNames[cnt][4] = {"Ail", "Ele", "Yaw", "Thr", "Ax1", "Ax2", "Ax3", "Ax4"};
  uint16_t unit;
  for (uint8_t n = 0; n < cnt; n++)
  {
    tft.setPrintPos(5, 27 + n * 13);
  //  i = linenr++ % 8; // 8 channels
    //strcpy_P(line1,PSTR("-Thr ---- "));
    //                   0123456789.12345
    template3[0] = ( '0' + n + 1); // channel numbering [1;8]
    tft.print(template3);
    tft.print(channelNames[n]);
    unit = rcData[n];
    template7[1] = digit1000(unit);
    template7[2] = digit100(unit);
    template7[3] = digit10(unit);
    template7[4] = digit1(unit);
    tft.print(template7);
  //  unit = constrain(rcData[n], 1000, 2000);
 //   bar(10,17 + n * 13,25,1500, unit);
  }
}

void screen_GPS(void)
{
  screen_FRAME("GPS", 128, 70);
  // tft.setFont(ucg_font_6x10_tr);
  char buff[18];
  int32_t n;
  tft.setFont(ucg_font_7x13_mf);
  tft.setColor(0, 255, 0);
  n = abs(GPS_coord[LAT]);
  strcpy_P(buff, PSTR("Lat. .---.-------"));
  buff[5]  = GPS_coord[LAT] < 0 ? 'S' : 'N';
  buff[6]  = '0' + n  / 1000000000;
  buff[7]  = '0' + n  / 100000000 - (n / 1000000000) * 10;
  buff[8]  = '0' + n  / 10000000  - (n / 100000000)  * 10;
  buff[10] = '0' + n  / 1000000   - (n / 10000000)   * 10;
  buff[11] = '0' + n  / 100000    - (n / 1000000)    * 10;
  buff[12] = '0' + n  / 10000     - (n / 100000)     * 10;
  buff[13] = '0' + n  / 1000      - (n / 10000)      * 10;
  buff[14] = '0' + n  / 100       - (n / 1000)       * 10;
  buff[15] = '0' + n  / 10        - (n / 100)        * 10;
  buff[16] = '0' + n              - (n / 10)         * 10;
  tft.setPrintPos(5, 27);
  tft.print(buff);
  n = abs(GPS_coord[LON]);
  strcpy_P(buff, PSTR("Lon. .---.-------"));
  buff[5]  = GPS_coord[LON] < 0 ? 'W' : 'E';
  buff[6]  = '0' + n  / 1000000000;
  buff[7]  = '0' + n  / 100000000 - (n / 1000000000) * 10;
  buff[8]  = '0' + n  / 10000000  - (n / 100000000)  * 10;
  buff[10] = '0' + n  / 1000000   - (n / 10000000)   * 10;
  buff[11] = '0' + n  / 100000    - (n / 1000000)    * 10;
  buff[12] = '0' + n  / 10000     - (n / 100000)     * 10;
  buff[13] = '0' + n  / 1000      - (n / 10000)      * 10;
  buff[14] = '0' + n  / 100       - (n / 1000)       * 10;
  buff[15] = '0' + n  / 10        - (n / 100)        * 10;
  buff[16] = '0' + n              - (n / 10)         * 10;
  tft.setPrintPos(5, 40);
  tft.print(buff);
  n = GPS_altitude;
  strcpy_P(buff, PSTR("Alt.        ----m"));
  buff[12] = '0' + n  / 1000      - (n / 10000)      * 10;
  buff[13] = '0' + n  / 100       - (n / 1000)       * 10;
  buff[14] = '0' + n  / 10        - (n / 100)        * 10;
  buff[15] = '0' + n              - (n / 10)         * 10;
  tft.setPrintPos(5, 53);
  tft.print(buff);
  n = GPS_speed * 0.036f;
  strcpy_P(buff, PSTR("--km/h max --km/h"));
  buff[0] = '0' + n  / 10        - (n / 100)        * 10;
  buff[1] = '0' + n              - (n / 10)         * 10;
  n = (GPS_speedMax * 0.036f);
  buff[11] = '0' + n  / 10        - (n / 100)        * 10;
  buff[12] = '0' + n              - (n / 10)         * 10;
  tft.setPrintPos(5, 66);
  tft.print(buff);
}
void icon_CB(bool stat, uint8_t x, uint8_t y, uint8_t s) {
  tft.drawFrame(x, y, s, s);
  if (stat)
    tft.drawBox(x + 2, y + 2, s - 4, s - 4);
}


void icon_GPS(void) {
#ifdef GPS
  const uint8_t PROGMEM gps_bits[] = {/* 8x12 */
    0x1c, 0x3e, 0x77, 0x63, 0x77, 0x3e, 0x3e, 0x1c, 0x1c, 0x08, 0x08, 0x00
  };
  char buff[3];
  uint8_t n = GPS_numSat;

  if (n > 0)
  {
    strcpy_P(buff, PSTR("#--"));
    buff[1] = '0' + n  / 10        - (n / 100)        * 10;
    buff[2] = '0' + n              - (n / 10)         * 10;
    tft.setColor(0, 0, 0);
    tft.drawBox(106, 145, 28, 13);
    tft.setColor(255, 255, 0);
    tft.setFont(ucg_font_orgv01_tr);
    tft.setPrintPos(106, 155);
    tft.print(buff);
  }
  else
    tft.setColor(255, 0, 0);
  drawBitmap(96, 147, gps_bits, 8, 12);
#else
  const uint8_t PROGMEM nogps_bits[] = {
    0x1c, 0x22, 0x49, 0x49, 0x41, 0x2a, 0x22, 0x14, 0x14, 0x08, 0x08, 0x00
  };
  tft.setColor(255, 0, 0);
  drawBitmap(96, 147, nogps_bits, 8, 12);
#endif
}

void icon_POWER(void) {
#ifdef VBAT
  const uint8_t PROGMEM ok_bits[] = {/* 8x12 */
    0x1c, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x00
  };
  char buff[4];
  strcpy_P(buff, PSTR("--.-V"));
  /*                   01234*/
  buff[0] = '0' + analog.vbat  / 100       - (analog.vbat / 1000)       * 10;
  buff[1] = '0' + analog.vbat  / 10        - (analog.vbat / 100)        * 10;
  buff[3] = '0' + analog.vbat              - (analog.vbat / 10)         * 10;
  if (analog.vbat < conf.vbatlevel_warn1)
    tft.setColor(255, 0, 0);
  else
    tft.setColor(255, 255, 0);
  drawBitmap(2, 147, ok_bits, 8, 12);
  //ucg_font_chikita_hf manr font
  //ucg_font_fixed_v0_hf manr harmar
  tft.setFont(ucg_font_orgv01_tr);
  tft.setPrintPos(13, 154);
  tft.print(buff);
#else
  const uint8_t PROGMEM ko_bits[] = {/* 8x12 */
    0x1c, 0x7f, 0x41, 0x49, 0x49, 0x49, 0x49, 0x41, 0x49, 0x41, 0x7f, 0x00
  };
  tft.setColor(255, 0, 0);
  drawBitmap(2, 147, ko_bits, 8, 12);
#endif
}


//--------------------------------------------------------------------
/*
  void output_gyroX() {
  LCDprintInt16(imu.gyroData[0]); LCDprint(' ');
  outputSensor(DISPLAY_COLUMNS - 10, imu.gyroData[0], GYROLIMIT);
  }
*/
void output_gyroX() {
  LCDprintInt16(imu.gyroData[1], 12, 35);
  tft.setFont(ucg_font_7x14B_mf);
}
void output_gyroY() {
  char buff[10];
  tft.setFont(ucg_font_7x14B_mf);
  sprintf(buff, "Gy %d", imu.gyroData[1]);
  tft.setPrintPos(12, 50);
  tft.print(buff);
}
void output_gyroZ() {
  char buff[10];
  tft.setFont(ucg_font_7x14B_mf);
  sprintf(buff, "Gz %d", imu.gyroData[2]);
  tft.setPrintPos(12, 65);
  tft.print(buff);
}
/*
  void output_gyroY() {
  LCDprintInt16(imu.gyroData[1]); LCDprint(' ');
  outputSensor(DISPLAY_COLUMNS - 10, imu.gyroData[1], GYROLIMIT);
  }
  void output_gyroZ() {
  LCDprintInt16(imu.gyroData[2]); LCDprint(' ');
  outputSensor(DISPLAY_COLUMNS - 10, imu.gyroData[2], GYROLIMIT);
  }
*/
void output_accX() {
  LCDprintInt16(imu.accSmooth[0], 0, 0); LCDprint(' ');
  outputSensor(DISPLAY_COLUMNS - 10, imu.accSmooth[0], ACCLIMIT);
}
void output_accY() {
  LCDprintInt16(imu.accSmooth[1], 0, 0); LCDprint(' ');
  outputSensor(DISPLAY_COLUMNS - 10, imu.accSmooth[1], ACCLIMIT);
}
void output_accZ() {
  LCDprintInt16(imu.accSmooth[2], 0, 0); LCDprint(' ');
  outputSensor(DISPLAY_COLUMNS - 10, imu.accSmooth[2] - ACC_1G, ACCLIMIT);
}

void output_debug0() {
  LCDprintChar("D1 ");
  LCDprintInt16(debug[0], 0, 0);
}
void output_debug1() {
  LCDprintChar("D2 ");
  LCDprintInt16(debug[1], 0, 0);
}
void output_debug2() {
  LCDprintChar("D3 ");
  LCDprintInt16(debug[2], 0, 0);
}
void output_debug3() {
  LCDprintChar("D4 ");
  LCDprintInt16(debug[3], 0, 0);
}


#if defined(DEBUG) || defined(DEBUG_FREE)
#define PRINT_FREE_RAM  { \
    extern unsigned int __bss_end; \
    /*extern unsigned int __heap_start;*/ \
    extern void *__brkval; \
    int free_memory; \
    if((int)__brkval == 0) \
      free_memory = ((int)&free_memory) - ((int)&__bss_end); \
    else \
      free_memory = ((int)&free_memory) - ((int)__brkval); \
    strcpy_P(line1,PSTR(" Free ----")); \
    line1[6] = digit1000( free_memory ); \
    line1[7] = digit100( free_memory ); \
    line1[8] = digit10( free_memory ); \
    line1[9] = digit1( free_memory ); \
    LCDsetLine(1); LCDprintChar(line1); \
    telemetry = 0; \
  }
#define PRINT_FREE_RAM_v2  { \
    const uint8_t *ptr = &_end; \
    uint16_t       free_memory = 0; \
    while(*ptr == 0xa5 && ptr <= &__stack) \
    {   ptr++; free_memory++; } \
    strcpy_P(line1,PSTR(" Free ----")); \
    line1[6] = digit1000( free_memory ); \
    line1[7] = digit100( free_memory ); \
    line1[8] = digit10( free_memory ); \
    line1[9] = digit1( free_memory ); \
    LCDsetLine(1); LCDprintChar(line1); \
    telemetry = 0; \
  }
#endif

/* ------------ DISPLAY_2LINES ------------------------------------*/
/*
  #ifdef DISPLAY_2LINES
  void lcd_telemetry() {
  static uint8_t linenr = 0;
  switch (telemetry) { // output telemetry data
      uint16_t unit;
      uint8_t i;
  #ifndef SUPPRESS_TELEMETRY_PAGE_1
    case 1: // button A on Textstar LCD -> angles
    case '1':
      if (linenr++ % 2) {
        fill_line1_deg();
        LCDsetLine(1);
        LCDprintChar(line1);
      } else {
  #ifdef POWERMETER_HARD
        LCDsetLine(2);
        output_AmaxA();
  #endif
      }
      break;
  #endif
  #ifndef SUPPRESS_TELEMETRY_PAGE_2
    case 2: // button B on Textstar LCD -> Voltage, PowerSum and power alarm trigger value
    case '2':
      if (linenr++ % 2) {
        LCDsetLine(1);
        output_V();
      } else {
        LCDsetLine(2);
        output_mAh();
      }
      break;
  #endif
  #ifndef SUPPRESS_TELEMETRY_PAGE_3
    case 3: // button C on Textstar LCD -> cycle time
    case '3':
      if (linenr++ % 2) {
        LCDsetLine(1);
        output_cycle();
      } else {
  #if (LOG_VALUES >= 2)
        LCDsetLine(2);
        output_cycleMinMax());
  #endif
      }
      break;
  #endif
  #ifndef SUPPRESS_TELEMETRY_PAGE_4
    case 4: // button D on Textstar LCD -> sensors
    case '4':
      if (linenr++ % 2) {
        LCDsetLine(1); LCDprintChar("G "); //refresh line 1 of LCD
        outputSensor(4, imu.gyroData[0], GYROLIMIT); LCDprint(' ');
        outputSensor(4, imu.gyroData[1], GYROLIMIT); LCDprint(' ');
        outputSensor(4, imu.gyroData[2], GYROLIMIT);
      } else {
        LCDsetLine(2); LCDprintChar("A "); //refresh line 2 of LCD
        outputSensor(4, imu.accSmooth[0], ACCLIMIT); LCDprint(' ');
        outputSensor(4, imu.accSmooth[1], ACCLIMIT); LCDprint(' ');
        outputSensor(4, imu.accSmooth[2] - ACC_1G, ACCLIMIT);
      }
      break;
  #endif
  #ifndef SUPPRESS_TELEMETRY_PAGE_5
    case 5:
    case '5':
      if (linenr++ % 2) {
        LCDsetLine(1);
        output_fails();
      } else {
        LCDsetLine(2);
        output_annex();
      }
      break;
  #endif
  #ifndef SUPPRESS_TELEMETRY_PAGE_6
    case 6: // RX inputs
    case '6':
      if (linenr++ % 2) {
        strcpy_P(line1, PSTR("Roll Pitch Throt"));
        if (f.ARMED) line2[14] = 'A'; else line2[14] = 'a';
        if (failsafeCnt > 5) line2[15] = 'F'; else line2[15] = 'f';
        LCDsetLine(1); LCDprintChar(line1);
      } else {
        // 0123456789012345
        strcpy_P(line2, PSTR("---- ---- ----xx"));
        line2[0] = digit1000( rcData[ROLL] );
        line2[1] = digit100( rcData[ROLL] );
        line2[2] = digit10( rcData[ROLL] );
        line2[3] = digit1( rcData[ROLL] );
        line2[5] = digit1000( rcData[PITCH] );
        line2[6] = digit100( rcData[PITCH] );
        line2[7] = digit10( rcData[PITCH] );
        line2[8] = digit1( rcData[PITCH] );
        line2[10] = digit1000( rcData[THROTTLE] );
        line2[11] = digit100( rcData[THROTTLE] );
        line2[12] = digit10( rcData[THROTTLE] );
        line2[13] = digit1( rcData[THROTTLE] );
        LCDsetLine(2); LCDprintChar(line2);
      }
      break;
  #endif
  #ifndef SUPPRESS_TELEMETRY_PAGE_7
    case 7:
    case '7':
  #if GPS
      if (linenr++ % 2) {
        fill_line1_gps_lat(1); // including #sat
        LCDsetLine(1); LCDprintChar(line1);

      } else {
        fill_line2_gps_lon(1); // including status info
        LCDsetLine(2); LCDprintChar(line2);
      }
  #endif // case 7 : GPS
      break;
  #endif
  #ifndef SUPPRESS_TELEMETRY_PAGE_9
    case 9:
    case '9':
      LCDsetLine(1);
      LCDprintInt16(debug[0]); LCDprint(' ');
      LCDprintInt16(debug[1]); LCDprint(' ');
      LCDprintInt16(debug[2]); LCDprint(' ');
      LCDprintInt16(debug[3]); LCDprint(' ');
      break;
  #endif

  #if defined(LOG_VALUES) && defined(DEBUG)
    case 'R':
      //Reset logvalues
      cycleTimeMax = 0;// reset min/max on transition on->off
      cycleTimeMin = 65535;
      telemetry = 0;// no use to repeat this forever
      break;
  #endif // case R
  #if defined(DEBUG) || defined(DEBUG_FREE)
    case 'F':
      PRINT_FREE_RAM;
      break;
  #endif // DEBUG
      // WARNING: if you add another case here, you should also add a case: in Serial.pde, so users can access your case via terminal input
  } // end switch (telemetry)
  } // end function lcd_telemetry
  #endif // DISPLAY_2LINES
  /* ------------ DISPLAY_MULTILINE ------------------------------------*/
/*
  #ifdef DISPLAY_MULTILINE
  #ifndef SUPPRESS_TELEMETRY_PAGE_5
  void outputMotorServo(uint8_t i, uint16_t unit) {
  #ifdef HELICOPTER
  static char outputNames[16][3] = {"M1", " 2", " 3", " 4", " 5", " 6", " 7", " 8",
                                    "S1", "S2", "S3", "SN", "SL", "ST", "SR", "SM",
                                   };
  #else
  static char outputNames[16][3] = {"M1", " 2", " 3", " 4", " 5", " 6", " 7", " 8",
                                    "S1", "S2", "S3", "S4", "S5", "S6", "S7", "S8",
                                   };
  #endif
  LCDprintChar(outputNames[i]);
  template7[1] = digit1000(unit);
  template7[2] = digit100(unit);
  template7[3] = digit10(unit);
  template7[4] = digit1(unit);
  LCDprintChar(template7);
  unit = constrain(unit, 1000, 2000);
  LCDbar(DISPLAY_COLUMNS - 8, (unit - 1000) / 10 );
  LCDcrlf();
  }
  #endif
  #ifdef LCD_TELEMETRY_PAGE1
  void (*page1_func_ptr[]) () = LCD_TELEMETRY_PAGE1 ;
  #else
  void (*page1_func_ptr[]) () = {
  #ifdef VBAT
  output_V, //1
  #endif
  #ifdef POWERMETER
  output_mAh,
  #endif
  #ifdef VBAT
  output_Vmin,
  #endif
  output_errors_or_armedTime,
  output_checkboxstatus,
  #if BARO
  output_altitude,
  #endif
  #ifdef POWERMETER_HARD
  output_AmaxA,
  #endif
  #ifdef WATTS
  output_WmaxW,
  #endif
  output_uptime_cset,
  };
  #endif
  #ifdef LCD_TELEMETRY_PAGE9
  void (*page9_func_ptr[]) () = LCD_TELEMETRY_PAGE9 ;
  #else
  void (*page9_func_ptr[]) () = {
  output_cycle,
  #if (LOG_VALUES >= 2)
  output_cycleMinMax,
  #endif
  output_fails,
  output_annex,
  #ifdef DEBUG
  output_debug0, output_debug1, output_debug2, output_debug3,
  #endif
  };
  #endif
  #ifdef LCD_TELEMETRY_PAGE2
  void (*page2_func_ptr[]) () = LCD_TELEMETRY_PAGE2 ;
  #else
  void (*page2_func_ptr[]) () = { output_gyroX, output_gyroY, output_gyroZ, output_accX, output_accY, output_accZ, };
  #endif

  void lcd_telemetry() {
  static uint8_t linenr = 0;
  #ifdef DISPLAY_FONT_DSIZE
  uint8_t offset = 0;
  #define POSSIBLE_OFFSET offset
  #else
  #define POSSIBLE_OFFSET 0
  #endif
  switch (telemetry) { // output telemetry data
      uint16_t unit;
      uint8_t i;
    case '0': // request to turn telemetry off - workaround, cannot enter binary zero \000 into string
      telemetry = 0;
      break;
  #ifndef SUPPRESS_TELEMETRY_PAGE_1
  #ifdef DISPLAY_FONT_DSIZE
    case '!':
      {
        offset = MULTILINE_PRE + MULTILINE_POST;
      }
      // no break !!
  #endif
    case 1:// overall display
    case '1':
      {
        linenr++;
        linenr %= min(MULTILINE_PRE + MULTILINE_POST, (sizeof(page1_func_ptr) / 2) - POSSIBLE_OFFSET);
        LCDsetLine(linenr + 1);
        (*page1_func_ptr [linenr + POSSIBLE_OFFSET] ) (); // not really linenumbers
        LCDcrlf();
        break;
      }
  #endif
  #ifndef SUPPRESS_TELEMETRY_PAGE_2
  #ifdef DISPLAY_FONT_DSIZE
    case '@':
      {
        offset = 3;
      }
      // no break !!
  #endif
    case 2: // sensor readings
    case '2':
      static char sensorNames[6][3] = {"Gx", "Gy", "Gz", "Ax", "Ay", "Az"};
      i = linenr++ % min(MULTILINE_PRE + MULTILINE_POST, 6 - POSSIBLE_OFFSET);
      LCDsetLine(i + 1);
      LCDprintChar(sensorNames[i + POSSIBLE_OFFSET]);
      LCDprint(' ');
      (*page2_func_ptr [i + POSSIBLE_OFFSET] ) (); // not really linenumbers
      LCDcrlf();
      break;
  #endif
  #ifndef SUPPRESS_TELEMETRY_PAGE_3
  #ifdef DISPLAY_FONT_DSIZE
    case '#':
      {
        offset = MULTILINE_PRE + MULTILINE_POST;
      }
      // no break !!
  #endif
    case 3: // checkboxes and modes
    case '3':
      {
        i = linenr++ % min(MULTILINE_PRE + MULTILINE_POST, CHECKBOXITEMS - POSSIBLE_OFFSET);
        LCDsetLine(i + 1);
        LCDprintChar(checkboxitemNames[i + POSSIBLE_OFFSET]);
        //LCDprintChar((PGM_P)(boxnames[index]));
        LCDprint(' ');
        LCDprint( rcOptions[i + POSSIBLE_OFFSET] ? 'X' : '.');
        LCDcrlf();
        break;
      }
  #endif
  #ifndef SUPPRESS_TELEMETRY_PAGE_4
  #ifdef DISPLAY_FONT_DSIZE
    case '$':
      {
        offset = 4;
      }
      // no break !!
  #endif
    case 4: // RX inputs
    case '4':
      {
        static char channelNames[8][4] = {"Ail", "Ele", "Yaw", "Thr", "Ax1", "Ax2", "Ax3", "Ax4"};
        i = linenr++ % 8; // 8 channels
        LCDsetLine((i - POSSIBLE_OFFSET) % 8 + 1);
        //strcpy_P(line1,PSTR("-Thr ---- "));
        //                   0123456789.12345
        template3[0] = ( '0' + i + 1); // channel numbering [1;8]
        LCDprintChar(template3);
        LCDprintChar(channelNames[i]);
        unit = rcData[i];
        template7[1] = digit1000(unit);
        template7[2] = digit100(unit);
        template7[3] = digit10(unit);
        template7[4] = digit1(unit);
        LCDprintChar(template7);
        unit = constrain(rcData[i], 1000, 2000);
        LCDbar(DISPLAY_COLUMNS - 11, (unit - 1000) / 10 );
        LCDcrlf();
        break;
      }
  #endif
  #ifndef SUPPRESS_TELEMETRY_PAGE_5
  #ifdef DISPLAY_FONT_DSIZE
    case '%':
      {
        offset = MULTILINE_PRE + MULTILINE_POST;
      }
      // no break !!
  #endif
    case 5: // outputs motors+servos
    case '5':
      {
        //      static char outputNames[16][3] = {"M1", " 2"," 3", " 4", " 5", " 6", " 7", " 8",
        //          "S1", "S2","S3", "S4", "S5", "S6", "S7", "S8",};
        static uint8_t index = 0;
        i = index++ % 16;
        if (i == 0) linenr = 0; //vt100 starts linenumbering @1
        LCDsetLine((linenr - POSSIBLE_OFFSET) % 8 + 1);
        if (i < 8) {
          if (i < NUMBER_MOTOR) {
            outputMotorServo(i, motor[i]);
            linenr++;
          } else {
            index = 8;
          }
        } else {
          uint8_t j = i - 7; // [8;15] -> [1;8]
  #if defined(PRI_SERVO_FROM) && defined(SEC_SERVO_FROM)
          if ((PRI_SERVO_FROM <= j && PRI_SERVO_TO >= j) || (SEC_SERVO_FROM <= j && SEC_SERVO_TO >= j))
  #elif defined(PRI_SERVO_FROM)
          if (j < PRI_SERVO_FROM) index = 7 + PRI_SERVO_FROM;
          else if (j > PRI_SERVO_TO) index = 16;
          else // (PRI_SERVO_FROM <= j && PRI_SERVO_TO >= j)
  #endif
  #if defined(PRI_SERVO_FROM) || defined(SEC_SERVO_FROM)
          {
            outputMotorServo(i, servo[j - 1]);
            linenr++;
            break;
          }
  #endif

        }
        break;
      }
  #endif // page 5

  #ifndef SUPPRESS_TELEMETRY_PAGE_6
  #if defined(VBAT_CELLS)
  #ifdef DISPLAY_FONT_DSIZE
    case '^':
      {
        offset = 4;
      }
      // no break !!
  #endif
    case 6: // alarms states
    case '6':
      {
        i = linenr++ % VBAT_CELLS_NUM; // VBAT_CELLS_NUM cells
        LCDsetLine((i - POSSIBLE_OFFSET) % VBAT_CELLS_NUM + 1);
        strcpy_P(line1, PSTR("_:-.-V __._V"));
        //                   0123456789.12345
        line1[0] = digit1(i + 1);
        uint16_t v = analog.vbatcells[i];
        if (i > 0) v = (analog.vbatcells[i] > analog.vbatcells[i - 1] ? analog.vbatcells[i] - analog.vbatcells[i - 1] : 0);
        line1[2] = digit10(v);
        line1[4] =  digit1(v);
        line1[7] = digit100(analog.vbatcells[i]);
        line1[8] =  digit10(analog.vbatcells[i]);
        line1[10] =   digit1(analog.vbatcells[i]);
        //      #ifndef OLED_I2C_128x64
        //        if (analog.vbat < conf.vbatlevel_warn1) { LCDattributesReverse(); }
        //      #endif
        if (v > VBATNOMINAL / VBAT_CELLS_NUM) v = VBATNOMINAL / VBAT_CELLS_NUM;
        LCDbar(DISPLAY_COLUMNS - 12, (v * 100 * VBAT_CELLS_NUM) / VBATNOMINAL );
        //      LCDattributesOff(); // turn Reverse off for rest of display
        LCDprintChar(line1);
        LCDcrlf();
        break;
      }
  #endif // vbat.cells
  #endif // page 6

  #ifndef SUPPRESS_TELEMETRY_PAGE_7
  #if GPS
  #ifdef DISPLAY_FONT_DSIZE
    case '&':
      {
        offset = MULTILINE_PRE + MULTILINE_POST;
      }
      // no break !!
  #endif
    case 7: // GPS
    case '7':
      linenr++;
      linenr %= 6;
      LCDsetLine(linenr + 1);
      switch (linenr + POSSIBLE_OFFSET) {
        case 0: // lat
          fill_line1_gps_lat(0); // skip #sat
          LCDprintChar(line1);
          break;
        case 1: // lon
          fill_line2_gps_lon(0);
          LCDprintChar(line2);
          break;
        case 2: // # Sats
          strcpy_P(line1, PSTR("-- Sats"));
          //                   0123456789012345
          line1[0] = digit10(GPS_numSat);
          line1[1] = digit1(GPS_numSat);
          LCDprintChar(line1);
          break;
        //        case 3: //
        //          strcpy_P(line1,PSTR("Status "));
        //          //                   0123456789012345
        //          LCDprintChar(line1);
        //          if (1)
        //            LCDprintChar("OK");
        //          else {
        //            LCDattributesReverse();
        //            LCDprintChar("KO");
        //            LCDattributesOff();
        //          }
        //        break;
        case 4: // gps speed
          {
            uint8_t v = (GPS_speed * 0.036f);
            strcpy_P(line1, PSTR("--km/h max--km/h"));
            //                   0123456789012345
            line1[0] = digit10(v);
            line1[1] = digit1(v);
            v = (GPS_speedMax * 0.036f);
            line1[10] = digit10(v);
            line1[11] = digit1(v);
            LCDprintChar(line1);
            break;
          }
        case 5: // vbat
          output_V();
          break;
      }
      LCDcrlf();
      break;
  #endif // gps
  #endif // page 7

  #ifndef SUPPRESS_TELEMETRY_PAGE_8
  #ifdef DISPLAY_FONT_DSIZE
    case '*':
      {
        offset = 5;
      }
      // no break !!
  #endif
    case 8: // alarms states
    case '8':
      //   123456789.1234567890
      static char alarmsNames[][12] = {
        "0: toggle  ",
        "1: failsafe",
        "2: noGPS   ",
        "3: beeperOn",
        "4: pMeter  ",
        "5: runtime ",
        "6: vBat    ",
        "7: confirma",
        "8: Acc     ",
        "9: I2Cerror"
      };
      linenr++;
      linenr %= min(MULTILINE_PRE + MULTILINE_POST, 10 - POSSIBLE_OFFSET);
      LCDsetLine(linenr + 1);
      // [linenr + POSSIBLE_OFFSET]
      LCDprintChar( alarmsNames[linenr + POSSIBLE_OFFSET] );
      LCDprint(' ');
      LCDprint( digit1( alarmArray[linenr + POSSIBLE_OFFSET] ) );
      LCDcrlf();
      break;
  #endif // page 8

  #ifndef SUPPRESS_TELEMETRY_PAGE_9
  #ifdef DISPLAY_FONT_DSIZE
    case '(':
      {
        offset = 4;
      }
      // no break !!
  #endif
    case 9: // diagnostics
    case '9':
      linenr++;
      linenr %= min(MULTILINE_PRE + MULTILINE_POST, (sizeof(page9_func_ptr) / 2) - POSSIBLE_OFFSET);
      LCDsetLine(linenr + 1);
      (*page9_func_ptr [linenr + POSSIBLE_OFFSET] ) (); // not really linenumbers
      LCDcrlf();
      break;
  #endif // page 9
  #ifndef SUPPRESS_TELEMETRY_PAGE_R
    case 'R':
      {
        //Reset logvalues
  #if defined(LOG_VALUES) && (LOG_VALUES >= 2)
        cycleTimeMax = 0;
        cycleTimeMin = 65535;
  #endif
  #if BARO
  #if defined(LOG_VALUES)
        BAROaltMax = 0;
  #endif
  #endif
  #if defined(FAILSAFE)
        failsafeEvents = 0; // reset failsafe counter
  #endif
        i2c_errors_count = 0;
        f.OK_TO_ARM = 1; // allow arming again
        telemetry = 0; // no use to repeat this forever
        break;
      }
  #endif // case R
  #if defined(DEBUG) || defined(DEBUG_FREE)
    case 'F':
      PRINT_FREE_RAM;
      break;
  #endif // DEBUG
      // WARNING: if you add another case here, you should also add a case: in Serial.pde, so users can access your case via terminal input
  } // end switch (telemetry)
  } // end function lcd_telemetry

  #endif // DISPLAY_MULTILINE
  /* ------------ DISPLAY_MATRIX ------------------------------------*/
#ifdef DISPLAY_MATRIX
#ifndef SUPPRESS_TELEMETRY_PAGE_5
void outputMotorServo(uint8_t i, uint16_t unit) {
#ifdef HELICOPTER
  static char outputNames[16][3] = {"M1", " 2", " 3", " 4", " 5", " 6", " 7", " 8",
                                    "S1", "S2", "S3", "SN", "SL", "ST", "SR", "SM",
                                   };
#else
  static char outputNames[16][3] = {"M1", " 2", " 3", " 4", " 5", " 6", " 7", " 8",
                                    "S1", "S2", "S3", "S4", "S5", "S6", "S7", "S8",
                                   };
#endif
  LCDprintChar(outputNames[i]);
  template7[1] = digit1000(unit);
  template7[2] = digit100(unit);
  template7[3] = digit10(unit);
  template7[4] = digit1(unit);
  LCDprintChar(template7);
  unit = constrain(unit, 1000, 2000);
  LCDbar(DISPLAY_COLUMNS - 8, (unit - 1000) / 10 );
  LCDcrlf();
}
#endif
#ifdef LCD_TELEMETRY_PAGE1
void (*page1_func_ptr[]) () = LCD_TELEMETRY_PAGE1 ;
#else
void (*page1_func_ptr[]) () = {
#ifdef VBAT
  output_V, //1
#endif
#ifdef POWERMETER
  output_mAh,
#endif
#ifdef VBAT
  output_Vmin,
#endif
  output_errors_or_armedTime,
  output_checkboxstatus,
#if BARO
  output_altitude,
#endif
#ifdef POWERMETER_HARD
  output_AmaxA,
#endif
#ifdef WATTS
  output_WmaxW,
#endif
  output_uptime_cset,
};
#endif
#ifdef LCD_TELEMETRY_PAGE9
void (*page9_func_ptr[]) () = LCD_TELEMETRY_PAGE9 ;
#else
void (*page9_func_ptr[]) () = {
  output_cycle,
#if (LOG_VALUES >= 2)
  output_cycleMinMax,
#endif
  output_fails,
  output_annex,
#ifdef DEBUG
  output_debug0, output_debug1, output_debug2, output_debug3,
#endif
};
#endif
#ifdef LCD_TELEMETRY_PAGE2
void (*page2_func_ptr[]) () = LCD_TELEMETRY_PAGE2 ;
#else
void (*page2_func_ptr[]) () = { output_gyroX, output_gyroY, output_gyroZ, output_accX, output_accY, output_accZ, };
#endif

void lcd_telemetry() {
  static uint8_t linenr = 0;
#define POSSIBLE_OFFSET 0

  //###########################################################
  //              Frame & Icons
  //###########################################################

  icon_POWER();
  icon_GPS();
  //###########################################################
  switch (telemetry) { // output telemetry data
      uint16_t unit;
      uint8_t i;
    case '0': // request to turn telemetry off - workaround, cannot enter binary zero \000 into string
      telemetry = 0;
      break;
#ifndef SUPPRESS_TELEMETRY_PAGE_1
#ifdef DISPLAY_FONT_DSIZE
    case '!':
      {
        offset = MULTILINE_PRE + MULTILINE_POST;
      }
      // no break !!
#endif
    case 1:// overall display
    case '1':
      {
        linenr++;
        linenr %= min(MULTILINE_PRE + MULTILINE_POST, (sizeof(page1_func_ptr) / 2));
        LCDsetLine(linenr + 1);
        (*page1_func_ptr [linenr] ) (); // not really linenumbers
        LCDcrlf();
        break;
      }
#endif
#ifndef SUPPRESS_TELEMETRY_PAGE_2
    /*
      #ifdef DISPLAY_FONT_DSIZE
        case '@':
          {
            offset = 3;
          }
          // no break !!
      #endif
    */
    case 2: // sensor readings
    case '2':
screen_S();

      /*
                  static char sensorNames[6][3] = {"Gx", "Gy", "Gz", "Ax", "Ay", "Az"};
                  i = linenr++ % min(MULTILINE_PRE + MULTILINE_POST, 6 - POSSIBLE_OFFSET);
                  tft.setPrintPos(4, 35 + i * 15);
                  LCDprintChar(sensorNames[i + POSSIBLE_OFFSET]);
                   tft.print(' ');
                  (*page2_func_ptr [i + POSSIBLE_OFFSET] ) (); // not really linenumbers
                  LCDcrlf();
      */
      break;
#endif
#ifndef SUPPRESS_TELEMETRY_PAGE_3
#ifdef DISPLAY_FONT_DSIZE
    case '#':
      {
        offset = MULTILINE_PRE + MULTILINE_POST;
      }
      // no break !!
#endif
    case 3: // checkboxes and modes
    case '3':
      {
        screen_CB();

        /*
          i = linenr++ % min(MULTILINE_PRE + MULTILINE_POST, CHECKBOXITEMS - POSSIBLE_OFFSET);
          LCDsetLine(i + 1);
          LCDprintChar(checkboxitemNames[i + POSSIBLE_OFFSET]);
          //LCDprintChar((PGM_P)(boxnames[index]));
          LCDprint(' ');
          LCDprint( rcOptions[i + POSSIBLE_OFFSET] ? 'X' : '.');
          LCDcrlf();
        */
        break;
      }
#endif
#ifndef SUPPRESS_TELEMETRY_PAGE_4
#ifdef DISPLAY_FONT_DSIZE
    case '$':
      {
        offset = 4;
      }
      // no break !!
#endif
    case 4: // RX inputs
    case '4':
      {
        screen_RX();

        break;
      }
#endif
#ifndef SUPPRESS_TELEMETRY_PAGE_5
#ifdef DISPLAY_FONT_DSIZE
    case '%':
      {
        offset = MULTILINE_PRE + MULTILINE_POST;
      }
      // no break !!
#endif
    case 5: // outputs motors+servos
    case '5':
      {
        //      static char outputNames[16][3] = {"M1", " 2"," 3", " 4", " 5", " 6", " 7", " 8",
        //          "S1", "S2","S3", "S4", "S5", "S6", "S7", "S8",};
        static uint8_t index = 0;
        i = index++ % 16;
        if (i == 0) linenr = 0; //vt100 starts linenumbering @1
        LCDsetLine((linenr - POSSIBLE_OFFSET) % 8 + 1);
        if (i < 8) {
          if (i < NUMBER_MOTOR) {
            outputMotorServo(i, motor[i]);
            linenr++;
          } else {
            index = 8;
          }
        } else {
          uint8_t j = i - 7; // [8;15] -> [1;8]
#if defined(PRI_SERVO_FROM) && defined(SEC_SERVO_FROM)
          if ((PRI_SERVO_FROM <= j && PRI_SERVO_TO >= j) || (SEC_SERVO_FROM <= j && SEC_SERVO_TO >= j))
#elif defined(PRI_SERVO_FROM)
          if (j < PRI_SERVO_FROM) index = 7 + PRI_SERVO_FROM;
          else if (j > PRI_SERVO_TO) index = 16;
          else // (PRI_SERVO_FROM <= j && PRI_SERVO_TO >= j)
#endif
#if defined(PRI_SERVO_FROM) || defined(SEC_SERVO_FROM)
          {
            outputMotorServo(i, servo[j - 1]);
            linenr++;
            break;
          }
#endif

        }
        break;
      }
#endif // page 5

#ifndef SUPPRESS_TELEMETRY_PAGE_6
#if defined(VBAT_CELLS)
#ifdef DISPLAY_FONT_DSIZE
    case '^':
      {
        offset = 4;
      }
      // no break !!
#endif
    case 6: // alarms states
    case '6':
      {
        i = linenr++ % VBAT_CELLS_NUM; // VBAT_CELLS_NUM cells
        LCDsetLine((i - POSSIBLE_OFFSET) % VBAT_CELLS_NUM + 1);
        strcpy_P(line1, PSTR("_:-.-V __._V"));
        //                   0123456789.12345
        line1[0] = digit1(i + 1);
        uint16_t v = analog.vbatcells[i];
        if (i > 0) v = (analog.vbatcells[i] > analog.vbatcells[i - 1] ? analog.vbatcells[i] - analog.vbatcells[i - 1] : 0);
        line1[2] = digit10(v);
        line1[4] =  digit1(v);
        line1[7] = digit100(analog.vbatcells[i]);
        line1[8] =  digit10(analog.vbatcells[i]);
        line1[10] =   digit1(analog.vbatcells[i]);
        //      #ifndef OLED_I2C_128x64
        //        if (analog.vbat < conf.vbatlevel_warn1) { LCDattributesReverse(); }
        //      #endif
        if (v > VBATNOMINAL / VBAT_CELLS_NUM) v = VBATNOMINAL / VBAT_CELLS_NUM;
        LCDbar(DISPLAY_COLUMNS - 12, (v * 100 * VBAT_CELLS_NUM) / VBATNOMINAL );
        //      LCDattributesOff(); // turn Reverse off for rest of display
        LCDprintChar(line1);
        LCDcrlf();
        break;
      }
#endif // vbat.cells
#endif // page 6

#ifndef SUPPRESS_TELEMETRY_PAGE_7
#if GPS
#ifdef DISPLAY_FONT_DSIZE
    case '&':
      {
        offset = MULTILINE_PRE + MULTILINE_POST;
      }
      // no break !!
#endif
    case 7: // GPS
    case '7':

      screen_GPS();
      /*
        linenr++;
        linenr %= 6;
        LCDsetLine(linenr + 1);
        switch (linenr + POSSIBLE_OFFSET) {
          case 0: // lat
            fill_line1_gps_lat(0); // skip #sat
            LCDprintChar(line1);
            break;
          case 1: // lon
            fill_line2_gps_lon(0);
            LCDprintChar(line2);
            break;
          case 2: // # Sats
            strcpy_P(line1, PSTR("-- Sats"));
            //                   0123456789012345
            line1[0] = digit10(GPS_numSat);
            line1[1] = digit1(GPS_numSat);
            LCDprintChar(line1);
            break;
          //        case 3: //
          //          strcpy_P(line1,PSTR("Status "));
          //          //                   0123456789012345
          //          LCDprintChar(line1);
          //          if (1)
          //            LCDprintChar("OK");
          //          else {
          //            LCDattributesReverse();
          //            LCDprintChar("KO");
          //            LCDattributesOff();
          //          }
          //        break;
          case 4: // gps speed
            {
              uint8_t v = (GPS_speed * 0.036f);
              strcpy_P(line1, PSTR("--km/h max--km/h"));
              //                   0123456789012345
              line1[0] = digit10(v);
              line1[1] = digit1(v);
              v = (GPS_speedMax * 0.036f);
              line1[10] = digit10(v);
              line1[11] = digit1(v);
              LCDprintChar(line1);
              break;
            }
          case 5: // vbat
            output_V();
            break;
        }
        LCDcrlf();
      */
      break;
#endif // gps
#endif // page 7

#ifndef SUPPRESS_TELEMETRY_PAGE_8
#ifdef DISPLAY_FONT_DSIZE
    case '*':
      {
        offset = 5;
      }
      // no break !!
#endif
    case 8: // alarms states
    case '8':
      //   123456789.1234567890
      static char alarmsNames[][12] = {
        "0: toggle  ",
        "1: failsafe",
        "2: noGPS   ",
        "3: beeperOn",
        "4: pMeter  ",
        "5: runtime ",
        "6: vBat    ",
        "7: confirma",
        "8: Acc     ",
        "9: I2Cerror"
      };
      linenr++;
      linenr %= min(MULTILINE_PRE + MULTILINE_POST, 10 - POSSIBLE_OFFSET);
      LCDsetLine(linenr + 1);
      // [linenr + POSSIBLE_OFFSET]
      LCDprintChar( alarmsNames[linenr + POSSIBLE_OFFSET] );
      LCDprint(' ');
      LCDprint( digit1( alarmArray[linenr + POSSIBLE_OFFSET] ) );
      LCDcrlf();
      break;
#endif // page 8

#ifndef SUPPRESS_TELEMETRY_PAGE_9
#ifdef DISPLAY_FONT_DSIZE
    case '(':
      {
        offset = 4;
      }
      // no break !!
#endif
    case 9: // diagnostics
    case '9':
      /*
        linenr++;
        linenr %= min(MULTILINE_PRE + MULTILINE_POST, (sizeof(page9_func_ptr) / 2) - POSSIBLE_OFFSET);
        LCDsetLine(linenr + 1);
        (*page9_func_ptr [linenr + POSSIBLE_OFFSET] ) (); // not really linenumbers
        LCDcrlf();
      */
      break;
#endif // page 9
#ifndef SUPPRESS_TELEMETRY_PAGE_R
    case 'R':
      {
        //Reset logvalues
#if defined(LOG_VALUES) && (LOG_VALUES >= 2)
        cycleTimeMax = 0;
        cycleTimeMin = 65535;
#endif
#if BARO
#if defined(LOG_VALUES)
        BAROaltMax = 0;
#endif
#endif
#if defined(FAILSAFE)
        failsafeEvents = 0; // reset failsafe counter
#endif
        i2c_errors_count = 0;
        f.OK_TO_ARM = 1; // allow arming again
        telemetry = 0; // no use to repeat this forever
        break;
      }
#endif // case R
#if defined(DEBUG) || defined(DEBUG_FREE)
    case 'F':
      PRINT_FREE_RAM;
      break;
#endif // DEBUG
      // WARNING: if you add another case here, you should also add a case: in Serial.pde, so users can access your case via terminal input
  } // end switch (telemetry)
} // end function lcd_telemetry

#endif // DISPLAY_MULTILINE
void toggle_telemetry(uint8_t t) {
  if (telemetry == t) telemetry = 0;
  else {
    telemetry = t;
#if defined( OLED_I2C_128x64)
    if (telemetry != 0) i2c_OLED_init();
#elif defined(OLED_DIGOLE)
    if (telemetry != 0) i2c_OLED_DIGOLE_init();
#endif
    LCDclear();
  }
}
#endif //  LCD_TELEMETRY

#ifdef LOG_PERMANENT
void dumpPLog(uint8_t full) {
#ifdef HAS_LCD
  /*LCDclear();*/ LCDnextline();
  LCDprintChar("LastOff   "); LCDprintChar(plog.running ? "KO" : "ok");  LCDnextline();
  LCDprintChar("#On      "); LCDprintInt16(plog.start, 0, 0); LCDnextline();
  LCDprintChar("Life[min]"); LCDprintInt16(plog.lifetime / 60, 0, 0); LCDnextline();
  if (full) {
#ifdef DEBUG
    LCDprintChar("#arm   "); LCDprintInt16(plog.arm, 0, 0); LCDnextline();
    LCDprintChar("#disarm"); LCDprintInt16(plog.disarm, 0, 0); LCDnextline();
    LCDprintChar("last[s]"); LCDprintInt16(plog.armed_time / 1000000, 0, 0); LCDnextline();
    LCDprintChar("#fail@dis"); LCDprintInt16(plog.failsafe, 0, 0); LCDnextline();
    LCDprintChar("#i2c@dis "); LCDprintInt16(plog.i2c, 0, 0); LCDnextline();
    //            0123456789012345
#endif
  }
  /*strcpy_P(line2,PSTR("Fail --- i2c ---"));
    line2[5] = digit100(plog.failsafe);
    line2[6] = digit10(plog.failsafe);
    line2[7] = digit1(plog.failsafe);
    line2[13] = digit100(plog.i2c);
    line2[14] = digit10(plog.i2c);
    line2[15] = digit1(plog.i2c);
    LCDprintChar(line2); LCDnextline();*/
  delay(4000);
#endif
#ifdef LOG_PERMANENT_SERVICE_LIFETIME
  serviceCheckPLog();
#endif
#ifdef HAS_LCD
  LCDclear();
#endif
}

void LCDnextline(void) {
#if ( defined(DISPLAY_MULTILINE) )
  lnr++;
  if (lnr > (MULTILINE_PRE + MULTILINE_POST)) {
    lnr = 1;
    delay(4000);
    LCDclear();
  }
  LCDsetLine(lnr);
  LCD_FLUSH;
#elif ( defined(DISPLAY_2LINES))
#if (! (defined(LCD_TTY)  ) )
  delay(600);
#endif
#ifdef HAS_LCD
  LCDprintChar("\r\n");
#endif
#else
  // no LCD, nothing to do here
#endif
}

#ifdef LOG_PERMANENT_SERVICE_LIFETIME
void serviceCheckPLog(void) {
  if ( (!f.ARMED) && (plog.lifetime > LOG_PERMANENT_SERVICE_LIFETIME) ) {
    for (uint8_t i = 0; i < max(1, min(9, (plog.lifetime - LOG_PERMANENT_SERVICE_LIFETIME) >> 10 )); i++) {
#ifdef HAS_LCD
      LCDprintChar("SERVICE lifetime"); LCDnextline();
#endif
      blinkLED(5, 200, 5);
      delay(5000);
    }
    SET_ALARM(ALRM_FAC_CONFIRM, ALRM_LVL_CONFIRM_ELSE);
  }
}
#endif // LOG_PERMANENT_SERVICE_LIFETIME

#endif // LOG_PERMANENT
