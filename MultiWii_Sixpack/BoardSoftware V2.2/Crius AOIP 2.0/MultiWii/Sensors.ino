// ************************************************************************************************************
// board orientation and setup
// ************************************************************************************************************
//default board orientation
#if !defined(ACC_ORIENTATION) 
  #define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = X; accADC[PITCH]  = Y; accADC[YAW]  = Z;}
#endif
#if !defined(GYRO_ORIENTATION) 
  #define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] = X; gyroADC[PITCH] = Y; gyroADC[YAW] = Z;}
#endif
#if !defined(MAG_ORIENTATION) 
  #define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  = X; magADC[PITCH]  = Y; magADC[YAW]  = Z;}
#endif

/*** I2C address ***/

//#if !defined(ADXL345_ADDRESS) 
//  #define ADXL345_ADDRESS 0x1D
  //#define ADXL345_ADDRESS 0x53   //WARNING: Conflicts with a Wii Motion plus!
//#endif

#if !defined(MS561101BA_ADDRESS) 
  #define MS561101BA_ADDRESS 0x77 //CBR=0 0xEE I2C address when pin CSB is connected to LOW (GND)
  //#define MS561101BA_ADDRESS 0x76 //CBR=1 0xEC I2C address when pin CSB is connected to HIGH (VCC)
#endif

#if !defined(MPU6050_ADDRESS)
  #define MPU6050_ADDRESS     0x68 // address pin AD0 low (GND), default for FreeIMU v0.4 and InvenSense evaluation board
  //#define MPU6050_ADDRESS     0x69 // address pin AD0 high (VCC)
#endif

//MPU6050 Gyro LPF setting
#if defined(MPU6050_LPF_256HZ) || defined(MPU6050_LPF_188HZ) || defined(MPU6050_LPF_98HZ) || defined(MPU6050_LPF_42HZ) || defined(MPU6050_LPF_20HZ) || defined(MPU6050_LPF_10HZ) || defined(MPU6050_LPF_5HZ)
  #if defined(MPU6050_LPF_256HZ)
    #define MPU6050_DLPF_CFG   0
  #endif
  #if defined(MPU6050_LPF_188HZ)
    #define MPU6050_DLPF_CFG   1
  #endif
  #if defined(MPU6050_LPF_98HZ)
    #define MPU6050_DLPF_CFG   2
  #endif
  #if defined(MPU6050_LPF_42HZ)
    #define MPU6050_DLPF_CFG   3
  #endif
  #if defined(MPU6050_LPF_20HZ)
    #define MPU6050_DLPF_CFG   4
  #endif
  #if defined(MPU6050_LPF_10HZ)
    #define MPU6050_DLPF_CFG   5
  #endif
  #if defined(MPU6050_LPF_5HZ)
    #define MPU6050_DLPF_CFG   6
  #endif
#else
    //Default settings LPF 256Hz/8000Hz sample
    #define MPU6050_DLPF_CFG   0
#endif

#if defined(TINY_GPS) | defined(TINY_GPS_SONAR)
#define TINY_GPS_TWI_ADD 0x11
#include "tinygps.h"
#endif

uint8_t rawADC[6];
static uint32_t neutralizeTime = 0;
  
// ************************************************************************************************************
// I2C general functions
// ************************************************************************************************************

void i2c_init(void) {
  #if defined(INTERNAL_I2C_PULLUPS)
    I2C_PULLUPS_ENABLE
  #else
    I2C_PULLUPS_DISABLE
  #endif
  TWSR = 0;                                    // no prescaler => prescaler = 1
  TWBR = ((F_CPU / I2C_SPEED) - 16) / 2;       // change the I2C clock rate
  TWCR = 1<<TWEN;                              // enable twi module, no interrupt
}

void i2c_rep_start(uint8_t address) {
  TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN) ; // send REPEAT START condition
  waitTransmissionI2C();                       // wait until transmission completed
  TWDR = address;                              // send device address
  TWCR = (1<<TWINT) | (1<<TWEN);
  waitTransmissionI2C();                       // wail until transmission completed
}

void i2c_stop(void) {
  TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
  //  while(TWCR & (1<<TWSTO));                // <- can produce a blocking state with some WMP clones
}

void i2c_write(uint8_t data ) {
  TWDR = data;                                 // send data to the previously addressed device
  TWCR = (1<<TWINT) | (1<<TWEN);
  waitTransmissionI2C();
}

uint8_t i2c_read(uint8_t ack) {
  TWCR = (1<<TWINT) | (1<<TWEN) | (ack? (1<<TWEA) : 0);
  waitTransmissionI2C();
  uint8_t r = TWDR;
  if (!ack) i2c_stop();
  return r;
}

uint8_t i2c_readAck() {
  return i2c_read(1);
}

uint8_t i2c_readNak(void) {
  return i2c_read(0);
}

void waitTransmissionI2C() {
  uint16_t count = 255;
  while (!(TWCR & (1<<TWINT))) {
    count--;
    if (count==0) {              //we are in a blocking state => we don't insist
      TWCR = 0;                  //and we force a reset on TWINT register
      neutralizeTime = micros(); //we take a timestamp here to neutralize the value during a short delay
      i2c_errors_count++;
      break;
    }
  }
}

size_t i2c_read_to_buf(uint8_t add, void *buf, size_t size) {
  i2c_rep_start((add<<1) | 1);  // I2C read direction
  size_t bytes_read = 0;
  uint8_t *b = (uint8_t*)buf;
  while (size--) {
    /* acknowledge all but the final byte */
    *b++ = i2c_read(size > 0);
    /* TODO catch I2C errors here and abort */
    bytes_read++;
  }
  return bytes_read;
}

size_t i2c_read_reg_to_buf(uint8_t add, uint8_t reg, void *buf, size_t size) {
  i2c_rep_start(add<<1); // I2C write direction
  i2c_write(reg);        // register selection
  return i2c_read_to_buf(add, buf, size);
}

/* transform a series of bytes from big endian to little
   endian and vice versa. */
void swap_endianness(void *buf, size_t size) {
  /* we swap in-place, so we only have to
  * place _one_ element on a temporary tray
  */
  uint8_t tray;
  uint8_t *from;
  uint8_t *to;
  /* keep swapping until the pointers have assed each other */
  for (from = (uint8_t*)buf, to = &from[size-1]; from < to; from++, to--) {
    tray = *from;
    *from = *to;
    *to = tray;
  }
}

void i2c_getSixRawADC(uint8_t add, uint8_t reg) {
  i2c_read_reg_to_buf(add, reg, &rawADC, 6);
}

void i2c_writeReg(uint8_t add, uint8_t reg, uint8_t val) {
  i2c_rep_start(add<<1); // I2C write direction
  i2c_write(reg);        // register selection
  i2c_write(val);        // value to write in register
  i2c_stop();
}

uint8_t i2c_readReg(uint8_t add, uint8_t reg) {
  uint8_t val;
  i2c_read_reg_to_buf(add, reg, &val, 1);
  return val;
}

// ****************
// GYRO common part
// ****************
void GYRO_Common() {
  static int16_t previousGyroADC[3] = {0,0,0};
  static int32_t g[3];
  uint8_t axis, tilt=0;

#if defined MMGYRO       
  // Moving Average Gyros by Magnetron1
  //---------------------------------------------------
  static int16_t mediaMobileGyroADC[3][MMGYROVECTORLENGTH];
  static int32_t mediaMobileGyroADCSum[3];
  static uint8_t mediaMobileGyroIDX;
  //---------------------------------------------------
#endif

  if (calibratingG>0) {
    for (axis = 0; axis < 3; axis++) {
      // Reset g[axis] at start of calibration
      if (calibratingG == 512) {
        g[axis]=0;
        
        #if defined(GYROCALIBRATIONFAILSAFE)
            previousGyroADC[axis] = gyroADC[axis];
          }
          if (calibratingG % 10 == 0) {
            if(abs(gyroADC[axis] - previousGyroADC[axis]) > 8) tilt=1;
            previousGyroADC[axis] = gyroADC[axis];
       #endif
      }
      // Sum up 512 readings
      g[axis] +=gyroADC[axis];
      // Clear global variables for next reading
      gyroADC[axis]=0;
      gyroZero[axis]=0;
      if (calibratingG == 1) {
        gyroZero[axis]=g[axis]>>9;
      #if defined(BUZZER)
        alarmArray[7] = 4;
      #else
        blinkLED(10,15,1); //the delay causes to beep the buzzer really long 
      #endif
      }
    }
    #if defined(GYROCALIBRATIONFAILSAFE)
      if(tilt) {
        calibratingG=1000;
        LEDPIN_ON;
      } else {
        calibratingG--;
        LEDPIN_OFF;
      }
      return;
    #else
      calibratingG--;
    #endif
    
  }

#ifdef MMGYRO       
  mediaMobileGyroIDX = ++mediaMobileGyroIDX % conf.mmgyro;
  for (axis = 0; axis < 3; axis++) {
    gyroADC[axis]  -= gyroZero[axis];
    mediaMobileGyroADCSum[axis] -= mediaMobileGyroADC[axis][mediaMobileGyroIDX];
    //anti gyro glitch, limit the variation between two consecutive readings
    mediaMobileGyroADC[axis][mediaMobileGyroIDX] = constrain(gyroADC[axis],previousGyroADC[axis]-800,previousGyroADC[axis]+800);
    mediaMobileGyroADCSum[axis] += mediaMobileGyroADC[axis][mediaMobileGyroIDX];
    gyroADC[axis] = mediaMobileGyroADCSum[axis] / conf.mmgyro;
#else 
  for (axis = 0; axis < 3; axis++) {
    gyroADC[axis]  -= gyroZero[axis];
    //anti gyro glitch, limit the variation between two consecutive readings
    gyroADC[axis] = constrain(gyroADC[axis],previousGyroADC[axis]-800,previousGyroADC[axis]+800);
#endif    
    previousGyroADC[axis] = gyroADC[axis];
  }

  #if defined(SENSORS_TILT_45DEG_LEFT)
    int16_t temp  = ((gyroADC[PITCH] - gyroADC[ROLL] )*7)/10;
    gyroADC[ROLL] = ((gyroADC[ROLL]  + gyroADC[PITCH])*7)/10;
    gyroADC[PITCH]= temp;
  #endif
  #if defined(SENSORS_TILT_45DEG_RIGHT)
    int16_t temp  = ((gyroADC[PITCH] + gyroADC[ROLL] )*7)/10;
    gyroADC[ROLL] = ((gyroADC[ROLL]  - gyroADC[PITCH])*7)/10;
    gyroADC[PITCH]= temp;
  #endif
}

// ****************
// ACC common part
// ****************
void ACC_Common() {
  static int32_t a[3];
  
  if (calibratingA>0) {
    for (uint8_t axis = 0; axis < 3; axis++) {
      // Reset a[axis] at start of calibration
      if (calibratingA == 512) a[axis]=0;
      // Sum up 512 readings
      a[axis] +=accADC[axis];
      // Clear global variables for next reading
      accADC[axis]=0;
      global_conf.accZero[axis]=0;
    }
    // Calculate average, shift Z down by acc_1G and store values in EEPROM at end of calibration
    if (calibratingA == 1) {
      global_conf.accZero[ROLL]  = a[ROLL]>>9;
      global_conf.accZero[PITCH] = a[PITCH]>>9;
      global_conf.accZero[YAW]   = (a[YAW]>>9)-acc_1G; // for nunchuk 200=1G
      conf.angleTrim[ROLL]   = 0;
      conf.angleTrim[PITCH]  = 0;
      writeGlobalSet(1); // write accZero in EEPROM
    }
    calibratingA--;
  }
  #if defined(INFLIGHT_ACC_CALIBRATION)
      static int32_t b[3];
      static int16_t accZero_saved[3]  = {0,0,0};
      static int16_t  angleTrim_saved[2] = {0, 0};
      //Saving old zeropoints before measurement
      if (InflightcalibratingA==50) {
         accZero_saved[ROLL]  = global_conf.accZero[ROLL] ;
         accZero_saved[PITCH] = global_conf.accZero[PITCH];
         accZero_saved[YAW]   = global_conf.accZero[YAW] ;
         angleTrim_saved[ROLL]  = conf.angleTrim[ROLL] ;
         angleTrim_saved[PITCH] = conf.angleTrim[PITCH] ;
      }
      if (InflightcalibratingA>0) {
        for (uint8_t axis = 0; axis < 3; axis++) {
          // Reset a[axis] at start of calibration
          if (InflightcalibratingA == 50) b[axis]=0;
          // Sum up 50 readings
          b[axis] +=accADC[axis];
          // Clear global variables for next reading
          accADC[axis]=0;
          global_conf.accZero[axis]=0;
        }
        //all values are measured
        if (InflightcalibratingA == 1) {
          AccInflightCalibrationActive = 0;
          AccInflightCalibrationMeasurementDone = 1;
          #if defined(BUZZER)
            alarmArray[7] = 1;      //buzzer for indicatiing the end of calibration
          #endif
          // recover saved values to maintain current flight behavior until new values are transferred
          global_conf.accZero[ROLL]  = accZero_saved[ROLL] ;
          global_conf.accZero[PITCH] = accZero_saved[PITCH];
          global_conf.accZero[YAW]   = accZero_saved[YAW] ;
          conf.angleTrim[ROLL]  = angleTrim_saved[ROLL] ;
          conf.angleTrim[PITCH] = angleTrim_saved[PITCH] ;
        }
        InflightcalibratingA--;
      }
      // Calculate average, shift Z down by acc_1G and store values in EEPROM at end of calibration
      if (AccInflightCalibrationSavetoEEProm == 1){  //the copter is landed, disarmed and the combo has been done again
        AccInflightCalibrationSavetoEEProm = 0;
        global_conf.accZero[ROLL]  = b[ROLL]/50;
        global_conf.accZero[PITCH] = b[PITCH]/50;
        global_conf.accZero[YAW]   = b[YAW]/50-acc_1G; // for nunchuk 200=1G
        conf.angleTrim[ROLL]   = 0;
        conf.angleTrim[PITCH]  = 0;
        writeGlobalSet(1); // write accZero in EEPROM
      }
  #endif
  accADC[ROLL]  -=  global_conf.accZero[ROLL] ;
  accADC[PITCH] -=  global_conf.accZero[PITCH];
  accADC[YAW]   -=  global_conf.accZero[YAW] ;

  #if defined(SENSORS_TILT_45DEG_LEFT)
    int16_t temp = ((accADC[PITCH] - accADC[ROLL] )*7)/10;
    accADC[ROLL] = ((accADC[ROLL]  + accADC[PITCH])*7)/10;
    accADC[PITCH] = temp;
  #endif
  #if defined(SENSORS_TILT_45DEG_RIGHT)
    int16_t temp = ((accADC[PITCH] + accADC[ROLL] )*7)/10;
    accADC[ROLL] = ((accADC[ROLL]  - accADC[PITCH])*7)/10;
    accADC[PITCH] = temp;
  #endif
}


// ************************************************************************************************************
// I2C Barometer BOSCH BMP085
// ************************************************************************************************************
// I2C adress: 0x77 (7bit)
// principle:
//  1) read the calibration register (only once at the initialization)
//  2) read uncompensated temperature (not mandatory at every cycle)
//  3) read uncompensated pressure
//  4) raw temp + raw pressure => calculation of the adjusted pressure
//  the following code uses the maximum precision setting (oversampling setting 3)
// ************************************************************************************************************

#if defined(BMP085)
#define BMP085_ADDRESS 0x77

static struct {
  // sensor registers from the BOSCH BMP085 datasheet
  int16_t  ac1, ac2, ac3;
  uint16_t ac4, ac5, ac6;
  int16_t  b1, b2, mb, mc, md;
  union {uint16_t val; uint8_t raw[2]; } ut; //uncompensated T
  union {uint32_t val; uint8_t raw[4]; } up; //uncompensated P
  uint8_t  state;
  uint32_t deadline;
} bmp085_ctx;  
#define OSS 3

void i2c_BMP085_readCalibration(){
  delay(10);
  //read calibration data in one go
  size_t s_bytes = (uint8_t*)&bmp085_ctx.md - (uint8_t*)&bmp085_ctx.ac1 + sizeof(bmp085_ctx.ac1);
  i2c_read_reg_to_buf(BMP085_ADDRESS, 0xAA, &bmp085_ctx.ac1, s_bytes);
  // now fix endianness
  int16_t *p;
  for (p = &bmp085_ctx.ac1; p <= &bmp085_ctx.md; p++) {
    swap_endianness(p, sizeof(*p));
  }
}

void  Baro_init() {
  delay(10);
  i2c_BMP085_readCalibration();
  delay(5);
  i2c_BMP085_UT_Start(); 
  bmp085_ctx.deadline = currentTime+5000;
}

// read uncompensated temperature value: send command first
void i2c_BMP085_UT_Start() {
  i2c_writeReg(BMP085_ADDRESS,0xf4,0x2e);
  i2c_rep_start(BMP085_ADDRESS<<1);
  i2c_write(0xF6);
  i2c_stop();
}

// read uncompensated pressure value: send command first
void i2c_BMP085_UP_Start () {
  i2c_writeReg(BMP085_ADDRESS,0xf4,0x34+(OSS<<6)); // control register value for oversampling setting 3
  i2c_rep_start(BMP085_ADDRESS<<1); //I2C write direction => 0
  i2c_write(0xF6);
  i2c_stop();
}

// read uncompensated pressure value: read result bytes
// the datasheet suggests a delay of 25.5 ms (oversampling settings 3) after the send command
void i2c_BMP085_UP_Read () {
  i2c_rep_start((BMP085_ADDRESS<<1) | 1);//I2C read direction => 1
  bmp085_ctx.up.raw[2] = i2c_readAck();
  bmp085_ctx.up.raw[1] = i2c_readAck();
  bmp085_ctx.up.raw[0] = i2c_readNak();
}

// read uncompensated temperature value: read result bytes
// the datasheet suggests a delay of 4.5 ms after the send command
void i2c_BMP085_UT_Read() {
  i2c_rep_start((BMP085_ADDRESS<<1) | 1);//I2C read direction => 1
  bmp085_ctx.ut.raw[1] = i2c_readAck();
  bmp085_ctx.ut.raw[0] = i2c_readNak();
}

void i2c_BMP085_Calculate() {
  int32_t  x1, x2, x3, b3, b5, b6, p, tmp;
  uint32_t b4, b7;
  // Temperature calculations
  x1 = ((int32_t)bmp085_ctx.ut.val - bmp085_ctx.ac6) * bmp085_ctx.ac5 >> 15;
  x2 = ((int32_t)bmp085_ctx.mc << 11) / (x1 + bmp085_ctx.md);
  b5 = x1 + x2;
  baroTemperature = (b5 * 10 + 8) >> 4; // in 0.01 degC (same as MS561101BA temperature)
  //Baro_T_gradC = (baroTemperature / 100); // in gradC, only for LabVIEW GUI
  // Pressure calculations
  b6 = b5 - 4000;
  x1 = (bmp085_ctx.b2 * (b6 * b6 >> 12)) >> 11; 
  x2 = bmp085_ctx.ac2 * b6 >> 11;
  x3 = x1 + x2;
  tmp = bmp085_ctx.ac1;
  tmp = (tmp*4 + x3) << OSS;
  b3 = (tmp+2)/4;
  x1 = bmp085_ctx.ac3 * b6 >> 13;
  x2 = (bmp085_ctx.b1 * (b6 * b6 >> 12)) >> 16;
  x3 = ((x1 + x2) + 2) >> 2;
  b4 = (bmp085_ctx.ac4 * (uint32_t)(x3 + 32768)) >> 15;
  b7 = ((uint32_t) (bmp085_ctx.up.val >> (8-OSS)) - b3) * (50000 >> OSS);
  p = b7 < 0x80000000 ? (b7 * 2) / b4 : (b7 / b4) * 2;
  x1 = (p >> 8) * (p >> 8);
  x1 = (x1 * 3038) >> 16;
  x2 = (-7357 * p) >> 16;
  baroPressure = p + ((x1 + x2 + 3791) >> 4);
  //Baro_P_mbara = (baroPressure / 100); // in mbar absolute, only for LabVIEW GUI
}

//return 0: no data available, no computation ;  1: new value available  ; 2: no new value, but computation time
uint8_t Baro_update() {                   // first UT conversion is started in init procedure
  if (currentTime < bmp085_ctx.deadline) return 0; 
  bmp085_ctx.deadline = currentTime+6000; // 1.5ms margin according to the spec (4.5ms T convetion time)
  TWBR = ((F_CPU / 400000L) - 16) / 2; // change the I2C clock rate to 400kHz, BMP085 is ok with this speed
  if (bmp085_ctx.state == 0) {
    i2c_BMP085_UT_Read(); 
    i2c_BMP085_UP_Start(); 
    bmp085_ctx.state = 1; 
    Baro_Common();
    bmp085_ctx.deadline += 21000;   // 6000+21000=27000 1.5ms margin according to the spec (25.5ms P convetion time with OSS=3)
    return 1;
  } else {
    i2c_BMP085_UP_Read(); 
    i2c_BMP085_UT_Start(); 
    i2c_BMP085_Calculate(); 
    bmp085_ctx.state = 0; 
    return 2;
  }
}
#endif


//#if BARO
//  void Baro_Common() {
//    static int32_t baroHistTab[BARO_TAB_SIZE];
//    static uint8_t baroHistIdx;
//    uint8_t indexplus1 = (baroHistIdx + 1);
//    if (indexplus1 == BARO_TAB_SIZE) indexplus1 = 0;
//  baroHistTab[baroHistIdx] = baroPressure;
//  baroPressureSum += baroHistTab[baroHistIdx];
//  baroPressureSum -= baroHistTab[indexplus1];
//  baroHistIdx = indexplus1;  
//}
//#endif


// ************************************************************************************************************
// I2C Barometer MS561101BA
// ************************************************************************************************************
//
// specs are here: http://www.meas-spec.com/downloads/MS5611-01BA03.pdf
// useful info on pages 7 -> 12
#if defined(MS561101BA)

// registers of the device
#define MS561101BA_PRESSURE    0x40
#define MS561101BA_TEMPERATURE 0x50
#define MS561101BA_RESET       0x1E

// OSR (Over Sampling Ratio) constants
#define MS561101BA_OSR_256  0x00
#define MS561101BA_OSR_512  0x02
#define MS561101BA_OSR_1024 0x04
#define MS561101BA_OSR_2048 0x06
#define MS561101BA_OSR_4096 0x08

#define OSR MS561101BA_OSR_4096

static struct {
  // sensor registers from the MS561101BA datasheet
  uint16_t c[7];
  union {uint32_t val; uint8_t raw[4]; } ut; //uncompensated T
  union {uint32_t val; uint8_t raw[4]; } up; //uncompensated P
  uint8_t  state;
  uint32_t deadline;
} ms561101ba_ctx;

void i2c_MS561101BA_reset(){
  i2c_writeReg(MS561101BA_ADDRESS, MS561101BA_RESET, 0);
}

void i2c_MS561101BA_readCalibration(){
  union {uint16_t val; uint8_t raw[2]; } data;
  for(uint8_t i=0;i<6;i++) {
    i2c_rep_start(MS561101BA_ADDRESS<<1);
    i2c_write(0xA2+2*i);
    delay(10);
    i2c_rep_start((MS561101BA_ADDRESS<<1) | 1);//I2C read direction => 1
    delay(10);
    data.raw[1] = i2c_readAck();  // read a 16 bit register
    data.raw[0] = i2c_readNak();
    ms561101ba_ctx.c[i+1] = data.val;
  }
}

void  Baro_init() {
  delay(10);
  i2c_MS561101BA_reset();
  delay(100);
  i2c_MS561101BA_readCalibration();
  delay(10);
  i2c_MS561101BA_UT_Start(); 
  ms561101ba_ctx.deadline = currentTime+10000; 
}

// read uncompensated temperature value: send command first
void i2c_MS561101BA_UT_Start() {
  i2c_rep_start(MS561101BA_ADDRESS<<1);      // I2C write direction
  i2c_write(MS561101BA_TEMPERATURE + OSR);  // register selection
  i2c_stop();
}

// read uncompensated pressure value: send command first
void i2c_MS561101BA_UP_Start () {
  i2c_rep_start(MS561101BA_ADDRESS<<1);      // I2C write direction
  i2c_write(MS561101BA_PRESSURE + OSR);     // register selection
  i2c_stop();
}

// read uncompensated pressure value: read result bytes
void i2c_MS561101BA_UP_Read () {
  i2c_rep_start(MS561101BA_ADDRESS<<1);
  i2c_write(0);
  i2c_rep_start((MS561101BA_ADDRESS<<1) | 1);
  ms561101ba_ctx.up.raw[2] = i2c_readAck();
  ms561101ba_ctx.up.raw[1] = i2c_readAck();
  ms561101ba_ctx.up.raw[0] = i2c_readNak();
}

// read uncompensated temperature value: read result bytes
void i2c_MS561101BA_UT_Read() {
  i2c_rep_start(MS561101BA_ADDRESS<<1);
  i2c_write(0);
  i2c_rep_start((MS561101BA_ADDRESS<<1) | 1);
  ms561101ba_ctx.ut.raw[2] = i2c_readAck();
  ms561101ba_ctx.ut.raw[1] = i2c_readAck();
  ms561101ba_ctx.ut.raw[0] = i2c_readNak();
}

void i2c_MS561101BA_Calculate() {
  int32_t off2,sens2,delt;

  int64_t dT       = (int32_t)ms561101ba_ctx.ut.val - ((int32_t)ms561101ba_ctx.c[5] << 8);
  baroTemperature  = 2000 + ((dT * ms561101ba_ctx.c[6])>>23);
  int64_t off      = ((uint32_t)ms561101ba_ctx.c[2] <<16) + ((dT * ms561101ba_ctx.c[4]) >> 7);
  int64_t sens     = ((uint32_t)ms561101ba_ctx.c[1] <<15) + ((dT * ms561101ba_ctx.c[3]) >> 8);

  if (baroTemperature < 2000) { // temperature lower than 20st.C 
    delt = baroTemperature-2000;
    delt  = 5*delt*delt;
    off2  = delt>>1;
    sens2 = delt>>2;
    if (baroTemperature < -1500) { // temperature lower than -15st.C
      delt  = baroTemperature+1500;
      delt  = delt*delt;
      off2  += 7 * delt;
      sens2 += (11 * delt)>>1;
    }
    off  -= off2; 
    sens -= sens2;
  }

  baroPressure     = (( (ms561101ba_ctx.up.val * sens ) >> 21) - off) >> 15;
}

//return 0: no data available, no computation ;  1: new value available  ; 2: no new value, but computation time
uint8_t Baro_update() {                            // first UT conversion is started in init procedure
  if (currentTime < ms561101ba_ctx.deadline) return 0; 
  ms561101ba_ctx.deadline = currentTime+10000;  // UT and UP conversion take 8.5ms so we do next reading after 10ms 
  TWBR = ((F_CPU / 400000L) - 16) / 2;          // change the I2C clock rate to 400kHz, MS5611 is ok with this speed
  if (ms561101ba_ctx.state == 0) {
    i2c_MS561101BA_UT_Read(); 
    i2c_MS561101BA_UP_Start(); 
    Baro_Common();                              // moved here for less timecycle spike
    ms561101ba_ctx.state = 1;
    return 1;
  } else {
    i2c_MS561101BA_UP_Read();
    i2c_MS561101BA_UT_Start(); 
    i2c_MS561101BA_Calculate();
    ms561101ba_ctx.state = 0; 
    return 2;
  }
}
#endif

#if BARO
  void Baro_Common() {
    static int32_t baroHistTab[BARO_TAB_SIZE];
    static uint8_t baroHistIdx;
  
    uint8_t indexplus1 = (baroHistIdx + 1);
    if (indexplus1 == BARO_TAB_SIZE) indexplus1 = 0;
    baroHistTab[baroHistIdx] = baroPressure;
    baroPressureSum += baroHistTab[baroHistIdx];
    baroPressureSum -= baroHistTab[indexplus1];
    baroHistIdx = indexplus1;  
  }
#endif

// ************************************************************************************************************
// I2C Accelerometer ADXL345 
// ************************************************************************************************************
// I2C adress: 0x3A (8bit)    0x1D (7bit)
// Resolution: 10bit (Full range - 14bit, but this is autoscaling 10bit ADC to the range +- 16g)
// principle:
//  1) CS PIN must be linked to VCC to select the I2C mode
//  2) SD0 PIN must be linked to VCC to select the right I2C adress
//  3) bit  b00000100 must be set on register 0x2D to read data (only once at the initialization)
//  4) bits b00001011 must be set on register 0x31 to select the data format (only once at the initialization)
// ************************************************************************************************************
#if defined(ADXL345)
void ACC_init () {
  delay(10);
  i2c_writeReg(ADXL345_ADDRESS,0x2D,1<<3); //  register: Power CTRL  -- value: Set measure bit 3 on
  i2c_writeReg(ADXL345_ADDRESS,0x31,0x0B); //  register: DATA_FORMAT -- value: Set bits 3(full range) and 1 0 on (+/- 16g-range)
  i2c_writeReg(ADXL345_ADDRESS,0x2C,0x09); //  register: BW_RATE     -- value: rate=50hz, bw=20hz
  acc_1G = 265;
}

void ACC_getADC () {
  TWBR = ((F_CPU / 400000L) - 16) / 2; // change the I2C clock rate to 400kHz, ADXL435 is ok with this speed
  i2c_getSixRawADC(ADXL345_ADDRESS,0x32);

  ACC_ORIENTATION( ((rawADC[1]<<8) | rawADC[0]) ,
                   ((rawADC[3]<<8) | rawADC[2]) ,
                   ((rawADC[5]<<8) | rawADC[4]) );
  ACC_Common();
}
#endif

// ************************************************************************************************************
// I2C Accelerometer BMA020
// ************************************************************************************************************
// I2C adress: 0x70 (8bit)
// Resolution: 10bit
// Control registers:
//
// Datasheet: After power on reset or soft reset it is recommended to set the SPI4-bit to the correct value.
//            0x80 = SPI four-wire = Default setting
// | 0x15: | SPI4 | enable_adv_INT | new_data_INT | latch_INT | shadow_dis | wake_up_pause<1:0> | wake_up |
// |       |    1 |              0 |            0 |         0 |          0 |                 00 |       0 |
//
// | 0x14: |                       reserved <2:0> |            range <1:0> |               bandwith <2:0> |
// |       |                      !!Calibration!! |                     2g |                         25Hz |
//
// ************************************************************************************************************
#if defined(BMA020)
void ACC_init(){
  i2c_writeReg(0x38,0x15,0x80);    // set SPI4 bit
  uint8_t control = i2c_readReg(0x70, 0x14);
  control = control & 0xE0;        // save bits 7,6,5
  control = control | (0x02 << 3); // Range 8G (10)
  control = control | 0x00;        // Bandwidth 25 Hz 000
  i2c_writeReg(0x38,0x14,control); 
  acc_1G = 63;
}

void ACC_getADC(){
  TWBR = ((F_CPU / 400000L) - 16) / 2;
  i2c_getSixRawADC(0x38,0x02);
  ACC_ORIENTATION( ((rawADC[1]<<8) | rawADC[0])>>6 ,
                   ((rawADC[3]<<8) | rawADC[2])>>6 ,
                   ((rawADC[5]<<8) | rawADC[4])>>6 );
  ACC_Common();
}
#endif

// ************************************************************************************************************
// I2C Compass common function
// ************************************************************************************************************
#if MAG
static float   magGain[3] = {1.0,1.0,1.0};  // gain for each axis, populated at sensor init
static uint8_t magInit = 0;

uint8_t Mag_getADC() { // return 1 when news values are available, 0 otherwise
  static uint32_t t,tCal = 0;
  static int16_t magZeroTempMin[3];
  static int16_t magZeroTempMax[3];
  uint8_t axis;
  if ( currentTime < t ) return 0; //each read is spaced by 100ms
  t = currentTime + 100000;
  TWBR = ((F_CPU / 400000L) - 16) / 2; // change the I2C clock rate to 400kHz
  Device_Mag_getADC();
  magADC[ROLL]  = magADC[ROLL]  * magGain[ROLL];
  magADC[PITCH] = magADC[PITCH] * magGain[PITCH];
  magADC[YAW]   = magADC[YAW]   * magGain[YAW];
  if (f.CALIBRATE_MAG) {
    tCal = t;
    for(axis=0;axis<3;axis++) {
      global_conf.magZero[axis] = 0;
      magZeroTempMin[axis] = magADC[axis];
      magZeroTempMax[axis] = magADC[axis];
    }
    f.CALIBRATE_MAG = 0;
  }
  if (magInit) { // we apply offset only once mag calibration is done
    magADC[ROLL]  -= global_conf.magZero[ROLL];
    magADC[PITCH] -= global_conf.magZero[PITCH];
    magADC[YAW]   -= global_conf.magZero[YAW];
  }
 
  if (tCal != 0) {
    if ((t - tCal) < 30000000) { // 30s: you have 30s to turn the multi in all directions
      LEDPIN_TOGGLE;
      for(axis=0;axis<3;axis++) {
        if (magADC[axis] < magZeroTempMin[axis]) magZeroTempMin[axis] = magADC[axis];
        if (magADC[axis] > magZeroTempMax[axis]) magZeroTempMax[axis] = magADC[axis];
      }
    } else {
      tCal = 0;
      for(axis=0;axis<3;axis++)
        global_conf.magZero[axis] = (magZeroTempMin[axis] + magZeroTempMax[axis])>>1;
      writeGlobalSet(1);
    }
  } else {
    #if defined(SENSORS_TILT_45DEG_LEFT)
      int16_t temp = ((magADC[PITCH] - magADC[ROLL] )*7)/10;
      magADC[ROLL] = ((magADC[ROLL]  + magADC[PITCH])*7)/10;
      magADC[PITCH] = temp;
    #endif
    #if defined(SENSORS_TILT_45DEG_RIGHT)
      int16_t temp = ((magADC[PITCH] + magADC[ROLL] )*7)/10;
      magADC[ROLL] = ((magADC[ROLL]  - magADC[PITCH])*7)/10;
      magADC[PITCH] = temp;
    #endif
  }
  return 1;
}
#endif

// ************************************************************************************************************
// I2C Compass HMC5883
// ************************************************************************************************************
// I2C adress: 0x3C (8bit)   0x1E (7bit)
// ************************************************************************************************************

#if defined(HMC5883)

#define HMC58X3_R_CONFA 0
#define HMC58X3_R_CONFB 1
#define HMC58X3_R_MODE 2
#define HMC58X3_X_SELF_TEST_GAUSS (+1.16)                       //!< X axis level when bias current is applied.
#define HMC58X3_Y_SELF_TEST_GAUSS (+1.16)   //!< Y axis level when bias current is applied.
#define HMC58X3_Z_SELF_TEST_GAUSS (+1.08)                       //!< Y axis level when bias current is applied.
#define SELF_TEST_LOW_LIMIT  (243.0/390.0)   //!< Low limit when gain is 5.
#define SELF_TEST_HIGH_LIMIT (575.0/390.0)   //!< High limit when gain is 5.
#define HMC_POS_BIAS 1
#define HMC_NEG_BIAS 2

#define MAG_ADDRESS 0x1E
#define MAG_DATA_REGISTER 0x03

void Mag_init() {
  int32_t xyz_total[3]={0,0,0};  // 32 bit totals so they won't overflow.
  bool bret=true;                // Error indicator

  delay(50);  //Wait before start
  i2c_writeReg(MAG_ADDRESS, HMC58X3_R_CONFA, 0x010 + HMC_POS_BIAS); // Reg A DOR=0x010 + MS1,MS0 set to pos bias

  // Note that the  very first measurement after a gain change maintains the same gain as the previous setting. 
  // The new gain setting is effective from the second measurement and on.

  i2c_writeReg(MAG_ADDRESS, HMC58X3_R_CONFB, 2 << 5);  //Set the Gain
  i2c_writeReg(MAG_ADDRESS,HMC58X3_R_MODE, 1);
  delay(100);
  getADC();  //Get one sample, and discard it

  for (uint8_t i=0; i<10; i++) { //Collect 10 samples
    i2c_writeReg(MAG_ADDRESS,HMC58X3_R_MODE, 1);
    delay(100);
    getADC();   // Get the raw values in case the scales have already been changed.
                
    // Since the measurements are noisy, they should be averaged rather than taking the max.
    xyz_total[0]+=magADC[0];
    xyz_total[1]+=magADC[1];
    xyz_total[2]+=magADC[2];
                
    // Detect saturation.
    if (-(1<<12) >= min(magADC[0],min(magADC[1],magADC[2]))) {
      bret=false;
      break;  // Breaks out of the for loop.  No sense in continuing if we saturated.
    }
  }

  // Apply the negative bias. (Same gain)
  i2c_writeReg(MAG_ADDRESS,HMC58X3_R_CONFA, 0x010 + HMC_NEG_BIAS); // Reg A DOR=0x010 + MS1,MS0 set to negative bias.
  for (uint8_t i=0; i<10; i++) { 
    i2c_writeReg(MAG_ADDRESS,HMC58X3_R_MODE, 1);
    delay(100);
    getADC();  // Get the raw values in case the scales have already been changed.
                
    // Since the measurements are noisy, they should be averaged.
    xyz_total[0]-=magADC[0];
    xyz_total[1]-=magADC[1];
    xyz_total[2]-=magADC[2];

    // Detect saturation.
    if (-(1<<12) >= min(magADC[0],min(magADC[1],magADC[2]))) {
      bret=false;
      break;  // Breaks out of the for loop.  No sense in continuing if we saturated.
    }
  }

  magGain[0]=fabs(820.0*HMC58X3_X_SELF_TEST_GAUSS*2.0*10.0/xyz_total[0]);
  magGain[1]=fabs(820.0*HMC58X3_Y_SELF_TEST_GAUSS*2.0*10.0/xyz_total[1]);
  magGain[2]=fabs(820.0*HMC58X3_Z_SELF_TEST_GAUSS*2.0*10.0/xyz_total[2]);

  // leave test mode
  i2c_writeReg(MAG_ADDRESS ,HMC58X3_R_CONFA ,0x70 ); //Configuration Register A  -- 0 11 100 00  num samples: 8 ; output rate: 15Hz ; normal measurement mode
  i2c_writeReg(MAG_ADDRESS ,HMC58X3_R_CONFB ,0x20 ); //Configuration Register B  -- 001 00000    configuration gain 1.3Ga
  i2c_writeReg(MAG_ADDRESS ,HMC58X3_R_MODE  ,0x00 ); //Mode register             -- 000000 00    continuous Conversion Mode
  delay(100);
  magInit = 1;

  if (!bret) { //Something went wrong so get a best guess
    magGain[0] = 1.0;
    magGain[1] = 1.0;
    magGain[2] = 1.0;
  }
} //  Mag_init().
#endif

// ************************************************************************************************************
// I2C Compass HMC5843
// ************************************************************************************************************
// I2C adress: 0x3C (8bit)   0x1E (7bit)
// ************************************************************************************************************
#if defined(HMC5843)

  #define MAG_ADDRESS 0x1E
  #define MAG_DATA_REGISTER 0x03
  
  void Mag_init() { 
    delay(100);
    // force positiveBias
    i2c_writeReg(MAG_ADDRESS ,0x00 ,0x71 ); //Configuration Register A  -- 0 11 100 01  num samples: 8 ; output rate: 15Hz ; positive bias
    delay(50);
    // set gains for calibration
    i2c_writeReg(MAG_ADDRESS ,0x01 ,0x60 ); //Configuration Register B  -- 011 00000    configuration gain 2.5Ga
    i2c_writeReg(MAG_ADDRESS ,0x02 ,0x01 ); //Mode register             -- 000000 01    single Conversion Mode

    // read values from the compass -  self test operation
    // by placing the mode register into single-measurement mode (0x01), two data acquisition cycles will be made on each magnetic vector.
    // The first acquisition values will be subtracted from the second acquisition, and the net measurement will be placed into the data output registers
    delay(100);
      getADC();
    delay(10);
      magGain[ROLL]  =  1000.0 / abs(magADC[ROLL]);
      magGain[PITCH] =  1000.0 / abs(magADC[PITCH]);
      magGain[YAW]   =  1000.0 / abs(magADC[YAW]);

    // leave test mode
    i2c_writeReg(MAG_ADDRESS ,0x00 ,0x70 ); //Configuration Register A  -- 0 11 100 00  num samples: 8 ; output rate: 15Hz ; normal measurement mode
    i2c_writeReg(MAG_ADDRESS ,0x01 ,0x20 ); //Configuration Register B  -- 001 00000    configuration gain 1.3Ga
    i2c_writeReg(MAG_ADDRESS ,0x02 ,0x00 ); //Mode register             -- 000000 00    continuous Conversion Mode

    magInit = 1;
  }
#endif
  
#if defined(HMC5843) || defined(HMC5883)
void getADC() {
  i2c_getSixRawADC(MAG_ADDRESS,MAG_DATA_REGISTER);
  #if defined(HMC5843)
    MAG_ORIENTATION( ((rawADC[0]<<8) | rawADC[1]) ,
                     ((rawADC[2]<<8) | rawADC[3]) ,
                     ((rawADC[4]<<8) | rawADC[5]) );
  #endif
  #if defined (HMC5883)  
    MAG_ORIENTATION( ((rawADC[0]<<8) | rawADC[1]) ,
                     ((rawADC[4]<<8) | rawADC[5]) ,
                     ((rawADC[2]<<8) | rawADC[3]) );
  #endif
}

#if !defined(MPU6050_I2C_AUX_MASTER)
void Device_Mag_getADC() {
  getADC();
}
#endif

#endif

// ************************************************************************************************************
// I2C Gyroscope and Accelerometer MPU6050
// ************************************************************************************************************
#if defined(MPU6050)

void Gyro_init() {
  TWBR = ((F_CPU / 400000L) - 16) / 2; // change the I2C clock rate to 400kHz
  i2c_writeReg(MPU6050_ADDRESS, 0x6B, 0x80);             //PWR_MGMT_1    -- DEVICE_RESET 1
  delay(5);
  i2c_writeReg(MPU6050_ADDRESS, 0x6B, 0x03);             //PWR_MGMT_1    -- SLEEP 0; CYCLE 0; TEMP_DIS 0; CLKSEL 3 (PLL with Z Gyro reference)
  i2c_writeReg(MPU6050_ADDRESS, 0x1A, MPU6050_DLPF_CFG); //CONFIG        -- EXT_SYNC_SET 0 (disable input pin for data sync) ; default DLPF_CFG = 0 => ACC bandwidth = 260Hz  GYRO bandwidth = 256Hz)
  i2c_writeReg(MPU6050_ADDRESS, 0x1B, 0x18);             //GYRO_CONFIG   -- FS_SEL = 3: Full scale set to 2000 deg/sec
  // enable I2C bypass for AUX I2C
  #if defined(MAG)
    i2c_writeReg(MPU6050_ADDRESS, 0x37, 0x02);           //INT_PIN_CFG   -- INT_LEVEL=0 ; INT_OPEN=0 ; LATCH_INT_EN=0 ; INT_RD_CLEAR=0 ; FSYNC_INT_LEVEL=0 ; FSYNC_INT_EN=0 ; I2C_BYPASS_EN=1 ; CLKOUT_EN=0
  #endif
}

void Gyro_getADC () {
  i2c_getSixRawADC(MPU6050_ADDRESS, 0x43);
  GYRO_ORIENTATION( ((rawADC[0]<<8) | rawADC[1])>>2 , // range: +/- 8192; +/- 2000 deg/sec
                    ((rawADC[2]<<8) | rawADC[3])>>2 ,
                    ((rawADC[4]<<8) | rawADC[5])>>2 );
  GYRO_Common();
}

void ACC_init () {
  i2c_writeReg(MPU6050_ADDRESS, 0x1C, 0x10);             //ACCEL_CONFIG  -- AFS_SEL=2 (Full Scale = +/-8G)  ; ACCELL_HPF=0   //note something is wrong in the spec.
  //note: something seems to be wrong in the spec here. With AFS=2 1G = 4096 but according to my measurement: 1G=2048 (and 2048/8 = 256)
  //confirmed here: http://www.multiwii.com/forum/viewtopic.php?f=8&t=1080&start=10#p7480
  #if defined(FREEIMUv04)
    acc_1G = 255;
  #else
    acc_1G = 512;
  #endif

  #if defined(MPU6050_I2C_AUX_MASTER)
    //at this stage, the MAG is configured via the original MAG init function in I2C bypass mode
    //now we configure MPU as a I2C Master device to handle the MAG via the I2C AUX port (done here for HMC5883)
    i2c_writeReg(MPU6050_ADDRESS, 0x6A, 0b00100000);       //USER_CTRL     -- DMP_EN=0 ; FIFO_EN=0 ; I2C_MST_EN=1 (I2C master mode) ; I2C_IF_DIS=0 ; FIFO_RESET=0 ; I2C_MST_RESET=0 ; SIG_COND_RESET=0
    i2c_writeReg(MPU6050_ADDRESS, 0x37, 0x00);             //INT_PIN_CFG   -- INT_LEVEL=0 ; INT_OPEN=0 ; LATCH_INT_EN=0 ; INT_RD_CLEAR=0 ; FSYNC_INT_LEVEL=0 ; FSYNC_INT_EN=0 ; I2C_BYPASS_EN=0 ; CLKOUT_EN=0
    i2c_writeReg(MPU6050_ADDRESS, 0x24, 0x0D);             //I2C_MST_CTRL  -- MULT_MST_EN=0 ; WAIT_FOR_ES=0 ; SLV_3_FIFO_EN=0 ; I2C_MST_P_NSR=0 ; I2C_MST_CLK=13 (I2C slave speed bus = 400kHz)
    i2c_writeReg(MPU6050_ADDRESS, 0x25, 0x80|MAG_ADDRESS);//I2C_SLV0_ADDR -- I2C_SLV4_RW=1 (read operation) ; I2C_SLV4_ADDR=MAG_ADDRESS
    i2c_writeReg(MPU6050_ADDRESS, 0x26, MAG_DATA_REGISTER);//I2C_SLV0_REG  -- 6 data bytes of MAG are stored in 6 registers. First register address is MAG_DATA_REGISTER
    i2c_writeReg(MPU6050_ADDRESS, 0x27, 0x86);             //I2C_SLV0_CTRL -- I2C_SLV0_EN=1 ; I2C_SLV0_BYTE_SW=0 ; I2C_SLV0_REG_DIS=0 ; I2C_SLV0_GRP=0 ; I2C_SLV0_LEN=3 (3x2 bytes)
  #endif
}

void ACC_getADC () {
  i2c_getSixRawADC(MPU6050_ADDRESS, 0x3B);
  ACC_ORIENTATION( ((rawADC[0]<<8) | rawADC[1])>>3 ,
                   ((rawADC[2]<<8) | rawADC[3])>>3 ,
                   ((rawADC[4]<<8) | rawADC[5])>>3 );
  ACC_Common();
}

//The MAG acquisition function must be replaced because we now talk to the MPU device
  #if defined(MPU6050_I2C_AUX_MASTER)
    void Device_Mag_getADC() {
      i2c_getSixRawADC(MPU6050_ADDRESS, 0x49);               //0x49 is the first memory room for EXT_SENS_DATA
      #if defined(HMC5843)
        MAG_ORIENTATION( ((rawADC[0]<<8) | rawADC[1]) ,
                         ((rawADC[2]<<8) | rawADC[3]) ,
                         ((rawADC[4]<<8) | rawADC[5]) );
      #endif
      #if defined (HMC5883)  
        MAG_ORIENTATION( ((rawADC[0]<<8) | rawADC[1]) ,
                         ((rawADC[4]<<8) | rawADC[5]) ,
                         ((rawADC[2]<<8) | rawADC[3]) );
      #endif
      #if defined (MAG3110)
        MAG_ORIENTATION( ((rawADC[0]<<8) | rawADC[1]) ,          
                         ((rawADC[2]<<8) | rawADC[3]) ,     
                         ((rawADC[4]<<8) | rawADC[5]) );
      #endif
    }
  #endif
#endif

//*********************************************************************************************************************

#if defined(TINY_GPS) | defined(TINY_GPS_SONAR)
void tinygps_query(void) {
  struct nav_data_t navi;
  int16_t i2c_errors = i2c_errors_count;
  /* copy GPS data to local struct */
  i2c_read_to_buf(TINY_GPS_TWI_ADD, &navi, sizeof(navi));
  /* did we generate any errors? */
  if (i2c_errors == i2c_errors_count) {
    #if defined(TINY_GPS)
    GPS_numSat = navi.gps.sats;
    f.GPS_FIX = (navi.gps.quality > 0);
    GPS_coord[LAT] = (navi.gps.flags & 1<<NMEA_RMC_FLAGS_LAT_NORTH ? 1 : -1) * GPS_coord_to_decimal(&navi.gps.lat);
    GPS_coord[LON] = (navi.gps.flags & 1<<NMEA_RMC_FLAGS_LON_EAST ? 1 : -1) * GPS_coord_to_decimal(&navi.gps.lon);
    GPS_altitude = navi.gps.alt.m;
    #endif

    #if defined(TINY_GPS_SONAR)
    sonarAlt = navi.sonar.distance;
    #endif
  }
}
#endif

//*****************************************************************************************************************

void initSensors() {
  delay(200);
  POWERPIN_ON;
  delay(100);
  i2c_init();
  delay(100);
  if (GYRO) Gyro_init();
  if (BARO) Baro_init();
  if (MAG) Mag_init();
  if (ACC) {ACC_init();acc_25deg = acc_1G * 0.423;}
  //if (SONAR) Sonar_init();
  f.I2C_INIT_DONE = 1;
}
