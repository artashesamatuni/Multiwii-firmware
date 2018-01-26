#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "MultiWii.h"
#include "Alarms.h"
#include "EEPROM.h"
#include "IMU.h"
#include "LCD.h"
#include "Sensors.h"

static void Device_Mag_getADC();
static void Baro_init();
static void Mag_init();
static void ACC_init();

// ************************************************************************************************************
// board orientation and setup
// ************************************************************************************************************
//default board orientation
#if !defined(ACC_ORIENTATION) 
  #define ACC_ORIENTATION(X, Y, Z)  {imu.accADC[ROLL]  = X; imu.accADC[PITCH]  = Y; imu.accADC[YAW]  = Z;}
#endif
#if !defined(GYRO_ORIENTATION) 
  #define GYRO_ORIENTATION(X, Y, Z) {imu.gyroADC[ROLL] = X; imu.gyroADC[PITCH] = Y; imu.gyroADC[YAW] = Z;}
#endif
#if !defined(MAG_ORIENTATION) 
  #define MAG_ORIENTATION(X, Y, Z)  {imu.magADC[ROLL]  = X; imu.magADC[PITCH]  = Y; imu.magADC[YAW]  = Z;}
#endif


//ITG3200 / ITG3205 / ITG3050 / MPU6050 / MPU3050 Gyro LPF setting
#if defined(GYRO_LPF_256HZ) || defined(GYRO_LPF_188HZ) || defined(GYRO_LPF_98HZ) || defined(GYRO_LPF_42HZ) || defined(GYRO_LPF_20HZ) || defined(GYRO_LPF_10HZ) || defined(GYRO_LPF_5HZ)
  #if defined(GYRO_LPF_256HZ)
    #define GYRO_DLPF_CFG   0
  #endif
  #if defined(GYRO_LPF_188HZ)
    #define GYRO_DLPF_CFG   1
  #endif
  #if defined(GYRO_LPF_98HZ)
    #define GYRO_DLPF_CFG   2
  #endif
  #if defined(GYRO_LPF_42HZ)
    #define GYRO_DLPF_CFG   3
  #endif
  #if defined(GYRO_LPF_20HZ)
    #define GYRO_DLPF_CFG   4
  #endif
  #if defined(GYRO_LPF_10HZ)
    #define GYRO_DLPF_CFG   5
  #endif
  #if defined(GYRO_LPF_5HZ)
    #define GYRO_DLPF_CFG   6
  #endif
#else
    #define GYRO_DLPF_CFG   0 //Default settings LPF 256Hz/8000Hz sample
#endif

static uint8_t rawADC[6];
#if defined(WMP)
static uint32_t neutralizeTime = 0;
#endif

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
  TWBR = ((F_CPU / 400000) - 16) / 2;          // set the I2C clock rate to 400kHz
  TWCR = 1<<TWEN;                              // enable twi module, no interrupt
  i2c_errors_count = 0;
}

void __attribute__ ((noinline)) waitTransmissionI2C(uint8_t twcr) {
  TWCR = twcr;
  uint8_t count = 255;
  while (!(TWCR & (1<<TWINT))) {
    count--;
    if (count==0) {              //we are in a blocking state => we don't insist
      TWCR = 0;                  //and we force a reset on TWINT register
      #if defined(WMP)
      neutralizeTime = micros(); //we take a timestamp here to neutralize the value during a short delay
      #endif
      i2c_errors_count++;
      break;
    }
  }
}

void i2c_rep_start(uint8_t address) {
  waitTransmissionI2C((1<<TWINT) | (1<<TWSTA) | (1<<TWEN)); // send REPEAT START condition and wait until transmission completed
  TWDR = address;                                           // send device address
  waitTransmissionI2C((1<<TWINT) | (1<<TWEN));              // wail until transmission completed
}

void i2c_stop(void) {
  TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
  //  while(TWCR & (1<<TWSTO));                // <- can produce a blocking state with some WMP clones
}

void i2c_write(uint8_t data ) {
  TWDR = data;                                 // send data to the previously addressed device
  waitTransmissionI2C((1<<TWINT) | (1<<TWEN));
}

uint8_t i2c_readAck() {
  waitTransmissionI2C((1<<TWINT) | (1<<TWEN) | (1<<TWEA));
  return TWDR;
}

uint8_t i2c_readNak() {
  waitTransmissionI2C((1<<TWINT) | (1<<TWEN));
  uint8_t r = TWDR;
  i2c_stop();
  return r;
}

void i2c_read_reg_to_buf(uint8_t add, uint8_t reg, uint8_t *buf, uint8_t size) {
  i2c_rep_start(add<<1); // I2C write direction
  i2c_write(reg);        // register selection
  i2c_rep_start((add<<1) | 1);  // I2C read direction
  uint8_t *b = buf;
  while (--size) *b++ = i2c_readAck(); // acknowledge all but the final byte
  *b = i2c_readNak();
}

void i2c_getSixRawADC(uint8_t add, uint8_t reg) {
  i2c_read_reg_to_buf(add, reg, rawADC, 6);
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
      if (calibratingG == 512) { // Reset g[axis] at start of calibration
        g[axis]=0;
    #if defined(GYROCALIBRATIONFAILSAFE)
        previousGyroADC[axis] = imu.gyroADC[axis];
      }
      if (calibratingG % 10 == 0) {
        if(abs(imu.gyroADC[axis] - previousGyroADC[axis]) > 8) tilt=1;
        previousGyroADC[axis] = imu.gyroADC[axis];
    #endif
      }
      g[axis] +=imu.gyroADC[axis]; // Sum up 512 readings
      gyroZero[axis]=g[axis]>>9;
      if (calibratingG == 1) {
        SET_ALARM_BUZZER(ALRM_FAC_CONFIRM, ALRM_LVL_CONFIRM_ELSE);
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
    imu.gyroADC[axis]  -= gyroZero[axis];
    mediaMobileGyroADCSum[axis] -= mediaMobileGyroADC[axis][mediaMobileGyroIDX];
    //anti gyro glitch, limit the variation between two consecutive readings
    mediaMobileGyroADC[axis][mediaMobileGyroIDX] = constrain(imu.gyroADC[axis],previousGyroADC[axis]-800,previousGyroADC[axis]+800);
    mediaMobileGyroADCSum[axis] += mediaMobileGyroADC[axis][mediaMobileGyroIDX];
    imu.gyroADC[axis] = mediaMobileGyroADCSum[axis] / conf.mmgyro;
  #else
  for (axis = 0; axis < 3; axis++) {
    imu.gyroADC[axis]  -= gyroZero[axis];
    //anti gyro glitch, limit the variation between two consecutive readings
    imu.gyroADC[axis] = constrain(imu.gyroADC[axis],previousGyroADC[axis]-800,previousGyroADC[axis]+800);
  #endif    
    previousGyroADC[axis] = imu.gyroADC[axis];
  }

  #if defined(SENSORS_TILT_45DEG_LEFT)
    int16_t temp  = ((imu.gyroADC[PITCH] - imu.gyroADC[ROLL] )*7)/10;
    imu.gyroADC[ROLL] = ((imu.gyroADC[ROLL]  + imu.gyroADC[PITCH])*7)/10;
    imu.gyroADC[PITCH]= temp;
  #endif
  #if defined(SENSORS_TILT_45DEG_RIGHT)
    int16_t temp  = ((imu.gyroADC[PITCH] + imu.gyroADC[ROLL] )*7)/10;
    imu.gyroADC[ROLL] = ((imu.gyroADC[ROLL]  - imu.gyroADC[PITCH])*7)/10;
    imu.gyroADC[PITCH]= temp;
  #endif
}

// ****************
// ACC common part
// ****************
void ACC_Common() {
  static int32_t a[3];
  if (calibratingA>0) {
    calibratingA--;
    for (uint8_t axis = 0; axis < 3; axis++) {
      if (calibratingA == 511) a[axis]=0;   // Reset a[axis] at start of calibration
      a[axis] +=imu.accADC[axis];           // Sum up 512 readings
      global_conf.accZero[axis] = a[axis]>>9; // Calculate average, only the last itteration where (calibratingA == 0) is relevant
    }
    if (calibratingA == 0) {
      global_conf.accZero[YAW] -= ACC_1G;   // shift Z down by ACC_1G and store values in EEPROM at end of calibration
      conf.angleTrim[ROLL]   = 0;
      conf.angleTrim[PITCH]  = 0;
      writeGlobalSet(1); // write accZero in EEPROM
    }
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
          b[axis] +=imu.accADC[axis];
          // Clear global variables for next reading
          imu.accADC[axis]=0;
          global_conf.accZero[axis]=0;
        }
        //all values are measured
        if (InflightcalibratingA == 1) {
          AccInflightCalibrationActive = 0;
          AccInflightCalibrationMeasurementDone = 1;
          SET_ALARM_BUZZER(ALRM_FAC_CONFIRM, ALRM_LVL_CONFIRM_1);     //buzzer for indicatiing the end of calibration
          // recover saved values to maintain current flight behavior until new values are transferred
          global_conf.accZero[ROLL]  = accZero_saved[ROLL] ;
          global_conf.accZero[PITCH] = accZero_saved[PITCH];
          global_conf.accZero[YAW]   = accZero_saved[YAW] ;
          conf.angleTrim[ROLL]  = angleTrim_saved[ROLL] ;
          conf.angleTrim[PITCH] = angleTrim_saved[PITCH] ;
        }
        InflightcalibratingA--;
      }
      // Calculate average, shift Z down by ACC_1G and store values in EEPROM at end of calibration
      if (AccInflightCalibrationSavetoEEProm == 1){  //the copter is landed, disarmed and the combo has been done again
        AccInflightCalibrationSavetoEEProm = 0;
        global_conf.accZero[ROLL]  = b[ROLL]/50;
        global_conf.accZero[PITCH] = b[PITCH]/50;
        global_conf.accZero[YAW]   = b[YAW]/50-ACC_1G;
        conf.angleTrim[ROLL]   = 0;
        conf.angleTrim[PITCH]  = 0;
        writeGlobalSet(1); // write accZero in EEPROM
      }
  #endif
  imu.accADC[ROLL]  -=  global_conf.accZero[ROLL] ;
  imu.accADC[PITCH] -=  global_conf.accZero[PITCH];
  imu.accADC[YAW]   -=  global_conf.accZero[YAW] ;

  #if defined(SENSORS_TILT_45DEG_LEFT)
    int16_t temp = ((imu.accADC[PITCH] - imu.accADC[ROLL] )*7)/10;
    imu.accADC[ROLL] = ((imu.accADC[ROLL]  + imu.accADC[PITCH])*7)/10;
    imu.accADC[PITCH] = temp;
  #endif
  #if defined(SENSORS_TILT_45DEG_RIGHT)
    int16_t temp = ((imu.accADC[PITCH] + imu.accADC[ROLL] )*7)/10;
    imu.accADC[ROLL] = ((imu.accADC[ROLL]  - imu.accADC[PITCH])*7)/10;
    imu.accADC[PITCH] = temp;
  #endif
}

// ************************************************************************************************************
// BARO section
// ************************************************************************************************************
#if BARO
static void Baro_Common() {
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
  
  void i2c_BMP085_readCalibration(){
    delay(10);
    //read calibration data in one go
    size_t s_bytes = (uint8_t*)&bmp085_ctx.md - (uint8_t*)&bmp085_ctx.ac1 + sizeof(bmp085_ctx.ac1);
    i2c_read_reg_to_buf(BMP085_ADDRESS, 0xAA, (uint8_t*)&bmp085_ctx.ac1, s_bytes);
    // now fix endianness
    int16_t *p;
    for (p = &bmp085_ctx.ac1; p <= &bmp085_ctx.md; p++) {
      swap_endianness(p, sizeof(*p));
    }
  }
  
  // read uncompensated temperature value: send command first
  void i2c_BMP085_UT_Start(void) {
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
  }
  
  void  Baro_init() {
    delay(10);
    i2c_BMP085_readCalibration();
    delay(5);
    i2c_BMP085_UT_Start(); 
    bmp085_ctx.deadline = currentTime+5000;
  }
  
  //return 0: no data available, no computation ;  1: new value available  ; 2: no new value, but computation time
  uint8_t Baro_update() {                   // first UT conversion is started in init procedure
    if (currentTime < bmp085_ctx.deadline) return 0; 
    bmp085_ctx.deadline = currentTime+6000; // 1.5ms margin according to the spec (4.5ms T convetion time)
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
  #if !defined(ADXL345_ADDRESS) 
    #define ADXL345_ADDRESS 0x1D
    //#define ADXL345_ADDRESS 0x53   //WARNING: Conflicts with a Wii Motion plus!
  #endif
  
  void ACC_init () {
    delay(10);
    i2c_writeReg(ADXL345_ADDRESS,0x2D,1<<3); //  register: Power CTRL  -- value: Set measure bit 3 on
    i2c_writeReg(ADXL345_ADDRESS,0x31,0x0B); //  register: DATA_FORMAT -- value: Set bits 3(full range) and 1 0 on (+/- 16g-range)
    i2c_writeReg(ADXL345_ADDRESS,0x2C,0x09); //  register: BW_RATE     -- value: rate=50hz, bw=20hz
  }
  
  void ACC_getADC () {
    i2c_getSixRawADC(ADXL345_ADDRESS,0x32);
  
    ACC_ORIENTATION( ((rawADC[1]<<8) | rawADC[0]) ,
                     ((rawADC[3]<<8) | rawADC[2]) ,
                     ((rawADC[5]<<8) | rawADC[4]) );
    ACC_Common();
  }
  #endif


// ************************************************************************************************************
// ADC ACC
// ************************************************************************************************************

  #if defined(ADCACC)
  void ACC_init(){
    pinMode(A1,INPUT);
    pinMode(A2,INPUT);
    pinMode(A3,INPUT);
  }
  
  void ACC_getADC() {
    ACC_ORIENTATION(  analogRead(A1) ,
                      analogRead(A2) ,
                      analogRead(A3) );
    ACC_Common();
  }
#endif


// ************************************************************************************************************
// I2C Gyroscope L3G4200D 
// ************************************************************************************************************

  #if defined(L3G4200D)
  #define L3G4200D_ADDRESS 0x69
  void Gyro_init() {
    delay(100);
    i2c_writeReg(L3G4200D_ADDRESS ,0x20 ,0x8F ); // CTRL_REG1   400Hz ODR, 20hz filter, run!
    delay(5);
    i2c_writeReg(L3G4200D_ADDRESS ,0x24 ,0x02 ); // CTRL_REG5   low pass filter enable
    delay(5);
    i2c_writeReg(L3G4200D_ADDRESS ,0x23 ,0x30); // CTRL_REG4 Select 2000dps
  }

  void Gyro_getADC () {
    i2c_getSixRawADC(L3G4200D_ADDRESS,0x80|0x28);
  
    GYRO_ORIENTATION( ((rawADC[1]<<8) | rawADC[0])>>2  ,
                      ((rawADC[3]<<8) | rawADC[2])>>2  ,
                      ((rawADC[5]<<8) | rawADC[4])>>2  );
    GYRO_Common();
  }
  #endif


// ************************************************************************************************************
// I2C Compass common function
// ************************************************************************************************************

#if MAG
static float magGain[3] = {1.0,1.0,1.0};  // gain for each axis, populated at sensor init

uint8_t Mag_getADC() { // return 1 when news values are available, 0 otherwise
  static uint32_t t,tCal = 0;
  static int16_t magZeroTempMin[3],magZeroTempMax[3];
  uint8_t axis;

  if ( currentTime < t ) return 0; //each read is spaced by 100ms
  t = currentTime + 100000;
  Device_Mag_getADC();

  for(axis=0;axis<3;axis++) {
    imu.magADC[axis]  = imu.magADC[axis]  * magGain[axis];
    if (!f.CALIBRATE_MAG) imu.magADC[axis]  -= global_conf.magZero[axis];
  }
  
  if (f.CALIBRATE_MAG) {
    if (tCal == 0) // init mag calibration
      tCal = t;
    if ((t - tCal) < 30000000) { // 30s: you have 30s to turn the multi in all directions
      LEDPIN_TOGGLE;
      for(axis=0;axis<3;axis++) {
        if(tCal == t) { // it happens only in the first step, initialize the zero
          magZeroTempMin[axis] = imu.magADC[axis];
          magZeroTempMax[axis] = imu.magADC[axis];
        }
        if (imu.magADC[axis] < magZeroTempMin[axis]) {magZeroTempMin[axis] = imu.magADC[axis]; SET_ALARM(ALRM_FAC_TOGGLE, ALRM_LVL_TOGGLE_1);}
        if (imu.magADC[axis] > magZeroTempMax[axis]) {magZeroTempMax[axis] = imu.magADC[axis]; SET_ALARM(ALRM_FAC_TOGGLE, ALRM_LVL_TOGGLE_1);}
        global_conf.magZero[axis] = (magZeroTempMin[axis] + magZeroTempMax[axis])>>1;
      }
    } else {
      f.CALIBRATE_MAG = 0;
      tCal = 0;
      writeGlobalSet(1);
    }
  }

  #if defined(SENSORS_TILT_45DEG_LEFT)
    int16_t temp = ((imu.magADC[PITCH] - imu.magADC[ROLL] )*7)/10;
    imu.magADC[ROLL] = ((imu.magADC[ROLL]  + imu.magADC[PITCH])*7)/10;
    imu.magADC[PITCH] = temp;
  #endif
  #if defined(SENSORS_TILT_45DEG_RIGHT)
    int16_t temp = ((imu.magADC[PITCH] + imu.magADC[ROLL] )*7)/10;
    imu.magADC[ROLL] = ((imu.magADC[ROLL]  - imu.magADC[PITCH])*7)/10;
    imu.magADC[PITCH] = temp;
  #endif
  
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

  static int32_t xyz_total[3]={0,0,0};  // 32 bit totals so they won't overflow.
  
  static void getADC() {
    i2c_getSixRawADC(MAG_ADDRESS,MAG_DATA_REGISTER);
    MAG_ORIENTATION( ((rawADC[0]<<8) | rawADC[1]) ,
                     ((rawADC[4]<<8) | rawADC[5]) ,
                     ((rawADC[2]<<8) | rawADC[3]) );
  }

  static uint8_t bias_collect(uint8_t bias) {
    int16_t abs_magADC;
  
    i2c_writeReg(MAG_ADDRESS, HMC58X3_R_CONFA, bias);            // Reg A DOR=0x010 + MS1,MS0 set to pos or negative bias
    for (uint8_t i=0; i<10; i++) {                               // Collect 10 samples
      i2c_writeReg(MAG_ADDRESS,HMC58X3_R_MODE, 1);
      delay(100);
      getADC();                                                  // Get the raw values in case the scales have already been changed.
      for (uint8_t axis=0; axis<3; axis++) {
        abs_magADC =  abs(imu.magADC[axis]);
        xyz_total[axis]+= abs_magADC;                            // Since the measurements are noisy, they should be averaged rather than taking the max.
        if ((int16_t)(1<<12) < abs_magADC) return false;         // Detect saturation.   if false Breaks out of the for loop.  No sense in continuing if we saturated.
      }
    }
    return true;
  }

  static void Mag_init() {
    bool bret=true;                // Error indicator
  
    // Note that the  very first measurement after a gain change maintains the same gain as the previous setting. 
    // The new gain setting is effective from the second measurement and on.
    i2c_writeReg(MAG_ADDRESS, HMC58X3_R_CONFB, 2 << 5);  //Set the Gain
    i2c_writeReg(MAG_ADDRESS,HMC58X3_R_MODE, 1);
    delay(100);
    getADC();  //Get one sample, and discard it
  
    if (!bias_collect(0x010 + HMC_POS_BIAS)) bret = false;
    if (!bias_collect(0x010 + HMC_NEG_BIAS)) bret = false;
  
    if (bret) // only if no saturation detected, compute the gain. otherwise, the default 1.0 is used
      for (uint8_t axis=0; axis<3; axis++)
        magGain[axis]=820.0*HMC58X3_X_SELF_TEST_GAUSS*2.0*10.0/xyz_total[axis];  // note: xyz_total[axis] is always positive
  
    // leave test mode
    i2c_writeReg(MAG_ADDRESS ,HMC58X3_R_CONFA ,0x70 ); //Configuration Register A  -- 0 11 100 00  num samples: 8 ; output rate: 15Hz ; normal measurement mode
    i2c_writeReg(MAG_ADDRESS ,HMC58X3_R_CONFB ,0x20 ); //Configuration Register B  -- 001 00000    configuration gain 1.3Ga
    i2c_writeReg(MAG_ADDRESS ,HMC58X3_R_MODE  ,0x00 ); //Mode register             -- 000000 00    continuous Conversion Mode
    delay(100);
  }

  #if !defined(MPU6050_I2C_AUX_MASTER)
    static void Device_Mag_getADC() {
      getADC();
    }
  #endif
#endif

// ************************************************************************************************************
// I2C Sonar SRF08
// ************************************************************************************************************
// first contribution from guru_florida (02-25-2012)
// ************************************************************************************************************

  #if defined(SRF02) || defined(SRF08) || defined(SRF10) || defined(SRC235)
  
  // the default address for any new sensor found on the bus
  // the code will move new sonars to the next available sonar address in range of F0-FE so that another
  // sonar sensor can be added again.
  // Thus, add only 1 sonar sensor at a time, poweroff, then wire the next, power on, wait for flashing light and repeat
  #if !defined(SRF08_DEFAULT_ADDRESS) 
    #define SRF08_DEFAULT_ADDRESS (0xE0>>1)
  #endif
  
  #if !defined(SRF08_RANGE_WAIT) 
    #define SRF08_RANGE_WAIT      70000      // delay between Ping and Range Read commands (65ms is safe in any case)
  #endif
  
  #if !defined(SRF08_RANGE_SLEEP) 
    #define SRF08_RANGE_SLEEP     5000       // sleep this long before starting another Ping
  #endif
  
  #if !defined(SRF08_SENSOR_FIRST) 
    #define SRF08_SENSOR_FIRST    (0xF0>>1)  // the first sensor i2c address (after it has been moved)
  #endif
  
  #if !defined(SRF08_MAX_SENSORS) 
    #define SRF08_MAX_SENSORS     4          // maximum number of sensors we'll allow (can go up to 8)
  #endif
  
  //#define SONAR_MULTICAST_PING
  
  // registers of the device
  #define SRF08_REV_COMMAND    0
  #define SRF08_LIGHT_GAIN     1
  #define SRF08_ECHO_RANGE     2
  
  
  static struct {
    // sensor registers from the MS561101BA datasheet
    int32_t  range[SRF08_MAX_SENSORS];
    int8_t   sensors;              // the number of sensors present
    int8_t   current;              // the current sensor being read
    uint8_t  state;
    uint32_t deadline;
  } srf08_ctx;
  
  
  // read uncompensated temperature value: send command first
  void Sonar_init() {
    memset(&srf08_ctx, 0, sizeof(srf08_ctx));
    srf08_ctx.deadline = 4000000;
  }
  
  // this function works like readReg accept a failed read is a normal expectation
  // use for testing the existence of sensors on the i2c bus
  // a 0xffff code is returned if the read failed
  uint16_t i2c_try_readReg(uint8_t add, uint8_t reg) {
    uint16_t count = 255;
    i2c_rep_start(add<<1);  // I2C write direction
    i2c_write(reg);        // register selection
    i2c_rep_start((add<<1)|1);  // I2C read direction
    TWCR = (1<<TWINT) | (1<<TWEN);
    while (!(TWCR & (1<<TWINT))) {
      count--;
      if (count==0) {              //we are in a blocking state => we don't insist
        TWCR = 0;                  //and we force a reset on TWINT register
        return 0xffff;  // return failure to read
      }
    }
    uint8_t r = TWDR;
    i2c_stop();
    return r;  
  }
  
  // read a 16bit unsigned int from the i2c bus
  uint16_t i2c_readReg16(int8_t addr, int8_t reg) {
    uint8_t b[2];
    i2c_read_reg_to_buf(addr, reg, (uint8_t*)&b, sizeof(b));
    return (b[0]<<8) | b[1];
  }
  
  void i2c_srf08_change_addr(int8_t current, int8_t moveto) {
    // to change a srf08 address, we must write the following sequence to the command register
    // this sequence must occur as 4 seperate i2c transactions!!   A0 AA A5 [addr]
    i2c_writeReg(current, SRF08_REV_COMMAND, 0xA0);    delay(30);
    i2c_writeReg(current, SRF08_REV_COMMAND, 0xAA);    delay(30);
    i2c_writeReg(current, SRF08_REV_COMMAND, 0xA5);    delay(30);
    i2c_writeReg(current, SRF08_REV_COMMAND, moveto);  delay(30); // now change i2c address
  }
  
  // discover previously known sensors and any new sensor (move new sensors to assigned area)
  void i2c_srf08_discover() {
    uint8_t addr;
    uint16_t x;
  
    srf08_ctx.sensors=0;                                     // determine how many sensors are plugged in
    addr = SRF08_SENSOR_FIRST;                               // using the I2C address range we choose, starting with first one
    for(uint8_t i=0; i<SRF08_MAX_SENSORS && x!=0xff; i++) {  // 0xff means a mesure is currently running, so try again
      x = i2c_try_readReg(addr, SRF08_REV_COMMAND);          // read the revision as a way to check if sensor exists at this location
      if(x!=0xffff) {                                        // detected a sensor at this address
        i2c_writeReg(addr, SRF08_LIGHT_GAIN, 0x15);          // not set to max to avoid bad echo indoor
        i2c_writeReg(addr, SRF08_ECHO_RANGE, 46);            // set to 2m max
        srf08_ctx.sensors++;
        addr += 1;                                           // 7 bit address => +1 is +2 for 8 bit address
      }
    }
    if(srf08_ctx.sensors < SRF08_MAX_SENSORS) {                 // do not add sensors if we are already maxed
      // now determine if any sensor is on the 'new sensor' address (srf08 default address)
      x = i2c_try_readReg(SRF08_DEFAULT_ADDRESS, SRF08_REV_COMMAND); // we try to read the revision number
      if(x!=0xffff) {                                           // new sensor detected at SRF08 default address
        i2c_srf08_change_addr(SRF08_DEFAULT_ADDRESS, addr<<1);  // move sensor to the next address (8 bit format expected by the device)
        srf08_ctx.sensors++;
      }
    }
  }
  
  void Sonar_update() {
    if ((int32_t)(currentTime - srf08_ctx.deadline)<0) return;
    srf08_ctx.deadline = currentTime;
    switch (srf08_ctx.state) {
      case 0: 
        i2c_srf08_discover();
        if(srf08_ctx.sensors>0) srf08_ctx.state++; 
        else                    srf08_ctx.deadline += 5000000; // wait 5 secs before trying search again
        break;
      case 1: 
        srf08_ctx.current=0;
        srf08_ctx.state++;
        srf08_ctx.deadline += SRF08_RANGE_SLEEP;
        break;
      #if defined(SONAR_MULTICAST_PING)
      case 2:
        // send a ping via the general broadcast address
        i2c_writeReg(0, SRF08_REV_COMMAND, 0x51);  // start ranging, result in centimeters
        srf08_ctx.state++;
        srf08_ctx.deadline += SRF08_RANGE_WAIT;
        break;
      case 3: 
        srf08_ctx.range[srf08_ctx.current] = i2c_readReg16( SRF08_SENSOR_FIRST+srf08_ctx.current, SRF08_ECHO_RANGE);
        srf08_ctx.current++;
        if(srf08_ctx.current >= srf08_ctx.sensors) srf08_ctx.state=1;
        break;
      #else
      case 2:
        // send a ping to the current sensor
        i2c_writeReg(SRF08_SENSOR_FIRST+srf08_ctx.current, SRF08_REV_COMMAND, 0x51);  // start ranging, result in centimeters
        srf08_ctx.state++;
        srf08_ctx.deadline += SRF08_RANGE_WAIT;
        break;
      case 3: 
        srf08_ctx.range[srf08_ctx.current] = i2c_readReg16(SRF08_SENSOR_FIRST+srf08_ctx.current, SRF08_ECHO_RANGE);
        srf08_ctx.current++;
        if(srf08_ctx.current >= srf08_ctx.sensors) srf08_ctx.state=1;
        else                                       srf08_ctx.state=2; 
        break;
      #endif
    } 
    sonarAlt = srf08_ctx.range[0]; // only one sensor considered for the moment
  }
  #else
  inline void Sonar_init() {}
  void Sonar_update() {}
  #endif


void initS() {
  i2c_init();
  if (GYRO)  Gyro_init();
  if (BARO)  Baro_init();
  if (MAG)   Mag_init();
  if (ACC)   ACC_init();
  if (SONAR) Sonar_init();
}

void initSensors() {
  uint8_t c = 5;
  #if !defined(DISABLE_POWER_PIN)
    POWERPIN_ON;
    delay(200);
  #endif
  while(c) { // We try several times to init all sensors without any i2c errors. An I2C error at this stage might results in a wrong sensor settings
    c--;
    initS();
    if (i2c_errors_count == 0) break; // no error during init => init ok
  }
}
