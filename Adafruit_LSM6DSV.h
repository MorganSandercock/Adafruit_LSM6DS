/*!
 *  @file Adafruit_LSM6DSV.h
 *
 * 	I2C Driver base for Adafruit LSM6DSVxx Dual-Channel Accelerometer and Gyroscope
 *      library
 *
 *    LSM6DSV - "dual channel" allows a 16G accelerometer scale to appear on the "high G" outputs
 *    LSM6DSV16 - "dual channel" is always 16G, adds Qvar electrostatic inputs (not implemented)
 *
 *  Sub-class for chip with different chip ID
 *    LSM6DSV16B -  replaces second I2C interface with TDM audio interface (not implemented)
 *                  NOTE: for your PCB layout, INT2 is moved to pin 10 instead of pin 9.
 *
 *  Sub-class for chip with different max range:  
 *    LSM6DSV32 - "dual channel" is always 32G
 *
 *  Separate .h for true dual accelerometer chips:
 *    LSM6DSV320 9-DoF 16G Accelerometer, 4000dps Gyroscope and 320G "high-G" Accelerometer
 *    LSM6DSV80 9-DoF 16G Accelerometer, 4000dps Gyroscope and 80G "high-G" Accelerometer
 *
 *  The register map on these chips with dual accelerometers is quite different to the 
 *  LSM6DSO series chips. All chips except the V80 and V16B have OIS abilities, which isn't supported
 *  by this library. 
 *
 *  Chips without a true high-g accelerometer return 0x70 for the who-am-I.
 *  The "B" TDM chip who-am-I is 0x71
 *  The 80 and 320 who-am-I is 0x73
 *
 *  Note on units used: 
 *      Acceleration is always meters/sec/sec
 *      Gyro is usually degrees/sec except when using the "event" sensor objects, where radians/sec is used
 *
 *  There are multiple ways of getting the data from the IMU:
 *      1. Call _read() or _readFast() then access the raw or scaled public values *RECOMMENDED*
 *      2. Call getEvent(sensor, sensor, sensor, sensor) to fill in Adafruit sensor objects
 *      3. Call read...(x, y, z) to get 12 bytes of XYZ values direct from the chip and scale to 
 *         floating-point output units. "Arduino API" *GOOD FOR JUST ONE SENSOR OF INTEREST*
 *      4. Call get...Sensor() to get a pointer to just one sensor object inside the class. 
 *         Then you can call .getEvent() on each sensor. Each getEvent does a full 26-byte data read on the chip.
 *         .getSensor() on a sensor will show the sensor details such as max/min but that is erased by a .getEvent()
 *
 *
 * 	Adafruit invests time and resources providing this open source code,
 *      please support Adafruit and open-source hardware by purchasing products
 *      from Adafruit!
 *
 *	BSD license (see license.txt)
 */

#ifndef _ADAFRUIT_LSM6DSV_H
#define _ADAFRUIT_LSM6DSV_H

#include "Arduino.h"
#include <Adafruit_BusIO_Register.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define LSM6DSV_I2CADDR_DEFAULT 0x6A ///< LSM6DSxxx default i2c address (address pin grounded)
#define LSM6DSV_CHIP_ID 0x70         ///< LSM6DSV default device id from WHOAMI register 0x0F
#define LSM6DSV16B_CHIP_ID 0x71      ///< LSM6DSV16B default device id from WHOAMI register 0x0F
 
#define LSM6DSV_FUNC_CFG_ACCESS 0x1  ///< Enable embedded functions register
#define LSM6DSV_PIN_CTRL 0x02        ///< Output pin control (interrupt drive strength)
#define LSM6DSV_IF_CFG 0x03          ///< Interface config
#define LSM6DSV_INT1_CTRL 0x0D       ///< Interrupt control for INT 1
#define LSM6DSV_INT2_CTRL 0x0E       ///< Interrupt control for INT 2
#define LSM6DSV_WHOAMI 0x0F          ///< Chip ID register
#define LSM6DSV_CTRL1 0x10           ///< Accelerometer mode and data rate
#define LSM6DSV_CTRL2 0x11           ///< Gyro mode and data rate
#define LSM6DSV_CTRL3 0x12           ///< Not used: software reset, reboot, block-update mode, block-read mode
#define LSM6DSV_CTRL6 0x15           ///< location of the gyro control register for LPF+range
#define LSM6DSV_CTRL7 0x16           ///< location of the interrupt-enable for high-G and Gyro LPF enable
#define LSM6DSV_CTRL8 0x17           ///< Accelerometer range and low/high pass
#define LSM6DSV_CTRL9 0x18           ///< Accelerometer filters enable
#define LSM6DSV_CTRL10 0x19          ///< debug/self-test
#define LSM6DSV_STATUS_REG 0X1E      ///< Status register (high-G+tepmerature+accel+gyro data-ready)
#define LSM6DSV_OUT_TEMP_L 0x20      ///< First data register (temperature low)
#define LSM6DSV_OUTX_L_G 0x22        ///< First gyro data register
#define LSM6DSV_OUTX_L_A 0x28        ///< First accel data register
#define LSM6DSV_OUTX_L_HG 0x34       ///< First high-g accel data register
#define LSM6DSV_INTERNAL_FREQ 0X4F   ///< Adjust internal clock frequency +/- in steps of 0.13%
#define LSM6DSV_MD1_CFG 0x5E         ///< Functions routing on INT1 register
#define LSM6DSV_X_OFS_USR 0x73       ///< Accelerometer user offset
#define LSM6DSV_Y_OFS_USR 0x74       ///< Accelerometer user offset
#define LSM6DSV_Z_OFS_USR 0x75       ///< Accelerometer user offset

/** The accel or gyro data rate - note the gyro can't do 1.875Hz */
typedef enum data_rate {
  LSM6DSV_RATE_SHUTDOWN,
  LSM6DSV_RATE_1_875_HZ,
  LSM6DSV_RATE_7_5_HZ,
  LSM6DSV_RATE_15_HZ,
  LSM6DSV_RATE_30_HZ,
  LSM6DSV_RATE_60_HZ,
  LSM6DSV_RATE_120_HZ,
  LSM6DSV_RATE_240_HZ,
  LSM6DSV_RATE_480_HZ,
  LSM6DSV_RATE_960_HZ,
  LSM6DSV_RATE_1_92K_HZ,
  LSM6DSV_RATE_3_84K_HZ,
  LSM6DSV_RATE_7_68K_HZ
} lsm6dsv_data_rate_t;


/** The accelerometer data range - identical for all except DSV32*/
typedef enum accel_range {
  LSM6DSV_ACCEL_RANGE_2_G,
  LSM6DSV_ACCEL_RANGE_4_G,
  LSM6DSV_ACCEL_RANGE_8_G,
  LSM6DSV_ACCEL_RANGE_16_G
} lsm6dsv_accel_range_t;
#define LSM6DSV32_ACCEL_RANGE_4_G LSM6DSV_ACCEL_RANGE_2_G
#define LSM6DSV32_ACCEL_RANGE_8_G LSM6DSV_ACCEL_RANGE_4_G
#define LSM6DSV32_ACCEL_RANGE_16_G LSM6DSV_ACCEL_RANGE_8_G
#define LSM6DSV32_ACCEL_RANGE_32_G LSM6DSV_ACCEL_RANGE_16_G

/** The accelerometer operating modes*/
typedef enum accel_op_mode {
  LSM6DSV_ACCEL_MODE_HIGH_PERF, //default
  LSM6DSV_ACCEL_MODE_HIGH_ACC,  //not recommended as it requires other settings for HAODR mode
  LSM6DSV_ACCEL_MODE_RESERVED,
  LSM6DSV_ACCEL_MODE_ODR_TRIG, //External trigger on INT2
  LSM6DSV_ACCEL_MODE_LP_1,     //low power, allows speeds below 7.5Hz, some restrictions
  LSM6DSV_ACCEL_MODE_LP_2,
  LSM6DSV_ACCEL_MODE_LP_3,
  LSM6DSV_ACCEL_MODE_NORMAL    //data rate 7.5Hz to 1.92kHz, use high-performance for faster, low-power for slower
} lsm6dsv_accel_op_mode_t;

/** The gyro full-scale range */
typedef enum gyro_range {
  LSM6DSV_GYRO_RANGE_RESERVED,
  LSM6DSV_GYRO_RANGE_250_DPS,
  LSM6DSV_GYRO_RANGE_500_DPS,
  LSM6DSV_GYRO_RANGE_1000_DPS,
  LSM6DSV_GYRO_RANGE_2000_DPS,
  LSM6DSV_GYRO_RANGE_4000_DPS
} lsm6dsv_gyro_range_t;

/** The high pass and low pass filter bandwidth */
typedef enum hpf_range {
  LSM6DSV_FILTER_ODR_DIV_4,
  LSM6DSV_FILTER_ODR_DIV_10,
  LSM6DSV_FILTER_ODR_DIV_20,
  LSM6DSV_FILTER_ODR_DIV_45,
  LSM6DSV_FILTER_ODR_DIV_100,
  LSM6DSV_FILTER_ODR_DIV_200,
  LSM6DSV_FILTER_ODR_DIV_400,
  LSM6DSV_FILTER_ODR_DIV_800
} lsm6dsv_filter_t;

class Adafruit_LSM6DSV;

/** Adafruit Unified Sensor interface for temperature component of LSM6DS */
class Adafruit_LSM6DSV_Temp : public Adafruit_Sensor {
public:
  /** @brief Create an Adafruit_Sensor compatible object for the temp sensor
      @param parent A pointer to the LSM6DSV class */
  Adafruit_LSM6DSV_Temp(Adafruit_LSM6DSV *parent) { _theLSM6DSV = parent; }
  bool getEvent(sensors_event_t *);
  void getSensor(sensor_t *);

private:
  int _sensorID = 0x6D0; //placeholder for unique sensor ID to be assigned later
  Adafruit_LSM6DSV *_theLSM6DSV = NULL;
};

/** Adafruit Unified Sensor interface for accelerometer component of LSM6DSV */
class Adafruit_LSM6DSV_Accelerometer : public Adafruit_Sensor {
public:
  /** @brief Create an Adafruit_Sensor compatible object for the accelerometer
     sensor
      @param parent A pointer to the LSM6DSV class */
  Adafruit_LSM6DSV_Accelerometer(Adafruit_LSM6DSV *parent) {
    _theLSM6DSV = parent;
  }
  bool getEvent(sensors_event_t *);
  void getSensor(sensor_t *);

private:
  int _sensorID = 0x6D1;
  Adafruit_LSM6DSV *_theLSM6DSV = NULL;
};

/** Adafruit Unified Sensor interface for gyro component of LSM6DSV */
class Adafruit_LSM6DSV_Gyro : public Adafruit_Sensor {
public:
  /** @brief Create an Adafruit_Sensor compatible object for the gyro sensor
      @param parent A pointer to the LSM6DS class */
  Adafruit_LSM6DSV_Gyro(Adafruit_LSM6DSV *parent) { _theLSM6DSV = parent; }
  bool getEvent(sensors_event_t *);
  void getSensor(sensor_t *);

private:
  int _sensorID = 0x6D2;
  Adafruit_LSM6DSV *_theLSM6DSV = NULL;
};

/** Adafruit Unified Sensor interface for high-g component of LSM6DSV */
class Adafruit_LSM6DSV_HighG : public Adafruit_Sensor {
public:
  /** @brief Create an Adafruit_Sensor compatible object for the accelerometer
     sensor
      @param parent A pointer to the LSM6DSV class */
  Adafruit_LSM6DSV_HighG(Adafruit_LSM6DSV *parent) {
    _theLSM6DSV = parent;
  }
  bool getEvent(sensors_event_t *);
  void getSensor(sensor_t *);

private:
  int _sensorID = 0x6D3;
  Adafruit_LSM6DSV *_theLSM6DSV = NULL;
};

/*!
 *    @brief  Class that stores state and functions for interacting with
 *            the LSM6DSV I2C Accel/Gyro/accel
 */
class Adafruit_LSM6DSV {
public:
  Adafruit_LSM6DSV();
  virtual ~Adafruit_LSM6DSV();

  bool begin_I2C(uint8_t i2c_addr = LSM6DSV_I2CADDR_DEFAULT,
                 TwoWire *wire = &Wire, int32_t sensorID = 0);

  bool begin_SPI(uint8_t cs_pin, SPIClass *theSPI = &SPI, int32_t sensorID = 0,
                 uint32_t frequency = 1000000);
  bool begin_SPI(int8_t cs_pin, int8_t sck_pin, int8_t miso_pin,
                 int8_t mosi_pin, int32_t sensorID = 0,
                 uint32_t frequency = 1000000);

  bool getEvent(sensors_event_t *accel, 
	            sensors_event_t *gyro);
  bool getEvent(sensors_event_t *accel, 
	            sensors_event_t *gyro,
                sensors_event_t *temp);
  bool getEvent(sensors_event_t *accel, 
	            sensors_event_t *gyro,
                sensors_event_t *temp,
                sensors_event_t *highG);

  lsm6dsv_data_rate_t getAccelDataRate(void);
  void setAccelDataRate(lsm6dsv_data_rate_t data_rate);

  lsm6dsv_accel_range_t getAccelRange(void);
  void setAccelRange(lsm6dsv_accel_range_t new_range);

  lsm6dsv_data_rate_t getGyroDataRate(void);
  void setGyroDataRate(lsm6dsv_data_rate_t data_rate);

  lsm6dsv_gyro_range_t getGyroRange(void);
  void setGyroRange(lsm6dsv_gyro_range_t new_range);

  lsm6dsv_data_rate_t getHighGDataRate(void);
  void setHighGDataRate(lsm6dsv_data_rate_t data_rate);

  lsm6dsv_accel_range_t getHighGRange(void) {return LSM6DSV_ACCEL_RANGE_16_G;};
  void setHighGRange(lsm6dsv_accel_range_t new_range) {};

  int8_t getInternalFreqFine(void);

  void reset(void);
  void configIntOutputs(bool active_low, bool open_drain, uint8_t drive_strength = 0b11);
  void configInt1(bool drdy_g, bool drdy_xl);
  void configInt2(bool drdy_g, bool drdy_xl);
  virtual void enableHighG(bool en);
  void accelHighPassFilter(bool enabled, lsm6dsv_filter_t filter);
  void accelLowPassFilter(bool enabled, lsm6dsv_filter_t filter);
  void gyroLowPassFilter(bool enabled, uint8_t filter);
  //high-G accelerometer only has a low-pass filter and it's not configurable
  void accelOffset(bool enabled, 
			bool weight = false, 
			int8_t offsetX = 0, 
			int8_t offsetY = 0, 
			int8_t offsetZ = 0);

  /* wakeup and pedometer functions not implemented yet*/
  //void enableWakeup(bool enable, uint8_t duration = 0, uint8_t thresh = 20);
  //bool awake(void);
  //bool shake(void);

  //void enablePedometer(bool enable);
  //void resetPedometer(void);
  //uint16_t readPedometer(void);

  // Arduino compatible API
  int readAcceleration(float &x, float &y, float &z);
  float accelerationSampleRate(void);
  int accelerationAvailable(void);

  int readGyroscope(float &x, float &y, float &z);
  float gyroscopeSampleRate(void);
  int gyroscopeAvailable(void);

  int readHighG(float &x, float &y, float &z);
  virtual float highGSampleRate(void);
  virtual int highGAvailable(void);

  int readTemperature(float &tempC);
  float temperatureSampleRate(void);
  int temperatureAvailable(void);

  int16_t rawAccX, ///< Last reading's raw accelerometer X axis
      rawAccY,     ///< Last reading's raw accelerometer Y axis
      rawAccZ,     ///< Last reading's raw accelerometer Z axis
      rawTemp,     ///< Last reading's raw temperature reading
      rawGyroX,    ///< Last reading's raw gyro X axis
      rawGyroY,    ///< Last reading's raw gyro Y axis
      rawGyroZ,    ///< Last reading's raw gyro Z axis
      rawHighGX,    ///< Last reading's raw high-g X axis
      rawHighGY,    ///< Last reading's raw high-g Y axis
      rawHighGZ;    ///< Last reading's raw high-g Z axis

  float temperature, ///< Last reading's temperature (C)
      accX,          ///< Last reading's accelerometer X axis m/s^2
      accY,          ///< Last reading's accelerometer Y axis m/s^2
      accZ,          ///< Last reading's accelerometer Z axis m/s^2
      gyroX,         ///< Last reading's gyro X axis in deg/s
      gyroY,         ///< Last reading's gyro Y axis in deg/s
      gyroZ,         ///< Last reading's gyro Z axis in deg/s
      highGX,        ///< Last reading's accelerometer X axis m/s^2
      highGY,        ///< Last reading's accelerometer Y axis m/s^2
      highGZ;        ///< Last reading's accelerometer Z axis m/s^2

  Adafruit_Sensor *getTemperatureSensor(void);
  Adafruit_Sensor *getAccelerometerSensor(void);
  Adafruit_Sensor *getGyroSensor(void);
  Adafruit_Sensor *getHighGSensor(void);

  virtual void _read(void);
  virtual void _readFast(void);

protected:
  uint8_t chipID(void);
  uint8_t status(void);
  virtual bool _init(int32_t sensor_id);

  uint16_t _sensorid_accel, ///< ID number for accelerometer
      _sensorid_gyro,       ///< ID number for gyro
      _sensorid_temp,       ///< ID number for temperature
      _sensorid_high_g;     ///< ID number for high-g accelerometer

  Adafruit_I2CDevice *i2c_dev = NULL; ///< Pointer to I2C bus interface
  Adafruit_SPIDevice *spi_dev = NULL; ///< Pointer to SPI bus interface

  Adafruit_LSM6DSV_Temp *temp_sensor = NULL;           ///< Temp sensor data object
  Adafruit_LSM6DSV_Accelerometer *accel_sensor = NULL; ///< Accelerometer data object
  Adafruit_LSM6DSV_Gyro *gyro_sensor = NULL;           ///< Gyro data object
  Adafruit_LSM6DSV_HighG *high_g_sensor = NULL;        ///< high-g data object 

  //! buffer for the accelerometer range
  lsm6dsv_accel_range_t accelRangeBuffered = LSM6DSV_ACCEL_RANGE_2_G;
  //! buffer for the gyroscope range
  lsm6dsv_gyro_range_t gyroRangeBuffered = LSM6DSV_GYRO_RANGE_2000_DPS;
  //! buffer for the high-g accelerometer range
  lsm6dsv_accel_range_t accelHighGBuffered = LSM6DSV_ACCEL_RANGE_16_G;

  //store scale factor for each sensor instead of re-calculating it on every read
  //The magic numbers come from the datasheet, as milli-gees/sec or milli-degrees/sec
  float accelScale = 0.122 * SENSORS_GRAVITY_EARTH / 1000;        //convert raw to meters/sec/sec
  float gyroScale = 70.0 * SENSORS_DPS_TO_RADS / 1000;            //convert raw to radians/sec/sec
  float highGScale = 0.488 * SENSORS_GRAVITY_EARTH / 1000;        //convert raw to meters/sec/sec
  virtual void setAccelScale(void);
  void setGyroScale(void);
  virtual void setHighGScale(void) {};
  float temperature_sensitivity = 256.0; ///< Temp sensor sensitivity in LSB/degC

  bool _high_g_enabled = false;

private:
  friend class Adafruit_LSM6DSV_Temp; ///< Gives access to private members to
                                     ///< Temp data object
  friend class Adafruit_LSM6DSV_Accelerometer; ///< Gives access to private
                                              ///< members to Accelerometer data
                                              ///< object
  friend class Adafruit_LSM6DSV_Gyro; ///< Gives access to private members to
                                     ///< Gyro data object
  friend class Adafruit_LSM6DSV_HighG; ///< Gives access to private members to
                                     ///< HighG data object

  void fillTempEvent(sensors_event_t *temp, uint32_t timestamp);
  void fillAccelEvent(sensors_event_t *accel, uint32_t timestamp);
  void fillGyroEvent(sensors_event_t *gyro, uint32_t timestamp);
  void fillHighGEvent(sensors_event_t *accel, uint32_t timestamp);
};

/*!
 *    @brief  Sub-class for the "B" version which has the TDM interface.
              The only difference that concerns us is the different chip ID
 */
class Adafruit_LSM6DSV16B : public Adafruit_LSM6DSV  {
public:
  Adafruit_LSM6DSV16B() {};
private:
  bool _init(int32_t sensor_id) {Adafruit_LSM6DSV::_init(sensor_id); return (chipID() == LSM6DSV16B_CHIP_ID);};
};

/*!
 *    @brief  Sub-class for the 32G version.
              The only difference that concerns us is the different accelerometer scale
 */
class Adafruit_LSM6DSV32 : public Adafruit_LSM6DSV  {
public:
  Adafruit_LSM6DSV32() {};
private:
  bool _init(int32_t sensor_id) {bool tmp = Adafruit_LSM6DSV::_init(sensor_id); highGScale = 0.976 * SENSORS_GRAVITY_EARTH / 1000; return tmp;};
  void setAccelScale(void) override {Adafruit_LSM6DSV::setAccelScale(); accelScale*=2;};
};

#endif
