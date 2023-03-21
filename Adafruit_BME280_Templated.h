/*!
 * @file Adafruit_BME280_Templated.h
 *
 * @mainpage Adafruit BME280 humidity, temperature & pressure sensor
 *
 * @section intro_sec Introduction
 *
 *  Driver for the BME280 humidity, temperature & pressure sensor
 *
 * These sensors use I2C or SPI to communicate, 2 or 4 pins are required
 * to interface.
 *
 * Designed specifically to work with the Adafruit BME280 Breakout
 * ----> http://www.adafruit.com/products/2652
 *
 *  Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing
 *  products from Adafruit!
 * 
 * kmm:
 * This fork has been modified to add templates that allow use of both
 * Wire (hardware I2C) and the mostly interface-compatible SoftWire (software I2C)
 * low-level drivers. If anyone knows how to use the stock lib with SoftWire 
 * let me know, I wasn't able to get it to do anything but crash using things like
 * reinterpret_cast to force the type and I don't think it's possible without changing
 * this library.
 * 
 * Depends on a similarly modified implementation of Adafruit_BusIO.
 * 
 * Implementation has been moved into this header by necessity.
 *
 * @section author Author
 *
 * Written by Kevin "KTOWN" Townsend for Adafruit Industries.
 * Hacked up fork by github/kmm
 * 
 * @section license License
 *
 * BSD license, all text here must be included in any redistribution.
 * See the LICENSE file for details.
 *
 */

#ifndef __BME280_H__
#define __BME280_H__

#include "Arduino.h"
#include <Adafruit_I2CDevice_Templated.h>
#include <Adafruit_SPIDevice.h>
#include <Adafruit_Sensor.h>

/*!
 *  @brief  default I2C address
 */
#define BME280_ADDRESS (0x77)           // Primary I2C Address
                                        /*!
                                         *  @brief  alternate I2C address
                                         */
#define BME280_ADDRESS_ALTERNATE (0x76) // Alternate Address

/*!
 *  @brief Register addresses
 */
enum {
  BME280_REGISTER_DIG_T1 = 0x88,
  BME280_REGISTER_DIG_T2 = 0x8A,
  BME280_REGISTER_DIG_T3 = 0x8C,

  BME280_REGISTER_DIG_P1 = 0x8E,
  BME280_REGISTER_DIG_P2 = 0x90,
  BME280_REGISTER_DIG_P3 = 0x92,
  BME280_REGISTER_DIG_P4 = 0x94,
  BME280_REGISTER_DIG_P5 = 0x96,
  BME280_REGISTER_DIG_P6 = 0x98,
  BME280_REGISTER_DIG_P7 = 0x9A,
  BME280_REGISTER_DIG_P8 = 0x9C,
  BME280_REGISTER_DIG_P9 = 0x9E,

  BME280_REGISTER_DIG_H1 = 0xA1,
  BME280_REGISTER_DIG_H2 = 0xE1,
  BME280_REGISTER_DIG_H3 = 0xE3,
  BME280_REGISTER_DIG_H4 = 0xE4,
  BME280_REGISTER_DIG_H5 = 0xE5,
  BME280_REGISTER_DIG_H6 = 0xE7,

  BME280_REGISTER_CHIPID = 0xD0,
  BME280_REGISTER_VERSION = 0xD1,
  BME280_REGISTER_SOFTRESET = 0xE0,

  BME280_REGISTER_CAL26 = 0xE1, // R calibration stored in 0xE1-0xF0

  BME280_REGISTER_CONTROLHUMID = 0xF2,
  BME280_REGISTER_STATUS = 0XF3,
  BME280_REGISTER_CONTROL = 0xF4,
  BME280_REGISTER_CONFIG = 0xF5,
  BME280_REGISTER_PRESSUREDATA = 0xF7,
  BME280_REGISTER_TEMPDATA = 0xFA,
  BME280_REGISTER_HUMIDDATA = 0xFD
};

/**************************************************************************/
/*!
    @brief  calibration data
*/
/**************************************************************************/
typedef struct {
  uint16_t dig_T1; ///< temperature compensation value
  int16_t dig_T2;  ///< temperature compensation value
  int16_t dig_T3;  ///< temperature compensation value

  uint16_t dig_P1; ///< pressure compensation value
  int16_t dig_P2;  ///< pressure compensation value
  int16_t dig_P3;  ///< pressure compensation value
  int16_t dig_P4;  ///< pressure compensation value
  int16_t dig_P5;  ///< pressure compensation value
  int16_t dig_P6;  ///< pressure compensation value
  int16_t dig_P7;  ///< pressure compensation value
  int16_t dig_P8;  ///< pressure compensation value
  int16_t dig_P9;  ///< pressure compensation value

  uint8_t dig_H1; ///< humidity compensation value
  int16_t dig_H2; ///< humidity compensation value
  uint8_t dig_H3; ///< humidity compensation value
  int16_t dig_H4; ///< humidity compensation value
  int16_t dig_H5; ///< humidity compensation value
  int8_t dig_H6;  ///< humidity compensation value
} bme280_calib_data;
/*=========================================================================*/
template <class TWire>
class Adafruit_BME280_Templated;

/** Adafruit Unified Sensor interface for temperature component of BME280 */
template<class TWire>
class Adafruit_SoftBME280_Temp : public Adafruit_Sensor {
public:
  /** @brief Create an Adafruit_Sensor compatible object for the temp sensor
      @param parent A pointer to the BME280 class */
  Adafruit_SoftBME280_Temp(Adafruit_BME280_Templated<TWire> *parent) { _theBME280 = parent; }
  bool getEvent(sensors_event_t *);
  void getSensor(sensor_t *);

private:
  int _sensorID = 280;
  Adafruit_BME280_Templated<TWire> *_theBME280 = NULL;
};

/** Adafruit Unified Sensor interface for pressure component of BME280 */
template<class TWire>
class Adafruit_SoftBME280_Pressure : public Adafruit_Sensor {
public:
  /** @brief Create an Adafruit_Sensor compatible object for the pressure sensor
      @param parent A pointer to the BME280 class */
  Adafruit_SoftBME280_Pressure(Adafruit_BME280_Templated<TWire> *parent) { _theBME280 = parent; }
  bool getEvent(sensors_event_t *);
  void getSensor(sensor_t *);

private:
  int _sensorID = 280;
  Adafruit_BME280_Templated<TWire> *_theBME280 = NULL;
};

/** Adafruit Unified Sensor interface for humidity component of BME280 */
template<class TWire>
class Adafruit_SoftBME280_Humidity : public Adafruit_Sensor {
public:
  /** @brief Create an Adafruit_Sensor compatible object for the humidity sensor
      @param parent A pointer to the BME280 class */
  Adafruit_SoftBME280_Humidity(Adafruit_BME280_Templated<TWire> *parent) { _theBME280 = parent; }
  bool getEvent(sensors_event_t *);
  void getSensor(sensor_t *);

private:
  int _sensorID = 280;
  Adafruit_BME280_Templated<TWire> *_theBME280 = NULL;
};

/**************************************************************************/
/*!
    @brief  Class that stores state and functions for interacting with BME280 IC
*/
/**************************************************************************/
template<class TWire>
class Adafruit_BME280_Templated {
public:
  /**************************************************************************/
  /*!
      @brief  sampling rates
  */
  /**************************************************************************/
  enum sensor_sampling {
    SAMPLING_NONE = 0b000,
    SAMPLING_X1 = 0b001,
    SAMPLING_X2 = 0b010,
    SAMPLING_X4 = 0b011,
    SAMPLING_X8 = 0b100,
    SAMPLING_X16 = 0b101
  };

  /**************************************************************************/
  /*!
      @brief  power modes
  */
  /**************************************************************************/
  enum sensor_mode {
    MODE_SLEEP = 0b00,
    MODE_FORCED = 0b01,
    MODE_NORMAL = 0b11
  };

  /**************************************************************************/
  /*!
      @brief  filter values
  */
  /**************************************************************************/
  enum sensor_filter {
    FILTER_OFF = 0b000,
    FILTER_X2 = 0b001,
    FILTER_X4 = 0b010,
    FILTER_X8 = 0b011,
    FILTER_X16 = 0b100
  };

  /**************************************************************************/
  /*!
      @brief  standby duration in ms
  */
  /**************************************************************************/
  enum standby_duration {
    STANDBY_MS_0_5 = 0b000,
    STANDBY_MS_10 = 0b110,
    STANDBY_MS_20 = 0b111,
    STANDBY_MS_62_5 = 0b001,
    STANDBY_MS_125 = 0b010,
    STANDBY_MS_250 = 0b011,
    STANDBY_MS_500 = 0b100,
    STANDBY_MS_1000 = 0b101
  };

  // constructors
  Adafruit_BME280_Templated();
  Adafruit_BME280_Templated(int8_t cspin, SPIClass *theSPI = &SPI);
  Adafruit_BME280_Templated(int8_t cspin, int8_t mosipin, int8_t misopin, int8_t sckpin);
  ~Adafruit_BME280_Templated(void);
  bool begin(uint8_t addr = BME280_ADDRESS, TWire *theWire = NULL);
  bool init();

  void setSampling(sensor_mode mode = MODE_NORMAL,
                   sensor_sampling tempSampling = SAMPLING_X16,
                   sensor_sampling pressSampling = SAMPLING_X16,
                   sensor_sampling humSampling = SAMPLING_X16,
                   sensor_filter filter = FILTER_OFF,
                   standby_duration duration = STANDBY_MS_0_5);

  bool takeForcedMeasurement(void);
  float readTemperature(void);
  float readPressure(void);
  float readHumidity(void);

  float readAltitude(float seaLevel);
  float seaLevelForAltitude(float altitude, float pressure);
  uint32_t sensorID(void);

  float getTemperatureCompensation(void);
  void setTemperatureCompensation(float);

  Adafruit_Sensor *getTemperatureSensor(void);
  Adafruit_Sensor *getPressureSensor(void);
  Adafruit_Sensor *getHumiditySensor(void);

protected:
  Adafruit_I2CDevice_Templated<TWire> *i2c_dev = NULL; ///< Pointer to I2C bus interface
  Adafruit_SPIDevice *spi_dev = NULL; ///< Pointer to SPI bus interface

  Adafruit_SoftBME280_Temp<TWire> *temp_sensor = NULL;
  //!< Adafruit_Sensor compat temperature sensor component

  Adafruit_SoftBME280_Pressure<TWire> *pressure_sensor = NULL;
  //!< Adafruit_Sensor compat pressure sensor component

  Adafruit_SoftBME280_Humidity<TWire> *humidity_sensor = NULL;
  //!< Adafruit_Sensor compat humidity sensor component

  void readCoefficients(void);
  bool isReadingCalibration(void);

  void write8(byte reg, byte value);
  uint8_t read8(byte reg);
  uint16_t read16(byte reg);
  uint32_t read24(byte reg);
  int16_t readS16(byte reg);
  uint16_t read16_LE(byte reg); // little endian
  int16_t readS16_LE(byte reg); // little endian

  uint8_t _i2caddr;  //!< I2C addr for the TwoWire interface
  int32_t _sensorID; //!< ID of the BME Sensor
  int32_t t_fine; //!< temperature with high resolution, stored as an attribute
                  //!< as this is used for temperature compensation reading
                  //!< humidity and pressure

  int32_t t_fine_adjust = 0; //!< add to compensate temp readings and in turn
                             //!< to pressure and humidity readings

  bme280_calib_data _bme280_calib; //!< here calibration data is stored

  /**************************************************************************/
  /*!
      @brief  config register
  */
  /**************************************************************************/
  struct config {
    // inactive duration (standby time) in normal mode
    // 000 = 0.5 ms
    // 001 = 62.5 ms
    // 010 = 125 ms
    // 011 = 250 ms
    // 100 = 500 ms
    // 101 = 1000 ms
    // 110 = 10 ms
    // 111 = 20 ms
    unsigned int t_sb : 3; ///< inactive duration (standby time) in normal mode

    // filter settings
    // 000 = filter off
    // 001 = 2x filter
    // 010 = 4x filter
    // 011 = 8x filter
    // 100 and above = 16x filter
    unsigned int filter : 3; ///< filter settings

    // unused - don't set
    unsigned int none : 1;     ///< unused - don't set
    unsigned int spi3w_en : 1; ///< unused - don't set

    /// @return combined config register
    unsigned int get() { return (t_sb << 5) | (filter << 2) | spi3w_en; }
  };
  config _configReg; //!< config register object

  /**************************************************************************/
  /*!
      @brief  ctrl_meas register
  */
  /**************************************************************************/
  struct ctrl_meas {
    // temperature oversampling
    // 000 = skipped
    // 001 = x1
    // 010 = x2
    // 011 = x4
    // 100 = x8
    // 101 and above = x16
    unsigned int osrs_t : 3; ///< temperature oversampling

    // pressure oversampling
    // 000 = skipped
    // 001 = x1
    // 010 = x2
    // 011 = x4
    // 100 = x8
    // 101 and above = x16
    unsigned int osrs_p : 3; ///< pressure oversampling

    // device mode
    // 00       = sleep
    // 01 or 10 = forced
    // 11       = normal
    unsigned int mode : 2; ///< device mode

    /// @return combined ctrl register
    unsigned int get() { return (osrs_t << 5) | (osrs_p << 2) | mode; }
  };
  ctrl_meas _measReg; //!< measurement register object

  /**************************************************************************/
  /*!
      @brief  ctrl_hum register
  */
  /**************************************************************************/
  struct ctrl_hum {
    /// unused - don't set
    unsigned int none : 5;

    // pressure oversampling
    // 000 = skipped
    // 001 = x1
    // 010 = x2
    // 011 = x4
    // 100 = x8
    // 101 and above = x16
    unsigned int osrs_h : 3; ///< pressure oversampling

    /// @return combined ctrl hum register
    unsigned int get() { return (osrs_h); }
  };
  ctrl_hum _humReg; //!< hum register object
};

/******************************************************************************/
/*!                     IMPLEMENTATION BEGINS HERE                            */
/******************************************************************************/

/*!
 *  @brief  class constructor
 */
template<class TWire>
Adafruit_BME280_Templated<TWire>::Adafruit_BME280_Templated() {}

/*!
 *   @brief  class constructor if using hardware SPI
 *   @param  cspin the chip select pin to use
 *   @param  *theSPI
 *           optional SPI object
 */
template<class TWire>
Adafruit_BME280_Templated<TWire>::Adafruit_BME280_Templated(int8_t cspin, SPIClass *theSPI) {
  spi_dev = new Adafruit_SPIDevice(cspin, 1000000, SPI_BITORDER_MSBFIRST,
                                   SPI_MODE0, theSPI);
}

/*!
 *   @brief  class constructor if using software SPI
 *   @param cspin the chip select pin to use
 *   @param mosipin the MOSI pin to use
 *   @param misopin the MISO pin to use
 *   @param sckpin the SCK pin to use
 */
template<class TWire>
Adafruit_BME280_Templated<TWire>::Adafruit_BME280_Templated(int8_t cspin, int8_t mosipin, int8_t misopin,
                                 int8_t sckpin) {
  spi_dev = new Adafruit_SPIDevice(cspin, sckpin, misopin, mosipin);
}
template<class TWire>
Adafruit_BME280_Templated<TWire>::~Adafruit_BME280_Templated(void) {
  if (spi_dev) {
    delete spi_dev;
  }
  if (i2c_dev) {
    delete i2c_dev;
  }
  if (temp_sensor) {
    delete temp_sensor;
  }
  if (pressure_sensor) {
    delete pressure_sensor;
  }
  if (humidity_sensor) {
    delete humidity_sensor;
  }
}

/*!
 *   @brief  Initialise sensor with given parameters / settings
 *   @param addr the I2C address the device can be found on
 *   @param theWire the I2C object to use, defaults to &Wire
 *   @returns true on success, false otherwise
 */
template<class TWire>
bool Adafruit_BME280_Templated<TWire>::begin(uint8_t addr, TWire *theWire) {
  if (spi_dev == NULL) {
    // I2C mode
    if (i2c_dev)
      delete i2c_dev;
    i2c_dev = new Adafruit_I2CDevice_Templated<TWire>(addr, theWire);
    if (!i2c_dev->begin())
      return false;
  } else {
    // SPI mode
    if (!spi_dev->begin())
      return false;
  }
  return init();
}

/*!
 *   @brief  Initialise sensor with given parameters / settings
 *   @returns true on success, false otherwise
 */
template<class TWire>
bool Adafruit_BME280_Templated<TWire>::init() {
  // check if sensor, i.e. the chip ID is correct
  _sensorID = read8(BME280_REGISTER_CHIPID);
  if (_sensorID != 0x60)
    return false;

  // reset the device using soft-reset
  // this makes sure the IIR is off, etc.
  write8(BME280_REGISTER_SOFTRESET, 0xB6);

  // wait for chip to wake up.
  delay(10);

  // if chip is still reading calibration, delay
  while (isReadingCalibration())
    delay(10);

  readCoefficients(); // read trimming parameters, see DS 4.2.2

  setSampling(); // use defaults

  delay(100);

  return true;
}

/*!
 *   @brief  setup sensor with given parameters / settings
 *
 *   This is simply a overload to the normal begin()-function, so SPI users
 *   don't get confused about the library requiring an address.
 *   @param mode the power mode to use for the sensor
 *   @param tempSampling the temp samping rate to use
 *   @param pressSampling the pressure sampling rate to use
 *   @param humSampling the humidity sampling rate to use
 *   @param filter the filter mode to use
 *   @param duration the standby duration to use
 */
template<class TWire>
void Adafruit_BME280_Templated<TWire>::setSampling(sensor_mode mode,
                                  sensor_sampling tempSampling,
                                  sensor_sampling pressSampling,
                                  sensor_sampling humSampling,
                                  sensor_filter filter,
                                  standby_duration duration) {
  _measReg.mode = mode;
  _measReg.osrs_t = tempSampling;
  _measReg.osrs_p = pressSampling;

  _humReg.osrs_h = humSampling;
  _configReg.filter = filter;
  _configReg.t_sb = duration;
  _configReg.spi3w_en = 0;

  // making sure sensor is in sleep mode before setting configuration
  // as it otherwise may be ignored
  write8(BME280_REGISTER_CONTROL, MODE_SLEEP);

  // you must make sure to also set REGISTER_CONTROL after setting the
  // CONTROLHUMID register, otherwise the values won't be applied (see
  // DS 5.4.3)
  write8(BME280_REGISTER_CONTROLHUMID, _humReg.get());
  write8(BME280_REGISTER_CONFIG, _configReg.get());
  write8(BME280_REGISTER_CONTROL, _measReg.get());
}

/*!
 *   @brief  Writes an 8 bit value over I2C or SPI
 *   @param reg the register address to write to
 *   @param value the value to write to the register
 */
template<class TWire>
void Adafruit_BME280_Templated<TWire>::write8(byte reg, byte value) {
  byte buffer[2];
  buffer[1] = value;
  if (i2c_dev) {
    buffer[0] = reg;
    i2c_dev->write(buffer, 2);
  } else {
    buffer[0] = reg & ~0x80;
    spi_dev->write(buffer, 2);
  }
}

/*!
 *   @brief  Reads an 8 bit value over I2C or SPI
 *   @param reg the register address to read from
 *   @returns the data byte read from the device
 */
template<class TWire>
uint8_t Adafruit_BME280_Templated<TWire>::read8(byte reg) {
  uint8_t buffer[1];
  if (i2c_dev) {
    buffer[0] = uint8_t(reg);
    i2c_dev->write_then_read(buffer, 1, buffer, 1);
  } else {
    buffer[0] = uint8_t(reg | 0x80);
    spi_dev->write_then_read(buffer, 1, buffer, 1);
  }
  return buffer[0];
}

/*!
 *   @brief  Reads a 16 bit value over I2C or SPI
 *   @param reg the register address to read from
 *   @returns the 16 bit data value read from the device
 */
template<class TWire>
uint16_t Adafruit_BME280_Templated<TWire>::read16(byte reg) {
  uint8_t buffer[2];

  if (i2c_dev) {
    buffer[0] = uint8_t(reg);
    i2c_dev->write_then_read(buffer, 1, buffer, 2);
  } else {
    buffer[0] = uint8_t(reg | 0x80);
    spi_dev->write_then_read(buffer, 1, buffer, 2);
  }
  return uint16_t(buffer[0]) << 8 | uint16_t(buffer[1]);
}

/*!
 *   @brief  Reads a signed 16 bit little endian value over I2C or SPI
 *   @param reg the register address to read from
 *   @returns the 16 bit data value read from the device
 */
template<class TWire>
uint16_t Adafruit_BME280_Templated<TWire>::read16_LE(byte reg) {
  uint16_t temp = read16(reg);
  return (temp >> 8) | (temp << 8);
}

/*!
 *   @brief  Reads a signed 16 bit value over I2C or SPI
 *   @param reg the register address to read from
 *   @returns the 16 bit data value read from the device
 */
template<class TWire>
int16_t Adafruit_BME280_Templated<TWire>::readS16(byte reg) { return (int16_t)read16(reg); }

/*!
 *   @brief  Reads a signed little endian 16 bit value over I2C or SPI
 *   @param reg the register address to read from
 *   @returns the 16 bit data value read from the device
 */
template<class TWire>
int16_t Adafruit_BME280_Templated<TWire>::readS16_LE(byte reg) {
  return (int16_t)read16_LE(reg);
}

/*!
 *   @brief  Reads a 24 bit value over I2C
 *   @param reg the register address to read from
 *   @returns the 24 bit data value read from the device
 */
template<class TWire>
uint32_t Adafruit_BME280_Templated<TWire>::read24(byte reg) {
  uint8_t buffer[3];

  if (i2c_dev) {
    buffer[0] = uint8_t(reg);
    i2c_dev->write_then_read(buffer, 1, buffer, 3);
  } else {
    buffer[0] = uint8_t(reg | 0x80);
    spi_dev->write_then_read(buffer, 1, buffer, 3);
  }

  return uint32_t(buffer[0]) << 16 | uint32_t(buffer[1]) << 8 |
         uint32_t(buffer[2]);
}

/*!
 *  @brief  Take a new measurement (only possible in forced mode)
    @returns true in case of success else false
 */
template<class TWire>
bool Adafruit_BME280_Templated<TWire>::takeForcedMeasurement(void) {
  bool return_value = false;
  // If we are in forced mode, the BME sensor goes back to sleep after each
  // measurement and we need to set it to forced mode once at this point, so
  // it will take the next measurement and then return to sleep again.
  // In normal mode simply does new measurements periodically.
  if (_measReg.mode == MODE_FORCED) {
    return_value = true;
    // set to forced mode, i.e. "take next measurement"
    write8(BME280_REGISTER_CONTROL, _measReg.get());
    // Store current time to measure the timeout
    uint32_t timeout_start = millis();
    // wait until measurement has been completed, otherwise we would read the
    // the values from the last measurement or the timeout occurred after 2 sec.
    while (read8(BME280_REGISTER_STATUS) & 0x08) {
      // In case of a timeout, stop the while loop
      if ((millis() - timeout_start) > 2000) {
        return_value = false;
        break;
      }
      delay(1);
    }
  }
  return return_value;
}

/*!
 *   @brief  Reads the factory-set coefficients
 */
template<class TWire>
void Adafruit_BME280_Templated<TWire>::readCoefficients(void) {
  _bme280_calib.dig_T1 = read16_LE(BME280_REGISTER_DIG_T1);
  _bme280_calib.dig_T2 = readS16_LE(BME280_REGISTER_DIG_T2);
  _bme280_calib.dig_T3 = readS16_LE(BME280_REGISTER_DIG_T3);

  _bme280_calib.dig_P1 = read16_LE(BME280_REGISTER_DIG_P1);
  _bme280_calib.dig_P2 = readS16_LE(BME280_REGISTER_DIG_P2);
  _bme280_calib.dig_P3 = readS16_LE(BME280_REGISTER_DIG_P3);
  _bme280_calib.dig_P4 = readS16_LE(BME280_REGISTER_DIG_P4);
  _bme280_calib.dig_P5 = readS16_LE(BME280_REGISTER_DIG_P5);
  _bme280_calib.dig_P6 = readS16_LE(BME280_REGISTER_DIG_P6);
  _bme280_calib.dig_P7 = readS16_LE(BME280_REGISTER_DIG_P7);
  _bme280_calib.dig_P8 = readS16_LE(BME280_REGISTER_DIG_P8);
  _bme280_calib.dig_P9 = readS16_LE(BME280_REGISTER_DIG_P9);

  _bme280_calib.dig_H1 = read8(BME280_REGISTER_DIG_H1);
  _bme280_calib.dig_H2 = readS16_LE(BME280_REGISTER_DIG_H2);
  _bme280_calib.dig_H3 = read8(BME280_REGISTER_DIG_H3);
  _bme280_calib.dig_H4 = ((int8_t)read8(BME280_REGISTER_DIG_H4) << 4) |
                         (read8(BME280_REGISTER_DIG_H4 + 1) & 0xF);
  _bme280_calib.dig_H5 = ((int8_t)read8(BME280_REGISTER_DIG_H5 + 1) << 4) |
                         (read8(BME280_REGISTER_DIG_H5) >> 4);
  _bme280_calib.dig_H6 = (int8_t)read8(BME280_REGISTER_DIG_H6);
}

/*!
 *   @brief return true if chip is busy reading cal data
 *   @returns true if reading calibration, false otherwise
 */
template<class TWire>
bool Adafruit_BME280_Templated<TWire>::isReadingCalibration(void) {
  uint8_t const rStatus = read8(BME280_REGISTER_STATUS);

  return (rStatus & (1 << 0)) != 0;
}

/*!
 *   @brief  Returns the temperature from the sensor
 *   @returns the temperature read from the device
 */
template<class TWire>
float Adafruit_BME280_Templated<TWire>::readTemperature(void) {
  int32_t var1, var2;

  int32_t adc_T = read24(BME280_REGISTER_TEMPDATA);
  if (adc_T == 0x800000) // value in case temp measurement was disabled
    return NAN;
  adc_T >>= 4;

  var1 = (int32_t)((adc_T / 8) - ((int32_t)_bme280_calib.dig_T1 * 2));
  var1 = (var1 * ((int32_t)_bme280_calib.dig_T2)) / 2048;
  var2 = (int32_t)((adc_T / 16) - ((int32_t)_bme280_calib.dig_T1));
  var2 = (((var2 * var2) / 4096) * ((int32_t)_bme280_calib.dig_T3)) / 16384;

  t_fine = var1 + var2 + t_fine_adjust;

  int32_t T = (t_fine * 5 + 128) / 256;

  return (float)T / 100;
}

/*!
 *   @brief  Returns the pressure from the sensor
 *   @returns the pressure value (in Pascal) read from the device
 */
template<class TWire>
float Adafruit_BME280_Templated<TWire>::readPressure(void) {
  int64_t var1, var2, var3, var4;

  readTemperature(); // must be done first to get t_fine

  int32_t adc_P = read24(BME280_REGISTER_PRESSUREDATA);
  if (adc_P == 0x800000) // value in case pressure measurement was disabled
    return NAN;
  adc_P >>= 4;

  var1 = ((int64_t)t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)_bme280_calib.dig_P6;
  var2 = var2 + ((var1 * (int64_t)_bme280_calib.dig_P5) * 131072);
  var2 = var2 + (((int64_t)_bme280_calib.dig_P4) * 34359738368);
  var1 = ((var1 * var1 * (int64_t)_bme280_calib.dig_P3) / 256) +
         ((var1 * ((int64_t)_bme280_calib.dig_P2) * 4096));
  var3 = ((int64_t)1) * 140737488355328;
  var1 = (var3 + var1) * ((int64_t)_bme280_calib.dig_P1) / 8589934592;

  if (var1 == 0) {
    return 0; // avoid exception caused by division by zero
  }

  var4 = 1048576 - adc_P;
  var4 = (((var4 * 2147483648) - var2) * 3125) / var1;
  var1 = (((int64_t)_bme280_calib.dig_P9) * (var4 / 8192) * (var4 / 8192)) /
         33554432;
  var2 = (((int64_t)_bme280_calib.dig_P8) * var4) / 524288;
  var4 = ((var4 + var1 + var2) / 256) + (((int64_t)_bme280_calib.dig_P7) * 16);

  float P = var4 / 256.0;

  return P;
}

/*!
 *  @brief  Returns the humidity from the sensor
 *  @returns the humidity value read from the device
 */
template<class TWire>
float Adafruit_BME280_Templated<TWire>::readHumidity(void) {
  int32_t var1, var2, var3, var4, var5;

  readTemperature(); // must be done first to get t_fine

  int32_t adc_H = read16(BME280_REGISTER_HUMIDDATA);
  if (adc_H == 0x8000) // value in case humidity measurement was disabled
    return NAN;

  var1 = t_fine - ((int32_t)76800);
  var2 = (int32_t)(adc_H * 16384);
  var3 = (int32_t)(((int32_t)_bme280_calib.dig_H4) * 1048576);
  var4 = ((int32_t)_bme280_calib.dig_H5) * var1;
  var5 = (((var2 - var3) - var4) + (int32_t)16384) / 32768;
  var2 = (var1 * ((int32_t)_bme280_calib.dig_H6)) / 1024;
  var3 = (var1 * ((int32_t)_bme280_calib.dig_H3)) / 2048;
  var4 = ((var2 * (var3 + (int32_t)32768)) / 1024) + (int32_t)2097152;
  var2 = ((var4 * ((int32_t)_bme280_calib.dig_H2)) + 8192) / 16384;
  var3 = var5 * var2;
  var4 = ((var3 / 32768) * (var3 / 32768)) / 128;
  var5 = var3 - ((var4 * ((int32_t)_bme280_calib.dig_H1)) / 16);
  var5 = (var5 < 0 ? 0 : var5);
  var5 = (var5 > 419430400 ? 419430400 : var5);
  uint32_t H = (uint32_t)(var5 / 4096);

  return (float)H / 1024.0;
}

/*!
 *   Calculates the altitude (in meters) from the specified atmospheric
 *   pressure (in hPa), and sea-level pressure (in hPa).
 *   @param  seaLevel      Sea-level pressure in hPa
 *   @returns the altitude value read from the device
 */
template<class TWire>
float Adafruit_BME280_Templated<TWire>::readAltitude(float seaLevel) {
  // Equation taken from BMP180 datasheet (page 16):
  //  http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf

  // Note that using the equation from wikipedia can give bad results
  // at high altitude. See this thread for more information:
  //  http://forums.adafruit.com/viewtopic.php?f=22&t=58064

  float atmospheric = readPressure() / 100.0F;
  return 44330.0 * (1.0 - pow(atmospheric / seaLevel, 0.1903));
}

/*!
 *   Calculates the pressure at sea level (in hPa) from the specified
 * altitude (in meters), and atmospheric pressure (in hPa).
 *   @param  altitude      Altitude in meters
 *   @param  atmospheric   Atmospheric pressure in hPa
 *   @returns the pressure at sea level (in hPa) from the specified altitude
 */
template<class TWire>
float Adafruit_BME280_Templated<TWire>::seaLevelForAltitude(float altitude, float atmospheric) {
  // Equation taken from BMP180 datasheet (page 17):
  //  http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf

  // Note that using the equation from wikipedia can give bad results
  // at high altitude. See this thread for more information:
  //  http://forums.adafruit.com/viewtopic.php?f=22&t=58064

  return atmospheric / pow(1.0 - (altitude / 44330.0), 5.255);
}

/*!
 *   Returns Sensor ID found by init() for diagnostics
 *   @returns Sensor ID 0x60 for BME280, 0x56, 0x57, 0x58 BMP280
 */
template<class TWire>
uint32_t Adafruit_BME280_Templated<TWire>::sensorID(void) { return _sensorID; }

/*!
 *   Returns the current temperature compensation value in degrees Celsius
 *   @returns the current temperature compensation value in degrees Celsius
 */
template<class TWire>
float Adafruit_BME280_Templated<TWire>::getTemperatureCompensation(void) {
  return float((t_fine_adjust * 5) >> 8) / 100.0;
};

/*!
 *  Sets a value to be added to each temperature reading. This adjusted
 *  temperature is used in pressure and humidity readings.
 *  @param  adjustment  Value to be added to each temperature reading in Celsius
 */
template<class TWire>
void Adafruit_BME280_Templated<TWire>::setTemperatureCompensation(float adjustment) {
  // convert the value in C into and adjustment to t_fine
  t_fine_adjust = ((int32_t(adjustment * 100) << 8)) / 5;
};

/*!
    @brief  Gets an Adafruit Unified Sensor object for the temp sensor component
    @return Adafruit_Sensor pointer to temperature sensor
 */
template<class TWire>
Adafruit_Sensor *Adafruit_BME280_Templated<TWire>::getTemperatureSensor(void) {
  if (!temp_sensor) {
    temp_sensor = new Adafruit_SoftBME280_Temp<TWire>(this);
  }

  return temp_sensor;
}

/*!
    @brief  Gets an Adafruit Unified Sensor object for the pressure sensor
   component
    @return Adafruit_Sensor pointer to pressure sensor
 */
template<class TWire>
Adafruit_Sensor *Adafruit_BME280_Templated<TWire>::getPressureSensor(void) {
  if (!pressure_sensor) {
    pressure_sensor = new Adafruit_SoftBME280_Pressure<TWire>(this);
  }
  return pressure_sensor;
}

/*!
    @brief  Gets an Adafruit Unified Sensor object for the humidity sensor
   component
    @return Adafruit_Sensor pointer to humidity sensor
 */
template<class TWire>
Adafruit_Sensor *Adafruit_BME280_Templated<TWire>::getHumiditySensor(void) {
  if (!humidity_sensor) {
    humidity_sensor = new Adafruit_SoftBME280_Humidity<TWire>(this);
  }
  return humidity_sensor;
}

/**************************************************************************/
/*!
    @brief  Gets the sensor_t data for the BME280's temperature sensor
*/
/**************************************************************************/
template<class TWire>
void Adafruit_SoftBME280_Temp<TWire>::getSensor(sensor_t *sensor) {
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy(sensor->name, "BME280", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name) - 1] = 0;
  sensor->version = 1;
  sensor->sensor_id = _sensorID;
  sensor->type = SENSOR_TYPE_AMBIENT_TEMPERATURE;
  sensor->min_delay = 0;
  sensor->min_value = -40.0; /* Temperature range -40 ~ +85 C  */
  sensor->max_value = +85.0;
  sensor->resolution = 0.01; /*  0.01 C */
}

/**************************************************************************/
/*!
    @brief  Gets the temperature as a standard sensor event
    @param  event Sensor event object that will be populated
    @returns True
*/
/**************************************************************************/
template<class TWire>
bool Adafruit_SoftBME280_Temp<TWire>::getEvent(sensors_event_t *event) {
  /* Clear the event */
  memset(event, 0, sizeof(sensors_event_t));

  event->version = sizeof(sensors_event_t);
  event->sensor_id = _sensorID;
  event->type = SENSOR_TYPE_AMBIENT_TEMPERATURE;
  event->timestamp = millis();
  event->temperature = _theBME280->readTemperature();
  return true;
}

/**************************************************************************/
/*!
    @brief  Gets the sensor_t data for the BME280's pressure sensor
*/
/**************************************************************************/
template<class TWire>
void Adafruit_SoftBME280_Pressure<TWire>::getSensor(sensor_t *sensor) {
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy(sensor->name, "BME280", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name) - 1] = 0;
  sensor->version = 1;
  sensor->sensor_id = _sensorID;
  sensor->type = SENSOR_TYPE_PRESSURE;
  sensor->min_delay = 0;
  sensor->min_value = 300.0; /* 300 ~ 1100 hPa  */
  sensor->max_value = 1100.0;
  sensor->resolution = 0.012; /* 0.12 hPa relative */
}

/**************************************************************************/
/*!
    @brief  Gets the pressure as a standard sensor event
    @param  event Sensor event object that will be populated
    @returns True
*/
/**************************************************************************/
template<class TWire>
bool Adafruit_SoftBME280_Pressure<TWire>::getEvent(sensors_event_t *event) {
  /* Clear the event */
  memset(event, 0, sizeof(sensors_event_t));

  event->version = sizeof(sensors_event_t);
  event->sensor_id = _sensorID;
  event->type = SENSOR_TYPE_PRESSURE;
  event->timestamp = millis();
  event->pressure = _theBME280->readPressure() / 100; // convert Pa to hPa
  return true;
}

/**************************************************************************/
/*!
    @brief  Gets the sensor_t data for the BME280's humidity sensor
*/
/**************************************************************************/
template<class TWire>
void Adafruit_SoftBME280_Humidity<TWire>::getSensor(sensor_t *sensor) {
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy(sensor->name, "BME280", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name) - 1] = 0;
  sensor->version = 1;
  sensor->sensor_id = _sensorID;
  sensor->type = SENSOR_TYPE_RELATIVE_HUMIDITY;
  sensor->min_delay = 0;
  sensor->min_value = 0;
  sensor->max_value = 100; /* 0 - 100 %  */
  sensor->resolution = 3;  /* 3% accuracy */
}

/**************************************************************************/
/*!
    @brief  Gets the humidity as a standard sensor event
    @param  event Sensor event object that will be populated
    @returns True
*/
/**************************************************************************/
template<class TWire>
bool Adafruit_SoftBME280_Humidity<TWire>::getEvent(sensors_event_t *event) {
  /* Clear the event */
  memset(event, 0, sizeof(sensors_event_t));

  event->version = sizeof(sensors_event_t);
  event->sensor_id = _sensorID;
  event->type = SENSOR_TYPE_RELATIVE_HUMIDITY;
  event->timestamp = millis();
  event->relative_humidity = _theBME280->readHumidity();
  return true;
}


#endif
