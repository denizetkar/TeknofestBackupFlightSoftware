/*!
 *  @file Adafruit_BMP280.cpp
 *
 *  This is a library for the BMP280 orientation sensor
 *
 *  Designed specifically to work with the Adafruit BMP280 Sensor.
 *
 *  Pick one up today in the adafruit shop!
 *  ------> https://www.adafruit.com/product/2651
 *
 *  These sensors use I2C to communicate, 2 pins are required to interface.
 *
 *  Adafruit invests time and resources providing this open source code,
 *  please support Adafruit andopen-source hardware by purchasing products
 *  from Adafruit!
 *
 *  K.Townsend (Adafruit Industries)
 *
 *  BSD license, all text above must be included in any redistribution
 */

#include "Adafruit_BMP280.h"
#include "Arduino.h"
#include <I2C.h>

#define I2C_TIMEOUT_MS 10

/*!
 * @brief  BMP280 constructor using i2c
 * @param  *theI2c
 *         optional i2c
 */
Adafruit_BMP280::Adafruit_BMP280(I2C *theI2c)
{
  _i2c = theI2c;
  temp_sensor = new Adafruit_BMP280_Temp(this);
  pressure_sensor = new Adafruit_BMP280_Pressure(this);
  memset(&_configReg, 0, sizeof(_configReg));
}

Adafruit_BMP280::~Adafruit_BMP280(void) {
  delete temp_sensor;
  delete pressure_sensor;
}

/*!
 *  Initialises the sensor.
 *  @param addr
 *         The I2C address to use (default = 0x77)
 *  @param chipid
 *         The expected chip ID (used to validate connection).
 *  @return True if the init was successful, otherwise false.
 */
bool Adafruit_BMP280::begin(uint8_t addr, uint8_t chipid) {
  uint8_t res;
  _i2caddr = addr;

  // i2c
  _i2c->begin();
  // set a timeout
  _i2c->timeOut(I2C_TIMEOUT_MS);

  if (readBytes(BMP280_REGISTER_CHIPID, &res, 1) != 0)
    return false;
  if (res != chipid)
    return false;

  if (!readCoefficients())
    return false;
  // writeByte(BMP280_REGISTER_CONTROL, 0x3F); /* needed? */
  if (!setSampling())
    return false;
  delay(100);
  return true;
}

/*!
 * Sets the sampling config for the device.
 * @param mode
 *        The operating mode of the sensor.
 * @param tempSampling
 *        The sampling scheme for temp readings.
 * @param pressSampling
 *        The sampling scheme for pressure readings.
 * @param filter
 *        The filtering mode to apply (if any).
 * @param duration
 *        The sampling duration.
 */
bool Adafruit_BMP280::setSampling(sensor_mode mode,
                                  sensor_sampling tempSampling,
                                  sensor_sampling pressSampling,
                                  sensor_filter filter,
                                  standby_duration duration) {
  _measReg.mode = mode;
  _measReg.osrs_t = tempSampling;
  _measReg.osrs_p = pressSampling;

  _configReg.filter = filter;
  _configReg.t_sb = duration;

  if (writeByte(BMP280_REGISTER_CONFIG, _configReg.get()))
    return false;
  if (writeByte(BMP280_REGISTER_CONTROL, _measReg.get()))
    return false;

  return true;
}

/*!
 *  @brief  Writes a byte of value over I2C
 *  @param  reg
 *          selected register
 *  @param  value
 *          1 byte of data to write
 *  @return operation status
 */
uint8_t Adafruit_BMP280::writeByte(uint8_t reg, uint8_t value) {
  return _i2c->write(_i2caddr, reg, value);
}

/*!
 *  @brief  Reads "length" byte(s) of value over I2C
 *  @param  reg
 *          selected register
 *  @param  buffer
 *          uint8_t buffer to read the data into
 *  @param  length
 *          number of available bytes in the buffer
 *  @return operation status
 */
uint8_t Adafruit_BMP280::readBytes(uint8_t reg, uint8_t *buffer, uint8_t length) {
  return _i2c->read(_i2caddr, reg, length, buffer);
}

uint8_t Adafruit_BMP280::read24(byte reg, uint32_t *out) {
  uint8_t buffer[3], res;

  res = _i2c->read((uint8_t)_i2caddr, (uint8_t)reg, 3, (uint8_t*)buffer);
  if (res == 0) {
    *out = (((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | (uint32_t)buffer[2]);
  }

  return res;
}

/*!
 *  @brief  Reads the factory-set coefficients
 */
bool Adafruit_BMP280::readCoefficients() {
  if (readBytes(BMP280_REGISTER_DIG_T1, (uint8_t*)&_bmp280_calib.dig_T1, 2))
    return false;
  if (readBytes(BMP280_REGISTER_DIG_T2, (uint8_t*)&_bmp280_calib.dig_T2, 2))
    return false;
  if (readBytes(BMP280_REGISTER_DIG_T3, (uint8_t*)&_bmp280_calib.dig_T3, 2))
    return false;

  if (readBytes(BMP280_REGISTER_DIG_P1, (uint8_t*)&_bmp280_calib.dig_P1, 2))
    return false;
  if (readBytes(BMP280_REGISTER_DIG_P2, (uint8_t*)&_bmp280_calib.dig_P2, 2))
    return false;
  if (readBytes(BMP280_REGISTER_DIG_P3, (uint8_t*)&_bmp280_calib.dig_P3, 2))
    return false;
  if (readBytes(BMP280_REGISTER_DIG_P4, (uint8_t*)&_bmp280_calib.dig_P4, 2))
    return false;
  if (readBytes(BMP280_REGISTER_DIG_P5, (uint8_t*)&_bmp280_calib.dig_P5, 2))
    return false;
  if (readBytes(BMP280_REGISTER_DIG_P6, (uint8_t*)&_bmp280_calib.dig_P6, 2))
    return false;
  if (readBytes(BMP280_REGISTER_DIG_P7, (uint8_t*)&_bmp280_calib.dig_P7, 2))
    return false;
  if (readBytes(BMP280_REGISTER_DIG_P8, (uint8_t*)&_bmp280_calib.dig_P8, 2))
    return false;
  if (readBytes(BMP280_REGISTER_DIG_P9, (uint8_t*)&_bmp280_calib.dig_P9, 2))
    return false;

  return true;
}

/*!
 * Reads the temperature from the device.
 * @return The temperature in degress celcius.
 */
bool Adafruit_BMP280::readTemperature(float *out) {
  int32_t var1, var2;
  int32_t adc_T;
  if (read24(BMP280_REGISTER_TEMPDATA, (uint32_t*)&adc_T))
    return false;

  adc_T >>= 4;

  var1 = ((((adc_T >> 3) - ((int32_t)_bmp280_calib.dig_T1 << 1))) *
          ((int32_t)_bmp280_calib.dig_T2)) >>
         11;

  var2 = (((((adc_T >> 4) - ((int32_t)_bmp280_calib.dig_T1)) *
            ((adc_T >> 4) - ((int32_t)_bmp280_calib.dig_T1))) >>
           12) *
          ((int32_t)_bmp280_calib.dig_T3)) >>
         14;

  t_fine = var1 + var2;

  float T = (t_fine * 5 + 128) >> 8;
  *out = T / 100;

  return true;
}

/*!
 * Reads the barometric pressure from the device.
 * @return Barometric pressure in Pa.
 */
bool Adafruit_BMP280::readPressure(float *out) {
  int64_t var1, var2, p;

  // Must be done first to get the t_fine variable set up
  if (!readTemperature((float*)&p))
    return false;

  int32_t adc_P;
  if (read24(BMP280_REGISTER_PRESSUREDATA, (uint32_t*)&adc_P))
    return false;

  adc_P >>= 4;

  var1 = ((int64_t)t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)_bmp280_calib.dig_P6;
  var2 = var2 + ((var1 * (int64_t)_bmp280_calib.dig_P5) << 17);
  var2 = var2 + (((int64_t)_bmp280_calib.dig_P4) << 35);
  var1 = ((var1 * var1 * (int64_t)_bmp280_calib.dig_P3) >> 8) +
         ((var1 * (int64_t)_bmp280_calib.dig_P2) << 12);
  var1 =
      (((((int64_t)1) << 47) + var1)) * ((int64_t)_bmp280_calib.dig_P1) >> 33;

  if (var1 == 0) {
    return false; // avoid exception caused by division by zero
  }
  p = 1048576 - adc_P;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (((int64_t)_bmp280_calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
  var2 = (((int64_t)_bmp280_calib.dig_P8) * p) >> 19;

  p = ((p + var1 + var2) >> 8) + (((int64_t)_bmp280_calib.dig_P7) << 4);
  *out = (float)p / 256;

  return true;
}

/*!
 * @brief Calculates the approximate altitude using barometric pressure and the
 * supplied sea level hPa as a reference.
 * @param seaLevelhPa
 *        The current hPa at sea level.
 * @return The approximate altitude above sea level in meters.
 */
bool Adafruit_BMP280::readAltitude(float *out, float seaLevelhPa) {
  float pressure;
  if (!readPressure(&pressure)) // in Si units for Pascal
    return false;

  pressure /= 100;

  *out = 44330 * (1.0 - pow(pressure / seaLevelhPa, 0.1903));

  return true;
}

/*!
 * Calculates the pressure at sea level (in hPa) from the specified altitude
 * (in meters), and atmospheric pressure (in hPa).
 * @param  altitude      Altitude in meters
 * @param  atmospheric   Atmospheric pressure in hPa
 * @return The approximate pressure
 */
float Adafruit_BMP280::seaLevelForAltitude(float altitude, float atmospheric) {
  // Equation taken from BMP180 datasheet (page 17):
  // http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf

  // Note that using the equation from wikipedia can give bad results
  // at high altitude.  See this thread for more information:
  // http://forums.adafruit.com/viewtopic.php?f=22&t=58064
  return atmospheric / pow(1.0 - (altitude / 44330.0), 5.255);
}

/*!
 *  @brief  Resets the chip via soft reset
 */
void Adafruit_BMP280::reset(void) {
  writeByte(BMP280_REGISTER_SOFTRESET, MODE_SOFT_RESET_CODE);
}

/*!
    @brief  Gets the most recent sensor event from the hardware status register.
    @return Sensor status as a byte.
 */
uint8_t Adafruit_BMP280::getStatus(void) {
  uint8_t res;
  readBytes(BMP280_REGISTER_STATUS, &res, 1);
  return res;
}

/*!
    @brief  Gets an Adafruit Unified Sensor object for the temp sensor component
    @return Adafruit_Sensor pointer to temperature sensor
 */
Adafruit_Sensor *Adafruit_BMP280::getTemperatureSensor(void) {
  return temp_sensor;
}

/*!
    @brief  Gets an Adafruit Unified Sensor object for the pressure sensor
   component
    @return Adafruit_Sensor pointer to pressure sensor
 */
Adafruit_Sensor *Adafruit_BMP280::getPressureSensor(void) {
  return pressure_sensor;
}

/**************************************************************************/
/*!
    @brief  Gets the sensor_t data for the BMP280's temperature sensor
*/
/**************************************************************************/
void Adafruit_BMP280_Temp::getSensor(sensor_t *sensor) {
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy(sensor->name, "BMP280", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name) - 1] = 0;
  sensor->version = 1;
  sensor->sensor_id = _sensorID;
  sensor->type = SENSOR_TYPE_AMBIENT_TEMPERATURE;
  sensor->min_delay = 0;
  sensor->max_value = -40.0; /* Temperature range -40 ~ +85 C  */
  sensor->min_value = +85.0;
  sensor->resolution = 0.01; /*  0.01 C */
}

/**************************************************************************/
/*!
    @brief  Gets the temperature as a standard sensor event
    @param  event Sensor event object that will be populated
    @returns True
*/
/**************************************************************************/
bool Adafruit_BMP280_Temp::getEvent(sensors_event_t *event) {
  /* Clear the event */
  memset(event, 0, sizeof(sensors_event_t));

  event->version = sizeof(sensors_event_t);
  event->sensor_id = _sensorID;
  event->type = SENSOR_TYPE_AMBIENT_TEMPERATURE;
  event->timestamp = millis();
  if (!_theBMP280->readTemperature(&event->temperature))
    return false;

  return true;
}

/**************************************************************************/
/*!
    @brief  Gets the sensor_t data for the BMP280's pressure sensor
*/
/**************************************************************************/
void Adafruit_BMP280_Pressure::getSensor(sensor_t *sensor) {
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy(sensor->name, "BMP280", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name) - 1] = 0;
  sensor->version = 1;
  sensor->sensor_id = _sensorID;
  sensor->type = SENSOR_TYPE_PRESSURE;
  sensor->min_delay = 0;
  sensor->max_value = 300.0; /* 300 ~ 1100 hPa  */
  sensor->min_value = 1100.0;
  sensor->resolution = 0.012; /* 0.12 hPa relative */
}

/**************************************************************************/
/*!
    @brief  Gets the pressure as a standard sensor event
    @param  event Sensor event object that will be populated
    @returns True
*/
/**************************************************************************/
bool Adafruit_BMP280_Pressure::getEvent(sensors_event_t *event) {
  /* Clear the event */
  memset(event, 0, sizeof(sensors_event_t));

  event->version = sizeof(sensors_event_t);
  event->sensor_id = _sensorID;
  event->type = SENSOR_TYPE_PRESSURE;
  event->timestamp = millis();
  if (!_theBMP280->readPressure(&event->pressure))
    return false;
  event->pressure /= 100; // convert Pa to hPa
  return true;
}
