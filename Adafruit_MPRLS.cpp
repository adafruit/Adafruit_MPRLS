/*!
 * @file Adafruit_MPRLS.cpp
 *
 * @mainpage Adafruit MPRLS Pressure sensor
 *
 * @section intro_sec Introduction
 *
 * Designed specifically to work with the MPRLS sensor from Adafruit
 * ----> https://www.adafruit.com/products/3965
 *
 * These sensors use I2C to communicate, 2 pins (SCL+SDA) are required
 * to interface with the breakout.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * @section dependencies Dependencies
 *
 *
 * @section author Author
 *
 * Written by Limor Fried/Ladyada for Adafruit Industries.
 *
 * @section license License
 *
 * MIT license, all text here must be included in any redistribution.
 *
 * @section changes Changes
 *
 * Changes by arkhipenko (https://github.com/arkhipenko) (December 2020)
 * Overall, with all the defaults this should be 99% backwards compatible and
 * could be a drop-in replacement. The 1% difference is that the library can now
 * return NAN due to a timeout
 *   - added parameters to constructor to support different transfer function
 * curves and a factor for conversion to desired units
 *   - PSI_min and PSI_max are 16 bit unsigned to support values > 255
 *   - readPressure(void) method calculates based on the provided curve values,
 *     and converts to desired units
 *   - readData(void) method may return NAN in case of timeout (20 millis
 * currently - could be changed)
 *   - public variable lastStatus could be accessed to check the error bits in
 * case of a NAN value
 *   - begin() method updates lastStatus, so in case of a failure, the reason
 * could be checked explicitly success is "true" if status ==
 * MPRLS_STATUS_POWERED and no other bits are set
 */

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "Adafruit_MPRLS.h"

/**************************************************************************/
/*!
    @brief constructor initializes default configuration value
    @param reset_pin Optional hardware reset pin, default set to -1 to skip
    @param EOC_pin Optional End-of-Convert indication pin, default set to -1 to
   skip
    @param PSI_min The minimum PSI measurement range of the sensor, default 0
    @param PSI_max The maximum PSI measurement range of the sensor, default 25
    @param OUTPUT_min The minimum transfer function curve value in %, default
   10%
    @param OUTPUT_max The maximum transfer function curve value in %, default
   90%
    @param K Conversion Factor to desired units, default is PSI to HPA
*/
/**************************************************************************/
Adafruit_MPRLS::Adafruit_MPRLS(int8_t reset_pin, int8_t EOC_pin,
                               uint16_t PSI_min, uint16_t PSI_max,
                               float OUTPUT_min, float OUTPUT_max, float K) {

  _reset = reset_pin;
  _eoc = EOC_pin;
  _PSI_min = PSI_min;
  _PSI_max = PSI_max;
  _OUTPUT_min = (uint32_t)((float)COUNTS_224 * (OUTPUT_min / 100.0) + 0.5);
  _OUTPUT_max = (uint32_t)((float)COUNTS_224 * (OUTPUT_max / 100.0) + 0.5);
  _K = K;
}

/**************************************************************************/
/*!
    @brief  setup and initialize communication with the hardware
    @param i2c_addr The I2C address for the sensor (default is 0x18)
    @param twoWire Optional pointer to the desired TwoWire I2C object. Defaults
   to &Wire
    @returns True on success, False if sensor not found
*/
/**************************************************************************/
boolean Adafruit_MPRLS::begin(uint8_t i2c_addr, TwoWire *twoWire) {
  if (i2c_dev)
    delete i2c_dev;
  i2c_dev = new Adafruit_I2CDevice(i2c_addr, twoWire);
  if (!i2c_dev->begin())
    return false;

  if (_reset != -1) {
    pinMode(_reset, OUTPUT);
    digitalWrite(_reset, HIGH);
    digitalWrite(_reset, LOW);
    delay(10);
    digitalWrite(_reset, HIGH);
  }
  if (_eoc != -1) {
    pinMode(_eoc, INPUT);
  }

  delay(10); // startup timing

  // Serial.print("Status: ");
  // Serial.println(stat);
  return ((readStatus() & MPRLS_STATUS_MASK) == MPRLS_STATUS_POWERED);
}

/**************************************************************************/
/*!
    @brief Read and calculate the pressure
    @returns The measured pressure, in hPa on success, NAN on failure

*/
/**************************************************************************/
float Adafruit_MPRLS::readPressure(void) {
  uint32_t raw_psi = readData();
  if (raw_psi == 0xFFFFFFFF || _OUTPUT_min == _OUTPUT_max) {
    return NAN;
  }

  // All is good, calculate and convert to desired units using provided factor
  // use the 10-90 calibration curve by default or whatever provided by the user
  float psi = (raw_psi - _OUTPUT_min) * (_PSI_max - _PSI_min);
  psi /= (float)(_OUTPUT_max - _OUTPUT_min);
  psi += _PSI_min;
  // convert to desired units
  return psi * _K;
}

/**************************************************************************/
/*!
    @brief Read 24 bits of measurement data from the device
    @returns -1 on failure (check status) or 24 bits of raw ADC reading
*/
/**************************************************************************/
uint32_t Adafruit_MPRLS::readData(void) {
  uint8_t buffer[4] = {0xAA, 0, 0, 0};

  // Request data
  i2c_dev->write(buffer, 3);

  // Use the gpio to tell end of conversion
  uint32_t t = millis();
  if (_eoc != -1) {
    while (!digitalRead(_eoc)) {
      if (millis() - t > MPRLS_READ_TIMEOUT)
        return 0xFFFFFFFF; // timeout
    }
  } else {
    // check the status byte
    //    uint8_t stat;
    while ((lastStatus = readStatus()) & MPRLS_STATUS_BUSY) {
      // Serial.print("Status: "); Serial.println(stat, HEX);
      if (millis() - t > MPRLS_READ_TIMEOUT)
        return 0xFFFFFFFF; // timeout
    }
  }

  // Read status byte and data
  i2c_dev->read(buffer, 4);

  // check status byte
  if (buffer[0] & MPRLS_STATUS_MATHSAT) {
    return 0xFFFFFFFF;
  }
  if (buffer[0] & MPRLS_STATUS_FAILED) {
    return 0xFFFFFFFF;
  }

  // all good, return data
  return (uint32_t(buffer[1]) << 16) | (uint32_t(buffer[2]) << 8) |
         (uint32_t(buffer[3]));
}

/**************************************************************************/
/*!
    @brief Read just the status byte, see datasheet for bit definitions
    @returns 8 bits of status data
*/
/**************************************************************************/
uint8_t Adafruit_MPRLS::readStatus(void) {
  uint8_t buffer[1];
  i2c_dev->read(buffer, 1);
  return buffer[0];
}
