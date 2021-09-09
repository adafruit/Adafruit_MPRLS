/*!
 * @file Adafruit_MPRLS.h
 *
 * Designed specifically to work with the MPRLS sensors from Adafruit
 * ----> https://www.adafruit.com/products/3965
 *
 * These sensors use I2C to communicate, 2 pins (SCL+SDA) are required
 * to interface with the breakout.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Written by Limor Fried/Ladyada for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <Adafruit_I2CDevice.h>

#define MPRLS_DEFAULT_ADDR (0x18)   ///< Most common I2C address
#define MPRLS_READ_TIMEOUT (20)     ///< millis
#define MPRLS_STATUS_POWERED (0x40) ///< Status SPI powered bit
#define MPRLS_STATUS_BUSY (0x20)    ///< Status busy bit
#define MPRLS_STATUS_FAILED (0x04)  ///< Status bit for integrity fail
#define MPRLS_STATUS_MATHSAT (0x01) ///< Status bit for math saturation
#define COUNTS_224 (16777216L)      ///< Constant: 2^24
#define PSI_to_HPA (68.947572932)   ///< Constant: PSI to HPA conversion factor
#define MPRLS_STATUS_MASK                                                      \
  (0b01100101) ///< Sensor status mask: only these bits are set

/**************************************************************************/
/*!
    @brief  Class that stores state and functions for interacting with MPRLS
   sensor IC
*/
/**************************************************************************/
class Adafruit_MPRLS {
public:
  Adafruit_MPRLS(int8_t reset_pin = -1, int8_t EOC_pin = -1,
                 uint16_t PSI_min = 0, uint16_t PSI_max = 25,
                 float OUTPUT_min = 10, float OUTPUT_max = 90,
                 float K = PSI_to_HPA);

  bool begin(uint8_t i2c_addr = MPRLS_DEFAULT_ADDR, TwoWire *twoWire = &Wire);

  uint8_t readStatus(void);
  float readPressure(void);

  uint8_t lastStatus; /*!< status byte after last operation */

private:
  Adafruit_I2CDevice *i2c_dev = NULL; ///< Pointer to I2C bus interface
  uint32_t readData(void);

  int8_t _reset, _eoc;
  uint16_t _PSI_min, _PSI_max;
  uint32_t _OUTPUT_min, _OUTPUT_max;
  float _K;
};
