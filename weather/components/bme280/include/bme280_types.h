#ifndef BME280_TYPES_H
#define BME280_TYPES_H

#include "driver/i2c_master.h"
#include "driver/i2c_types.h"
#include <stdint.h>

#define BME280_ADDRESS 0x76
#define BME280_CLK_HZ 100000

#define BME280_RESET_BYTE 0xB6

typedef enum {
  /**
   * @brief Start address of the first block of calibration data.
   */
  BME280_REG_CALIB_TP_START = 0x88,

  /**
   * @brief Chip ID register. Contains the fixed value 0x60 for the BME280.
   */
  BME280_REG_CHIP_ID = 0xD0,

  /**
   * @brief Software reset register. Writing 0xB6 to this register performs a
   */
  BME280_REG_RESET = 0xE0,

  /**
   * @brief Start address of the second block of calibration data (for
   * humidity).
   */
  BME280_REG_CALIB_H_START = 0xE1,

  /**
   * @brief Humidity control register. Controls the oversampling of humidity
   * data.
   */
  BME280_REG_CTRL_HUM = 0xF2,

  /**
   * @brief Measurement control register. Sets the sensor mode and oversampling
   * for temperature and pressure.
   */
  BME280_REG_CTRL_MEAS = 0xF4,

  /**
   * @brief Configuration register. Sets the standby time in normal mode and the
   * IIR filter coefficient.
   */
  BME280_REG_CONFIG = 0xF5,

  /**
   * @brief Start address of the temperature measurement data (MSB, LSB, XLSB).
   */
  BME280_REG_TEMP_MSB = 0xFA,

  BME280_REG_HUM_MSB = 0xFD,

  BME280_REG_PRES_MSB = 0xF7

} bme280_reg_t;

typedef enum bme280_mode_t {
  /** Sensor does no measurements. */
  BME280_MODE_SLEEP = 0,
  /** Sensor is in a forced measurement cycle. Sleeps after finishing. */
  BME280_MODE_FORCE = 1,
  /** Sensor does measurements. Never sleeps. */
  BME280_MODE_CYCLE = 3,
} bme280_mode_t;

// const char *bme280mode_to_string(enum bme280_mode_t mode) {
//   switch (mode) {
//   case BME280_MODE_SLEEP:
//     return "BME280_MODE_SLEEP";
//   case BME280_MODE_FORCE:
//     return "BME280_MODE_FORCE";
//   case BME280_MODE_CYCLE:
//     return "BME280_MODE_CYCLE";
//   }
//   return "UNKNOWN";
// }

typedef enum bme280_tsmpl_t {
  bme280_TEMPERATURE_OVERSAMPLING_NONE = 0x0,
  bme280_TEMPERATURE_OVERSAMPLING_X1,
  bme280_TEMPERATURE_OVERSAMPLING_X2,
  bme280_TEMPERATURE_OVERSAMPLING_X4,
  bme280_TEMPERATURE_OVERSAMPLING_X8,
  bme280_TEMPERATURE_OVERSAMPLING_X16,
} bme280_tsmpl_t;

typedef enum bme280_psmpl_t {
  bme280_PRESSURE_OVERSAMPLING_NONE = 0x0,
  bme280_PRESSURE_OVERSAMPLING_X1,
  bme280_PRESSURE_OVERSAMPLING_X2,
  bme280_PRESSURE_OVERSAMPLING_X4,
  bme280_PRESSURE_OVERSAMPLING_X8,
  bme280_PRESSURE_OVERSAMPLING_X16,
} bme280_psmpl_t;

typedef enum bme280_hsmpl_t {
  bme280_HUMIDITY_OVERSAMPLING_NONE = 0x0,
  bme280_HUMIDITY_OVERSAMPLING_X1,
  bme280_HUMIDITY_OVERSAMPLING_X2,
  bme280_HUMIDITY_OVERSAMPLING_X4,
  bme280_HUMIDITY_OVERSAMPLING_X8,
  bme280_HUMIDITY_OVERSAMPLING_X16,
} bme280_hsmpl_t;

typedef enum bme280_tstby_t {
  bme280_STANDBY_0M5 = 0x0,
  bme280_STANDBY_62M5,
  bme280_STANDBY_125M,
  bme280_STANDBY_250M,
  bme280_STANDBY_500M,
  bme280_STANDBY_1000M,
  BME280_STANDBY_10M,
  BME280_STANDBY_20M,
  BMP280_STANDBY_2000M = BME280_STANDBY_10M,
  BMP280_STANDBY_4000M = BME280_STANDBY_20M,
} bme280_tstby_t;

typedef enum bme280_iirf_t {
  bme280_IIR_NONE = 0x0,
  bme280_IIR_X2,
  bme280_IIR_X4,
  bme280_IIR_X8,
  bme280_IIR_X16,
} bme280_iirf_t;

typedef struct bme280_config_t {
  bme280_tsmpl_t t_sampling;
  bme280_psmpl_t p_sampling;
  bme280_tstby_t t_standby;
  bme280_iirf_t iir_filter;
  bme280_hsmpl_t h_sampling;
  bme280_mode_t mode;
} bme280_config_t;

#define bme280_DEFAULT_TEMPERATURE_OVERSAMPLING                                \
  bme280_TEMPERATURE_OVERSAMPLING_X16
#define bme280_DEFAULT_PRESSURE_OVERSAMPLING bme280_PRESSURE_OVERSAMPLING_X16
#define bme280_DEFAULT_STANDBY BME280_STANDBY_20M
#define bme280_DEFAULT_IIR bme280_IIR_X16
#define bme280_DEFAULT_HUMIDITY_OVERSAMPLING bme280_HUMIDITY_OVERSAMPLING_X16
#define bme280_DEFAULT_CONFIG                                                  \
  ((bme280_config_t){bme280_DEFAULT_TEMPERATURE_OVERSAMPLING,                  \
                     bme280_DEFAULT_PRESSURE_OVERSAMPLING,                     \
                     bme280_DEFAULT_STANDBY, bme280_DEFAULT_IIR,               \
                     bme280_DEFAULT_HUMIDITY_OVERSAMPLING, BME280_MODE_CYCLE})

typedef struct bme280_calib_data {
  uint16_t T1;
  int16_t T2;
  int16_t T3;
  uint16_t P1;
  int16_t P2;
  int16_t P3;
  int16_t P4;
  int16_t P5;
  int16_t P6;
  int16_t P7;
  int16_t P8;
  int16_t P9;
  uint8_t H1;
  int16_t H2;
  uint8_t H3;
  int16_t H4;
  int16_t H5;
  int8_t H6;
} bme280_calib_data;

typedef struct bme280_handle {
  i2c_master_dev_handle_t i2c_handle;
  bme280_calib_data calib_data;
  int32_t t_fine;
} bme280_handle;

#endif
