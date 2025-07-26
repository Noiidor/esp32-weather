#pragma once

#include <stdint.h>

typedef enum bme280_mode_t {
  /** Sensor does no measurements. */
  BME280_MODE_SLEEP = 0,
  /** Sensor is in a forced measurement cycle. Sleeps after finishing. */
  BME280_MODE_FORCE = 1,
  /** Sensor does measurements. Never sleeps. */
  BME280_MODE_CYCLE = 3,
} bme280_mode_t;

const char *bme280mode_to_string(enum bme280_mode_t mode) {
  switch (mode) {
  case BME280_MODE_SLEEP:
    return "BME280_MODE_SLEEP";
  case BME280_MODE_FORCE:
    return "BME280_MODE_FORCE";
  case BME280_MODE_CYCLE:
    return "BME280_MODE_CYCLE";
  }
  return "UNKNOWN";
}

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
                     bme280_DEFAULT_HUMIDITY_OVERSAMPLING})

struct bme280_calib_data {
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
} calib_data;
