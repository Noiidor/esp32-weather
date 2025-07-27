/*
 * Most of the code is based on https://github.com/utkumaden/esp-idf-bmx280
 * Huge thanks to the authors for an example
 * */
#include "bme280_types.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include <stdint.h>

const char *LOG_TAG = "BME280";

esp_err_t bme280_write_register(const bme280_handle *handle, bme280_reg_t reg,
                                uint8_t data) {
  uint8_t write_buf[2] = {reg, data};
  return i2c_master_transmit(handle->i2c_handle, write_buf, sizeof(write_buf),
                             -1);
}

esp_err_t bme280_read_registers(const bme280_handle *handle, bme280_reg_t reg,
                                uint8_t *data, size_t len) {
  return i2c_master_transmit_receive(handle->i2c_handle, (uint8_t *)&reg, 1,
                                     data, len, -1);
}

esp_err_t bme280_reset(const bme280_handle *handle) {

  ESP_LOGI(LOG_TAG, "Resetting...");

  esp_err_t err =
      bme280_write_register(handle, BME280_REG_RESET, BME280_RESET_BYTE);

  vTaskDelay(pdTICKS_TO_MS(10));

  return err;
}

esp_err_t bme280_calibrate(bme280_handle *handle) {
  ESP_LOGI(LOG_TAG, "Reading out calibration values.");

  esp_err_t err;
  uint8_t buf[26];

  const uint8_t reg_low = 0x88;

  err =
      i2c_master_transmit_receive(handle->i2c_handle, &reg_low, sizeof(reg_low),
                                  buf, sizeof(buf) / sizeof(buf[0]), -1);

  if (err != ESP_OK)
    return err;

  ESP_LOGI(LOG_TAG, "Read Low Bank.");

  handle->calib_data.T1 = buf[0] | (buf[1] << 8);
  handle->calib_data.T2 = buf[2] | (buf[3] << 8);
  handle->calib_data.T3 = buf[4] | (buf[5] << 8);
  handle->calib_data.P1 = buf[6] | (buf[7] << 8);
  handle->calib_data.P2 = buf[8] | (buf[9] << 8);
  handle->calib_data.P3 = buf[10] | (buf[11] << 8);
  handle->calib_data.P4 = buf[12] | (buf[13] << 8);
  handle->calib_data.P5 = buf[14] | (buf[15] << 8);
  handle->calib_data.P6 = buf[16] | (buf[17] << 8);
  handle->calib_data.P7 = buf[18] | (buf[19] << 8);
  handle->calib_data.P8 = buf[20] | (buf[21] << 8);
  handle->calib_data.P9 = buf[22] | (buf[23] << 8);
  handle->calib_data.H1 = buf[23];

  const uint8_t reg_hi = 0xE1;
  err = i2c_master_transmit_receive(handle->i2c_handle, &reg_hi, sizeof(reg_hi),
                                    buf, sizeof(buf) / sizeof(buf[0]), -1);

  if (err != ESP_OK)
    return err;

  ESP_LOGI(LOG_TAG, "Read High Bank.");

  handle->calib_data.H2 = buf[0] | (buf[1] << 8);
  handle->calib_data.H3 = buf[2];
  handle->calib_data.H4 = (buf[3] << 4) | (buf[4] & 0x0F);
  handle->calib_data.H5 = (buf[4] >> 4) | (buf[5] << 4);
  handle->calib_data.H6 = buf[6];

  return ESP_OK;
}

esp_err_t bme280_handle_init(bme280_handle *handle,
                             i2c_master_bus_handle_t bus_handle) {

  if (handle == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  bme280_handle hnld = {
      .i2c_handle = NULL,
      .calib_data = {},
  };

  *handle = hnld;

  i2c_device_config_t i2c_bme280_cfg = {.dev_addr_length = I2C_ADDR_BIT_LEN_7,
                                        .device_address = BME280_ADDRESS,
                                        .scl_speed_hz = BME280_CLK_HZ};

  esp_err_t err = i2c_master_bus_add_device(bus_handle, &i2c_bme280_cfg,
                                            &handle->i2c_handle);
  if (err) {
    return err;
  }

  err = bme280_reset(handle);
  if (err) {
    return err;
  }

  err = bme280_calibrate(handle);
  if (err) {
    return err;
  }

  return ESP_OK;
}

esp_err_t bme280_configure(const bme280_handle *handle, bme280_config_t *cfg) {
  if (cfg == NULL)
    return ESP_ERR_INVALID_ARG;

  uint8_t num = (cfg->t_sampling << 5) | (cfg->p_sampling << 2) | cfg->mode;

  esp_err_t err = bme280_write_register(handle, BME280_REG_CTRL_MEAS, num);
  if (err != 0) {
    return err;
  }

  num = (cfg->t_standby << 5) | (cfg->iir_filter << 2);

  err = bme280_write_register(handle, BME280_REG_CONFIG, num);
  if (err)
    return err;

  err = bme280_write_register(handle, BME280_REG_CTRL_HUM, num);
  if (err)
    return err;

  return ESP_OK;
}

esp_err_t bme280_read_chip_id(const bme280_handle *handle, uint8_t *chip_id) {

  const unsigned char chip_id_reg = 0xD0;
  uint8_t id;

  esp_err_t err = bme280_read_registers(handle, chip_id_reg, &id, sizeof(id));
  if (err)
    return err;

  *chip_id = id;

  return ESP_OK;
}

esp_err_t bme280_get_mode(const bme280_handle *handle, bme280_mode_t *mode) {
  uint8_t ctrl_mes;

  esp_err_t err = bme280_read_registers(handle, BME280_REG_CTRL_MEAS, &ctrl_mes,
                                        sizeof(ctrl_mes));
  if (err != 0) {
    return err;
  }

  ctrl_mes &= 3;

  switch (ctrl_mes) {
  default:
    *mode = ctrl_mes;
    break;
  case (BME280_MODE_FORCE + 1):
    *mode = BME280_MODE_FORCE;
    break;
  }

  return ESP_OK;
}

esp_err_t bme280_set_mode(const bme280_handle *handle, bme280_mode_t mode) {

  uint8_t ctrl_mes;

  esp_err_t err = bme280_read_registers(handle, BME280_REG_CTRL_MEAS, &ctrl_mes,
                                        sizeof(ctrl_mes));
  if (err != 0) {
    return err;
  }

  ctrl_mes = (ctrl_mes & (~3)) | mode;

  err = bme280_write_register(handle, BME280_REG_CTRL_MEAS, ctrl_mes);
  if (err != 0) {
    return err;
  }

  return ESP_OK;
}

// Low precision(32 bits) temperature compensation
int32_t bme280_compensate_T_int32(bme280_handle *handle, int32_t adc_T) {
  int32_t var1, var2, T;
  var1 = ((((adc_T >> 3) - ((int32_t)handle->calib_data.T1 << 1))) *
          ((int32_t)handle->calib_data.T2)) >>
         11;
  var2 = (((((adc_T >> 4) - ((int32_t)handle->calib_data.T1)) *
            ((adc_T >> 4) - ((int32_t)handle->calib_data.T1))) >>
           12) *
          ((int32_t)handle->calib_data.T3)) >>
         14;
  int32_t t_fine = var1 + var2;
  handle->t_fine = t_fine;
  T = (t_fine * 5 + 128) >> 8;
  return T;
}

// Should be divided by 100 to get temperature in degrees Celsius
esp_err_t bme280_read_temp(const bme280_handle *handle, int32_t *temp) {
  uint8_t temp_raw_buf[3];

  esp_err_t err = bme280_read_registers(handle, BME280_REG_TEMP_MSB,
                                        temp_raw_buf, sizeof(temp_raw_buf));
  if (err != 0) {
    return err;
  }

  int32_t t_raw =
      (temp_raw_buf[0] << 12) | (temp_raw_buf[1] << 4) | (temp_raw_buf[2] >> 4);

  int32_t t_calib = bme280_compensate_T_int32(handle, t_raw);

  *temp = t_calib;

  return ESP_OK;
}

uint32_t bme280_compensate_H_int32(const bme280_handle *handle, int32_t adc_H) {
  int32_t v_x1_u32r;

  // abomination
  v_x1_u32r = (handle->t_fine - ((int32_t)76800));
  v_x1_u32r = (((((adc_H << 14) - (((int32_t)handle->calib_data.H4) << 20) -
                  (((int32_t)handle->calib_data.H5) * v_x1_u32r)) +
                 ((int32_t)16384)) >>
                15) *
               (((((((v_x1_u32r * ((int32_t)handle->calib_data.H6)) >> 10) *
                    (((v_x1_u32r * ((int32_t)handle->calib_data.H3)) >> 11) +
                     ((int32_t)32768))) >>
                   10) +
                  ((int32_t)2097152)) *
                     ((int32_t)handle->calib_data.H2) +
                 8192) >>
                14));

  v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
                             ((int32_t)handle->calib_data.H1)) >>
                            4));

  v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
  v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);

  return (uint32_t)(v_x1_u32r >> 12);
}

// Should be divided by 1024 to get % relative humidity
esp_err_t bme280_read_hum(const bme280_handle *handle, int32_t *hum) {
  uint8_t hum_raw_buf[2];

  esp_err_t err = bme280_read_registers(handle, BME280_REG_HUM_MSB, hum_raw_buf,
                                        sizeof(hum_raw_buf));
  if (err)
    return err;

  int32_t h_raw = (hum_raw_buf[0] << 8) | (hum_raw_buf[1] << 8);

  *hum = (int32_t)bme280_compensate_H_int32(handle, h_raw);

  return ESP_OK;
}

// Low precision(32 bit) pressure compensation
uint32_t bme280_compensate_P_int32(const bme280_handle *handle, int32_t adc_P) {
  int32_t var1, var2;
  uint32_t p;

  var1 = ((handle->t_fine) >> 1) - (int32_t)(64000);
  var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t)handle->calib_data.P6);
  var2 = var2 + ((var1 * ((int32_t)handle->calib_data.P5)) << 1);
  var2 = (var2 >> 2) + (((int32_t)handle->calib_data.P4) << 16);
  var1 = (((handle->calib_data.P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) +
          ((((int32_t)handle->calib_data.P2) * var1) >> 1)) >>
         18;
  var1 = ((((32768 + var1)) * ((int32_t)handle->calib_data.P1)) >> 15);

  if (var1 == 0) {
    return 0; // avoid exception caused by division by zero
  }

  p = (((uint32_t)(((int32_t)1048576) - adc_P) - (var2 >> 12))) * 3125;
  if (p < 0x80000000) {
    p = (p << 1) / ((uint32_t)var1);
  } else {
    p = (p / (uint32_t)var1) * 2;
  }

  var1 = (((int32_t)handle->calib_data.P9) *
          ((int32_t)(((p >> 3) * (p >> 3)) >> 13))) >>
         12;

  var2 = (((int32_t)(p >> 2)) * ((int32_t)handle->calib_data.P8)) >> 13;

  p = (uint32_t)((int32_t)p + ((var1 + var2 + handle->calib_data.P7) >> 4));

  return p;
}

// Returns pressure in Pascals
esp_err_t bme280_read_pres(const bme280_handle *handle, int32_t *pres) {
  uint8_t pres_raw_buf[3];

  esp_err_t err = bme280_read_registers(handle, BME280_REG_PRES_MSB,
                                        pres_raw_buf, sizeof(pres_raw_buf));
  if (err)
    return err;

  int32_t p_raw =
      (pres_raw_buf[0] << 12) | (pres_raw_buf[1] << 4) | (pres_raw_buf[2] >> 4);

  *pres = (int32_t)bme280_compensate_P_int32(handle, p_raw);

  return ESP_OK;
}

// Burst read(most efficient)
esp_err_t bme280_read_all(const bme280_handle *handle, int32_t *temp,
                          int32_t *hum, int32_t *pres) {

  uint8_t raw_data_buf[8];
  esp_err_t err = bme280_read_registers(handle, BME280_REG_PRES_MSB,
                                        raw_data_buf, sizeof(raw_data_buf));
  if (err)
    return err;

  // Temp should be first to calculate t_fine
  int32_t t_raw =
      (raw_data_buf[3] << 12) | (raw_data_buf[4] << 4) | (raw_data_buf[5] >> 4);
  *temp = bme280_compensate_T_int32(handle, t_raw);

  int32_t p_raw =
      (raw_data_buf[0] << 12) | (raw_data_buf[1] << 4) | (raw_data_buf[2] >> 4);
  *pres = (int32_t)bme280_compensate_P_int32(handle, p_raw);

  int32_t h_raw = (raw_data_buf[6] << 8) | (raw_data_buf[7] << 8);
  *hum = (int32_t)bme280_compensate_H_int32(handle, h_raw);

  return ESP_OK;
}
