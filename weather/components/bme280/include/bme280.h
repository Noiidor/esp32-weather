#ifndef BME280_H
#define BME280_H

#include "bme280_types.h"
#include "esp_err.h"
#include <stdint.h>

esp_err_t bme280_handle_init(bme280_handle *handle,
                             i2c_master_bus_handle_t bus_handle);

esp_err_t bme280_read_temp(const bme280_handle *handle, int32_t *temp);
esp_err_t bme280_read_hum(const bme280_handle *handle, int32_t *hum);
esp_err_t bme280_read_pres(const bme280_handle *handle, int32_t *pres);
esp_err_t bme280_read_all(const bme280_handle *handle, int32_t *temp,
                          int32_t *hum, int32_t *pres);

esp_err_t bme280_get_mode(const bme280_handle *handle, bme280_mode_t *mode);
esp_err_t bme280_set_mode(const bme280_handle *handle, bme280_mode_t mode);
esp_err_t bme280_read_chip_id(const bme280_handle *handle, uint8_t *chip_id);
esp_err_t bme280_configure(const bme280_handle *handle, bme280_config_t *cfg);

#endif
