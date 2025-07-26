
#pragma once

#include "bme280_types.h"
#include "esp_err.h"

// esp_err_t bme280_read_temp(int32_t *temp);
// esp_err_t bme280_read_hum(int32_t *hum);
// esp_err_t bme280_read_pres(int32_t *pres);

esp_err_t bme280_read(int32_t *temp, int32_t *hum, int32_t *pres);
