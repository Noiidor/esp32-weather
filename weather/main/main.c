#include "bme280.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/i2c_slave.h"
#include "driver/i2c_types.h"
#include "driver/rmt_types.h"
#include "driver/temperature_sensor.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_wifi_ap_get_sta_list.h"
#include "esp_wifi_default.h"
#include "esp_wifi_types_generic.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "hal/gpio_types.h"
#include "hal/i2c_types.h"
#include "led_strip.h"
#include "led_strip_rmt.h"
#include "led_strip_types.h"
#include "portmacro.h"
#include "soc/clk_tree_defs.h"
#include "soc/gpio_num.h"
#include "stdint.h"
#include <stdint.h>
#include <stdio.h>

static led_strip_handle_t strip_handle;
static temperature_sensor_handle_t temp_handle;

static i2c_master_bus_handle_t i2c_bus_handle;
static i2c_master_dev_handle_t i2c_bme280_handle;

esp_err_t bme280_write_register(uint8_t reg, uint8_t data) {
  uint8_t write_buf[2] = {reg, data};
  return i2c_master_transmit(i2c_bme280_handle, write_buf, sizeof(write_buf),
                             -1);
}

esp_err_t bme280_read_registers(uint8_t reg, uint8_t *data, size_t len) {
  return i2c_master_transmit_receive(i2c_bme280_handle, &reg, 1, data, len, -1);
}

int lerp_int(int a, int b, float x) { return (1 - x) * a + x * b; }

double scale_value(double m, double r_min, double r_max, double t_min,
                   double t_max) {
  return ((m - r_min) / (r_max - r_min)) * (t_max - t_min) + t_min;
}

int32_t BME280_compensate_T_int32(int32_t adc_T) {
  int32_t var1, var2, T;
  var1 = ((((adc_T >> 3) - ((int32_t)calib_data.T1 << 1))) *
          ((int32_t)calib_data.T2)) >>
         11;
  var2 = (((((adc_T >> 4) - ((int32_t)calib_data.T1)) *
            ((adc_T >> 4) - ((int32_t)calib_data.T1))) >>
           12) *
          ((int32_t)calib_data.T3)) >>
         14;
  int32_t t_fine = var1 + var2;
  T = (t_fine * 5 + 128) >> 8;
  return T;
}

// uint32_t bme280_compensate_H_int32(int32_t adc_H) {
//   int32_t v_x1_u32r;
//   v_x1_u32r = (bmx280->t_fine - ((int32_t)76800));
//   v_x1_u32r = (((((adc_H << 14) - (((int32_t)bmx280->cmps.H4) << 20) -
//                   (((int32_t)bmx280->cmps.H5) * v_x1_u32r)) +
//                  ((int32_t)16384)) >>
//                 15) *
//                (((((((v_x1_u32r * ((int32_t)bmx280->cmps.H6)) >> 10) *
//                     (((v_x1_u32r * ((int32_t)bmx280->cmps.H3)) >> 11) +
//                      ((int32_t)32768))) >>
//                    10) +
//                   ((int32_t)2097152)) *
//                      ((int32_t)bmx280->cmps.H2) +
//                  8192) >>
//                 14));
//   v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
//                              ((int32_t)bmx280->cmps.H1)) >>
//                             4));
//   v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
//   v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
//   return (uint32_t)(v_x1_u32r >> 12);
// }

esp_err_t bme280_calibrate() {
  ESP_LOGI("bmx280", "Reading out calibration values...");

  esp_err_t err;
  uint8_t buf[26];

  const uint8_t reg_low = 0x88;

  err =
      i2c_master_transmit_receive(i2c_bme280_handle, &reg_low, sizeof(reg_low),
                                  buf, sizeof(buf) / sizeof(buf[0]), -1);

  if (err != ESP_OK)
    return err;

  ESP_LOGI("bmx280", "Read Low Bank.");

  calib_data.T1 = buf[0] | (buf[1] << 8);
  calib_data.T2 = buf[2] | (buf[3] << 8);
  calib_data.T3 = buf[4] | (buf[5] << 8);
  calib_data.P1 = buf[6] | (buf[7] << 8);
  calib_data.P2 = buf[8] | (buf[9] << 8);
  calib_data.P3 = buf[10] | (buf[11] << 8);
  calib_data.P4 = buf[12] | (buf[13] << 8);
  calib_data.P5 = buf[14] | (buf[15] << 8);
  calib_data.P6 = buf[16] | (buf[17] << 8);
  calib_data.P7 = buf[18] | (buf[19] << 8);
  calib_data.P8 = buf[20] | (buf[21] << 8);
  calib_data.P9 = buf[22] | (buf[23] << 8);

  calib_data.H1 = buf[23];

  const uint8_t reg_hi = 0xE1;
  err = i2c_master_transmit_receive(i2c_bme280_handle, &reg_hi, sizeof(reg_hi),
                                    buf, sizeof(buf) / sizeof(buf[0]), -1);

  if (err != ESP_OK)
    return err;

  ESP_LOGI("bmx280", "Read High Bank.");

  calib_data.H2 = buf[0] | (buf[1] << 8);
  calib_data.H3 = buf[2];
  calib_data.H4 = (buf[3] << 4) | (buf[4] & 0x0F);
  calib_data.H5 = (buf[4] >> 4) | (buf[5] << 4);
  calib_data.H6 = buf[6];

  return ESP_OK;
}

esp_err_t bme280_get_mode(bme280_mode_t *mode) {
  const unsigned char mode_reg = 0xF4;

  uint8_t ctrl_mes;

  esp_err_t err = i2c_master_transmit_receive(
      i2c_bme280_handle, &mode_reg, sizeof(mode_reg), &ctrl_mes, 1, -1);
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

esp_err_t bme280_set_mode(bme280_mode_t mode) {
  const unsigned char mode_reg = 0xF4;

  uint8_t ctrl_mes;

  esp_err_t err = i2c_master_transmit_receive(
      i2c_bme280_handle, &mode_reg, sizeof(mode_reg), &ctrl_mes, 1, -1);
  if (err != 0) {
    return err;
  }

  ctrl_mes = (ctrl_mes & (~3)) | mode;

  err = bme280_write_register(mode_reg, ctrl_mes);
  if (err != 0) {
    return err;
  }

  return ESP_OK;
}

esp_err_t bme280_reset() {
  const unsigned char reset_reg = 0xE0;
  const unsigned char reset_byte = 0xB6;
  const uint8_t data[2] = {reset_reg, reset_byte};

  return i2c_master_transmit(i2c_bme280_handle, data, sizeof(data), -1);
}

esp_err_t bme280_configure(bme280_config_t *cfg) {
  if (cfg == NULL)
    return ESP_ERR_INVALID_ARG;

  uint8_t num = (cfg->t_sampling << 5) | (cfg->p_sampling << 2) | cfg->mode;

  const unsigned char meas_ctrl_reg = 0xF4;

  esp_err_t err = bme280_write_register(meas_ctrl_reg, num);
  if (err != 0) {
    return err;
  }

  num = (cfg->t_standby << 5) | (cfg->iir_filter << 2);

  const unsigned char config_reg = 0xF5;
  const uint8_t config_data[2] = {config_reg, num};

  err = i2c_master_transmit(i2c_bme280_handle, config_data, sizeof(config_data),
                            -1);

  if (err)
    return err;

  const unsigned char hum_ctrl_reg = 0xF2;
  err = bme280_write_register(hum_ctrl_reg, num);
  if (err)
    return err;

  return ESP_OK;
}

esp_err_t bme280_read_chip_id(uint8_t *chip_id) {

  const unsigned char chip_id_reg = 0xD0;
  uint8_t id;

  esp_err_t err =
      i2c_master_transmit_receive(i2c_bme280_handle, &chip_id_reg,
                                  sizeof(chip_id_reg), &id, sizeof(id), -1);
  if (err)
    return err;

  *chip_id = id;

  return ESP_OK;
}

static void setup(void) {
  ESP_LOGI("SETUP", "Configuring LED");
  /* LED strip initialization with the GPIO and pixels number*/
  led_strip_config_t strip_config = {
      .strip_gpio_num = GPIO_NUM_48,
      .max_leds = 1, // at least one LED on board
  };

  led_strip_rmt_config_t strip_rtm_config = {
      .clk_src = RMT_CLK_SRC_DEFAULT,
  };

  ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &strip_rtm_config,
                                           &strip_handle));

  led_strip_clear(strip_handle);

  ESP_LOGI("SETUP", "Configuring temp sensor");

  temperature_sensor_config_t temp_conf =
      TEMPERATURE_SENSOR_CONFIG_DEFAULT(10, 40);

  ESP_ERROR_CHECK(temperature_sensor_install(&temp_conf, &temp_handle));

  temperature_sensor_enable(temp_handle);

  ESP_LOGI("SETUP", "Configuring I2C");

  i2c_master_bus_config_t i2c_bus_cfg = {.clk_source = I2C_CLK_SRC_DEFAULT,
                                         .i2c_port = I2C_NUM_0,
                                         .sda_io_num = GPIO_NUM_8,
                                         .scl_io_num = GPIO_NUM_9,
                                         .glitch_ignore_cnt = 7,
                                         .flags.enable_internal_pullup = true};

  ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus_handle));

  i2c_device_config_t i2c_bme280_cfg = {.dev_addr_length = I2C_ADDR_BIT_LEN_7,
                                        .device_address = 0x76,
                                        .scl_speed_hz = 100000};

  ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus_handle, &i2c_bme280_cfg,
                                            &i2c_bme280_handle));

  // esp_err_t err = i2c_master_probe(i2c_bus_handle, 0x76, 1000);
  // if (err != 0) {
  //   ESP_LOGI("MAIN.I2C", "error: %s", esp_err_to_name(probe_err));
  // }
  //
  ESP_ERROR_CHECK(bme280_reset());

  vTaskDelay(pdMS_TO_TICKS(10));

  ESP_ERROR_CHECK(bme280_calibrate());

  ESP_LOG_BUFFER_HEX("SETUP.BME280", &calib_data, sizeof(calib_data));

  uint8_t chip_id;
  ESP_ERROR_CHECK(bme280_read_chip_id(&chip_id));

  ESP_LOGI("SETUP.BME280", "Chip ID: %x", chip_id);

  bme280_config_t bme_cfg = bme280_DEFAULT_CONFIG;
  bme_cfg.mode = BME280_MODE_CYCLE;

  ESP_ERROR_CHECK(bme280_configure(&bme_cfg));

  // ESP_ERROR_CHECK(bme280_set_mode(3));
}

esp_err_t bme280_read_temp(int32_t *temp) {
  const unsigned char temp_reg = 0xFA;

  uint8_t buf[3];

  esp_err_t err = i2c_master_transmit_receive(i2c_bme280_handle, &temp_reg,
                                              sizeof(temp_reg), buf,
                                              sizeof(buf) / sizeof(buf[0]), -1);
  if (err != 0) {
    return err;
  }

  int32_t t_raw = (buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4);

  int32_t t_calib = BME280_compensate_T_int32(t_raw);

  *temp = t_calib;

  return ESP_OK;
}

esp_err_t bme280_read_hum(int32_t *hum) {
  const unsigned char hum_reg = 0xFD;

  uint8_t buf[2];

  esp_err_t err =
      i2c_master_transmit_receive(i2c_bme280_handle, &hum_reg, sizeof(hum_reg),
                                  buf, sizeof(buf) / sizeof(buf[0]), -1);
  if (err != 0) {
    return err;
  }

  int32_t h_raw = (buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4);

  int32_t t_calib = BME280_compensate_T_int32(h_raw);

  *hum = t_calib;

  return ESP_OK;
}

void loop(void) {
  uint8_t prev_color[3] = {0};

  while (1) {

    int light = 0;
    esp_err_t err;
    uint8_t addr = 0x76;
    err = i2c_master_probe(i2c_bus_handle, addr, 1000);
    if (err != 0) {
      ESP_LOGI("MAIN.I2C", "error: %s", esp_err_to_name(err));
    } else {
      light = 100;
    }

    led_strip_set_pixel(strip_handle, 0, light, light, light);
    led_strip_refresh(strip_handle);

    vTaskDelay(500 / portTICK_PERIOD_MS);

    err = bme280_set_mode(3);
    if (err != 0) {
      ESP_LOGI("MAIN.BME280", "error: %s", esp_err_to_name(err));
    }

    int32_t temp;
    err = bme280_read_temp(&temp);
    if (err != 0) {
      ESP_LOGI("MAIN.BME280", "error: %s", esp_err_to_name(err));
    }
    ESP_LOGI("MAIN.BME280", "temp: %d", (int)temp);

    bme280_mode_t mode;
    err = bme280_get_mode(&mode);
    if (err != 0) {
      ESP_LOGI("MAIN.BME280", "error: %s", esp_err_to_name(err));
    }
    ESP_LOGI("MAIN.BME280", "mode %s", bme280mode_to_string(mode));
  }
}

void app_main(void) {
  setup();

  loop();
}
