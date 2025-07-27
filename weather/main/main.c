#include "bme280.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/i2c_types.h"
#include "driver/temperature_sensor.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
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

bme280_handle bme_handle;

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

  ESP_ERROR_CHECK(bme280_handle_init(&bme_handle, i2c_bus_handle));

  uint8_t chip_id;
  ESP_ERROR_CHECK(bme280_read_chip_id(&bme_handle, &chip_id));

  ESP_LOGI("SETUP.BME280", "Chip ID: %x", chip_id);

  bme280_config_t bme_cfg = bme280_DEFAULT_CONFIG;
  bme_cfg.mode = BME280_MODE_CYCLE;

  ESP_ERROR_CHECK(bme280_configure(&bme_handle, &bme_cfg));
}

void loop(void) {
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

    int32_t temp;
    err = bme280_read_temp(&bme_handle, &temp);
    if (err != 0) {
      ESP_LOGI("MAIN.BME280", "error: %s", esp_err_to_name(err));
    }
    ESP_LOGI("MAIN.BME280", "temp: %d", (int)temp);
  }
}

void app_main(void) {
  setup();
  loop();
}
