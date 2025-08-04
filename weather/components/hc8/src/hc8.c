#include "hc8.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "hal/uart_types.h"
#include "soc/gpio_num.h"
#include <stdint.h>

#define LOG_TAG "HC8"

// Datasheet: Typical 1s, max 3s
#define READ_TIMEOUT 3000

#define INTERVAL_MSG_LEN 16
#define QUERY_MSG_LEN 14

// -1 forces to set state in init()
// before calling reading functions
static uart_port_t uart_port = -1;

esp_err_t hc8_init(uart_port_t port, gpio_num_t tx_pin, gpio_num_t rx_pin) {
  uart_config_t uart_config = {.baud_rate = 9600,
                               .data_bits = UART_DATA_8_BITS,
                               .parity = UART_PARITY_DISABLE,
                               .stop_bits = UART_STOP_BITS_1,
                               .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};

  uart_port = port;

  ESP_LOGI(LOG_TAG, "Installing driver");
  esp_err_t err = uart_driver_install(uart_port, 256, 0, 0, NULL, 0);
  if (err)
    return err;

  ESP_LOGI(LOG_TAG, "Applying UART conf");
  err = uart_param_config(uart_port, &uart_config);
  if (err)
    return err;

  ESP_LOGI(LOG_TAG, "Setting UART pins");
  err = uart_set_pin(uart_port, tx_pin, rx_pin, UART_PIN_NO_CHANGE,
                     UART_PIN_NO_CHANGE);
  if (err)
    return err;

  return ESP_OK;
}

esp_err_t hc8_free() {
  if (uart_port == -1) {
    return ESP_ERR_INVALID_STATE;
  }
  esp_err_t err = uart_driver_delete(uart_port);
  if (err)
    return err;
  uart_port = -1;
  return ESP_OK;
}

esp_err_t hc8_read(uint8_t *buf, size_t len) {
  if (uart_port == -1) {
    return ESP_ERR_INVALID_STATE;
  }
  int bytes_read =
      uart_read_bytes(uart_port, buf, len, pdMS_TO_TICKS(READ_TIMEOUT));
  if (bytes_read == -1)
    return ESP_ERR_INVALID_ARG;
  if (bytes_read != len) {
    return ESP_ERR_TIMEOUT;
  }

  return ESP_OK;
}

esp_err_t hc8_read_co2(int *ppm) {
  uint8_t buf[INTERVAL_MSG_LEN];
  esp_err_t err = hc8_read(buf, sizeof(buf));
  if (err) {
    return err;
  }

  ESP_LOG_BUFFER_HEX(LOG_TAG, buf, sizeof(buf));

  if (buf[0] != 0x42 || buf[1] != 0x4D) {
    return ESP_ERR_INVALID_RESPONSE;
  }

  // PPM formula from datasheet
  *ppm = buf[6] * 256 + buf[7];

  return ESP_OK;
}

// WARN: After querying interval readings stop sending
// BUG: First time after sensor powerups, it doesnt send readings
esp_err_t hc8_query_co2(int *ppm) {
  if (uart_port == -1) {
    return ESP_ERR_INVALID_STATE;
  }

  const uint8_t query[5] = {0x64, 0x69, 0x03, 0x5E, 0x4E};
  int bytes_wrote = uart_write_bytes(uart_port, query, sizeof(query));
  if (bytes_wrote == -1) {
    return ESP_ERR_INVALID_ARG;
  }
  if (bytes_wrote != sizeof(query)) {
    return ESP_ERR_INVALID_SIZE;
  }

  uint8_t buf[QUERY_MSG_LEN];
  esp_err_t err = hc8_read(buf, sizeof(buf));
  if (err) {
    return err;
  }

  if (buf[0] != 0x64 || buf[1] != 0x69) {
    return ESP_ERR_INVALID_RESPONSE;
  }

  // PPM formula from datasheet
  *ppm = buf[5] * 256 + buf[4];

  return ESP_OK;
}
