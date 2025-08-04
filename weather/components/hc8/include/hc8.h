#ifndef HC8_H
#define HC8_H

#include "esp_err.h"
#include "hal/uart_types.h"
#include "soc/gpio_num.h"

esp_err_t hc8_init(uart_port_t port, gpio_num_t tx_pin, gpio_num_t rx_pin);
esp_err_t hc8_free();

esp_err_t hc8_read_co2(int *co2);
esp_err_t hc8_query_co2(int *co2);

#endif
