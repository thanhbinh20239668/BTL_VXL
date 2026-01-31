#ifndef DS3231_H
#define DS3231_H

#include "driver/i2c.h"
#include <time.h>

#define DS3231_ADDR 0x68

esp_err_t ds3231_init(i2c_port_t port);
esp_err_t ds3231_get_time(i2c_port_t port, struct tm *timeinfo);
esp_err_t ds3231_set_time(i2c_port_t port, struct tm *timeinfo);

#endif