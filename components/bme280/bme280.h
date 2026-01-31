#ifndef BME280_H
#define BME280_H

#include "driver/i2c.h"

#define BME280_ADDR 0x76

typedef struct {
    float temperature;
    float humidity;
    float pressure;
} bme280_data_t;

// Cấu trúc lưu thông số hiệu chuẩn
typedef struct {
    uint16_t dig_T1; int16_t dig_T2, dig_T3;
    uint16_t dig_P1; int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
    uint8_t  dig_H1; int16_t dig_H2; uint8_t  dig_H3; int16_t dig_H4, dig_H5; int8_t  dig_H6;
} bme280_calib_t;

esp_err_t bme280_init(i2c_port_t port);
esp_err_t bme280_read_data(i2c_port_t port, bme280_data_t *data);

#endif