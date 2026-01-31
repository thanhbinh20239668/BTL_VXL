#include "bme280.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static bme280_calib_t cal;
static int32_t t_fine; // Biến trung gian dùng cho bù trừ

// Hàm đọc thông số hiệu chuẩn từ cảm biến
void read_calibration_data(i2c_port_t port) {
    uint8_t d[24], h[7];
    i2c_master_write_read_device(port, BME280_ADDR, (uint8_t[]){0x88}, 1, d, 24, 100);
    cal.dig_T1 = (d[1] << 8) | d[0]; cal.dig_T2 = (d[3] << 8) | d[2]; cal.dig_T3 = (d[5] << 8) | d[4];
    cal.dig_P1 = (d[7] << 8) | d[6]; cal.dig_P2 = (d[9] << 8) | d[8]; cal.dig_P3 = (d[11] << 8) | d[10];
    cal.dig_P4 = (d[13] << 8) | d[12]; cal.dig_P5 = (d[15] << 8) | d[14]; cal.dig_P6 = (d[17] << 8) | d[16];
    cal.dig_P7 = (d[19] << 8) | d[18]; cal.dig_P8 = (d[21] << 8) | d[20]; cal.dig_P9 = (d[23] << 8) | d[22];
    i2c_master_write_read_device(port, BME280_ADDR, (uint8_t[]){0xA1}, 1, &cal.dig_H1, 1, 100);
    i2c_master_write_read_device(port, BME280_ADDR, (uint8_t[]){0xE1}, 1, h, 7, 100);
    cal.dig_H2 = (h[1] << 8) | h[0]; cal.dig_H3 = h[2];
    cal.dig_H4 = (h[3] << 4) | (h[4] & 0x0F); cal.dig_H5 = (h[5] << 4) | (h[4] >> 4); cal.dig_H6 = h[6];
}

esp_err_t bme280_init(i2c_port_t port) {
    read_calibration_data(port);
    uint8_t config[] = {0xF2, 0x01, 0xF4, 0x25}; 
    return i2c_master_write_to_device(port, BME280_ADDR, config, 4, 100);
}

esp_err_t bme280_read_data(i2c_port_t port, bme280_data_t *data) {
    uint8_t raw[8];
    i2c_master_write_to_device(port, BME280_ADDR, (uint8_t[]){0xF4, 0x25}, 2, 100);
    vTaskDelay(pdMS_TO_TICKS(50));
    i2c_master_write_read_device(port, BME280_ADDR, (uint8_t[]){0xF7}, 1, raw, 8, 100);

    int32_t adc_P = (raw[0] << 12) | (raw[1] << 4) | (raw[2] >> 4);
    int32_t adc_T = (raw[3] << 12) | (raw[4] << 4) | (raw[5] >> 4);
    int32_t adc_H = (raw[6] << 8) | raw[7];

    // Bù trừ Nhiệt độ (Chuẩn Bosch)
    int32_t var1T = ((((adc_T >> 3) - ((int32_t)cal.dig_T1 << 1))) * ((int32_t)cal.dig_T2)) >> 11;
    int32_t var2T = (((((adc_T >> 4) - ((int32_t)cal.dig_T1)) * ((adc_T >> 4) - ((int32_t)cal.dig_T1))) >> 12) * ((int32_t)cal.dig_T3)) >> 14;
    t_fine = var1T + var2T;
    data->temperature = (float)((t_fine * 5 + 128) >> 8) / 100.0;

    // Bù trừ Độ ẩm (Chuẩn Bosch)
    int32_t h_res = (t_fine - ((int32_t)76800));
    h_res = (((((adc_H << 14) - (((int32_t)cal.dig_H4) << 20) - (((int32_t)cal.dig_H5) * h_res)) + ((int32_t)16384)) >> 15) * (((((((h_res * ((int32_t)cal.dig_H6)) >> 10) * (((h_res * ((int32_t)cal.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) * ((int32_t)cal.dig_H2) + 8192) >> 14));
    h_res = (h_res - (((((h_res >> 15) * (h_res >> 15)) >> 7) * ((int32_t)cal.dig_H1)) >> 4));
    h_res = (h_res < 0 ? 0 : h_res);
    h_res = (h_res > 419430400 ? 419430400 : h_res);
    data->humidity = (float)(h_res >> 12) / 1024.0;

    // Bù trừ Áp suất (Chuẩn Bosch)
    int64_t v1P = ((int64_t)t_fine) - 128000;
    int64_t v2P = v1P * v1P * (int64_t)cal.dig_P6;
    v2P = v2P + ((v1P * (int64_t)cal.dig_P5) << 17);
    v2P = v2P + (((int64_t)cal.dig_P4) << 35);
    v1P = ((v1P * v1P * (int64_t)cal.dig_P3) >> 8) + ((v1P * (int64_t)cal.dig_P2) << 12);
    v1P = (((((int64_t)1) << 47) + v1P)) * ((int64_t)cal.dig_P1) >> 33;
    if (v1P == 0) data->pressure = 0;
    else {
        int64_t p = 1048576 - adc_P;
        p = (((p << 31) - v2P) * 3125) / v1P;
        v1P = (((int64_t)cal.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
        v2P = (((int64_t)cal.dig_P8) * p) >> 19;
        p = ((p + v1P + v2P) >> 8) + (((int64_t)cal.dig_P7) << 4);
        data->pressure = (float)p / 256.0;
    }

    return ESP_OK;
}