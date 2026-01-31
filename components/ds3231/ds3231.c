#include "ds3231.h"

// Chuyển đổi BCD sang Decimal
static uint8_t bcd2dec(uint8_t val) { return ((val >> 4) * 10) + (val & 0x0f); }
// Chuyển đổi Decimal sang BCD
static uint8_t dec2bcd(uint8_t val) { return ((val / 10) << 4) + (val % 10); }

esp_err_t ds3231_init(i2c_port_t port) {
    uint8_t dummy;
    return i2c_master_write_read_device(port, DS3231_ADDR, (uint8_t[]){0x00}, 1, &dummy, 1, 100);
}

esp_err_t ds3231_get_time(i2c_port_t port, struct tm *timeinfo) {
    uint8_t data[7];
    esp_err_t err = i2c_master_write_read_device(port, DS3231_ADDR, (uint8_t[]){0x00}, 1, data, 7, 100);
    if (err != ESP_OK) return err;

    timeinfo->tm_sec  = bcd2dec(data[0]);
    timeinfo->tm_min  = bcd2dec(data[1]);
    timeinfo->tm_hour = bcd2dec(data[2]);
    timeinfo->tm_mday = bcd2dec(data[4]);
    timeinfo->tm_mon  = bcd2dec(data[5] & 0x7F) - 1;
    timeinfo->tm_year = bcd2dec(data[6]) + 100; // Năm kể từ 1900
    return ESP_OK;
}

esp_err_t ds3231_set_time(i2c_port_t port, struct tm *timeinfo) {
    uint8_t data[8] = {
        0x00, // Thanh ghi bắt đầu
        dec2bcd(timeinfo->tm_sec),
        dec2bcd(timeinfo->tm_min),
        dec2bcd(timeinfo->tm_hour),
        0x01, // Day of week (tạm bỏ qua)
        dec2bcd(timeinfo->tm_mday),
        dec2bcd(timeinfo->tm_mon + 1),
        dec2bcd(timeinfo->tm_year - 100)
    };
    return i2c_master_write_to_device(port, DS3231_ADDR, data, 8, 100);
}