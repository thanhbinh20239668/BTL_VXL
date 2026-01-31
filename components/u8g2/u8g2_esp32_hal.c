#include <stdio.h>
#include <string.h>

#include "sdkconfig.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "u8g2_esp32_hal.h"

static const char *TAG = "u8g2_hal";
static const unsigned int I2C_TIMEOUT_MS = 1000;

static spi_device_handle_t handle_spi;      // SPI handle.
static i2c_cmd_handle_t    handle_i2c;      // I2C handle.
static u8g2_esp32_hal_t    u8g2_esp32_hal;  // HAL state data.

#undef ESP_ERROR_CHECK
#define ESP_ERROR_CHECK(x)   do { esp_err_t rc = (x); if (rc != ESP_OK) { ESP_LOGE("err", "esp_err_t = %d", rc); assert(0 && #x);} } while(0);

/*
 * Initialze the ESP32 HAL.
 */
void u8g2_esp32_hal_init(u8g2_esp32_hal_t u8g2_esp32_hal_param) {
    u8g2_esp32_hal = u8g2_esp32_hal_param;
} // u8g2_esp32_hal_init

/*
 * HAL callback function for SPI communications.
 */
uint8_t u8g2_esp32_spi_byte_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
    switch(msg) {
        case U8X8_MSG_BYTE_SET_DC:
            if (u8g2_esp32_hal.dc != U8G2_ESP32_HAL_UNDEFINED) {
                gpio_set_level(u8g2_esp32_hal.dc, arg_int);
            }
            break;

        case U8X8_MSG_BYTE_INIT: {
            if (u8g2_esp32_hal.clk == U8G2_ESP32_HAL_UNDEFINED ||
                u8g2_esp32_hal.mosi == U8G2_ESP32_HAL_UNDEFINED ||
                u8g2_esp32_hal.cs == U8G2_ESP32_HAL_UNDEFINED) {
                break;
            }

            spi_bus_config_t bus_config;
            memset(&bus_config, 0, sizeof(spi_bus_config_t));
            bus_config.sclk_io_num   = u8g2_esp32_hal.clk;
            bus_config.mosi_io_num   = u8g2_esp32_hal.mosi;
            bus_config.miso_io_num   = -1;
            bus_config.quadwp_io_num = -1;
            bus_config.quadhd_io_num = -1;
            ESP_ERROR_CHECK(spi_bus_initialize(HSPI_HOST, &bus_config, 1));

            spi_device_interface_config_t dev_config;
            memset(&dev_config, 0, sizeof(spi_device_interface_config_t));
            dev_config.clock_speed_hz   = 10000;
            dev_config.spics_io_num     = u8g2_esp32_hal.cs;
            dev_config.queue_size       = 200;
            ESP_ERROR_CHECK(spi_bus_add_device(HSPI_HOST, &dev_config, &handle_spi));
            break;
        }

        case U8X8_MSG_BYTE_SEND: {
            spi_transaction_t trans_desc;
            memset(&trans_desc, 0, sizeof(spi_transaction_t));
            trans_desc.length    = 8 * arg_int;
            trans_desc.tx_buffer = arg_ptr;
            ESP_ERROR_CHECK(spi_device_transmit(handle_spi, &trans_desc));
            break;
        }
    }
    return 0;
}

/*
 * HAL callback function for I2C communications.
 */
uint8_t u8g2_esp32_i2c_byte_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
    switch(msg) {
        case U8X8_MSG_BYTE_SET_DC: {
            if (u8g2_esp32_hal.dc != U8G2_ESP32_HAL_UNDEFINED) {
                gpio_set_level(u8g2_esp32_hal.dc, arg_int);
            }
            break;
        }

        case U8X8_MSG_BYTE_INIT: {
            if (u8g2_esp32_hal.sda == U8G2_ESP32_HAL_UNDEFINED ||
                u8g2_esp32_hal.scl == U8G2_ESP32_HAL_UNDEFINED) {
                break;
            }

            static bool is_i2c_installed = false; // Biến tĩnh để ghi nhớ trạng thái

            i2c_config_t conf = {0}; 
            conf.mode = I2C_MODE_MASTER;
            conf.sda_io_num = u8g2_esp32_hal.sda;
            conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
            conf.scl_io_num = u8g2_esp32_hal.scl;
            conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
            conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
            conf.clk_flags = 0; 

            ESP_LOGI(TAG, "I2C Config: SDA=%d, SCL=%d", conf.sda_io_num, conf.scl_io_num);
            
            // Bước 1: Cấu hình lại thông số (Luôn thực hiện để khôi phục sau Sleep)
            i2c_param_config(I2C_MASTER_NUM, &conf);
            
            // Bước 2: Chỉ cài đặt driver nếu chưa có
            if (!is_i2c_installed) {
                esp_err_t res = i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
                if (res == ESP_OK) {
                    is_i2c_installed = true;
                    ESP_LOGI(TAG, "I2C Driver installed successfully.");
                } else {
                    ESP_LOGE(TAG, "I2C Driver install failed: %d", res);
                    // Không dùng ESP_ERROR_CHECK ở đây để tránh treo máy nếu lỗi nhẹ
                }
            } else {
                ESP_LOGI(TAG, "I2C Driver already exists, re-configured only.");
            }
            break;
        }

        case U8X8_MSG_BYTE_SEND: {
            uint8_t* data_ptr = (uint8_t*)arg_ptr;
            while( arg_int > 0 ) {
               ESP_ERROR_CHECK(i2c_master_write_byte(handle_i2c, *data_ptr, ACK_CHECK_EN));
               data_ptr++;
               arg_int--;
            }
            break;
        }

        case U8X8_MSG_BYTE_START_TRANSFER: {
            uint8_t i2c_address = u8x8_GetI2CAddress(u8x8);
            handle_i2c = i2c_cmd_link_create();
            ESP_ERROR_CHECK(i2c_master_start(handle_i2c));
            ESP_ERROR_CHECK(i2c_master_write_byte(handle_i2c, i2c_address | I2C_MASTER_WRITE, ACK_CHECK_EN));
            break;
        }

        case U8X8_MSG_BYTE_END_TRANSFER: {
            ESP_ERROR_CHECK(i2c_master_stop(handle_i2c));
            // --- SỬA TẠI ĐÂY: portTICK_RATE_MS -> portTICK_PERIOD_MS ---
            ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_MASTER_NUM, handle_i2c, I2C_TIMEOUT_MS / portTICK_PERIOD_MS));
            i2c_cmd_link_delete(handle_i2c);
            break;
        }
    }
    return 0;
}

/*
 * HAL callback function for GPIO and delay functions.
 */
uint8_t u8g2_esp32_gpio_and_delay_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
    switch(msg) {
        case U8X8_MSG_GPIO_AND_DELAY_INIT: {
            uint64_t bitmask = 0;
            if (u8g2_esp32_hal.dc != U8G2_ESP32_HAL_UNDEFINED) bitmask |= (1ull<<u8g2_esp32_hal.dc);
            if (u8g2_esp32_hal.reset != U8G2_ESP32_HAL_UNDEFINED) bitmask |= (1ull<<u8g2_esp32_hal.reset);
            if (u8g2_esp32_hal.cs != U8G2_ESP32_HAL_UNDEFINED) bitmask |= (1ull<<u8g2_esp32_hal.cs);

            if (bitmask == 0) break;

            gpio_config_t gpioConfig = {
                .pin_bit_mask = bitmask,
                .mode = GPIO_MODE_OUTPUT,
                .pull_up_en = GPIO_PULLUP_DISABLE,
                .pull_down_en = GPIO_PULLDOWN_ENABLE,
                .intr_type = GPIO_INTR_DISABLE
            };
            gpio_config(&gpioConfig);
            break;
        }

        case U8X8_MSG_GPIO_RESET:
            if (u8g2_esp32_hal.reset != U8G2_ESP32_HAL_UNDEFINED) {
                gpio_set_level(u8g2_esp32_hal.reset, arg_int);
            }
            break;

        case U8X8_MSG_GPIO_CS:
            if (u8g2_esp32_hal.cs != U8G2_ESP32_HAL_UNDEFINED) {
                gpio_set_level(u8g2_esp32_hal.cs, arg_int);
            }
            break;

        case U8X8_MSG_GPIO_I2C_CLOCK:
            if (u8g2_esp32_hal.scl != U8G2_ESP32_HAL_UNDEFINED) {
                gpio_set_level(u8g2_esp32_hal.scl, arg_int);
            }
            break;

        case U8X8_MSG_GPIO_I2C_DATA:
            if (u8g2_esp32_hal.sda != U8G2_ESP32_HAL_UNDEFINED) {
                gpio_set_level(u8g2_esp32_hal.sda, arg_int);
            }
            break;

        case U8X8_MSG_DELAY_MILLI:
            // --- SỬA TẠI ĐÂY: portTICK_RATE_MS -> portTICK_PERIOD_MS ---
            vTaskDelay(arg_int / portTICK_PERIOD_MS);
            break;
    }
    return 0;
}