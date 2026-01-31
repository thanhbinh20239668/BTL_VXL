/*
============================================================================================================================
 * PROJECT: AIR QUALITY MONITOR SYSTEM BY GROUP 8
 * LEAD BY: DO THANH BINH
 * MEMBERS: NGUYEN TRUNG HIEU, NGUYEN LE VIET ANH, VU HUONG THAO
 * SUPERVISOR: DR. HAN HUY DUNG
 * MENTORS: LE DINH HIEU - NGUYEN NGOC SON
 * HARDWARE: ESP32, PMS7003, BME280, OLED, DS3231, SD Card
 * Kịch bản 1: Full Load 
- ESP32: Wifi, Bluetooth quét, ép CPU làm việc nặng
- PMS7003: Hoạt động liên tục, không sleep
- BME280: Hoạt động liên tục
- DS3231: Hoạt động liên tục
- OLED: Hoạt động liên tục
- SD card: Ghi liên tục
* Kịch bản 2: Sleep BME280
* Kịch bản 3: Sleep BME280, Display off OLED
* Kịch bản 4: Sleep BME280, Display off OLED, Sleep SD card
* Kịch bản 5: Sleep BME280, Display off OLED, Sleep SD card, Sleep PMS7003
* Kịch bản 6: Sleep BME280, Display off OLED, Sleep SD card, Sleep PMS7003, ESP32 Wifi và Bluetooth off
* Kịch bản 7: Sleep BME280, Display off OLED, Sleep SD card, Sleep PMS7003, ESP32 Wifi và Bluetooth off, ESP32 Light sleep
* Kịch bản 8: Full tối ưu
- ESP32: Deepsleep, bật lên sau mỗi 15p Deep sleep, tắt những tính năng không cần thiết để tiết kiệm điện năng tiêu thụ
- PMS7003: Sleep 15p, thời gian warmup là 30s để đo ổn định
- OLED: Display off khi ESP32 Deep sleep
- SD Card: Sau khi ghi giá trị đo được thì sleep
- BME280: Forced mode
- DS3231: Hoạt động liên tục
============================================================================================================================
*/

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <sys/time.h>
#include <sys/stat.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Drivers
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/sdspi_host.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "esp_sleep.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_wifi.h"

// Thư viện
#include "u8g2.h"
#include "u8g2_esp32_hal.h"
#include "bme280.h"
#include "ds3231.h"

// ============================================================
// CẤU HÌNH NGƯỜI DÙNG
// ============================================================
#define CURRENT_SCENARIO    8   // Chọn kịch bản (1 -> 8)

// Cài đặt thời gian (1 để chỉnh sau đó chuyển về 0)
#define SET_RTC_TIME_NOW    0   

#define TIME_YEAR           2026 
#define TIME_MONTH          1    
#define TIME_DAY            1   
#define TIME_HOUR           20   
#define TIME_MIN            34   
#define TIME_SEC            0   

// Cấu hình
#define SLEEP_DURATION_MINS 15  
#define WARMUP_SECONDS      30  

// Cấu hình chân
#define PIN_PMS_SET         GPIO_NUM_33
#define PIN_PMS_TX          GPIO_NUM_26 // Nối vào RX của PMS
#define PIN_PMS_RX          GPIO_NUM_14 // Nối vào TX của PMS
#define PIN_SD_CS           5
#define PIN_SD_MOSI         23
#define PIN_SD_MISO         19
#define PIN_SD_CLK          18
#define PIN_I2C_SDA         21
#define PIN_I2C_SCL         22

#define PORT_PMS_UART       UART_NUM_2
#define PORT_APP_I2C        I2C_NUM_0
#define FILE_PATH           "/sdcard/data.csv"
#define PMS_BUF_SIZE        128

static const char *TAG = "AIR_MONITOR";
u8g2_t u8g2;
bool is_sd_mounted = false;
bme280_data_t bme_data;
struct tm rtc_time;

// Struct chứa dữ liệu bụi từ PMS7003
typedef struct {
    uint16_t pm1_0;
    uint16_t pm2_5;
    uint16_t pm10;
    bool valid;
} pms_data_t;

pms_data_t pms_last_reading = {0, 0, 0, false};

// ============================================================
// KHỞI TẠO PHẦN CỨNG
// ============================================================
void hal_init_pms() {
    gpio_config_t io_conf = { .pin_bit_mask = (1ULL << PIN_PMS_SET), .mode = GPIO_MODE_OUTPUT, .pull_up_en = 0, .pull_down_en = 1 };
    gpio_config(&io_conf);
    gpio_set_level(PIN_PMS_SET, 0);
}

void hal_init_uart() {
    uart_config_t cfg = { .baud_rate = 9600, .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE, .stop_bits = UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE };
    uart_param_config(PORT_PMS_UART, &cfg);
    uart_set_pin(PORT_PMS_UART, PIN_PMS_TX, PIN_PMS_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(PORT_PMS_UART, PMS_BUF_SIZE * 2, 0, 0, NULL, 0);
}

void hal_init_oled_i2c() {
    u8g2_esp32_hal_t u8g2_esp32_hal = U8G2_ESP32_HAL_DEFAULT;
    u8g2_esp32_hal.sda = PIN_I2C_SDA; u8g2_esp32_hal.scl = PIN_I2C_SCL;
    u8g2_esp32_hal_init(u8g2_esp32_hal);
    u8g2_Setup_ssd1306_i2c_128x64_noname_f(&u8g2, U8G2_R0, u8g2_esp32_i2c_byte_cb, u8g2_esp32_gpio_and_delay_cb);
    u8x8_SetI2CAddress(&u8g2.u8x8, 0x3C << 1);
    u8g2_InitDisplay(&u8g2);
    u8g2_SetPowerSave(&u8g2, 0); u8g2_ClearBuffer(&u8g2);
}

void hal_init_sd() {
    esp_vfs_fat_sdmmc_mount_config_t mount_config = { .format_if_mount_failed = true, .max_files = 2 };
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    spi_bus_config_t bus_cfg = { .mosi_io_num = PIN_SD_MOSI, .miso_io_num = PIN_SD_MISO, .sclk_io_num = PIN_SD_CLK, .quadwp_io_num = -1, .quadhd_io_num = -1 };
    spi_bus_initialize(host.slot, &bus_cfg, SPI_DMA_CH_AUTO);
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_SD_CS; slot_config.host_id = host.slot;
    if (esp_vfs_fat_sdspi_mount("/sdcard", &host, &slot_config, &mount_config, NULL) == ESP_OK) is_sd_mounted = true;
}

void hal_init_wifi_hog() {
    nvs_flash_init(); esp_netif_init(); esp_event_loop_create_default();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT(); esp_wifi_init(&cfg);
    esp_wifi_set_mode(WIFI_MODE_STA); esp_wifi_start();
}

// ============================================================
// ĐỌC DỮ LIỆU TỪ PMS7003
// ============================================================
pms_data_t pms_read_data() {
    pms_data_t result = {0, 0, 0, false};
    uint8_t data[PMS_BUF_SIZE];
    
    // Đọc dữ liệu từ UART buffer
    int len = uart_read_bytes(PORT_PMS_UART, data, PMS_BUF_SIZE, pdMS_TO_TICKS(100));
    
    if (len > 0) {
        // Duyệt qua buffer để tìm Header 0x42 0x4D
        for (int i = 0; i < len - 31; i++) {
            if (data[i] == 0x42 && data[i+1] == 0x4D) {
                // Tính Checksum
                uint16_t checksum = 0;
                uint16_t expected_checksum = (data[i+30] << 8) | data[i+31];
                
                for (int j = 0; j < 30; j++) {
                    checksum += data[i+j];
                }
                
                if (checksum == expected_checksum) {
                    result.pm1_0 = (data[i+10] << 8) | data[i+11];
                    result.pm2_5 = (data[i+12] << 8) | data[i+13];
                    result.pm10  = (data[i+14] << 8) | data[i+15];
                    result.valid = true;
                    ESP_LOGI(TAG, "PMS Read: PM1=%d, PM2.5=%d, PM10=%d", result.pm1_0, result.pm2_5, result.pm10);
                    return result;
                }
            }
        }
    }
    ESP_LOGW(TAG, "PMS Read Failed or No Data");
    return result; // Trả về false nếu không đọc được
}

// ============================================================
// LƯU VÀO SD CARD
// ============================================================
void check_and_write_header() {
    if (!is_sd_mounted) return;
    struct stat st;
    if (stat(FILE_PATH, &st) != 0) {
        FILE *f = fopen(FILE_PATH, "w");
        if (f) {
            fprintf(f, "Time,PM1.0,PM2.5,PM10,Temp,Hum,Pres\n");
            fclose(f);
        }
    }
}

void save_data_csv(struct tm t, bme280_data_t bme, pms_data_t pms) {
    if (!is_sd_mounted) return;
    FILE *f = fopen(FILE_PATH, "a");
    if (f) {
        fprintf(f, "%02d-%02d-%04d %02d:%02d:%02d,%d,%d,%d,%.1f,%.1f,%.1f\n",
                t.tm_mday, t.tm_mon + 1, t.tm_year + 1900,
                t.tm_hour, t.tm_min, t.tm_sec,
                pms.pm1_0, pms.pm2_5, pms.pm10,
                bme.temperature, bme.humidity, bme.pressure
        );
        fclose(f);
    }
}

// ============================================================
// HIỂN THỊ LÊN MÀN HÌNH OLED
// ============================================================
void ui_draw_dashboard(struct tm t, float temp, float hum, float pres, int pm25, const char* status) {
    char buf[32];
    u8g2_ClearBuffer(&u8g2);

    // Thời gian
    u8g2_SetFont(&u8g2, u8g2_font_6x10_tr); 
    sprintf(buf, "%02d:%02d:%02d", t.tm_hour, t.tm_min, t.tm_sec);
    u8g2_DrawStr(&u8g2, 0, 10, buf);
    sprintf(buf, "%02d/%02d/%02d", t.tm_mday, t.tm_mon + 1, t.tm_year % 100);
    int w = u8g2_GetStrWidth(&u8g2, buf);
    u8g2_DrawStr(&u8g2, 128 - w, 10, buf); 
    u8g2_DrawLine(&u8g2, 0, 12, 127, 12);

    // PM2.5
    u8g2_SetFont(&u8g2, u8g2_font_helvB14_tr); 
    sprintf(buf, "PM2.5: %d", pm25);
    u8g2_DrawStr(&u8g2, 0, 32, buf); 

    // Temp, Hum, Pres
    u8g2_SetFont(&u8g2, u8g2_font_profont11_tf);
    sprintf(buf, "Temp:%.1fC Hum:%.1f%%", temp, hum);
    u8g2_DrawStr(&u8g2, 0, 45, buf);
    sprintf(buf, "Pres:%.1fhPa", pres / 100.0);
    u8g2_DrawStr(&u8g2, 0, 55, buf);

    // Trạng thái, SD
    u8g2_DrawLine(&u8g2, 0, 57, 127, 57);
    u8g2_SetFont(&u8g2, u8g2_font_5x7_tr);
    u8g2_DrawStr(&u8g2, 0, 64, status);

    if (is_sd_mounted) {
        int w_sd = u8g2_GetStrWidth(&u8g2, "[SD]");
        u8g2_DrawStr(&u8g2, 128 - w_sd, 64, "[SD]");
    }
    u8g2_SendBuffer(&u8g2);
}

// ============================================================
// LOGIC CÁC KỊCH BẢN (SCENARIO)
// ============================================================
void run_active_scenarios(int scenario) {
    ESP_LOGI(TAG, "SCENARIO %d ACTIVE", scenario);
    bool use_bme = (scenario<2); bool use_oled = (scenario<3);
    bool use_sd = (scenario<4); bool use_fan = (scenario<5); bool use_wifi = (scenario<6);

    if(use_wifi) hal_init_wifi_hog(); else esp_wifi_stop();
    gpio_set_level(PIN_PMS_SET, use_fan ? 1:0);
    if(!use_oled) u8g2_SetPowerSave(&u8g2, 1);
    if (use_sd) check_and_write_header();

    while(1) {
        ds3231_get_time(PORT_APP_I2C, &rtc_time);
        if(use_bme) bme280_read_data(PORT_APP_I2C, &bme_data);
        
        // ĐỌC PMS
        pms_data_t current_pms = pms_read_data();
        if (current_pms.valid) pms_last_reading = current_pms; // Lưu giá trị tốt nhất

        if(use_oled) {
            char st[32]; sprintf(st, "Run S%d...", scenario);
            ui_draw_dashboard(rtc_time, bme_data.temperature, bme_data.humidity, bme_data.pressure, pms_last_reading.pm2_5, st);
        }
        if(use_sd && is_sd_mounted) {
            save_data_csv(rtc_time, bme_data, pms_last_reading);
        }
        if(use_wifi) {
            wifi_scan_config_t sc = {.show_hidden=true}; esp_wifi_scan_start(&sc, false);
            if(scenario==1) {volatile float x=0; for(int i=0;i<10000;i++) x+=i;}
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ============================================================
// KỊCH BẢN 7: ESP32 LIGHT SLEEP
// ============================================================
void run_scenario_7() {
    ESP_LOGI(TAG, "SCENARIO 7 START");
    esp_sleep_enable_timer_wakeup(SLEEP_DURATION_MINS * 60 * 1000000ULL);

    while(1) {
        u8g2_SetPowerSave(&u8g2, 0); gpio_set_level(PIN_PMS_SET, 1);
        
        // Warmup & Display
        for(int i=5; i>0; i--) {
            ds3231_get_time(PORT_APP_I2C, &rtc_time);
            bme280_read_data(PORT_APP_I2C, &bme_data);
            
            // Đọc PMS
            pms_data_t current_pms = pms_read_data();
            if (current_pms.valid) pms_last_reading = current_pms;

            char s[32]; sprintf(s, "Awake: %ds", i);
            ui_draw_dashboard(rtc_time, bme_data.temperature, bme_data.humidity, bme_data.pressure, pms_last_reading.pm2_5, s);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }

        char m[32]; sprintf(m, "Light Sleep %dm", SLEEP_DURATION_MINS);
        ui_draw_dashboard(rtc_time, bme_data.temperature, bme_data.humidity, bme_data.pressure, pms_last_reading.pm2_5, m);
        vTaskDelay(pdMS_TO_TICKS(2000));
        
        u8g2_SetPowerSave(&u8g2, 1); gpio_set_level(PIN_PMS_SET, 0);
        fflush(stdout); vTaskDelay(pdMS_TO_TICKS(100)); 
        esp_light_sleep_start();
    }
}

// ============================================================
// KỊCH BẢN 8: ESP32 DEEP SLEEP
// ============================================================
void run_scenario_8() {
    ESP_LOGI(TAG, "SCENARIO 8 START");

    // 1. Warmup (30s)
    gpio_set_level(PIN_PMS_SET, 1);
    
    // Xóa rác trong buffer UART trước khi warmup
    uart_flush_input(PORT_PMS_UART);

    for(int i = WARMUP_SECONDS; i > 0; i--) {
        ds3231_get_time(PORT_APP_I2C, &rtc_time);
        bme280_read_data(PORT_APP_I2C, &bme_data);
        
        // Đọc PMS trong lúc Warmup để cập nhật màn hình
        pms_data_t current_pms = pms_read_data();
        if (current_pms.valid) pms_last_reading = current_pms;

        char status[32]; sprintf(status, "Warmup: %ds", i);
        ui_draw_dashboard(rtc_time, bme_data.temperature, bme_data.humidity, bme_data.pressure, pms_last_reading.pm2_5, status);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // 2. Đọc dữ liệu lần cuối
    bme280_read_data(PORT_APP_I2C, &bme_data);
    
    // Đọc PMS lần cuối
    pms_data_t final_pms = pms_read_data();
    // Nếu đọc lỗi, dùng giá trị gần nhất
    if (!final_pms.valid) final_pms = pms_last_reading;

    char log_status[32] = "Saving...";
    if (is_sd_mounted) {
        check_and_write_header();
        save_data_csv(rtc_time, bme_data, final_pms);
        strcpy(log_status, "Saved OK");
    } else {
        strcpy(log_status, "No SD");
    }

    ui_draw_dashboard(rtc_time, bme_data.temperature, bme_data.humidity, bme_data.pressure, final_pms.pm2_5, log_status);
    vTaskDelay(pdMS_TO_TICKS(2000));

    // 3. Chuẩn bị Deep sleep
    char m[32]; sprintf(m, "Deep Sleep %dm", SLEEP_DURATION_MINS);
    ui_draw_dashboard(rtc_time, bme_data.temperature, bme_data.humidity, bme_data.pressure, final_pms.pm2_5, m);
    vTaskDelay(pdMS_TO_TICKS(1000));

    u8g2_SetPowerSave(&u8g2, 1); gpio_set_level(PIN_PMS_SET, 0);
    gpio_hold_en(PIN_PMS_SET); gpio_deep_sleep_hold_en();
    esp_sleep_enable_timer_wakeup(SLEEP_DURATION_MINS * 60 * 1000000ULL);
    
    fflush(stdout); 
    vTaskDelay(pdMS_TO_TICKS(100)); 

    esp_deep_sleep_start();
}

// ============================================================
// HÀM MAIN
// ============================================================
void app_main(void) {
    hal_init_pms(); hal_init_uart(); hal_init_oled_i2c(); hal_init_sd();
    bme280_init(PORT_APP_I2C); ds3231_init(PORT_APP_I2C);

    #if SET_RTC_TIME_NOW == 1
        struct tm ts = {
            .tm_year = TIME_YEAR - 1900, 
            .tm_mon  = TIME_MONTH - 1,   
            .tm_mday = TIME_DAY,
            .tm_hour = TIME_HOUR,
            .tm_min  = TIME_MIN,
            .tm_sec  = TIME_SEC
        };
        ds3231_set_time(PORT_APP_I2C, &ts);
        ESP_LOGW(TAG, "DA CAP NHAT GIO! HAY NAP LAI CODE VOI SET_RTC_TIME_NOW = 0");
    #endif

    if (CURRENT_SCENARIO <= 6) run_active_scenarios(CURRENT_SCENARIO);
    else if (CURRENT_SCENARIO == 7) run_scenario_7();
    else if (CURRENT_SCENARIO == 8) run_scenario_8();
}