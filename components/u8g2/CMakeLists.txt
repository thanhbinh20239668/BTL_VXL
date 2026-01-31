# Tự động lấy tất cả các file .c trong thư mục csrc
file(GLOB_RECURSE u8g2_srcs "csrc/*.c")

# Thêm file HAL cho ESP32 (đảm bảo file này nằm cùng cấp với CMakeLists.txt)
list(APPEND u8g2_srcs "u8g2_esp32_hal.c")

idf_component_register(SRCS ${u8g2_srcs}
                      INCLUDE_DIRS "." "csrc"
                      PRIV_REQUIRES driver)