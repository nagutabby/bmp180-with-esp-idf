#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>

#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_err.h"

#include "freertos/FreeRTOS.h"

#define I2C_PRIMARY_SCL_IO 33
#define I2C_PRIMARY_SDA_IO 32
#define I2C_PRIMARY_FREQUENCY_HZ 400000
#define I2C_PRIMARY_TX_BUFFER_DISABLE 0
#define I2C_PRIMARY_RX_BUFFER_DISABLE 0

#define BMP180_I2C_ADDRESS 0x77

#define BMP180_CALIBRATION_AC1 0xAA
#define BMP180_CALIBRATION_AC2 0xAC
#define BMP180_CALIBRATION_AC3 0xAE
#define BMP180_CALIBRATION_AC4 0xB0
#define BMP180_CALIBRATION_AC5 0xB2
#define BMP180_CALIBRATION_AC6 0xB4
#define BMP180_CALIBRATION_B1 0xB6
#define BMP180_CALIBRATION_B2 0xB8
#define BMP180_CALIBRATION_MB 0xBA
#define BMP180_CALIBRATION_MC 0xBC
#define BMP180_CALIBRATION_MD 0xBE

#define BMP180_CONTROL 0xF4
#define BMP180_TEMPERATURE_OR_PRESSURE_DATA 0xF6
#define BMP180_READ_TEMPERATURE 0x2E
#define BMP180_READ_PRESSURE 0x34

#define BMP180_ULTRA_HIGH_RESOLUTION 3

#define ACK_CHECK_ENABLE 0x1
#define ACK_CHECK_DISABLE 0x0
#define ACK_VALUE 0x0
#define NACK_VALUE 0x1

#define TAG "MAIN"

static int16_t ac1;
static int16_t ac2;
static int16_t ac3;
static uint16_t ac4;
static uint16_t ac5;
static uint16_t ac6;
static int16_t b1;
static int16_t b2;
static int16_t mb;
static int16_t mc;
static int16_t md;
static uint8_t oversampling = BMP180_ULTRA_HIGH_RESOLUTION;

esp_err_t i2c_primary_init();
static esp_err_t bmp180_read_int16(i2c_port_t, uint8_t, int16_t*);
static esp_err_t bmp180_read_uint16(i2c_port_t, uint8_t, uint16_t*);
static esp_err_t bmp180_read_uint32(i2c_port_t, uint8_t, uint32_t*);
static esp_err_t bmp180_primary_write_secondary(i2c_port_t, uint8_t*, size_t);
static esp_err_t bmp180_primary_read_secondary(i2c_port_t, uint8_t*, size_t);
static esp_err_t bmp180_write_reg(i2c_port_t, uint8_t, uint8_t);
static esp_err_t bmp180_read_raw_pressure(uint32_t*);
esp_err_t bmp180_read_pressure(uint32_t*);
static esp_err_t bmp180_read_raw_temperature(int16_t*);
esp_err_t bmp180_read_temperature(float*);
static esp_err_t bmp180_calculate_b5(int32_t*);

int app_main() {
    ESP_ERROR_CHECK(i2c_primary_init());

    while (true) {
        uint32_t pressure;
        float temperature;

        bmp180_read_pressure(&pressure);
        bmp180_read_temperature(&temperature);

        ESP_LOGI(TAG, "Pressure: %"PRIu32" Pa, Temperature: %.1f °C", pressure, temperature);
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }

    return 0;
}

esp_err_t i2c_primary_init() {
    esp_err_t error;

    i2c_config_t config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_PRIMARY_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_PRIMARY_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_PRIMARY_FREQUENCY_HZ
    };

    error = i2c_param_config(I2C_NUM_0, &config);
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver configuration failed with error = %d", error);
    }

    error = i2c_driver_install(I2C_NUM_0, config.mode, I2C_PRIMARY_RX_BUFFER_DISABLE, I2C_PRIMARY_TX_BUFFER_DISABLE, 0);
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver installation failed with error = %d", error);
    }

    uint8_t reg = 0x00;
    error = bmp180_primary_write_secondary(I2C_NUM_0, &reg, 1);
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "BMP180 sensor not found at 0x%02x", BMP180_I2C_ADDRESS);
    } else {
        ESP_LOGI(TAG, "BMP180 sensor found at 0x%02x", BMP180_I2C_ADDRESS);
    }

    error = bmp180_read_int16(I2C_NUM_0, BMP180_CALIBRATION_AC1, &ac1);
    error |= bmp180_read_int16(I2C_NUM_0, BMP180_CALIBRATION_AC2, &ac2);
    error |= bmp180_read_int16(I2C_NUM_0, BMP180_CALIBRATION_AC3, &ac3);
    error |= bmp180_read_uint16(I2C_NUM_0, BMP180_CALIBRATION_AC4, &ac4);
    error |= bmp180_read_uint16(I2C_NUM_0, BMP180_CALIBRATION_AC5, &ac5);
    error |= bmp180_read_uint16(I2C_NUM_0, BMP180_CALIBRATION_AC6, &ac6);
    error |= bmp180_read_int16(I2C_NUM_0, BMP180_CALIBRATION_B1, &b1);
    error |= bmp180_read_int16(I2C_NUM_0, BMP180_CALIBRATION_B2, &b2);
    error |= bmp180_read_int16(I2C_NUM_0, BMP180_CALIBRATION_MB, &mb);
    error |= bmp180_read_int16(I2C_NUM_0, BMP180_CALIBRATION_MC, &mc);
    error |= bmp180_read_int16(I2C_NUM_0, BMP180_CALIBRATION_MD, &md);
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "BMP180 sensor calibration read failure, error = %d", error);
    }

    return ESP_OK;
}

static esp_err_t bmp180_read_int16(i2c_port_t i2c_num, uint8_t reg, int16_t* value) {
    esp_err_t error = bmp180_primary_write_secondary(i2c_num, &reg, 1);
    if (error == ESP_OK) {
        uint8_t data_rd[2] = {0};
        error = bmp180_primary_read_secondary(i2c_num, data_rd, 2);
        if (error == ESP_OK) {
            *value = (int16_t) ((data_rd[0] << 8) | data_rd[1]);
        }
    }
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "Read [0x%02x] int16 failed, error = %d", reg, error);
    }
    return error;
}

static esp_err_t bmp180_read_uint16(i2c_port_t i2c_num, uint8_t reg, uint16_t* value) {
    esp_err_t error = bmp180_primary_write_secondary(i2c_num, &reg, 1);
    if (error == ESP_OK) {
        uint8_t data_rd[2] = {0};
        error = bmp180_primary_read_secondary(i2c_num, data_rd, 2);
        if (error == ESP_OK) {
            *value = (uint16_t) ((data_rd[0] << 8) | data_rd[1]);
        }
    }
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "Read [0x%02x] uint16 failed, error = %d", reg, error);
    }
    return error;
}

static esp_err_t bmp180_read_uint32(i2c_port_t i2c_num, uint8_t reg, uint32_t* value) {
    esp_err_t error = bmp180_primary_write_secondary(i2c_num, &reg, 1);
    if (error == ESP_OK) {
        uint8_t data_rd[3] = {0};
        error = bmp180_primary_read_secondary(i2c_num, data_rd, 3);
        if (error == ESP_OK) {
            *value = (uint32_t) ((data_rd[0] << 16) | (data_rd[1] << 8) | data_rd[2]);
        }
    }
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "Read [0x%02x] uint16 failed, error = %d", reg, error);
    }
    return error;
}


static esp_err_t bmp180_primary_write_secondary(i2c_port_t i2c_num, uint8_t* data_wr, size_t size) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( BMP180_I2C_ADDRESS  << 1 ) | I2C_MASTER_WRITE, ACK_CHECK_ENABLE);
    i2c_master_write(cmd, data_wr, size, ACK_CHECK_ENABLE);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t bmp180_primary_read_secondary(i2c_port_t i2c_num, uint8_t* data_rd, size_t size) {
    if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( BMP180_I2C_ADDRESS << 1 ) | I2C_MASTER_READ, ACK_CHECK_ENABLE);
    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VALUE);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VALUE);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t bmp180_write_reg(i2c_port_t i2c_num, uint8_t reg, uint8_t cmd) {
    uint8_t data_wr[] = {reg, cmd};
    esp_err_t error = bmp180_primary_write_secondary(i2c_num, data_wr, 2);
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "Write [0x%02x] = 0x%02x failed, error = %d", reg, cmd, error);
    }
    return error;
}

static esp_err_t bmp180_read_raw_pressure(uint32_t* raw_pressure) {
    esp_err_t error = bmp180_write_reg(I2C_NUM_0, BMP180_CONTROL, BMP180_READ_PRESSURE + (oversampling << 6));
    if (error == ESP_OK) {
        TickType_t xDelay = (2 + (3 << oversampling)) / portTICK_PERIOD_MS;
        if (xDelay == 0) {
            xDelay = 1;
        }
        vTaskDelay(xDelay);
        error = bmp180_read_uint32(I2C_NUM_0, BMP180_TEMPERATURE_OR_PRESSURE_DATA, raw_pressure);
        if (error == ESP_OK) {
            *raw_pressure >>= (8 - oversampling);
        }
    }
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "Read uncompensated pressure failed, error = %d", error);
    }
    return error;
}


static esp_err_t bmp180_read_raw_temperature(int16_t* raw_temperature) {
    esp_err_t error = bmp180_write_reg(I2C_NUM_0, BMP180_CONTROL, BMP180_READ_TEMPERATURE);
    if (error == ESP_OK) {
        TickType_t xDelay = 5 / portTICK_PERIOD_MS;
        if (xDelay == 0) {
            xDelay = 1;
        }
        vTaskDelay(xDelay);
        error = bmp180_read_int16(I2C_NUM_0, BMP180_TEMPERATURE_OR_PRESSURE_DATA, raw_temperature);
    }
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "Read uncompensated temperature failed, error = %d", error);
    }
    return error;
}

esp_err_t bmp180_read_temperature(float* temperature) {
    int32_t b5;

    esp_err_t error = bmp180_calculate_b5(&b5);
    if (error == ESP_OK) {
        *temperature = ((b5 + 8) >> 4) / 10.0;
    } else {
        ESP_LOGE(TAG, "Read temperature failed, error = %d", error);
    }
    return error;
}

esp_err_t bmp180_read_pressure(uint32_t* pressure) {
    int32_t b3, b5, b6, x1, x2, x3, p;
    uint32_t raw_pressure, b4, b7;
    esp_err_t error;

    error = bmp180_calculate_b5(&b5);
    if (error == ESP_OK) {
        b6 = b5 - 4000;
        x1 = (b2 * (b6 * b6) >> 12) >> 11;
        x2 = (ac2 * b6) >> 11;
        x3 = x1 + x2;
        b3 = (((((int32_t)ac1) * 4 + x3) << oversampling) + 2) >> 2;

        x1 = (ac3 * b6) >> 13;
        x2 = (b1 * ((b6 * b6) >> 12)) >> 16;
        x3 = ((x1 + x2) + 2) >> 2;
        b4 = (ac4 * (uint32_t)(x3 + 32768)) >> 15;

        error  = bmp180_read_raw_pressure(&raw_pressure);
        if (error == ESP_OK) {
            b7 = ((uint32_t)(raw_pressure - b3) * (50000 >> oversampling));
            if (b7 < 0x80000000) {
                p = (b7 << 1) / b4;
            } else {
                p = (b7 / b4) << 1;
            }

            x1 = (p >> 8) * (p >> 8);
            x1 = (x1 * 3038) >> 16;
            x2 = (-7357 * p) >> 16;
            p += (x1 + x2 + 3791) >> 4;
            *pressure = p;
        }
    }

    if (error != ESP_OK) {
        ESP_LOGE(TAG, "Pressure compensation failed, error = %d", error);
    }

    return error;
}

static esp_err_t bmp180_calculate_b5(int32_t* b5) {
    int16_t raw_temperature;
    int32_t x1, x2;

    esp_err_t error = bmp180_read_raw_temperature(&raw_temperature);
    if (error == ESP_OK) {
        x1 = ((raw_temperature - (int32_t) ac6) * (int32_t) ac5) >> 15;
        x2 = ((int32_t) mc << 11) / (x1 + md);
        *b5 = x1 + x2;
    } else {
        ESP_LOGE(TAG, "Calculate b5 failed, error = %d", error);
    }
    return error;
}
