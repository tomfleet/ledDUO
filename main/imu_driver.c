#include "imu_driver.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "config.h"

static esp_err_t i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t value)
{
    uint8_t data[2] = { reg, value };
    return i2c_master_write_to_device(I2C_PORT, addr, data, sizeof(data), pdMS_TO_TICKS(100));
}

static esp_err_t i2c_read_reg(uint8_t addr, uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_PORT, addr, &reg, 1, data, len, pdMS_TO_TICKS(100));
}

static esp_err_t qmi8658_init_addr(uint8_t addr, uint8_t *whoami_out)
{
    uint8_t whoami = 0;
    esp_err_t err = i2c_read_reg(addr, QMI8658_REG_WHOAMI, &whoami, 1);
    if (err != ESP_OK) {
        return err;
    }

    err = i2c_write_reg(addr, QMI8658_REG_CTRL1, 0x60);
    if (err != ESP_OK) {
        return err;
    }
    err = i2c_write_reg(addr, QMI8658_REG_CTRL2, 0x23);
    if (err != ESP_OK) {
        return err;
    }
    err = i2c_write_reg(addr, QMI8658_REG_CTRL3, 0x23);
    if (err != ESP_OK) {
        return err;
    }
    err = i2c_write_reg(addr, QMI8658_REG_CTRL7, 0x03);
    if (err != ESP_OK) {
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(20));
    if (whoami_out) {
        *whoami_out = whoami;
    }
    return ESP_OK;
}

esp_err_t imu_driver_init(imu_device_t *imu, uint8_t *whoami_out)
{
    if (!imu) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(imu, 0, sizeof(*imu));

    i2c_config_t i2c_cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
    };

    esp_err_t err = i2c_param_config(I2C_PORT, &i2c_cfg);
    if (err != ESP_OK) {
        return err;
    }

    err = i2c_driver_install(I2C_PORT, i2c_cfg.mode, 0, 0, 0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    uint8_t whoami = 0;
    err = qmi8658_init_addr(IMU_ADDR_PRIMARY, &whoami);
    if (err == ESP_OK) {
        imu->addr = IMU_ADDR_PRIMARY;
        imu->ok = true;
        if (whoami_out) {
            *whoami_out = whoami;
        }
        return ESP_OK;
    }

    err = qmi8658_init_addr(IMU_ADDR_SECONDARY, &whoami);
    if (err == ESP_OK) {
        imu->addr = IMU_ADDR_SECONDARY;
        imu->ok = true;
        if (whoami_out) {
            *whoami_out = whoami;
        }
        return ESP_OK;
    }

    return err;
}

esp_err_t imu_driver_read(const imu_device_t *imu, imu_sample_t *sample)
{
    if (!imu || !imu->ok || !sample) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t data[12] = {0};
    esp_err_t err = i2c_read_reg(imu->addr, QMI8658_REG_AX_L, data, sizeof(data));
    if (err != ESP_OK) {
        return err;
    }

    sample->ax = (int16_t)((data[1] << 8) | data[0]);
    sample->ay = (int16_t)((data[3] << 8) | data[2]);
    sample->az = (int16_t)((data[5] << 8) | data[4]);
    sample->gx = (int16_t)((data[7] << 8) | data[6]);
    sample->gy = (int16_t)((data[9] << 8) | data[8]);
    sample->gz = (int16_t)((data[11] << 8) | data[10]);
    return ESP_OK;
}
