#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

typedef struct {
    uint8_t addr;
    bool ok;
} imu_device_t;

typedef struct {
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t gx;
    int16_t gy;
    int16_t gz;
} imu_sample_t;

esp_err_t imu_driver_init(imu_device_t *imu, uint8_t *whoami_out);
esp_err_t imu_driver_read(const imu_device_t *imu, imu_sample_t *sample);
