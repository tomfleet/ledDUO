/* IMU + WS2812 8x8 demo (ESP32-S3 + QMI8658 + WS2812) */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "config.h"
#include "led_ws2812.h"
#include "espnow_link.h"
#include "game_logic.h"
#include "imu_driver.h"
#include "captive_portal.h"
#include "web_dashboard.h"

static const char *TAG = "imu_led";

typedef struct {
    bool imu_ok;
    imu_device_t imu;
} app_ctx_t;

static ws2812_strip_t s_strip;
static game_logic_t s_game;
static bool s_idle_mode;
static int64_t s_last_motion_us;
static float s_motion_level;
static int64_t s_last_send_us;

static float clamp_f(float v, float lo, float hi)
{
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

static void imu_led_task(void *pv)
{
    app_ctx_t *ctx = (app_ctx_t *)pv;
    int64_t last_send_us = 0;
    int64_t last_motion_us = 0;
    int64_t last_debug_us = 0;
    bool idle_mode = false;
    while (true) {
        imu_sample_t sample = {0};
        int64_t now = esp_timer_get_time();

        const espnow_link_state_t *link = espnow_link_get_state();
        if (espnow_link_is_pairing_active()) {
            int64_t elapsed_ms = (now - espnow_link_get_pairing_start_us()) / 1000;
            if (elapsed_ms > ESPNOW_PAIR_WINDOW_MS) {
                espnow_link_close_pairing();
                link = espnow_link_get_state();
                ESP_LOGW(TAG, "Pairing window closed; local-only.");
            } else if ((now - espnow_link_get_last_hello_us()) > (ESPNOW_HELLO_INTERVAL_MS * 1000)) {
                espnow_link_send_hello();
                espnow_link_set_last_hello_us(now);
            }
        }

        if (ctx->imu_ok && imu_driver_read(&ctx->imu, &sample) == ESP_OK) {
            float motion = (fabsf((float)sample.gx) + fabsf((float)sample.gy)
                + fabsf((float)sample.gz)) / MOTION_GYRO_SCALE;
            s_motion_level = motion;
            if (motion > MOTION_EXIT_IDLE) {
                idle_mode = false;
                last_motion_us = now;
                s_last_motion_us = now;
            }
            if (!idle_mode && motion > MOTION_ENTER_IDLE) {
                last_motion_us = now;
                s_last_motion_us = now;
            }
            if (!idle_mode && last_motion_us > 0 && ((now - last_motion_us) > (IDLE_TIMEOUT_MS * 1000))) {
                idle_mode = true;
            }
            s_idle_mode = idle_mode;
            if (DEBUG_MOTION_LOGS && (now - last_debug_us) > (DEBUG_MOTION_PERIOD_MS * 1000)) {
                int64_t since_motion_ms = (last_motion_us > 0) ? ((now - last_motion_us) / 1000) : -1;
                int8_t rssi_min = link->rssi_has_obs ? link->rssi_min_obs : ESPNOW_RSSI_MIN_DBM;
                int8_t rssi_max = link->rssi_has_obs ? link->rssi_max_obs : ESPNOW_RSSI_MAX_DBM;
                float denom = (float)(rssi_max - rssi_min);
                float rssi_norm = denom > 1.0f ? ((link->remote_rssi - rssi_min) / denom) : 0.0f;
                rssi_norm = clamp_f(rssi_norm, 0.0f, 1.0f);
                float glow = ESPNOW_RSSI_GLOW_MAX * rssi_norm;
                ESP_LOGI(TAG,
                         "imu ax=%d ay=%d az=%d gx=%d gy=%d gz=%d motion=%.3f idle=%d since=%lldms rssi=%d norm=%.2f glow=%.1f",
                         sample.ax, sample.ay, sample.az, sample.gx, sample.gy, sample.gz,
                         motion, idle_mode ? 1 : 0, since_motion_ms,
                         link->remote_rssi, rssi_norm, glow);
                last_debug_us = now;
            }
            game_logic_render_frame(&s_game,
                                    sample.ax, sample.ay, sample.az,
                                    sample.gx, sample.gy, sample.gz,
                                    now, idle_mode);

            if (espnow_link_has_peer() && (now - last_send_us) > (ESPNOW_SEND_INTERVAL_MS * 1000)) {
                espnow_link_send_data(s_game.local_vpos_x, s_game.local_vpos_y, s_game.local_hue);
                last_send_us = now;
                s_last_send_us = now;
            }
        } else {
            game_logic_render_demo(&s_game, now);
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void app_main(void)
{
    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_err);

    gpio_config_t gpio_cfg = {
        .pin_bit_mask = 1ULL << LED_STRIP_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&gpio_cfg));
    ESP_ERROR_CHECK(gpio_set_level(LED_STRIP_GPIO, 0));
    vTaskDelay(pdMS_TO_TICKS(50));

    uint8_t whoami = 0;
    imu_device_t imu = {0};
    bool imu_ok = (imu_driver_init(&imu, &whoami) == ESP_OK);
    ESP_LOGI(TAG, "IMU %s at 0x%02X (WHOAMI=0x%02X)",
             imu_ok ? "found" : "not found", imu.addr, whoami);

    ESP_ERROR_CHECK(espnow_link_init());
    web_dashboard_state_t dashboard_state = {
        .idle_mode = &s_idle_mode,
        .motion_level = &s_motion_level,
        .last_motion_us = &s_last_motion_us,
        .game = &s_game,
    };
    captive_portal_t portal = {0};
    captive_portal_start(&portal, espnow_link_get_ap_netif(), &dashboard_state);

    ws2812_init();
    game_logic_init(&s_game, &s_strip);
    ws2812_clear(&s_strip);
    ws2812_show(&s_strip);
    vTaskDelay(pdMS_TO_TICKS(10));
    ws2812_show(&s_strip);

    static app_ctx_t ctx = {0};
    ctx.imu_ok = imu_ok;
    ctx.imu = imu;

    xTaskCreate(imu_led_task, "imu_led_task", 4096, &ctx, 5, NULL);
}
