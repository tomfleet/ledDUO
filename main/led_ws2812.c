#include "led_ws2812.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rmt_tx.h"
#include "esp_log.h"

static const char *TAG = "ws2812";

static rmt_channel_handle_t s_rmt_chan;
static rmt_encoder_handle_t s_rmt_encoder;

void ws2812_init(void)
{
    rmt_tx_channel_config_t tx_cfg = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = LED_STRIP_GPIO,
        .mem_block_symbols = 64,
        .resolution_hz = WS2812_RESOLUTION_HZ,
        .trans_queue_depth = 4,
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_cfg, &s_rmt_chan));

    rmt_bytes_encoder_config_t enc_cfg = {
        .bit0 = {
            .level0 = 1,
            .duration0 = WS2812_T0H_TICKS,
            .level1 = 0,
            .duration1 = WS2812_T0L_TICKS,
        },
        .bit1 = {
            .level0 = 1,
            .duration0 = WS2812_T1H_TICKS,
            .level1 = 0,
            .duration1 = WS2812_T1L_TICKS,
        },
        .flags.msb_first = 1,
    };
    ESP_ERROR_CHECK(rmt_new_bytes_encoder(&enc_cfg, &s_rmt_encoder));
    ESP_ERROR_CHECK(rmt_enable(s_rmt_chan));
}

void ws2812_set_pixel(ws2812_strip_t *strip, int index, uint8_t r, uint8_t g, uint8_t b)
{
    if (index < 0 || index >= LED_STRIP_LEN) {
        return;
    }
    if (LED_BRIGHTNESS_SCALE < 255) {
        r = (uint8_t)((r * LED_BRIGHTNESS_SCALE) / 255);
        g = (uint8_t)((g * LED_BRIGHTNESS_SCALE) / 255);
        b = (uint8_t)((b * LED_BRIGHTNESS_SCALE) / 255);
    }
    int base = index * 3;
    strip->pixels[base + 0] = g;
    strip->pixels[base + 1] = r;
    strip->pixels[base + 2] = b;
}

void ws2812_clear(ws2812_strip_t *strip)
{
    memset(strip->pixels, 0, sizeof(strip->pixels));
}

void ws2812_show(ws2812_strip_t *strip)
{
    rmt_transmit_config_t tx_cfg = {
        .loop_count = 0,
    };
    esp_err_t err = rmt_transmit(s_rmt_chan, s_rmt_encoder, strip->pixels,
                                 sizeof(strip->pixels), &tx_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "RMT transmit failed: 0x%x", err);
        return;
    }

    err = rmt_tx_wait_all_done(s_rmt_chan, pdMS_TO_TICKS(200));
    if (err == ESP_ERR_TIMEOUT) {
        ESP_LOGW(TAG, "RMT flush timeout, resetting channel");
        rmt_disable(s_rmt_chan);
        rmt_enable(s_rmt_chan);
        return;
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "RMT wait failed: 0x%x", err);
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(1));
}
