#pragma once

#include <stdint.h>
#include "config.h"

typedef struct {
    uint8_t pixels[LED_STRIP_LEN * 3];
} ws2812_strip_t;

void ws2812_init(void);
void ws2812_set_pixel(ws2812_strip_t *strip, int index, uint8_t r, uint8_t g, uint8_t b);
void ws2812_clear(ws2812_strip_t *strip);
void ws2812_show(ws2812_strip_t *strip);
