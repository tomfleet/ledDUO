#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "led_ws2812.h"

typedef struct {
    ws2812_strip_t *strip;
    float pos_x;
    float pos_y;
    float vel_x;
    float vel_y;
    uint8_t trail[LED_STRIP_LEN];
    uint8_t ripple[LED_STRIP_LEN];
    bool idle_active;
    bool ripple_active;
    int64_t ripple_start_us;
    int64_t ripple_last_us;
    float ripple_vx;
    float ripple_vy;
    float ripple_phase;
    bool ripple_contact;
    uint8_t local_vpos_x;
    uint8_t local_vpos_y;
    uint8_t local_hue;
    float roll_deg;
    float pitch_deg;
} game_logic_t;

void game_logic_init(game_logic_t *game, ws2812_strip_t *strip);
void game_logic_render_frame(game_logic_t *game,
                             int16_t ax, int16_t ay, int16_t az,
                             int16_t gx, int16_t gy, int16_t gz,
                             int64_t t_us, bool idle_mode);
void game_logic_render_demo(game_logic_t *game, int64_t t_us);
