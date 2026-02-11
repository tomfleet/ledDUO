#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "esp_http_server.h"
#include "game_logic.h"

typedef struct {
    const bool *idle_mode;
    const float *motion_level;
    const int64_t *last_motion_us;
    const game_logic_t *game;
} web_dashboard_state_t;

esp_err_t web_dashboard_init(const web_dashboard_state_t *state);
esp_err_t web_dashboard_register_handlers(httpd_handle_t server);
