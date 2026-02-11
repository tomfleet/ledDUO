#pragma once

#include "esp_err.h"
#include "esp_http_server.h"
#include "esp_netif.h"
#include "web_dashboard.h"

typedef struct {
    httpd_handle_t server;
} captive_portal_t;

esp_err_t captive_portal_start(captive_portal_t *portal,
                               esp_netif_t *ap_netif,
                               const web_dashboard_state_t *state);
