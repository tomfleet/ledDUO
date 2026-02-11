#include "web_dashboard.h"

#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_spiffs.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "config.h"
#include "espnow_link.h"
#include "http_root_html.h"

static const char *TAG = "web_dashboard";

static web_dashboard_state_t s_state;
static bool s_state_ready = false;

static void format_mac(const uint8_t *mac, char *buf, size_t len)
{
    snprintf(buf, len, "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

static int build_status_json(char *buf, size_t len)
{
    const espnow_link_state_t *link = espnow_link_get_state();
    uint8_t mac[6] = {0};
    char mac_str[18] = "";
    char peer_str[18] = "";
    int64_t now = esp_timer_get_time();
    int64_t since_motion_ms = 0;
    int64_t pair_left_ms = link->pairing_active
        ? (ESPNOW_PAIR_WINDOW_MS - ((now - link->pairing_start_us) / 1000))
        : 0;
    int64_t rssi_left_ms = link->rssi_track_end_us > 0
        ? ((link->rssi_track_end_us - now) / 1000)
        : 0;
    bool idle_mode = false;
    float motion_level = 0.0f;
    uint8_t local_vpos_x = 0;
    uint8_t local_vpos_y = 0;
    uint8_t local_hue = 0;
    float roll_deg = 0.0f;
    float pitch_deg = 0.0f;

    if (s_state_ready) {
        if (s_state.last_motion_us && *s_state.last_motion_us > 0) {
            since_motion_ms = (now - *s_state.last_motion_us) / 1000;
        }
        if (s_state.idle_mode) {
            idle_mode = *s_state.idle_mode;
        }
        if (s_state.motion_level) {
            motion_level = *s_state.motion_level;
        }
        if (s_state.game) {
            local_vpos_x = s_state.game->local_vpos_x;
            local_vpos_y = s_state.game->local_vpos_y;
            local_hue = s_state.game->local_hue;
            roll_deg = s_state.game->roll_deg;
            pitch_deg = s_state.game->pitch_deg;
        }
    }

    if (pair_left_ms < 0) {
        pair_left_ms = 0;
    }
    if (rssi_left_ms < 0) {
        rssi_left_ms = 0;
    }

    esp_wifi_get_mac(WIFI_IF_STA, mac);
    format_mac(mac, mac_str, sizeof(mac_str));
    if (link->has_peer) {
        format_mac(link->peer_mac, peer_str, sizeof(peer_str));
    }

    return snprintf(buf, len,
                    "{\"mac\":\"%s\",\"peer\":\"%s\",\"has_peer\":%s,"
                    "\"idle\":%s,\"motion\":%.3f,\"since_motion_ms\":%lld,"
                    "\"rssi\":%d,\"rssi_min\":%d,\"rssi_max\":%d,\"rssi_track_ms\":%lld,"
                    "\"pairing\":%s,\"pairing_ms\":%lld,"
                    "\"local_vpos\":[%u,%u],\"local_hue\":%u,"
                    "\"remote_vpos\":[%u,%u],\"remote_hue\":%u,"
                    "\"roll\":%.2f,\"pitch\":%.2f}",
                    mac_str,
                    link->has_peer ? peer_str : "",
                    link->has_peer ? "true" : "false",
                    idle_mode ? "true" : "false",
                    motion_level,
                    since_motion_ms,
                    (int)link->remote_rssi,
                    (int)link->rssi_min_obs,
                    (int)link->rssi_max_obs,
                    rssi_left_ms,
                    link->pairing_active ? "true" : "false",
                    pair_left_ms,
                    local_vpos_x,
                    local_vpos_y,
                    local_hue,
                    link->remote_vpos_x,
                    link->remote_vpos_y,
                    link->remote_hue,
                    roll_deg,
                    pitch_deg);
}

static esp_err_t http_root_get(httpd_req_t *req)
{
    static char body[20000];
    snprintf(body, sizeof(body), "%s", kHttpRootHtml);

    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, body, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t http_status_get(httpd_req_t *req)
{
    char body[512];
    build_status_json(body, sizeof(body));
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, body, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t http_overlay_png_get(httpd_req_t *req)
{
    FILE *f = fopen("/spiffs/overlay.png", "rb");
    if (!f) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "overlay missing");
        return ESP_FAIL;
    }
    httpd_resp_set_type(req, "image/png");
    char buf[2048];
    size_t read_len = 0;
    while ((read_len = fread(buf, 1, sizeof(buf), f)) > 0) {
        esp_err_t err = httpd_resp_send_chunk(req, buf, read_len);
        if (err != ESP_OK) {
            fclose(f);
            httpd_resp_sendstr_chunk(req, NULL);
            return err;
        }
    }
    fclose(f);
    return httpd_resp_sendstr_chunk(req, NULL);
}

static esp_err_t http_pair_clear(httpd_req_t *req)
{
    espnow_link_clear_peer();
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_send(req, "pairing cleared\n", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static void init_spiffs(void)
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 4,
        .format_if_mount_failed = false,
    };
    esp_err_t err = esp_vfs_spiffs_register(&conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPIFFS mount failed: 0x%x", err);
        return;
    }
    size_t total = 0;
    size_t used = 0;
    err = esp_spiffs_info(NULL, &total, &used);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "SPIFFS: total=%u used=%u", (unsigned)total, (unsigned)used);
    }
}

esp_err_t web_dashboard_init(const web_dashboard_state_t *state)
{
    if (!state) {
        return ESP_ERR_INVALID_ARG;
    }

    s_state = *state;
    s_state_ready = true;
    init_spiffs();
    return ESP_OK;
}

esp_err_t web_dashboard_register_handlers(httpd_handle_t server)
{
    if (!server) {
        return ESP_ERR_INVALID_ARG;
    }

    httpd_uri_t root = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = http_root_get,
    };
    httpd_uri_t status = {
        .uri = "/status",
        .method = HTTP_GET,
        .handler = http_status_get,
    };
    httpd_uri_t clear = {
        .uri = "/pair/clear",
        .method = HTTP_GET,
        .handler = http_pair_clear,
    };
    httpd_uri_t overlay = {
        .uri = "/overlay.png",
        .method = HTTP_GET,
        .handler = http_overlay_png_get,
    };

    httpd_register_uri_handler(server, &root);
    httpd_register_uri_handler(server, &status);
    httpd_register_uri_handler(server, &clear);
    httpd_register_uri_handler(server, &overlay);
    return ESP_OK;
}
