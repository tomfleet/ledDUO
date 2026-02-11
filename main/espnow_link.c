#include "espnow_link.h"

#include <stdio.h>
#include <string.h>
#include "esp_event.h"
#include "esp_log.h"
#include "esp_now.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "nvs.h"
#include "config.h"
#if MDNS_ENABLE
#include "mdns.h"
#endif

static const char *TAG = "espnow_link";

typedef struct __attribute__((packed)) {
    uint8_t version;
    uint8_t msg_type;
    uint8_t vpos_x;
    uint8_t vpos_y;
    uint8_t hue;
} imu_led_packet_t;

static espnow_link_state_t s_state;
static bool s_ripple_pending;
static uint8_t s_ripple_pending_vx;
static uint8_t s_ripple_pending_vy;
static esp_netif_t *s_ap_netif;
static const uint8_t s_broadcast_mac[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

static void update_rssi_tracking(int8_t rssi, int64_t now)
{
    if (s_state.rssi_track_end_us > 0 && now > s_state.rssi_track_end_us) {
        return;
    }
    if (!s_state.rssi_has_obs) {
        s_state.rssi_min_obs = rssi;
        s_state.rssi_max_obs = rssi;
        s_state.rssi_has_obs = true;
        return;
    }
    if (rssi < s_state.rssi_min_obs) {
        s_state.rssi_min_obs = rssi;
    }
    if (rssi > s_state.rssi_max_obs) {
        s_state.rssi_max_obs = rssi;
    }
}

static bool mac_equal(const uint8_t *a, const uint8_t *b)
{
    return memcmp(a, b, ESP_NOW_ETH_ALEN) == 0;
}

static esp_err_t load_peer_mac_from_nvs(uint8_t *mac_out)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open("pairing", NVS_READONLY, &handle);
    if (err != ESP_OK) {
        return err;
    }
    size_t len = ESP_NOW_ETH_ALEN;
    err = nvs_get_blob(handle, "peer_mac", mac_out, &len);
    nvs_close(handle);
    if (err != ESP_OK || len != ESP_NOW_ETH_ALEN) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

static esp_err_t save_peer_mac_to_nvs(const uint8_t *mac)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open("pairing", NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_blob(handle, "peer_mac", mac, ESP_NOW_ETH_ALEN);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    return err;
}

static esp_err_t clear_peer_in_nvs(void)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open("pairing", NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_erase_key(handle, "peer_mac");
    if (err == ESP_OK || err == ESP_ERR_NVS_NOT_FOUND) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    return err;
}

static bool pair_with_sender(const uint8_t *mac)
{
    memcpy(s_state.peer_mac, mac, ESP_NOW_ETH_ALEN);
    esp_now_peer_info_t peer = {0};
    peer.channel = ESPNOW_CHANNEL;
    peer.ifidx = ESP_IF_WIFI_STA;
    peer.encrypt = false;
    memcpy(peer.peer_addr, s_state.peer_mac, ESP_NOW_ETH_ALEN);
    if (esp_now_add_peer(&peer) != ESP_OK) {
        return false;
    }
    s_state.has_peer = true;
    s_state.pairing_active = false;
    s_state.rssi_has_obs = false;
    s_state.rssi_track_end_us = esp_timer_get_time() + (ESPNOW_RSSI_TRACK_MS * 1000);
    save_peer_mac_to_nvs(s_state.peer_mac);
    ESP_LOGI(TAG, "Paired with %02X:%02X:%02X:%02X:%02X:%02X",
             s_state.peer_mac[0], s_state.peer_mac[1], s_state.peer_mac[2],
             s_state.peer_mac[3], s_state.peer_mac[4], s_state.peer_mac[5]);
    return true;
}

static esp_err_t add_broadcast_peer(void)
{
    esp_now_peer_info_t peer = {0};
    peer.channel = ESPNOW_CHANNEL;
    peer.ifidx = ESP_IF_WIFI_STA;
    peer.encrypt = false;
    memcpy(peer.peer_addr, s_broadcast_mac, ESP_NOW_ETH_ALEN);
    esp_err_t err = esp_now_add_peer(&peer);
    if (err == ESP_OK || err == ESP_ERR_ESPNOW_EXIST) {
        s_state.broadcast_added = true;
        return ESP_OK;
    }
    return err;
}

static void start_pairing_window(void)
{
    s_state.has_peer = false;
    s_state.pairing_active = true;
    s_state.pairing_start_us = esp_timer_get_time();
    s_state.last_hello_us = 0;
}

static void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    if (recv_info == NULL || data == NULL || len < (int)sizeof(imu_led_packet_t)) {
        return;
    }
    const imu_led_packet_t *pkt = (const imu_led_packet_t *)data;
    if (pkt->version != 1) {
        return;
    }
    bool from_peer = s_state.has_peer && mac_equal(recv_info->src_addr, s_state.peer_mac);
    if (pkt->msg_type == ESPNOW_MSG_HELLO) {
        if (!s_state.has_peer) {
            pair_with_sender(recv_info->src_addr);
        }
        return;
    }

    if (pkt->msg_type == ESPNOW_MSG_RIPPLE) {
        if (!s_state.has_peer) {
            if (!pair_with_sender(recv_info->src_addr)) {
                return;
            }
            from_peer = true;
        }
        if (!from_peer) {
            return;
        }
        s_ripple_pending_vx = pkt->vpos_x;
        s_ripple_pending_vy = pkt->vpos_y;
        s_ripple_pending = true;
        return;
    }

    if (pkt->msg_type == ESPNOW_MSG_DATA) {
        if (!s_state.has_peer) {
            if (!pair_with_sender(recv_info->src_addr)) {
                return;
            }
            from_peer = true;
        }
        if (!from_peer) {
            return;
        }
        s_state.remote_vpos_x = pkt->vpos_x;
        s_state.remote_vpos_y = pkt->vpos_y;
        s_state.remote_hue = pkt->hue;
        if (recv_info->rx_ctrl) {
            s_state.remote_rssi = recv_info->rx_ctrl->rssi;
            update_rssi_tracking(s_state.remote_rssi, esp_timer_get_time());
        }
        s_state.remote_rx_time_us = esp_timer_get_time();
        s_state.has_remote = true;
    }
}

esp_err_t espnow_link_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    s_ap_netif = esp_netif_create_default_wifi_ap();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));

    wifi_config_t ap_cfg = {0};
    uint8_t mac[6] = {0};
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    snprintf((char *)ap_cfg.ap.ssid, sizeof(ap_cfg.ap.ssid), "%s-%02X%02X",
             CAPTIVE_AP_SSID, mac[4], mac[5]);
    ap_cfg.ap.ssid_len = strlen((char *)ap_cfg.ap.ssid);
    ap_cfg.ap.channel = CAPTIVE_AP_CHANNEL;
    ap_cfg.ap.max_connection = CAPTIVE_AP_MAX_CONN;
    if (CAPTIVE_AP_OPEN) {
        ap_cfg.ap.authmode = WIFI_AUTH_OPEN;
    } else {
        snprintf((char *)ap_cfg.ap.password, sizeof(ap_cfg.ap.password), "%s", CAPTIVE_AP_PASS);
        ap_cfg.ap.authmode = WIFI_AUTH_WPA2_PSK;
    }
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

#if MDNS_ENABLE
    ESP_ERROR_CHECK(mdns_init());
    ESP_ERROR_CHECK(mdns_hostname_set(MDNS_HOSTNAME));
    ESP_ERROR_CHECK(mdns_instance_name_set(MDNS_INSTANCE));
    ESP_ERROR_CHECK(mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0));
#endif

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));

    if (load_peer_mac_from_nvs(s_state.peer_mac) == ESP_OK) {
        esp_now_peer_info_t peer = {0};
        peer.channel = ESPNOW_CHANNEL;
        peer.ifidx = ESP_IF_WIFI_STA;
        peer.encrypt = false;
        memcpy(peer.peer_addr, s_state.peer_mac, ESP_NOW_ETH_ALEN);
        ESP_ERROR_CHECK(esp_now_add_peer(&peer));
        s_state.has_peer = true;
        s_state.rssi_has_obs = false;
        s_state.rssi_track_end_us = esp_timer_get_time() + (ESPNOW_RSSI_TRACK_MS * 1000);
        ESP_LOGI(TAG, "ESP-NOW peer loaded: %02X:%02X:%02X:%02X:%02X:%02X",
                 s_state.peer_mac[0], s_state.peer_mac[1], s_state.peer_mac[2],
                 s_state.peer_mac[3], s_state.peer_mac[4], s_state.peer_mac[5]);
    } else {
        s_state.has_peer = false;
        s_state.pairing_active = true;
        s_state.pairing_start_us = esp_timer_get_time();
        s_state.last_hello_us = 0;
        add_broadcast_peer();
        ESP_LOGW(TAG, "No ESP-NOW peer in NVS; pairing window open.");
    }

    return ESP_OK;
}

const espnow_link_state_t *espnow_link_get_state(void)
{
    return &s_state;
}

void espnow_link_clear_remote(void)
{
    s_state.has_remote = false;
}

void espnow_link_start_pairing(void)
{
    start_pairing_window();
}

void espnow_link_close_pairing(void)
{
    s_state.pairing_active = false;
}

esp_err_t espnow_link_clear_peer(void)
{
    esp_err_t err = clear_peer_in_nvs();
    if (err == ESP_OK || err == ESP_ERR_NVS_NOT_FOUND) {
        start_pairing_window();
    }
    return err;
}

bool espnow_link_has_peer(void)
{
    return s_state.has_peer;
}

void espnow_link_get_peer_mac(uint8_t *mac_out)
{
    if (!mac_out) {
        return;
    }
    memcpy(mac_out, s_state.peer_mac, ESP_NOW_ETH_ALEN);
}

bool espnow_link_is_pairing_active(void)
{
    return s_state.pairing_active;
}

int64_t espnow_link_get_pairing_start_us(void)
{
    return s_state.pairing_start_us;
}

int64_t espnow_link_get_last_hello_us(void)
{
    return s_state.last_hello_us;
}

void espnow_link_set_last_hello_us(int64_t now_us)
{
    s_state.last_hello_us = now_us;
}

void espnow_link_send_hello(void)
{
    imu_led_packet_t hello = {
        .version = 1,
        .msg_type = ESPNOW_MSG_HELLO,
        .vpos_x = 0,
        .vpos_y = 0,
        .hue = 0,
    };
    esp_now_send(s_broadcast_mac, (uint8_t *)&hello, sizeof(hello));
}

void espnow_link_send_data(uint8_t vpos_x, uint8_t vpos_y, uint8_t hue)
{
    if (!s_state.has_peer) {
        return;
    }
    imu_led_packet_t pkt = {
        .version = 1,
        .msg_type = ESPNOW_MSG_DATA,
        .vpos_x = vpos_x,
        .vpos_y = vpos_y,
        .hue = hue,
    };
    esp_now_send(s_state.peer_mac, (uint8_t *)&pkt, sizeof(pkt));
}

void espnow_link_send_ripple(uint8_t vpos_x, uint8_t vpos_y)
{
    if (!s_state.has_peer) {
        return;
    }
    imu_led_packet_t pkt = {
        .version = 1,
        .msg_type = ESPNOW_MSG_RIPPLE,
        .vpos_x = vpos_x,
        .vpos_y = vpos_y,
        .hue = 0,
    };
    esp_now_send(s_state.peer_mac, (uint8_t *)&pkt, sizeof(pkt));
}

bool espnow_link_take_ripple_event(uint8_t *vx, uint8_t *vy)
{
    if (!s_ripple_pending) {
        return false;
    }
    if (vx) {
        *vx = s_ripple_pending_vx;
    }
    if (vy) {
        *vy = s_ripple_pending_vy;
    }
    s_ripple_pending = false;
    return true;
}

esp_netif_t *espnow_link_get_ap_netif(void)
{
    return s_ap_netif;
}
