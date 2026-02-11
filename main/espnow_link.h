#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "esp_netif.h"

typedef struct {
    bool has_peer;
    bool pairing_active;
    int64_t pairing_start_us;
    int64_t last_hello_us;
    bool broadcast_added;
    uint8_t peer_mac[6];
    bool has_remote;
    uint8_t remote_vpos_x;
    uint8_t remote_vpos_y;
    uint8_t remote_hue;
    int8_t remote_rssi;
    int8_t rssi_min_obs;
    int8_t rssi_max_obs;
    bool rssi_has_obs;
    int64_t rssi_track_end_us;
    int64_t remote_rx_time_us;
} espnow_link_state_t;

esp_err_t espnow_link_init(void);
const espnow_link_state_t *espnow_link_get_state(void);
void espnow_link_clear_remote(void);
void espnow_link_start_pairing(void);
void espnow_link_close_pairing(void);
esp_err_t espnow_link_clear_peer(void);

bool espnow_link_has_peer(void);
void espnow_link_get_peer_mac(uint8_t *mac_out);

bool espnow_link_is_pairing_active(void);
int64_t espnow_link_get_pairing_start_us(void);
int64_t espnow_link_get_last_hello_us(void);
void espnow_link_set_last_hello_us(int64_t now_us);

void espnow_link_send_hello(void);
void espnow_link_send_data(uint8_t vpos_x, uint8_t vpos_y, uint8_t hue);
void espnow_link_send_ripple(uint8_t vpos_x, uint8_t vpos_y);

bool espnow_link_take_ripple_event(uint8_t *vx, uint8_t *vy);

esp_netif_t *espnow_link_get_ap_netif(void);
