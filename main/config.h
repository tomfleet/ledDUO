#pragma once

/* I2C + IMU */
#define I2C_PORT             I2C_NUM_0
#define I2C_SDA_GPIO         11
#define I2C_SCL_GPIO         12
#define I2C_FREQ_HZ          400000

#define IMU_ADDR_PRIMARY     0x6B
#define IMU_ADDR_SECONDARY   0x6A

#define QMI8658_REG_WHOAMI   0x00
#define QMI8658_REG_CTRL1    0x02
#define QMI8658_REG_CTRL2    0x03
#define QMI8658_REG_CTRL3    0x04
#define QMI8658_REG_CTRL7    0x08
#define QMI8658_REG_AX_L     0x35

/* LED output + timing */
#define LED_STRIP_GPIO       14
#define LED_STRIP_LEN        64
#define LED_GRID_SIZE        8
#define LED_SERPENTINE       0
#define LED_BRIGHTNESS_SCALE 8   /* Lower = dimmer LEDs. */

#define WS2812_RESOLUTION_HZ 10000000
#define WS2812_T0H_TICKS     4
#define WS2812_T0L_TICKS     8
#define WS2812_T1H_TICKS     7
#define WS2812_T1L_TICKS     6

/* Motion + flow tuning */
#define TILT_INVERT_ROLL     1   /* Flip left/right if tilt feels reversed. */
#define TILT_INVERT_PITCH    1   /* Flip up/down if tilt feels reversed. */
#define TILT_GAIN_ROLL       0.65f /* Lower = less sensitive left/right. */
#define TILT_GAIN_PITCH      0.85f /* Lower = less sensitive up/down. */
#define FLOW_INPUT_SMOOTH    0.7f /* Higher = slower response to tilt changes. */
#define FLOW_RESISTANCE      0.5f /* Higher = more drag, shorter glide. */
#define VIRTUAL_GRID_SCALE   6.0f /* Higher = smoother sub-pixel motion. */
#define VIRTUAL_BLOB_SCALE   1.6f /* Blob size in virtual-grid units. */
#define BLOB_COLLIDE_RADIUS  (VIRTUAL_BLOB_SCALE * 1.4f) /* Virtual units. */
#define BLOB_COLLIDE_PUSH    0.75f /* 0-1, how hard blobs push apart. */
#define BLOB_COLLIDE_BOUNCE  0.35f /* 0-1, extra bounce on contact. */
#define RIPPLE_DURATION_MS   700  /* Collision ripple lifetime. */
#define RIPPLE_HOLDOFF_MS    450  /* Min time between ripple triggers. */
#define RIPPLE_SPEED_VU      20.0f /* Virtual units per second. */
#define RIPPLE_THICKNESS_VU  3.0f /* Ring thickness in virtual units. */
#define RIPPLE_INTENSITY     16 /* Brightness add (0-255). */
#define RIPPLE_ORGANIC       0.12f /* 0-1, ring wobble amount. */

/* Idle mode */
#define MOTION_GYRO_SCALE    30000.0f /* Higher = less sensitive motion detect. */
#define MOTION_ENTER_IDLE    0.5 /* Below this = can enter idle after timeout. */
#define MOTION_EXIT_IDLE     1.5f /* Above this = exit idle immediately. */
#define IDLE_TIMEOUT_MS      5000
#define IDLE_SPEED           0.22f /* Pixels per frame when idle. */
#define IDLE_LOCAL_DIM       60  /* 0-255, local trail dimming in idle. */
#define IDLE_REMOTE_DIM      45   /* 0-255, remote trail dimming in idle. */

/* Debug */
#define DEBUG_MOTION_LOGS    1   /* 1 = print IMU/motion state at 2 Hz. */
#define DEBUG_MOTION_PERIOD_MS 500
#define DEBUG_WS_ENABLE      1   /* 1 = enable WebSocket status stream. */
#define DEBUG_WS_PERIOD_MS   100 /* 10 Hz updates. */

/* ESP-NOW */
#define ESPNOW_CHANNEL       1   /* Must match on both boards. */
#define ESPNOW_SEND_INTERVAL_MS 40
#define ESPNOW_PAIR_WINDOW_MS 6000
#define ESPNOW_HELLO_INTERVAL_MS 300
#define ESPNOW_REMOTE_TIMEOUT_MS 800
#define ESPNOW_REMOTE_DIM    130 /* 0-255, lower = dimmer remote trail. */
#define ESPNOW_RSSI_MIN_DBM  -90
#define ESPNOW_RSSI_MAX_DBM  -40
#define ESPNOW_RSSI_GLOW_MAX 90  /* 0-255, glow intensity at strong RSSI. */
#define ESPNOW_RSSI_TRACK_MS 5000
#define ESPNOW_MSG_HELLO     1
#define ESPNOW_MSG_DATA      2
#define ESPNOW_MSG_RIPPLE    3

/* Captive portal (AP + status page) */
#define CAPTIVE_PORTAL_ENABLE 1
#define CAPTIVE_AP_SSID      "leDUO-debug"
#define CAPTIVE_AP_PASS      ""
#define CAPTIVE_AP_OPEN      1   /* 1 = open AP, 0 = WPA2 with CAPTIVE_AP_PASS. */
#define CAPTIVE_AP_MAX_CONN  2
#define CAPTIVE_AP_CHANNEL   ESPNOW_CHANNEL

/* mDNS (enable only if mdns component is available) */
#define MDNS_ENABLE          0
#define MDNS_HOSTNAME        "leduo"
#define MDNS_INSTANCE        "leDUO"
