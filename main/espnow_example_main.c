/* IMU + WS2812 8x8 demo (ESP32-S3 + QMI8658 + WS2812) */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_random.h"
#include "esp_http_server.h"
#if MDNS_ENABLE
#include "mdns.h"
#endif
#include "nvs_flash.h"
#include "nvs.h"
#include "config.h"

static const char *TAG = "imu_led";

typedef struct {
    bool imu_ok;
    uint8_t imu_addr;
} app_ctx_t;

typedef struct {
    uint8_t pixels[LED_STRIP_LEN * 3];
} ws2812_strip_t;

static ws2812_strip_t s_strip;
static rmt_channel_handle_t s_rmt_chan;
static rmt_encoder_handle_t s_rmt_encoder;
static esp_netif_t *s_ap_netif;
static httpd_handle_t s_httpd;
static bool s_idle_mode;
static int64_t s_last_motion_us;
static float s_motion_level;
static int64_t s_last_send_us;
static float s_roll_deg;
static float s_pitch_deg;

typedef struct __attribute__((packed)) {
    uint8_t version;
    uint8_t msg_type;
    uint8_t vpos_x;
    uint8_t vpos_y;
    uint8_t hue;
} imu_led_packet_t;

typedef struct {
    bool has_peer;
    bool pairing_active;
    int64_t pairing_start_us;
    int64_t last_hello_us;
    bool broadcast_added;
    uint8_t peer_mac[ESP_NOW_ETH_ALEN];
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
} espnow_state_t;

static espnow_state_t s_espnow;
static uint8_t s_local_vpos_x;
static uint8_t s_local_vpos_y;
static uint8_t s_local_hue;
static const uint8_t s_broadcast_mac[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

static int clamp_int(int v, int lo, int hi)
{
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

static int led_index(int x, int y)
{
    if (LED_SERPENTINE && (y & 1)) {
        x = (LED_GRID_SIZE - 1) - x;
    }
    return (y * LED_GRID_SIZE) + x;
}

static uint8_t add_u8_sat(uint8_t a, uint8_t b)
{
    uint16_t sum = (uint16_t)a + (uint16_t)b;
    return (sum > 255) ? 255 : (uint8_t)sum;
}

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

static void update_rssi_tracking(int8_t rssi, int64_t now)
{
    if (s_espnow.rssi_track_end_us > 0 && now > s_espnow.rssi_track_end_us) {
        return;
    }
    if (!s_espnow.rssi_has_obs) {
        s_espnow.rssi_min_obs = rssi;
        s_espnow.rssi_max_obs = rssi;
        s_espnow.rssi_has_obs = true;
        return;
    }
    if (rssi < s_espnow.rssi_min_obs) {
        s_espnow.rssi_min_obs = rssi;
    }
    if (rssi > s_espnow.rssi_max_obs) {
        s_espnow.rssi_max_obs = rssi;
    }
}

static void format_mac(const uint8_t *mac, char *buf, size_t len)
{
    snprintf(buf, len, "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

static int build_status_json(char *buf, size_t len)
{
    uint8_t mac[6] = {0};
    char mac_str[18] = "";
    char peer_str[18] = "";
    int64_t now = esp_timer_get_time();
    int64_t since_motion_ms = s_last_motion_us > 0 ? ((now - s_last_motion_us) / 1000) : -1;
    int64_t pair_left_ms = s_espnow.pairing_active
        ? (ESPNOW_PAIR_WINDOW_MS - ((now - s_espnow.pairing_start_us) / 1000))
        : 0;
    if (pair_left_ms < 0) {
        pair_left_ms = 0;
    }
    int64_t rssi_left_ms = s_espnow.rssi_track_end_us > 0
        ? ((s_espnow.rssi_track_end_us - now) / 1000)
        : 0;
    if (rssi_left_ms < 0) {
        rssi_left_ms = 0;
    }

    esp_wifi_get_mac(WIFI_IF_STA, mac);
    format_mac(mac, mac_str, sizeof(mac_str));
    if (s_espnow.has_peer) {
        format_mac(s_espnow.peer_mac, peer_str, sizeof(peer_str));
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
                    s_espnow.has_peer ? peer_str : "",
                    s_espnow.has_peer ? "true" : "false",
                    s_idle_mode ? "true" : "false",
                    s_motion_level,
                    since_motion_ms,
                    (int)s_espnow.remote_rssi,
                    (int)s_espnow.rssi_min_obs,
                    (int)s_espnow.rssi_max_obs,
                    rssi_left_ms,
                    s_espnow.pairing_active ? "true" : "false",
                    pair_left_ms,
                    s_local_vpos_x,
                    s_local_vpos_y,
                    s_local_hue,
                    s_espnow.remote_vpos_x,
                    s_espnow.remote_vpos_y,
                    s_espnow.remote_hue,
                    s_roll_deg,
                    s_pitch_deg);
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

static void start_pairing_window(void)
{
    s_espnow.has_peer = false;
    s_espnow.pairing_active = true;
    s_espnow.pairing_start_us = esp_timer_get_time();
    s_espnow.last_hello_us = 0;
}

static esp_err_t http_root_get(httpd_req_t *req)
{
    static char body[10000];
    snprintf(body, sizeof(body),
             "<!doctype html><html><head>"
             "<meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
             "<title>leDUO debug</title></head><body>"
             "<style>"
             "body{margin:0;font-family:'Palatino Linotype','Book Antiqua',Palatino,serif;"
             "background:radial-gradient(circle at 10%% 10%%,#1c2b3a 0%%,#0b0f14 60%%);"
             "color:#e9f1ff;}"
             ".wrap{padding:18px;}"
             ".grid{display:grid;grid-template-columns:1fr 1fr;gap:16px;}"
             ".portrait .grid{grid-template-columns:1fr;}"
             ".card{background:rgba(15,25,35,0.75);border:1px solid rgba(120,160,200,0.25);"
             "border-radius:12px;padding:14px;box-shadow:0 10px 30px rgba(0,0,0,0.35);}"
             "h2{margin:6px 0 12px 0;font-weight:600;letter-spacing:0.5px;}"
             ".kv{display:grid;grid-template-columns:130px 1fr;gap:6px;font-size:14px;}"
             "canvas{width:100%%;height:180px;background:#0a1118;border-radius:10px;}"
             ".graph-stack{position:relative;width:100%%;height:180px;}"
             ".graph-stack canvas{position:absolute;left:0;top:0;width:100%%;height:100%%;}"
             ".graph-layer{background:transparent;}"
             "a{color:#9fd6ff;text-decoration:none;}"
             "</style></head><body>"
             "<div class='wrap'>"
             "<h2>leDUO debug</h2>"
             "<div class='grid'>"
             "<div class='card'>"
             "<div class='kv'>"
             "<div>MAC</div><div id='mac'>-</div>"
             "<div>Peer</div><div id='peer'>-</div>"
             "<div>Idle</div><div id='idle'>-</div>"
             "<div>Orientation</div><div id='orientation'>-</div>"
             "<div>Motion</div><div id='motion'>-</div>"
             "<div>Since</div><div id='since'>-</div>"
             "<div>RSSI</div><div id='rssi'>-</div>"
             "<div>Pairing</div><div id='pairing'>-</div>"
             "<div>Local</div><div id='local'>-</div>"
             "<div>Remote</div><div id='remote'>-</div>"
             "</div>"
             "<p><a href='/status'>/status</a> | <a href='/pair/clear'>clear pairing</a></p>"
             "</div>"
             "<div class='card'>"
             "<div>Roll/Pitch</div><canvas id='tilt'></canvas>"
             "<div style='margin-top:10px'>LED Preview</div><canvas id='led'></canvas>"
             "<div style='margin-top:10px'>Rolling Graph</div>"
             "<div class='graph-stack'>"
             "<canvas id='graph-base'></canvas>"
             "<canvas id='graph-roll' class='graph-layer'></canvas>"
             "<canvas id='graph-pitch' class='graph-layer'></canvas>"
             "</div>"
             "</div>"
             "</div>"
             "</div>"
             "<script>"
             "const tilt=document.getElementById('tilt');"
             "const graphBase=document.getElementById('graph-base');"
             "const graphRoll=document.getElementById('graph-roll');"
             "const graphPitch=document.getElementById('graph-pitch');"
             "const led=document.getElementById('led');"
             "const rollBuf=[],pitchBuf=[];const maxN=120;"
             "function setText(id,v){document.getElementById(id).textContent=v;}"
             "function updateOrientation(){const o=window.innerWidth>window.innerHeight?'landscape':'portrait';"
             "document.body.classList.remove('landscape','portrait');document.body.classList.add(o);"
             "setText('orientation',o);}"
             "function drawTilt(r,p){const ctx=tilt.getContext('2d');"
             "const w=tilt.width=tilt.clientWidth,h=tilt.height=tilt.clientHeight;"
             "ctx.clearRect(0,0,w,h);"
             "ctx.strokeStyle='#2d516b';ctx.lineWidth=2;"
             "ctx.beginPath();ctx.arc(w/2,h/2,Math.min(w,h)*0.35,0,Math.PI*2);ctx.stroke();"
             "const rx=Math.max(-45,Math.min(45,r));"
             "const py=Math.max(-45,Math.min(45,p));"
             "const dx=rx/45* Math.min(w,h)*0.32;"
             "const dy=py/45* Math.min(w,h)*0.32;"
             "ctx.fillStyle='#9fd6ff';ctx.beginPath();ctx.arc(w/2+dx,h/2+dy,8,0,Math.PI*2);ctx.fill();"
             "}"
             "function drawGraph(){const ctxBase=graphBase.getContext('2d');"
             "const w=graphBase.width=graphBase.clientWidth,h=graphBase.height=graphBase.clientHeight;"
             "ctxBase.clearRect(0,0,w,h);"
             "ctxBase.strokeStyle='#1e2a38';ctxBase.lineWidth=1;"
             "for(let i=1;i<4;i++){ctxBase.beginPath();ctxBase.moveTo(0,h*i/4);ctxBase.lineTo(w,h*i/4);ctxBase.stroke();}"
             "const ctxRoll=graphRoll.getContext('2d');const ctxPitch=graphPitch.getContext('2d');"
             "graphRoll.width=w;graphRoll.height=h;graphPitch.width=w;graphPitch.height=h;"
             "ctxRoll.clearRect(0,0,w,h);ctxPitch.clearRect(0,0,w,h);"
             "let maxRoll=5;for(const v of rollBuf){maxRoll=Math.max(maxRoll,Math.abs(v));}"
             "let maxPitch=5;for(const v of pitchBuf){maxPitch=Math.max(maxPitch,Math.abs(v));}"
             "maxRoll=Math.min(maxRoll,90);maxPitch=Math.min(maxPitch,90);"
             "function plot(ctx,arr,color,scale){ctx.strokeStyle=color;ctx.lineWidth=2;ctx.beginPath();"
             "for(let i=0;i<arr.length;i++){const x=i/(maxN-1)*w;"
             "const y=h/2 - (arr[i]/scale)*(h*0.4);"
             "if(i===0)ctx.moveTo(x,y);else ctx.lineTo(x,y);}ctx.stroke();}"
             "plot(ctxRoll,rollBuf,'#ff9c6b',maxRoll);plot(ctxPitch,pitchBuf,'#7fd1ff',maxPitch);"
             "ctxRoll.font='12px serif';ctxRoll.fillStyle='#ffb493';ctxRoll.fillText('±'+maxRoll.toFixed(0)+'°',8,16);"
             "const rightLabel='±'+maxPitch.toFixed(0)+'°';"
             "ctxPitch.font='12px serif';ctxPitch.fillStyle='#8fd8ff';"
             "ctxPitch.fillText(rightLabel,w-ctxPitch.measureText(rightLabel).width-8,16);"
             "}"
             "function hsv(h,s,v){const c=v*s;const x=c*(1-Math.abs(((h/60)%%2)-1));"
             "const m=v-c;let r=0,g=0,b=0;"
             "if(h<60){r=c;g=x;}else if(h<120){r=x;g=c;}else if(h<180){g=c;b=x;}"
             "else if(h<240){g=x;b=c;}else if(h<300){r=x;b=c;}else{r=c;b=x;}"
             "return [Math.round((r+m)*255),Math.round((g+m)*255),Math.round((b+m)*255)];}"
             "function drawLed(d){const ctx=led.getContext('2d');"
             "const w=led.width=led.clientWidth,h=led.height=led.clientHeight;ctx.clearRect(0,0,w,h);"
             "const size=Math.min(w,h);const pad=8;const cell=(size-2*pad)/8;"
             "ctx.fillStyle='#0b141d';ctx.fillRect(0,0,w,h);"
             "for(let y=0;y<8;y++){for(let x=0;x<8;x++){"
             "ctx.strokeStyle='rgba(80,110,140,0.25)';ctx.strokeRect(pad+x*cell,pad+y*cell,cell,cell);}}"
             "function blob(vx,vy,hue,alpha){const fx=vx/4;const fy=vy/4;"
             "const cx=pad+(fx+0.5)*cell;const cy=pad+(fy+0.5)*cell;"
             "const [r,g,b]=hsv((hue||0)/255*360,1,1);"
             "ctx.fillStyle='rgba('+r+','+g+','+b+','+alpha+')';"
             "ctx.beginPath();ctx.arc(cx,cy,cell*0.7,0,Math.PI*2);ctx.fill();"
             "ctx.globalAlpha=alpha*0.5;ctx.beginPath();ctx.arc(cx,cy,cell*1.4,0,Math.PI*2);ctx.fill();"
             "ctx.globalAlpha=1;}"
             "if(d.local_vpos)blob(d.local_vpos[0],d.local_vpos[1],d.local_hue,0.9);"
             "if(d.remote_vpos)blob(d.remote_vpos[0],d.remote_vpos[1],d.remote_hue,0.5);"
             "}"
             "function ingest(d){"
             "setText('mac',d.mac||'-');setText('peer',d.peer||'-');"
             "setText('idle',d.idle?'yes':'no');"
             "updateOrientation();"
             "setText('motion',d.motion?.toFixed(3));"
             "setText('since',d.since_motion_ms+' ms');"
             "setText('rssi',d.rssi+' dBm ('+d.rssi_min+'/'+d.rssi_max+')');"
             "setText('pairing',d.pairing?'open':'closed');"
             "setText('local',d.local_vpos+' hue '+d.local_hue);"
             "setText('remote',d.remote_vpos+' hue '+d.remote_hue);"
             "rollBuf.push(d.roll||0);pitchBuf.push(d.pitch||0);"
             "while(rollBuf.length>maxN){rollBuf.shift();pitchBuf.shift();}"
             "drawTilt(d.roll||0,d.pitch||0);drawLed(d);drawGraph();"
             "}"
             "function poll(){fetch('/status').then(r=>r.json()).then(ingest).catch(()=>{});}"
             "setInterval(poll,100);poll();"
             "</script></body></html>");

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

static esp_err_t http_pair_clear(httpd_req_t *req)
{
    clear_peer_in_nvs();
    start_pairing_window();
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_send(req, "pairing cleared\n", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t http_404_handler(httpd_req_t *req, httpd_err_code_t err)
{
    (void)err;
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "/");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

static void start_captive_portal(void)
{
#if CAPTIVE_PORTAL_ENABLE
    if (!s_ap_netif) {
        return;
    }
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.uri_match_fn = httpd_uri_match_wildcard;
    if (httpd_start(&s_httpd, &config) == ESP_OK) {
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
        httpd_register_uri_handler(s_httpd, &root);
        httpd_register_uri_handler(s_httpd, &status);
        httpd_register_uri_handler(s_httpd, &clear);
        httpd_register_err_handler(s_httpd, HTTPD_404_NOT_FOUND, http_404_handler);
    }
#endif
}

static void add_trail_bilinear(uint8_t *trail, float x, float y, float intensity)
{
    int x0 = (int)floorf(x);
    int y0 = (int)floorf(y);
    float fx = x - (float)x0;
    float fy = y - (float)y0;

    for (int dy = 0; dy <= 1; ++dy) {
        for (int dx = 0; dx <= 1; ++dx) {
            int xi = x0 + dx;
            int yi = y0 + dy;
            if (xi < 0 || xi >= LED_GRID_SIZE || yi < 0 || yi >= LED_GRID_SIZE) {
                continue;
            }
            float wx = dx ? fx : (1.0f - fx);
            float wy = dy ? fy : (1.0f - fy);
            float w = wx * wy;
            uint8_t add = (uint8_t)lrintf(intensity * w);
            if (add == 0) {
                continue;
            }
            int idx = led_index(xi, yi);
            trail[idx] = add_u8_sat(trail[idx], add);
        }
    }
}

static void add_trail_blob_virtual(uint8_t *trail, float vx, float vy, float intensity)
{
    for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
            float ox = (float)dx * VIRTUAL_BLOB_SCALE;
            float oy = (float)dy * VIRTUAL_BLOB_SCALE;
            float dist = sqrtf((ox * ox) + (oy * oy));
            float w = 1.0f - (dist / (VIRTUAL_BLOB_SCALE * 1.5f));
            if (w <= 0.0f) {
                continue;
            }
            float x = (vx + ox) / VIRTUAL_GRID_SCALE;
            float y = (vy + oy) / VIRTUAL_GRID_SCALE;
            add_trail_bilinear(trail, x, y, intensity * w);
        }
    }
}

static void ws2812_init(void)
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

static void ws2812_set_pixel(ws2812_strip_t *strip, int index, uint8_t r, uint8_t g, uint8_t b)
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

static void ws2812_clear(ws2812_strip_t *strip)
{
    memset(strip->pixels, 0, sizeof(strip->pixels));
}

static void ws2812_show(ws2812_strip_t *strip)
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

static bool mac_equal(const uint8_t *a, const uint8_t *b)
{
    return memcmp(a, b, ESP_NOW_ETH_ALEN) == 0;
}

static bool pair_with_sender(const uint8_t *mac)
{
    memcpy(s_espnow.peer_mac, mac, ESP_NOW_ETH_ALEN);
    esp_now_peer_info_t peer = {0};
    peer.channel = ESPNOW_CHANNEL;
    peer.ifidx = ESP_IF_WIFI_STA;
    peer.encrypt = false;
    memcpy(peer.peer_addr, s_espnow.peer_mac, ESP_NOW_ETH_ALEN);
    if (esp_now_add_peer(&peer) != ESP_OK) {
        return false;
    }
    s_espnow.has_peer = true;
    s_espnow.pairing_active = false;
    s_espnow.rssi_has_obs = false;
    s_espnow.rssi_track_end_us = esp_timer_get_time() + (ESPNOW_RSSI_TRACK_MS * 1000);
    save_peer_mac_to_nvs(s_espnow.peer_mac);
    ESP_LOGI(TAG, "Paired with %02X:%02X:%02X:%02X:%02X:%02X",
             s_espnow.peer_mac[0], s_espnow.peer_mac[1], s_espnow.peer_mac[2],
             s_espnow.peer_mac[3], s_espnow.peer_mac[4], s_espnow.peer_mac[5]);
    return true;
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
    bool from_peer = s_espnow.has_peer && mac_equal(recv_info->src_addr, s_espnow.peer_mac);
    if (pkt->msg_type == ESPNOW_MSG_HELLO) {
        if (!s_espnow.has_peer) {
            pair_with_sender(recv_info->src_addr);
        }
        return;
    }

    if (pkt->msg_type == ESPNOW_MSG_DATA) {
        if (!s_espnow.has_peer) {
            if (!pair_with_sender(recv_info->src_addr)) {
                return;
            }
            from_peer = true;
        }
        if (!from_peer) {
            return;
        }
        s_espnow.remote_vpos_x = pkt->vpos_x;
        s_espnow.remote_vpos_y = pkt->vpos_y;
        s_espnow.remote_hue = pkt->hue;
        if (recv_info->rx_ctrl) {
            s_espnow.remote_rssi = recv_info->rx_ctrl->rssi;
            update_rssi_tracking(s_espnow.remote_rssi, esp_timer_get_time());
        }
        s_espnow.remote_rx_time_us = esp_timer_get_time();
        s_espnow.has_remote = true;
    }
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
        s_espnow.broadcast_added = true;
        return ESP_OK;
    }
    return err;
}

static esp_err_t espnow_init(void)
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
    snprintf((char *)ap_cfg.ap.ssid, sizeof(ap_cfg.ap.ssid), "%s", CAPTIVE_AP_SSID);
    ap_cfg.ap.ssid_len = strlen(CAPTIVE_AP_SSID);
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

    if (load_peer_mac_from_nvs(s_espnow.peer_mac) == ESP_OK) {
        esp_now_peer_info_t peer = {0};
        peer.channel = ESPNOW_CHANNEL;
        peer.ifidx = ESP_IF_WIFI_STA;
        peer.encrypt = false;
        memcpy(peer.peer_addr, s_espnow.peer_mac, ESP_NOW_ETH_ALEN);
        ESP_ERROR_CHECK(esp_now_add_peer(&peer));
        s_espnow.has_peer = true;
        s_espnow.rssi_has_obs = false;
        s_espnow.rssi_track_end_us = esp_timer_get_time() + (ESPNOW_RSSI_TRACK_MS * 1000);
        ESP_LOGI(TAG, "ESP-NOW peer loaded: %02X:%02X:%02X:%02X:%02X:%02X",
                 s_espnow.peer_mac[0], s_espnow.peer_mac[1], s_espnow.peer_mac[2],
                 s_espnow.peer_mac[3], s_espnow.peer_mac[4], s_espnow.peer_mac[5]);
    } else {
        s_espnow.has_peer = false;
        s_espnow.pairing_active = true;
        s_espnow.pairing_start_us = esp_timer_get_time();
        s_espnow.last_hello_us = 0;
        add_broadcast_peer();
        ESP_LOGW(TAG, "No ESP-NOW peer in NVS; pairing window open.");
    }

    return ESP_OK;
}

static void hsv_to_rgb(uint8_t h, uint8_t s, uint8_t v, uint8_t *r, uint8_t *g, uint8_t *b)
{
    uint8_t region = h / 43;
    uint8_t remainder = (h - (region * 43)) * 6;

    uint8_t p = (uint8_t)((v * (255 - s)) >> 8);
    uint8_t q = (uint8_t)((v * (255 - ((s * remainder) >> 8))) >> 8);
    uint8_t t = (uint8_t)((v * (255 - ((s * (255 - remainder)) >> 8))) >> 8);

    switch (region) {
        case 0: *r = v; *g = t; *b = p; break;
        case 1: *r = q; *g = v; *b = p; break;
        case 2: *r = p; *g = v; *b = t; break;
        case 3: *r = p; *g = q; *b = v; break;
        case 4: *r = t; *g = p; *b = v; break;
        default: *r = v; *g = p; *b = q; break;
    }
}

static esp_err_t i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t value)
{
    uint8_t data[2] = { reg, value };
    return i2c_master_write_to_device(I2C_PORT, addr, data, sizeof(data), pdMS_TO_TICKS(100));
}

static esp_err_t i2c_read_reg(uint8_t addr, uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_PORT, addr, &reg, 1, data, len, pdMS_TO_TICKS(100));
}

static esp_err_t qmi8658_init(uint8_t addr, uint8_t *whoami)
{
    if (i2c_read_reg(addr, QMI8658_REG_WHOAMI, whoami, 1) != ESP_OK) {
        return ESP_FAIL;
    }

    /* Minimal init: enable accel + gyro. Adjust if your module needs different settings. */
    ESP_ERROR_CHECK(i2c_write_reg(addr, QMI8658_REG_CTRL1, 0x60));
    ESP_ERROR_CHECK(i2c_write_reg(addr, QMI8658_REG_CTRL2, 0x23));
    ESP_ERROR_CHECK(i2c_write_reg(addr, QMI8658_REG_CTRL3, 0x23));
    ESP_ERROR_CHECK(i2c_write_reg(addr, QMI8658_REG_CTRL7, 0x03));
    vTaskDelay(pdMS_TO_TICKS(20));
    return ESP_OK;
}

static esp_err_t qmi8658_read_raw(uint8_t addr, int16_t *ax, int16_t *ay, int16_t *az,
                                  int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t data[12] = {0};
    if (i2c_read_reg(addr, QMI8658_REG_AX_L, data, sizeof(data)) != ESP_OK) {
        return ESP_FAIL;
    }

    *ax = (int16_t)((data[1] << 8) | data[0]);
    *ay = (int16_t)((data[3] << 8) | data[2]);
    *az = (int16_t)((data[5] << 8) | data[4]);
    *gx = (int16_t)((data[7] << 8) | data[6]);
    *gy = (int16_t)((data[9] << 8) | data[8]);
    *gz = (int16_t)((data[11] << 8) | data[10]);
    return ESP_OK;
}

static void render_frame(app_ctx_t *ctx, int16_t ax, int16_t ay, int16_t az,
                         int16_t gx, int16_t gy, int16_t gz, int64_t t_us,
                         bool idle_mode)
{
    static float pos_x = 3.5f;
    static float pos_y = 3.5f;
    static float vel_x = 0.0f;
    static float vel_y = 0.0f;
    static uint8_t trail[LED_STRIP_LEN] = {0};
    static bool idle_active = false;

    float fax = (float)ax;
    float fay = (float)ay;
    float faz = (float)az;

    float roll = atan2f(fay, faz);
    float pitch = atan2f(-fax, sqrtf((fay * fay) + (faz * faz)));
    s_roll_deg = roll * 57.2958f;
    s_pitch_deg = pitch * 57.2958f;

    float roll_norm = roll / (float)(M_PI / 2.0f);
    float pitch_norm = pitch / (float)(M_PI / 2.0f);
    if (roll_norm > 1.0f) roll_norm = 1.0f;
    if (roll_norm < -1.0f) roll_norm = -1.0f;
    if (pitch_norm > 1.0f) pitch_norm = 1.0f;
    if (pitch_norm < -1.0f) pitch_norm = -1.0f;

    if (TILT_INVERT_ROLL) {
        roll_norm = -roll_norm;
    }
    if (TILT_INVERT_PITCH) {
        pitch_norm = -pitch_norm;
    }

    roll_norm *= TILT_GAIN_ROLL;
    pitch_norm *= TILT_GAIN_PITCH;
    if (roll_norm > 1.0f) roll_norm = 1.0f;
    if (roll_norm < -1.0f) roll_norm = -1.0f;
    if (pitch_norm > 1.0f) pitch_norm = 1.0f;
    if (pitch_norm < -1.0f) pitch_norm = -1.0f;

    if (idle_mode) {
        if (!idle_active) {
            float angle = (float)(esp_random() & 0xFFFF) / 65535.0f * (float)(M_PI * 2.0f);
            vel_x = cosf(angle) * IDLE_SPEED;
            vel_y = sinf(angle) * IDLE_SPEED;
            idle_active = true;
        }
    } else {
        idle_active = false;
        float mag = sqrtf((roll_norm * roll_norm) + (pitch_norm * pitch_norm));
        if (mag > 1.0f) {
            mag = 1.0f;
        }

        float speed = 0.06f + (0.45f * mag);
        vel_x = (vel_x * FLOW_INPUT_SMOOTH) + (roll_norm * speed);
        vel_y = (vel_y * FLOW_INPUT_SMOOTH) + (pitch_norm * speed);
        vel_x *= (1.0f - FLOW_RESISTANCE);
        vel_y *= (1.0f - FLOW_RESISTANCE);
    }

    pos_x += vel_x;
    pos_y += vel_y;

    if (pos_x < 0.0f) { pos_x = 0.0f; vel_x *= -0.6f; }
    if (pos_x > (LED_GRID_SIZE - 1)) { pos_x = (LED_GRID_SIZE - 1); vel_x *= -0.6f; }
    if (pos_y < 0.0f) { pos_y = 0.0f; vel_y *= -0.6f; }
    if (pos_y > (LED_GRID_SIZE - 1)) { pos_y = (LED_GRID_SIZE - 1); vel_y *= -0.6f; }

    for (int i = 0; i < LED_STRIP_LEN; ++i) {
        trail[i] = (uint8_t)((trail[i] * 220) / 255);
    }

    float vpos_x = pos_x * VIRTUAL_GRID_SCALE;
    float vpos_y = pos_y * VIRTUAL_GRID_SCALE;
    float vtail_x = (pos_x - (vel_x * 2.0f)) * VIRTUAL_GRID_SCALE;
    float vtail_y = (pos_y - (vel_y * 2.0f)) * VIRTUAL_GRID_SCALE;
    s_local_vpos_x = (uint8_t)clamp_int((int)lrintf(vpos_x), 0,
                                        (int)lrintf((LED_GRID_SIZE - 1) * VIRTUAL_GRID_SCALE));
    s_local_vpos_y = (uint8_t)clamp_int((int)lrintf(vpos_y), 0,
                                        (int)lrintf((LED_GRID_SIZE - 1) * VIRTUAL_GRID_SCALE));
    s_local_hue = (uint8_t)((t_us / 12000) & 0xFF);
    float local_dim = idle_mode ? (IDLE_LOCAL_DIM / 255.0f) : 1.0f;
    add_trail_blob_virtual(trail, vpos_x, vpos_y, 220.0f * local_dim);
    add_trail_blob_virtual(trail, vtail_x, vtail_y, 120.0f * local_dim);

    if (s_espnow.has_remote) {
        int64_t age_ms = (t_us - s_espnow.remote_rx_time_us) / 1000;
        if (age_ms < ESPNOW_REMOTE_TIMEOUT_MS) {
            float fade = 1.0f - ((float)age_ms / (float)ESPNOW_REMOTE_TIMEOUT_MS);
            float base_dim = idle_mode ? IDLE_REMOTE_DIM : ESPNOW_REMOTE_DIM;
            int8_t rssi_min = s_espnow.rssi_has_obs ? s_espnow.rssi_min_obs : ESPNOW_RSSI_MIN_DBM;
            int8_t rssi_max = s_espnow.rssi_has_obs ? s_espnow.rssi_max_obs : ESPNOW_RSSI_MAX_DBM;
            float denom = (float)(rssi_max - rssi_min);
            float rssi_norm = denom > 1.0f ? ((s_espnow.remote_rssi - rssi_min) / denom) : 0.0f;
            rssi_norm = clamp_f(rssi_norm, 0.0f, 1.0f);
            float link = 0.2f + (0.8f * rssi_norm);
            float dim = (base_dim / 255.0f) * fade * link;
            float rvx = (float)s_espnow.remote_vpos_x;
            float rvy = (float)s_espnow.remote_vpos_y;
            add_trail_blob_virtual(trail, rvx, rvy, 200.0f * dim);
            add_trail_blob_virtual(trail, rvx, rvy, ESPNOW_RSSI_GLOW_MAX * fade * rssi_norm);
        } else {
            s_espnow.has_remote = false;
        }
    }

    uint8_t base_hue = s_local_hue;
    for (int y = 0; y < LED_GRID_SIZE; ++y) {
        for (int x = 0; x < LED_GRID_SIZE; ++x) {
            int idx = led_index(x, y);
            uint8_t v = trail[idx];
            uint8_t r = 0, g = 0, b = 0;
            uint8_t hue = (uint8_t)(base_hue + (x * 8) + (y * 6));
            hsv_to_rgb(hue, 255, v, &r, &g, &b);
            ws2812_set_pixel(&s_strip, idx, r, g, b);
        }
    }

    ws2812_show(&s_strip);
}

static void render_demo(app_ctx_t *ctx, int64_t t_us)
{
    float t = (float)t_us / 1000000.0f;
    int x = (int)lrintf((sinf(t * 1.4f) * 3.5f) + 3.5f);
    int y = (int)lrintf((cosf(t * 1.1f) * 3.5f) + 3.5f);
    x = clamp_int(x, 0, LED_GRID_SIZE - 1);
    y = clamp_int(y, 0, LED_GRID_SIZE - 1);

    uint8_t r = 0, g = 0, b = 0;
    uint8_t hue = (uint8_t)((t_us / 8000) & 0xFF);
    hsv_to_rgb(hue, 255, 200, &r, &g, &b);

    for (int i = 0; i < LED_STRIP_LEN; ++i) {
        ws2812_set_pixel(&s_strip, i, 2, 1, 4);
    }

    ws2812_set_pixel(&s_strip, led_index(x, y), r, g, b);
    ws2812_show(&s_strip);
}

static void imu_led_task(void *pv)
{
    app_ctx_t *ctx = (app_ctx_t *)pv;
    int64_t last_send_us = 0;
    int64_t last_motion_us = 0;
    int64_t last_debug_us = 0;
    bool idle_mode = false;
    while (true) {
        int16_t ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
        int64_t now = esp_timer_get_time();

        if (s_espnow.pairing_active) {
            int64_t elapsed_ms = (now - s_espnow.pairing_start_us) / 1000;
            if (elapsed_ms > ESPNOW_PAIR_WINDOW_MS) {
                s_espnow.pairing_active = false;
                ESP_LOGW(TAG, "Pairing window closed; local-only.");
            } else if ((now - s_espnow.last_hello_us) > (ESPNOW_HELLO_INTERVAL_MS * 1000)) {
                imu_led_packet_t hello = {
                    .version = 1,
                    .msg_type = ESPNOW_MSG_HELLO,
                    .vpos_x = 0,
                    .vpos_y = 0,
                    .hue = 0,
                };
                esp_now_send(s_broadcast_mac, (uint8_t *)&hello, sizeof(hello));
                s_espnow.last_hello_us = now;
            }
        }

        if (ctx->imu_ok && qmi8658_read_raw(ctx->imu_addr, &ax, &ay, &az, &gx, &gy, &gz) == ESP_OK) {
            float motion = (fabsf((float)gx) + fabsf((float)gy) + fabsf((float)gz)) / MOTION_GYRO_SCALE;
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
                int8_t rssi_min = s_espnow.rssi_has_obs ? s_espnow.rssi_min_obs : ESPNOW_RSSI_MIN_DBM;
                int8_t rssi_max = s_espnow.rssi_has_obs ? s_espnow.rssi_max_obs : ESPNOW_RSSI_MAX_DBM;
                float denom = (float)(rssi_max - rssi_min);
                float rssi_norm = denom > 1.0f ? ((s_espnow.remote_rssi - rssi_min) / denom) : 0.0f;
                rssi_norm = clamp_f(rssi_norm, 0.0f, 1.0f);
                float glow = ESPNOW_RSSI_GLOW_MAX * rssi_norm;
                ESP_LOGI(TAG,
                         "imu ax=%d ay=%d az=%d gx=%d gy=%d gz=%d motion=%.3f idle=%d since=%lldms rssi=%d norm=%.2f glow=%.1f",
                         ax, ay, az, gx, gy, gz, motion, idle_mode ? 1 : 0, since_motion_ms,
                         s_espnow.remote_rssi, rssi_norm, glow);
                last_debug_us = now;
            }
            render_frame(ctx, ax, ay, az, gx, gy, gz, now, idle_mode);

            if (s_espnow.has_peer && (now - last_send_us) > (ESPNOW_SEND_INTERVAL_MS * 1000)) {
                imu_led_packet_t pkt = {
                    .version = 1,
                    .msg_type = ESPNOW_MSG_DATA,
                    .vpos_x = s_local_vpos_x,
                    .vpos_y = s_local_vpos_y,
                    .hue = s_local_hue,
                };
                esp_now_send(s_espnow.peer_mac, (uint8_t *)&pkt, sizeof(pkt));
                last_send_us = now;
                s_last_send_us = now;
            }
        } else {
            render_demo(ctx, now);
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

    i2c_config_t i2c_cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &i2c_cfg));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, i2c_cfg.mode, 0, 0, 0));

    uint8_t whoami = 0;
    uint8_t imu_addr = IMU_ADDR_PRIMARY;
    bool imu_ok = (qmi8658_init(imu_addr, &whoami) == ESP_OK);
    if (!imu_ok) {
        imu_addr = IMU_ADDR_SECONDARY;
        imu_ok = (qmi8658_init(imu_addr, &whoami) == ESP_OK);
    }
    ESP_LOGI(TAG, "IMU %s at 0x%02X (WHOAMI=0x%02X)", imu_ok ? "found" : "not found", imu_addr, whoami);

    ESP_ERROR_CHECK(espnow_init());
    start_captive_portal();

    ws2812_init();
    ws2812_clear(&s_strip);
    ws2812_show(&s_strip);
    vTaskDelay(pdMS_TO_TICKS(10));
    ws2812_show(&s_strip);

    static app_ctx_t ctx = {0};
    ctx.imu_ok = imu_ok;
    ctx.imu_addr = imu_addr;

    xTaskCreate(imu_led_task, "imu_led_task", 4096, &ctx, 5, NULL);
}
