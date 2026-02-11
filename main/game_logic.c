#include "game_logic.h"

#include <math.h>
#include <string.h>
#include "esp_random.h"
#include "config.h"
#include "espnow_link.h"

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

static uint32_t xorshift32(uint32_t *state)
{
    uint32_t x = *state;
    if (x == 0) {
        x = 0x6C8E9CF5u;
    }
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *state = x;
    return x;
}

static uint16_t game_logic_rand_u16(game_logic_t *game)
{
    if (game && game->rng_seeded) {
        return (uint16_t)(xorshift32(&game->rng_state) & 0xFFFFu);
    }
    return (uint16_t)(esp_random() & 0xFFFFu);
}

void game_logic_init(game_logic_t *game, ws2812_strip_t *strip)
{
    if (!game) {
        return;
    }
    memset(game, 0, sizeof(*game));
    game->strip = strip;
    game->pos_x = 3.5f;
    game->pos_y = 3.5f;
}

void game_logic_set_seed(game_logic_t *game, uint32_t seed)
{
    if (!game) {
        return;
    }
    if (seed == 0) {
        seed = 0x6C8E9CF5u;
    }
    game->rng_state = seed;
    game->rng_seeded = true;
}

void game_logic_render_frame(game_logic_t *game,
                             int16_t ax, int16_t ay, int16_t az,
                             int16_t gx, int16_t gy, int16_t gz,
                             int64_t t_us, bool idle_mode)
{
    if (!game || !game->strip) {
        return;
    }

    uint8_t pending_vx = 0;
    uint8_t pending_vy = 0;
    if (espnow_link_take_ripple_event(&pending_vx, &pending_vy)) {
        game->ripple_active = true;
        game->ripple_start_us = t_us;
        game->ripple_last_us = t_us;
        game->ripple_vx = (float)pending_vx;
        game->ripple_vy = (float)pending_vy;
        game->ripple_phase = (float)(esp_random() & 0xFFFF) / 65535.0f * (float)(M_PI * 2.0f);
        game->ripple_contact = true;
    }

    float fax = (float)ax;
    float fay = (float)ay;
    float faz = (float)az;

    float roll = atan2f(fay, faz);
    float pitch = atan2f(-fax, sqrtf((fay * fay) + (faz * faz)));
    game->roll_deg = roll * 57.2958f;
    game->pitch_deg = pitch * 57.2958f;

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
        if (!game->idle_active) {
            float angle = (float)game_logic_rand_u16(game) / 65535.0f * (float)(M_PI * 2.0f);
            game->vel_x = cosf(angle) * IDLE_SPEED;
            game->vel_y = sinf(angle) * IDLE_SPEED;
            game->idle_active = true;
        }
    } else {
        game->idle_active = false;
        float mag = sqrtf((roll_norm * roll_norm) + (pitch_norm * pitch_norm));
        if (mag > 1.0f) {
            mag = 1.0f;
        }

        float speed = 0.06f + (0.45f * mag);
        game->vel_x = (game->vel_x * FLOW_INPUT_SMOOTH) + (roll_norm * speed);
        game->vel_y = (game->vel_y * FLOW_INPUT_SMOOTH) + (pitch_norm * speed);
        game->vel_x *= (1.0f - FLOW_RESISTANCE);
        game->vel_y *= (1.0f - FLOW_RESISTANCE);
    }

    game->pos_x += game->vel_x;
    game->pos_y += game->vel_y;

    const espnow_link_state_t *link = espnow_link_get_state();
    bool remote_active = false;
    float rvx = 0.0f;
    float rvy = 0.0f;
    int64_t remote_age_ms = 0;
    if (link->has_remote) {
        remote_age_ms = (t_us - link->remote_rx_time_us) / 1000;
        if (remote_age_ms < ESPNOW_REMOTE_TIMEOUT_MS) {
            remote_active = true;
            rvx = (float)link->remote_vpos_x;
            rvy = (float)link->remote_vpos_y;
        } else {
            espnow_link_clear_remote();
        }
    }

    bool contact = false;
    if (remote_active) {
        float lvx = game->pos_x * VIRTUAL_GRID_SCALE;
        float lvy = game->pos_y * VIRTUAL_GRID_SCALE;
        float dx = lvx - rvx;
        float dy = lvy - rvy;
        float dist2 = (dx * dx) + (dy * dy);
        float radius = BLOB_COLLIDE_RADIUS;
        if (dist2 < (radius * radius) && dist2 > 0.0001f) {
            contact = true;
            float dist = sqrtf(dist2);
            float overlap = radius - dist;
            float nx = dx / dist;
            float ny = dy / dist;
            float push = overlap / VIRTUAL_GRID_SCALE;

            if (!game->ripple_contact && ((t_us - game->ripple_last_us) / 1000) > RIPPLE_HOLDOFF_MS) {
                game->ripple_active = true;
                game->ripple_start_us = t_us;
                game->ripple_last_us = t_us;
                game->ripple_vx = (lvx + rvx) * 0.5f;
                game->ripple_vy = (lvy + rvy) * 0.5f;
                game->ripple_phase = (float)(esp_random() & 0xFFFF) / 65535.0f * (float)(M_PI * 2.0f);
                espnow_link_send_ripple(
                    (uint8_t)clamp_int((int)lrintf(game->ripple_vx), 0,
                                       (int)lrintf((LED_GRID_SIZE - 1) * VIRTUAL_GRID_SCALE)),
                    (uint8_t)clamp_int((int)lrintf(game->ripple_vy), 0,
                                       (int)lrintf((LED_GRID_SIZE - 1) * VIRTUAL_GRID_SCALE)));
            }

            game->pos_x += nx * (push * BLOB_COLLIDE_PUSH);
            game->pos_y += ny * (push * BLOB_COLLIDE_PUSH);
            game->vel_x += nx * (push * (BLOB_COLLIDE_PUSH * 0.5f));
            game->vel_y += ny * (push * (BLOB_COLLIDE_PUSH * 0.5f));

            float dot = (game->vel_x * nx) + (game->vel_y * ny);
            if (dot < 0.0f) {
                game->vel_x -= (1.0f + BLOB_COLLIDE_BOUNCE) * dot * nx;
                game->vel_y -= (1.0f + BLOB_COLLIDE_BOUNCE) * dot * ny;
            }
        }
    }
    game->ripple_contact = contact;

    if (game->pos_x < 0.0f) { game->pos_x = 0.0f; game->vel_x *= -0.6f; }
    if (game->pos_x > (LED_GRID_SIZE - 1)) { game->pos_x = (LED_GRID_SIZE - 1); game->vel_x *= -0.6f; }
    if (game->pos_y < 0.0f) { game->pos_y = 0.0f; game->vel_y *= -0.6f; }
    if (game->pos_y > (LED_GRID_SIZE - 1)) { game->pos_y = (LED_GRID_SIZE - 1); game->vel_y *= -0.6f; }

    for (int i = 0; i < LED_STRIP_LEN; ++i) {
        game->trail[i] = (uint8_t)((game->trail[i] * 220) / 255);
        game->ripple[i] = (uint8_t)((game->ripple[i] * 200) / 255);
    }

    float vpos_x = game->pos_x * VIRTUAL_GRID_SCALE;
    float vpos_y = game->pos_y * VIRTUAL_GRID_SCALE;
    float vtail_x = (game->pos_x - (game->vel_x * 2.0f)) * VIRTUAL_GRID_SCALE;
    float vtail_y = (game->pos_y - (game->vel_y * 2.0f)) * VIRTUAL_GRID_SCALE;
    game->local_vpos_x = (uint8_t)clamp_int((int)lrintf(vpos_x), 0,
                                            (int)lrintf((LED_GRID_SIZE - 1) * VIRTUAL_GRID_SCALE));
    game->local_vpos_y = (uint8_t)clamp_int((int)lrintf(vpos_y), 0,
                                            (int)lrintf((LED_GRID_SIZE - 1) * VIRTUAL_GRID_SCALE));
    game->local_hue = (uint8_t)((t_us / 12000) & 0xFF);
    float local_dim = idle_mode ? (IDLE_LOCAL_DIM / 255.0f) : 1.0f;
    add_trail_blob_virtual(game->trail, vpos_x, vpos_y, 220.0f * local_dim);
    add_trail_blob_virtual(game->trail, vtail_x, vtail_y, 120.0f * local_dim);

    if (remote_active) {
        float fade = 1.0f - ((float)remote_age_ms / (float)ESPNOW_REMOTE_TIMEOUT_MS);
        float base_dim = idle_mode ? IDLE_REMOTE_DIM : ESPNOW_REMOTE_DIM;
        int8_t rssi_min = link->rssi_has_obs ? link->rssi_min_obs : ESPNOW_RSSI_MIN_DBM;
        int8_t rssi_max = link->rssi_has_obs ? link->rssi_max_obs : ESPNOW_RSSI_MAX_DBM;
        float denom = (float)(rssi_max - rssi_min);
        float rssi_norm = denom > 1.0f ? ((link->remote_rssi - rssi_min) / denom) : 0.0f;
        rssi_norm = clamp_f(rssi_norm, 0.0f, 1.0f);
        float link_strength = 0.2f + (0.8f * rssi_norm);
        float dim = (base_dim / 255.0f) * fade * link_strength;
        add_trail_blob_virtual(game->trail, rvx, rvy, 200.0f * dim);
        add_trail_blob_virtual(game->trail, rvx, rvy, ESPNOW_RSSI_GLOW_MAX * fade * rssi_norm);
    }

    if (game->ripple_active) {
        int64_t ripple_age_ms = (t_us - game->ripple_start_us) / 1000;
        if (ripple_age_ms > RIPPLE_DURATION_MS) {
            game->ripple_active = false;
        } else {
            float age_s = (float)ripple_age_ms / 1000.0f;
            float base_r = RIPPLE_SPEED_VU * age_s;
            float fade = 1.0f - ((float)ripple_age_ms / (float)RIPPLE_DURATION_MS);
            const int samples = 24;
            for (int i = 0; i < samples; ++i) {
                float ang = ((float)i / (float)samples) * (float)(M_PI * 2.0f);
                float wobble = 1.0f + (RIPPLE_ORGANIC * sinf((3.0f * ang) + game->ripple_phase));
                float r = base_r * wobble;
                float vx = game->ripple_vx + cosf(ang) * r;
                float vy = game->ripple_vy + sinf(ang) * r;
                float gx = vx / VIRTUAL_GRID_SCALE;
                float gy = vy / VIRTUAL_GRID_SCALE;
                add_trail_bilinear(game->ripple, gx, gy, (float)RIPPLE_INTENSITY * fade);

                float r2 = r + RIPPLE_THICKNESS_VU;
                vx = game->ripple_vx + cosf(ang) * r2;
                vy = game->ripple_vy + sinf(ang) * r2;
                gx = vx / VIRTUAL_GRID_SCALE;
                gy = vy / VIRTUAL_GRID_SCALE;
                add_trail_bilinear(game->ripple, gx, gy, ((float)RIPPLE_INTENSITY * 0.5f) * fade);
            }
        }
    }

    uint8_t base_hue = game->local_hue;
    for (int y = 0; y < LED_GRID_SIZE; ++y) {
        for (int x = 0; x < LED_GRID_SIZE; ++x) {
            int idx = led_index(x, y);
            uint8_t v = game->trail[idx];
            uint8_t w = game->ripple[idx];
            uint8_t r = 0, g = 0, b = 0;
            uint8_t hue = (uint8_t)(base_hue + (x * 8) + (y * 6));
            hsv_to_rgb(hue, 255, v, &r, &g, &b);
            if (w) {
                r = add_u8_sat(r, w);
                g = add_u8_sat(g, w);
                b = add_u8_sat(b, w);
            }
            ws2812_set_pixel(game->strip, idx, r, g, b);
        }
    }

    ws2812_show(game->strip);
}

void game_logic_render_demo(game_logic_t *game, int64_t t_us)
{
    if (!game || !game->strip) {
        return;
    }

    float t = (float)t_us / 1000000.0f;
    int x = (int)lrintf((sinf(t * 1.4f) * 3.5f) + 3.5f);
    int y = (int)lrintf((cosf(t * 1.1f) * 3.5f) + 3.5f);
    x = clamp_int(x, 0, LED_GRID_SIZE - 1);
    y = clamp_int(y, 0, LED_GRID_SIZE - 1);

    uint8_t r = 0, g = 0, b = 0;
    uint8_t hue = (uint8_t)((t_us / 8000) & 0xFF);
    hsv_to_rgb(hue, 255, 200, &r, &g, &b);

    for (int i = 0; i < LED_STRIP_LEN; ++i) {
        ws2812_set_pixel(game->strip, i, 2, 1, 4);
    }

    ws2812_set_pixel(game->strip, led_index(x, y), r, g, b);
    ws2812_show(game->strip);
}
