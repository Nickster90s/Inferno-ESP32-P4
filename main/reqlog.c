#include "reqlog.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include <stdio.h>
#include <string.h>

#define RL_N        24
#define RL_BYTES    96

typedef struct {
    uint32_t t_ms;
    uint32_t ip;
    char     svc[6];
    uint16_t len;          // full request length
    uint16_t repeats;      // identical consecutive copies folded in
    uint8_t  data[RL_BYTES];
} rl_entry_t;

static rl_entry_t  s_rl[RL_N];
static uint32_t    s_wr;               // total entries ever written
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

void reqlog_add(const char *svc, uint32_t ip, const uint8_t *p, int n)
{
    if (n <= 0) return;
    uint16_t keep = n < RL_BYTES ? (uint16_t)n : RL_BYTES;
    taskENTER_CRITICAL(&s_mux);
    // Fold an identical repeat of the previous entry into a counter: a
    // controller polls, and the interesting request must not be pushed out.
    if (s_wr) {
        rl_entry_t *prev = &s_rl[(s_wr - 1) % RL_N];
        // Ignore the sequence number (bytes 4..5 in both framings) when comparing.
        if (prev->len == n && prev->ip == ip && strcmp(prev->svc, svc) == 0 &&
            memcmp(prev->data, p, 4) == 0 && memcmp(prev->data + 6, p + 6, keep - 6) == 0) {
            prev->repeats++;
            prev->t_ms = (uint32_t)(esp_timer_get_time() / 1000);
            taskEXIT_CRITICAL(&s_mux);
            return;
        }
    }
    rl_entry_t *e = &s_rl[s_wr % RL_N];
    e->t_ms = (uint32_t)(esp_timer_get_time() / 1000);
    e->ip = ip;
    strncpy(e->svc, svc, sizeof(e->svc) - 1);
    e->svc[sizeof(e->svc) - 1] = 0;
    e->len = (uint16_t)n;
    e->repeats = 0;
    memcpy(e->data, p, keep);
    s_wr++;
    taskEXIT_CRITICAL(&s_mux);
}

int reqlog_dump(char *out, size_t cap)
{
    size_t o = 0;
    uint32_t n = s_wr < RL_N ? s_wr : RL_N;
    for (uint32_t k = 0; k < n && o + 8 < cap; k++) {
        rl_entry_t e;
        taskENTER_CRITICAL(&s_mux);
        e = s_rl[(s_wr - n + k) % RL_N];
        taskEXIT_CRITICAL(&s_mux);
        const uint8_t *a = (const uint8_t *)&e.ip;
        int w = snprintf(out + o, cap - o, "%u %s %u.%u.%u.%u len=%u x%u ",
                         (unsigned)e.t_ms, e.svc, a[0], a[1], a[2], a[3],
                         (unsigned)e.len, (unsigned)e.repeats + 1);
        if (w < 0 || (size_t)w >= cap - o) break;
        o += w;
        uint16_t keep = e.len < RL_BYTES ? e.len : RL_BYTES;
        for (uint16_t i = 0; i < keep && o + 3 < cap; i++)
            o += snprintf(out + o, cap - o, "%02x", e.data[i]);
        if (o + 2 < cap) out[o++] = '\n';
    }
    if (o < cap) out[o] = 0;
    return (int)o;
}
