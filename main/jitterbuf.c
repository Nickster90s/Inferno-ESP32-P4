#include "jitterbuf.h"
#include "aoip_wire.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

// The ring. Internal SRAM, never PSRAM: this is read by the I2S feed path and
// PSRAM access latency is precisely the jitter this design exists to avoid.
static int32_t s_ring[AP_RING_FRAMES * AP_NCH];

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static uint64_t s_rx_head;
static uint64_t s_playout;
static jb_stats_t s_st;

void jb_init(void)
{
    memset(s_ring, 0, sizeof(s_ring));
    memset(&s_st, 0, sizeof(s_st));
    s_rx_head = 0;
    s_playout = 0;
}

void jb_reset(uint64_t playout_idx)
{
    taskENTER_CRITICAL(&s_mux);
    memset(s_ring, 0, sizeof(s_ring));
    s_playout = playout_idx;
    s_rx_head = playout_idx;
    taskEXIT_CRITICAL(&s_mux);
}

uint64_t jb_playout_idx(void)
{
    taskENTER_CRITICAL(&s_mux);
    uint64_t v = s_playout;
    taskEXIT_CRITICAL(&s_mux);
    return v;
}

uint64_t jb_rx_head(void)
{
    taskENTER_CRITICAL(&s_mux);
    uint64_t v = s_rx_head;
    taskEXIT_CRITICAL(&s_mux);
    return v;
}

void jb_step_playout(int64_t delta_frames)
{
    taskENTER_CRITICAL(&s_mux);
    s_playout = (uint64_t)((int64_t)s_playout + delta_frames);
    s_st.playout_steps++;
    taskEXIT_CRITICAL(&s_mux);
}

bool jb_write(uint64_t first_idx, uint32_t nframes,
              uint32_t nslots, const int8_t *slot_to_ch, const uint8_t *be24)
{
    if (nframes == 0 || nslots == 0) { s_st.pkt_malformed++; return false; }

    taskENTER_CRITICAL(&s_mux);
    uint64_t playout = s_playout;
    taskEXIT_CRITICAL(&s_mux);

    // Signed difference of two 64-bit indices. Never a plain unsigned subtract:
    // that is the shape of the bug that wedged the FPGA four times.
    int64_t ahead = (int64_t)first_idx - (int64_t)playout;

    if (ahead + (int64_t)nframes <= 0) {
        // Entirely in the past -- the DAC has already played this window.
        s_st.pkt_late++;
        return false;
    }
    if (ahead > AP_FUTURE_GUARD_FRAMES) {
        // A single corrupt timestamp would otherwise scribble the whole ring.
        s_st.pkt_future++;
        return false;
    }

    const uint8_t *src = be24;
    for (uint32_t f = 0; f < nframes; f++) {
        uint64_t idx = first_idx + f;
        // Partially-late packet: skip the frames already played, keep the rest.
        if ((int64_t)idx - (int64_t)playout < 0) { src += nslots * 3; continue; }

        int32_t *row = &s_ring[(size_t)(idx & AP_RING_MASK) * AP_NCH];
        for (uint32_t s = 0; s < nslots; s++) {
            int8_t ch = slot_to_ch[s];
            if (ch >= 0 && ch < AP_NCH) {
                // AoIP is 24-bit big-endian MSB-justified; shifting it up by 8
                // yields exactly the 32-bit MSB-justified word the I2S slot
                // wants. No scaling step, deliberately.
                row[ch] = (int32_t)(((uint32_t)src[0] << 24) |
                                    ((uint32_t)src[1] << 16) |
                                    ((uint32_t)src[2] << 8));
            }
            src += 3;
        }
    }

    uint64_t end = first_idx + nframes;
    taskENTER_CRITICAL(&s_mux);
    if ((int64_t)end - (int64_t)s_rx_head > 0) s_rx_head = end;
    s_st.pkt_written++;
    taskEXIT_CRITICAL(&s_mux);
    return true;
}

void jb_read(int32_t *dst, uint32_t nframes)
{
    taskENTER_CRITICAL(&s_mux);
    uint64_t idx  = s_playout;
    uint64_t head = s_rx_head;
    taskEXIT_CRITICAL(&s_mux);

    int64_t avail = (int64_t)head - (int64_t)idx;
    if (avail < (int64_t)nframes) {
        // Reading past what anyone wrote. The unwritten frames are already
        // zero (we clear behind ourselves), so the output is silence rather
        // than a ring-length-old repeat -- the correct concealment.
        uint32_t missing = (avail < 0) ? nframes : (uint32_t)((int64_t)nframes - avail);
        s_st.frames_underrun += missing;
    }

    size_t pos = (size_t)(idx & AP_RING_MASK);
    size_t first = nframes;
    if (pos + first > AP_RING_FRAMES) first = AP_RING_FRAMES - pos;
    size_t rest = nframes - first;

    memcpy(dst, &s_ring[pos * AP_NCH], first * AP_NCH * sizeof(int32_t));
    memset(&s_ring[pos * AP_NCH], 0, first * AP_NCH * sizeof(int32_t));
    if (rest) {
        memcpy(dst + first * AP_NCH, &s_ring[0], rest * AP_NCH * sizeof(int32_t));
        memset(&s_ring[0], 0, rest * AP_NCH * sizeof(int32_t));
    }

    taskENTER_CRITICAL(&s_mux);
    s_playout += nframes;
    s_st.level_frames = (int32_t)((int64_t)s_rx_head - (int64_t)s_playout);
    taskEXIT_CRITICAL(&s_mux);
}

void jb_get_stats(jb_stats_t *out)
{
    taskENTER_CRITICAL(&s_mux);
    *out = s_st;
    out->rx_head     = s_rx_head;
    out->playout_idx = s_playout;
    taskEXIT_CRITICAL(&s_mux);
}
