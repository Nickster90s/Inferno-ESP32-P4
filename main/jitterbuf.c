#include "jitterbuf.h"
#include "aoip_wire.h"
#include "rate.h"
#include "esp_attr.h"
#include "esp_cache.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

// ---------------------------------------------------------------------------
// The ring = the I2S DMA buffers, learned from the TX interrupt.
//
// Slots are counted UNWRAPPED from the moment the channel started: the DMA
// begins at descriptor 0, so slot s lives in buffer (s / F) % N at frame s % F.
// An absolute AoIP sample index A lives in slot A + s_off; s_off is set by the
// anchor and moved only by a step.
// ---------------------------------------------------------------------------
#define JB_MAX_BUFS  (AP_RING_FRAMES / 16)

static int32_t  *s_buf[JB_MAX_BUFS];
static uint32_t  s_nbufs;
static uint32_t  s_F, s_fshift;

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static uint32_t s_mapped;        // buffers learned so far (first lap)
static bool     s_ready;
static uint32_t s_next_k;        // descriptor expected to finish next
static uint64_t s_dma_end;       // unwrapped slot the DMA has read up to
static int64_t  s_t_isr;         // esp_timer us of the last TX interrupt
static int64_t  s_off;           // slot = A + s_off
static bool     s_anchored;
static uint64_t s_rx_head;
static jb_stats_t s_st;
static uint32_t s_isr_max_us;

void jb_init(void)
{
    memset(&s_st, 0, sizeof(s_st));
    s_rx_head = 0;
}

void jb_attach_dma(uint32_t nbufs, uint32_t frames_per_buf)
{
    s_nbufs = nbufs;
    s_F = frames_per_buf;
    s_fshift = 0;
    while ((1u << s_fshift) < s_F) s_fshift++;
    s_mapped = 0; s_ready = false; s_next_k = 0; s_dma_end = 0;
}

bool jb_ready(void) { return s_ready; }

void IRAM_ATTR jb_isr_sent(void *buf)
{
    int64_t t = esp_timer_get_time();
    portENTER_CRITICAL_ISR(&s_mux);
    if (!s_ready) {
        // First lap: descriptors finish in order from 0, so this is buffer
        // s_mapped. Record it.
        s_buf[s_mapped++] = (int32_t *)buf;
        s_dma_end += s_F;
        s_next_k = s_mapped % s_nbufs;
        if (s_mapped == s_nbufs) s_ready = true;
    } else {
        uint32_t k = s_next_k, adv = 1;
        if (s_buf[k] != buf) {
            // An interrupt was late enough to cover two descriptors: find
            // which one this is. Position stays exact; only the count shows it.
            for (uint32_t j = 0; j < s_nbufs; j++) {
                if (s_buf[j] == buf) { adv = ((j + s_nbufs - k) % s_nbufs) + 1; k = j; break; }
            }
            s_st.isr_gaps++;
        }
        s_dma_end += (uint64_t)adv * s_F;
        s_next_k = (k + 1) % s_nbufs;
    }
    s_t_isr = t;

    // Underrun: the descriptor just played held frames nobody wrote.
    if (s_anchored) {
        int64_t a_end   = (int64_t)s_dma_end - s_off;
        int64_t a_start = a_end - (int64_t)s_F;
        int64_t head    = (int64_t)s_rx_head;
        if (head < a_end) {
            int64_t from = head > a_start ? head : a_start;
            s_st.frames_underrun += (uint32_t)(a_end - from);
        }
    }
    portEXIT_CRITICAL_ISR(&s_mux);

    uint32_t us = (uint32_t)(esp_timer_get_time() - t);
    if (us > s_isr_max_us) s_isr_max_us = us;
}

// Where the DMA is now, in unwrapped slots: the last interrupt's position plus
// the frames played since. Clamped, so a stalled interrupt cannot make the
// estimate run away.
static int64_t dma_pos_now(int64_t *off_out)
{
    portENTER_CRITICAL(&s_mux);
    uint64_t e = s_dma_end;
    int64_t  ti = s_t_isr;
    int64_t  off = s_off;
    portEXIT_CRITICAL(&s_mux);
    int64_t el = (esp_timer_get_time() - ti) * (int64_t)rate_hz() / 1000000;
    if (el < 0) el = 0;
    if (el > 2 * (int64_t)s_F) el = 2 * (int64_t)s_F;
    if (off_out) *off_out = off;
    return (int64_t)e + el;
}

static void clear_ring(void)
{
    const size_t bytes = (size_t)s_F * AP_NCH * sizeof(int32_t);
    for (uint32_t k = 0; k < s_mapped; k++) {
        memset(s_buf[k], 0, bytes);
        esp_cache_msync(s_buf[k], bytes, ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_UNALIGNED);
    }
}

void jb_reset(uint64_t playout_idx)
{
    if (!s_ready) return;
    int64_t pos = dma_pos_now(NULL) - AP_I2S_FIFO_FRAMES;   // slot at the DAC
    portENTER_CRITICAL(&s_mux);
    s_off = pos - (int64_t)playout_idx;
    s_rx_head = playout_idx;
    s_anchored = true;
    portEXIT_CRITICAL(&s_mux);
    clear_ring();
}

uint64_t jb_playout_idx(void)
{
    int64_t off;
    int64_t pos = dma_pos_now(&off) - AP_I2S_FIFO_FRAMES;
    return (uint64_t)(pos - off);
}

uint64_t jb_rx_head(void)
{
    portENTER_CRITICAL(&s_mux);
    uint64_t v = s_rx_head;
    portEXIT_CRITICAL(&s_mux);
    return v;
}

void jb_step_playout(int64_t delta_frames)
{
    // playout = pos - off, so moving playout by +delta is off -= delta. What
    // was written under the old mapping would now play at the wrong time:
    // clear it. A step is audible either way.
    portENTER_CRITICAL(&s_mux);
    s_off -= delta_frames;
    s_st.playout_steps++;
    portEXIT_CRITICAL(&s_mux);
    clear_ring();
}

static inline void sync_out(int32_t *p, size_t frames)
{
    if (frames)
        esp_cache_msync(p, frames * AP_NCH * sizeof(int32_t),
                        ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_UNALIGNED);
}

bool jb_write(uint64_t first_idx, uint32_t nframes,
              uint32_t nslots, const int8_t *slot_to_ch, const uint8_t *be24)
{
    if (nframes == 0 || nslots == 0) { s_st.pkt_malformed++; return false; }
    if (!s_ready || !s_anchored) return false;

    // The DMA's READ head: where it is, plus what it may already have fetched.
    // A frame must land beyond it or it is not heard.
    int64_t off;
    int64_t head_slot = dma_pos_now(&off) + AP_DMA_AHEAD_FRAMES;
    int64_t read_a = head_slot - off;

    // Signed difference of two 64-bit indices. Never a plain unsigned subtract:
    // that is the shape of the bug that wedged the FPGA four times.
    int64_t ahead = (int64_t)first_idx - read_a;
    if (ahead + (int64_t)nframes <= 0) { s_st.pkt_late++; return false; }
    if (ahead + (int64_t)nframes > AP_FUTURE_GUARD_FRAMES) {
        // A single corrupt timestamp would otherwise scribble the whole ring.
        s_st.pkt_future++;
        return false;
    }

    const uint8_t *src = be24;
    int32_t *seg = NULL; size_t seg_n = 0;       // contiguous run to write back
    for (uint32_t f = 0; f < nframes; f++, src += nslots * 3) {
        int64_t a = (int64_t)first_idx + f;
        if (a < read_a) continue;                // partially late: keep the rest

        uint64_t slot = (uint64_t)(a + off) & AP_RING_MASK;
        int32_t *row = s_buf[(slot >> s_fshift) % s_nbufs] + (slot & (s_F - 1)) * AP_NCH;
        if (!seg || row != seg + seg_n * AP_NCH) { sync_out(seg, seg_n); seg = row; seg_n = 0; }
        seg_n++;

        const uint8_t *p = src;
        for (uint32_t s = 0; s < nslots; s++, p += 3) {
            int8_t ch = slot_to_ch[s];
            if (ch >= 0 && ch < AP_NCH) {
                // AoIP is 24-bit big-endian MSB-justified; shifting it up by 8
                // yields exactly the 32-bit MSB-justified word the I2S slot
                // wants. No scaling step, deliberately.
                row[ch] = (int32_t)(((uint32_t)p[0] << 24) |
                                    ((uint32_t)p[1] << 16) |
                                    ((uint32_t)p[2] << 8));
            }
        }
    }
    sync_out(seg, seg_n);                        // the DMA reads memory, not cache

    uint64_t end = first_idx + nframes;
    portENTER_CRITICAL(&s_mux);
    if ((int64_t)end - (int64_t)s_rx_head > 0) s_rx_head = end;
    s_st.pkt_written++;
    portEXIT_CRITICAL(&s_mux);
    return true;
}

void jb_peek_peaks(uint32_t peaks[AP_NCH])
{
    for (int c = 0; c < AP_NCH; c++) peaks[c] = 0;
    if (!s_ready) return;
    int64_t pos = dma_pos_now(NULL);
    const int32_t *b = s_buf[((uint64_t)pos >> s_fshift) % s_nbufs];
    for (uint32_t f = 0; f < s_F; f++) {
        for (int c = 0; c < AP_NCH; c++) {
            int32_t v = b[f * AP_NCH + c];
            uint32_t a = (uint32_t)(v < 0 ? -(int64_t)v : v);
            if (a > peaks[c]) peaks[c] = a;
        }
    }
}

void jb_get_stats(jb_stats_t *out)
{
    uint64_t play = s_ready ? jb_playout_idx() : 0;
    portENTER_CRITICAL(&s_mux);
    *out = s_st;
    out->rx_head     = s_rx_head;
    portEXIT_CRITICAL(&s_mux);
    out->playout_idx  = play;
    out->level_frames = (int32_t)((int64_t)out->rx_head - (int64_t)play);
}

uint32_t jb_take_isr_max_us(void)
{
    uint32_t v = s_isr_max_us;
    s_isr_max_us = 0;
    return v;
}
