// Timestamp-indexed jitter buffer that the I2S DMA PLAYS DIRECTLY.
//
// One ring, shared by every flow, addressed by ABSOLUTE AoIP sample index. All
// flows on a device share one media timeline, so a per-flow ring would only
// re-derive the same index with more bookkeeping. This is the shape inferno
// uses (ring_buffer.rs) and it is the right one.
//
// The ring IS the I2S driver's DMA buffers: N descriptors of F frames in a
// closed loop that the DMA walks forever. A packet is written straight to the
// slot where its samples will be played -- timestamp + latency -- and the
// driver zeroes each buffer once it has been played (auto_clear), so an
// unwritten slot plays silence and old audio can never come round again.
//
// There is no copying task and so no per-block deadline. The only rule left is
// the physical one: a packet must land before the DMA reaches its slot.
//
// Position comes from the TX interrupt: each "descriptor finished" event says
// where the DMA is, and the time since it (esp_timer) interpolates between
// events. That is what the media clock compares against PTP.

#ifndef JITTERBUF_H
#define JITTERBUF_H

#include <stdint.h>
#include <stdbool.h>
#include "app_config.h"

typedef struct {
    uint64_t rx_head;          // highest (timestamp + fpp) written, absolute
    uint64_t playout_idx;      // absolute sample at the DAC now (estimate)
    uint32_t pkt_written;
    uint32_t pkt_late;         // arrived after the DMA had passed -- dropped
    uint32_t pkt_future;       // timestamp beyond the guard -- dropped
    uint32_t pkt_malformed;
    uint32_t frames_underrun;  // played with nothing written (silence)
    uint32_t playout_steps;    // servo gave up and stepped the mapping
    int32_t  level_frames;     // rx_head - playout_idx, last observed
    uint32_t isr_gaps;         // TX interrupts that skipped a descriptor
} jb_stats_t;

void jb_init(void);

// Called by audio_out before the channel starts: the DMA ring is `nbufs`
// buffers of `frames_per_buf` frames. The buffer addresses themselves are
// learned from the interrupt during the first lap (jb_isr_sent).
void jb_attach_dma(uint32_t nbufs, uint32_t frames_per_buf);

// From the I2S TX interrupt, for every finished descriptor. IRAM.
void jb_isr_sent(void *buf);

// True once the first lap has mapped every buffer: writes are dropped before.
bool jb_ready(void);

// Map the timeline so that `playout_idx` is at the DAC now, and clear the
// ring. Called on (re)anchor only.
void jb_reset(uint64_t playout_idx);

// Write one packet's worth of audio.
//
//   first_idx   absolute sample index of the OLDEST frame in the packet
//   nframes     frames in the packet (fpp)
//   nslots      channels interleaved per frame on the wire
//   slot_to_ch  slot -> local channel, or -1 to discard that slot
//   be24        pointer to nframes * nslots * 3 bytes
//
// Returns true if the packet landed, false if it was dropped (late/future).
bool jb_write(uint64_t first_idx, uint32_t nframes,
              uint32_t nslots, const int8_t *slot_to_ch, const uint8_t *be24);

uint64_t jb_playout_idx(void);           // absolute sample at the DAC now
uint64_t jb_rx_head(void);
void     jb_step_playout(int64_t delta_frames);

// Peak |sample| per channel of the buffer the DMA is playing now.
void jb_peek_peaks(uint32_t peaks[AP_NCH]);

void jb_get_stats(jb_stats_t *out);
uint32_t jb_take_isr_max_us(void);       // longest TX interrupt since last call

#endif // JITTERBUF_H
