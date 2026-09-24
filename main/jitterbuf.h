// Timestamp-indexed jitter buffer: the whole receive data plane.
//
// One ring, shared by every flow, addressed by ABSOLUTE AoIP sample index. All
// flows on a device share one media timeline, so a per-flow ring would only
// re-derive the same index with more bookkeeping. This is the shape inferno
// uses (ring_buffer.rs) and it is the right one.
//
// Single writer (the aoip_rx task), single reader (the audio_out task). The
// only shared mutable state is two 64-bit counters, guarded by a spinlock
// because a 32-bit core tears a 64-bit read.

#ifndef JITTERBUF_H
#define JITTERBUF_H

#include <stdint.h>
#include <stdbool.h>
#include "app_config.h"

typedef struct {
    uint64_t rx_head;          // highest (timestamp + fpp) written, absolute
    uint64_t playout_idx;      // next frame the reader will hand to the DAC
    uint32_t pkt_written;
    uint32_t pkt_late;         // arrived after playout had passed -- dropped
    uint32_t pkt_future;       // timestamp beyond the guard -- dropped
    uint32_t pkt_malformed;
    uint32_t frames_underrun;  // read with nothing written -> silence emitted
    uint32_t playout_steps;    // servo gave up and stepped the pointer
    int32_t  level_frames;     // rx_head - playout_idx, last observed
} jb_stats_t;

void jb_init(void);

// Reset the ring and place the playout pointer. Called on (re)anchor only.
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

// Hand `nframes` interleaved AP_NCH frames to the caller and advance the
// playout pointer. Frames nobody wrote come out as silence and are counted.
// The region read is zeroed, so a packet that arrives late cannot be heard
// a ring-length later.
void jb_read(int32_t *dst, uint32_t nframes);

uint64_t jb_playout_idx(void);
uint64_t jb_rx_head(void);
void     jb_step_playout(int64_t delta_frames);

void jb_get_stats(jb_stats_t *out);

#endif // JITTERBUF_H
