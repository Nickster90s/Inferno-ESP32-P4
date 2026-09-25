// I2S TDM8 -> PCM1690, and the task that feeds it.
//
// This task is the closest thing this design has to the FPGA's hardware
// packetiser: it is the only thing between the jitter buffer and the DAC, it
// runs pinned to core 0 at the highest priority in the system, and its loop is
// paced by the I2S DMA rather than by a timer. If it is ever late, the DAC
// emits whatever the DMA still holds and the audio is audibly wrong.

#ifndef AUDIO_OUT_H
#define AUDIO_OUT_H

#include <stdint.h>
#include "esp_err.h"

esp_err_t audio_out_init(void);
esp_err_t audio_out_start(void);

// Frames of DMA ahead of the converter. A constant, folded into the media
// clock's phase target.
uint32_t audio_out_dma_depth_frames(void);

uint32_t audio_out_blocks(void);

// Peak |sample| per channel since the last call, 32-bit full scale 2^31.
void audio_out_take_peaks(uint32_t *out);          // AP_NCH entries

#endif // AUDIO_OUT_H
