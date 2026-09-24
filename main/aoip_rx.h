// Audio receive: UDP in, jitter buffer out.
//
// One socket per flow, bound to consecutive ports from AOIP_RX_AUDIO_PORT.
// Real receivers do the same -- a RedNet A16R and an AM2 both asked our
// transmitter to send to :14337 (FPGA project UNICAST_FLOWS.md) -- and it means
// the port identifies the flow with no per-packet lookup.

#ifndef AOIP_RX_H
#define AOIP_RX_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "app_config.h"

typedef struct {
    uint32_t packets;
    uint32_t bad_magic;
    uint32_t short_pkt;
    uint32_t wrong_len;
} aoip_rx_stats_t;

esp_err_t aoip_rx_start(void);

// Bind a flow to a socket. `slot_to_ch` maps each wire slot to a local DAC
// channel, or -1 to discard it.
//
// A flow is an arbitrary slot -> channel map, not a contiguous run, and two
// flows carrying overlapping channels is normal: flows are per-transmitter,
// not per-channel-set (FPGA project UNICAST_FLOWS.md).
esp_err_t aoip_rx_bind_flow(uint8_t idx, uint8_t nslots, const int8_t *slot_to_ch,
                             uint16_t fpp, uint16_t *out_port);
esp_err_t aoip_rx_unbind_flow(uint8_t idx);

// Change a bound flow's slot count and map in place, socket untouched -- the
// local half of a 0x0102 flow update.
esp_err_t aoip_rx_remap_flow(uint8_t idx, uint8_t nslots, const int8_t *slot_to_ch);

void aoip_rx_get_stats(aoip_rx_stats_t *out);

// Peak |sample| per DAC channel since the last call, 24-bit full scale 2^23.
void aoip_rx_take_peaks(uint32_t out[AP_NCH]);
uint32_t aoip_rx_flow_packets(uint8_t idx);

#endif // AOIP_RX_H
