// Resolve a transmit channel to something we can request a flow from.
//
// A subscription names a channel and a device -- "Out 1" on "RedNetA16R" --
// and nothing else. Turning that into an IP, a port, a numeric channel id and
// a legal fpp is an mDNS lookup of
//
//     <channel>@<device>._netaudio-chan._udp.local
//
// whose TXT record carries the rest. The key names below are the ones the FPGA
// transmitter emits (FPGA project firmware/mdns.c build_txt_chan), which were
// themselves matched against a RedNet A16R's.

#ifndef CHAN_RESOLVE_H
#define CHAN_RESOLVE_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

typedef struct {
    uint32_t ip;                // host order
    uint16_t flow_port;         // SRV port -- the flow-control server, 4455
    uint16_t tx_channel_id;     // TXT "id="
    uint16_t dbcp1;             // TXT "dbcp1="
    uint32_t sample_rate;       // TXT "rate="
    uint32_t bits;              // TXT "enc=" / "en="
    uint16_t fpp_min, fpp_max;  // TXT "fpp=<max>,<min>"
    uint16_t nchan;             // channels a flow of this device carries
    uint32_t latency_ns;        // TXT "latency_ns=" -- the transmitter's floor
    bool     multicast;         // any TXT key starting "b." -> already in a bundle
    uint16_t bundle_id;
    uint8_t  bundle_pos;
} tx_channel_t;

esp_err_t chan_resolve(const char *channel, const char *device, tx_channel_t *out);

#endif // CHAN_RESOLVE_H
