// Subscriptions: the control plane that turns "channel 3 <- Out 5 @ RedNetA16R"
// into audio arriving on a socket.
//
// The sequence, and why each step exists:
//
//   1. AoIP Controller sends ARC 0x3010 naming a tx channel and a tx device.
//   2. mDNS resolves <channel>@<device>._netaudio-chan._udp -> IP, SRV port,
//      numeric channel id, legal fpp range (chan_resolve.c).
//   3. Channels wanting the same transmitter are grouped into ONE flow, up to
//      8 slots, because a flow is per-transmitter and not per-channel-set.
//   4. flows_client asks that transmitter's server on 4455 to send to us.
//   5. We keep asking. A flow that is not refreshed dies with 0x0103.
//
// Step 5 is not optional and is easy to leave out: the request succeeds, audio
// flows, and it stops a few seconds later with every counter looking healthy.

#ifndef SUBSCRIBER_H
#define SUBSCRIBER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "app_config.h"

typedef struct {
    char     tx_channel[32];
    char     tx_device[32];
    char     friendly[32];      // our own name for this DAC output
    uint32_t status;            // ARC_SUB_*
} rx_channel_t;

esp_err_t subscriber_start(void);

// Called by the ARC server. An empty tx_channel clears the subscription.
esp_err_t subscriber_set(uint8_t local_ch, const char *tx_channel, const char *tx_device);
esp_err_t subscriber_rename(uint8_t local_ch, const char *friendly);

const rx_channel_t *subscriber_channel(uint8_t local_ch);
void subscriber_force_refresh(void);

typedef struct {
    uint8_t  flows_active;
    uint32_t requests_ok;
    uint32_t requests_failed;
    uint32_t keepalives_ok;
    uint32_t keepalives_lost;   // transmitter had forgotten the flow
    uint32_t resolve_failed;
    uint32_t fallbacks;         // replacement refused -> break-before-make used
} subscriber_stats_t;

void subscriber_get_stats(subscriber_stats_t *out);

// Active receive flows, for the controller's flow list (ARC 0x3200) -- which
// is what its Latency tab lists as "Receive Connections".
typedef struct {
    uint8_t  slot;                               // flow slot = aoip_rx flow index
    uint16_t rx_port;                            // our socket for this flow
    uint8_t  nslots;
    int8_t   slot_to_ch[AP_MAX_CH_PER_FLOW];     // flow slot -> local channel, -1 none
} subscriber_flow_t;

int subscriber_get_flows(subscriber_flow_t *out, int max);

#endif // SUBSCRIBER_H
