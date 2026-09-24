// Unicast flow-control CLIENT, port 4455.
//
// The FPGA project built the SERVER side of this and captured, off its own
// wire, exactly what two real receivers ask for (FPGA project UNICAST_FLOWS.md):
//
//   field            RedNet A16R        RedNet AM2
//   sample_rate      48000              48000
//   bits_per_sample  24                 24
//   num_channels     4                  2
//   channel slots    [1, 2, 0, 0]       [1, 2]
//   fpp              8                  16
//   destination      169.254.60.249:14337
//
// and that they repeat it every ~5 s as a KEEPALIVE, not a retry: a flow that
// is not refreshed dies with opcode2 = 0x0103, "stream expired". So this is the
// other half of a protocol this project already understands from the far side.
//
// Body layout is from inferno protocol/flows_control.rs, used as a
// specification. No code copied.

#ifndef FLOWS_CLIENT_H
#define FLOWS_CLIENT_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "app_config.h"

#define FLOW_HANDLE_LEN 6

typedef struct {
    uint16_t last_opcode;       // 0x0100 request, 0x0101 stop, 0x0102 update
    uint16_t last_code;         // transmitter's opcode2 on refusal; 0xFFFF = no reply
    uint32_t refused;
    uint32_t timeouts;
} flows_client_diag_t;
extern flows_client_diag_t g_flows_diag;

typedef struct {
    uint32_t tx_ip;                              // transmitter, host order
    uint16_t tx_flow_port;                       // from the channel's SRV (4455)
    uint16_t tx_channel_id[AP_MAX_CH_PER_FLOW];  // 0 = unused slot
    uint8_t  nslots;
    uint16_t fpp;
    uint16_t rx_port;                            // our aoip_rx socket
    char     rx_flow_name[32];
} flow_request_t;

// Ask the transmitter to start sending. Fills `handle` on success.
esp_err_t flows_client_request(const flow_request_t *req, uint8_t handle[FLOW_HANDLE_LEN]);

// Change an existing flow's channel list in place (opcode 0x0102, inferno
// flows_control.rs update_flow): what a patch on a transmitter we already
// receive from should send, instead of tearing the flow down. NOT the
// keepalive -- that is 0x13 0x37 on the audio socket (aoip_rx.c). Returns
// ESP_ERR_NOT_FOUND if the transmitter has forgotten the flow (0x0103).
esp_err_t flows_client_update(const flow_request_t *req,
                              const uint8_t handle[FLOW_HANDLE_LEN]);

esp_err_t flows_client_stop(const flow_request_t *req,
                            const uint8_t handle[FLOW_HANDLE_LEN]);

#endif // FLOWS_CLIENT_H
