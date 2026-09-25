// Ethernet bring-up plus the IEEE-1588 timestamp shim.
//
// EVERY ESP-IDF call whose exact spelling is version- or target-dependent lives
// in eth_ts.c and nowhere else. The PTP code above it is written against this
// header, so when the IDF API moves -- and the timestamping API has moved
// between versions -- exactly one file needs editing.
//
// Two things here are load-bearing for AoIP specifically:
//
// 1. TIMESTAMP ALL RECEIVED FRAMES, not just PTPv2-classified ones. AoIP
//    speaks PTPv1 (IEEE 1588-2002) over UDP 319/320, which a v2 packet
//    classifier does not recognise. The EMAC can be told to stamp everything;
//    that is the mode this uses.
//
// 2. A RAW TX PATH. Delay_Req needs a hardware TX timestamp of the frame we
//    just sent, which means building Ethernet/IP/UDP ourselves and handing the
//    whole frame to the MAC rather than to lwIP. It also lets us set TOS 0xE0
//    and TTL 1 exactly as a RedNet does.

#ifndef ETH_TS_H
#define ETH_TS_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "esp_netif.h"

typedef struct {
    uint32_t seconds;
    uint32_t nanoseconds;
} eth_ts_time_t;

// A PTP frame lifted off the RX path with its hardware timestamp.
typedef struct {
    uint8_t       payload[128];   // UDP payload: PTPv1 header + body
    uint32_t      len;
    uint16_t      dst_port;
    eth_ts_time_t ts;
    bool          ts_valid;
} eth_ts_ptp_frame_t;

// Called from the RX path for every PTPv1 frame. Keep it short -- it runs on
// the Ethernet receive task, which is also carrying the audio.
typedef void (*eth_ts_ptp_cb_t)(const eth_ts_ptp_frame_t *f);

esp_err_t eth_ts_init(void);
esp_err_t eth_ts_start(void);

// Receive watchdog counters (see eth_ts.c).
uint32_t eth_ts_rx_kicks(void);
uint32_t eth_ts_rx_restarts(void);
uint32_t eth_ts_fifo_hang_reboots(void);   // RX FIFO hang -> reboot, across reboots
uint32_t eth_ts_phy_resets(void);
// RX DMA state now, and cumulative missed / FIFO-overflow frame counts.
void eth_ts_mac_dump(void);
void eth_ts_rx_diag(uint32_t *dma_state_now, uint32_t *missed, uint32_t *fifo_ovf);
bool      eth_ts_link_up(void);
const uint8_t *eth_ts_mac(void);
esp_netif_t   *eth_ts_netif(void);

void eth_ts_register_ptp_cb(eth_ts_ptp_cb_t cb);

// --- PTP hardware clock ----------------------------------------------------

esp_err_t eth_ts_get_time(eth_ts_time_t *t);
esp_err_t eth_ts_set_time(const eth_ts_time_t *t);

// Coarse step and fine rate. The servo in ptp_servo.c drives these.
esp_err_t eth_ts_step_time(int64_t delta_ns);
esp_err_t eth_ts_set_rate_ppb(int32_t ppb);
int32_t   eth_ts_get_rate_ppb(void);

// --- Raw transmit ----------------------------------------------------------

// Build and send a UDP datagram to an IPv4 multicast group, returning the
// hardware TX timestamp. Used only for PTPv1 Delay_Req.
esp_err_t eth_ts_send_udp_mcast(uint32_t dst_ip, uint16_t dst_port, uint16_t src_port,
                                const uint8_t *payload, uint32_t len, uint8_t tos,
                                uint8_t ttl, eth_ts_time_t *tx_ts);

// --- Receive filtering -----------------------------------------------------

// Add a multicast group to the MAC's hardware filter.
//
// This is the ESP32-P4's built-in equivalent of the FPGA's rx_gate, and it
// exists for the same measured reason: RX_GATE.md found 21% of control-plane
// round-trips lost to a flooded segment, and the frame loss it caused was the
// SOURCE of the PTP offset excursions. On a 100 Mbit/s port there is less
// headroom, not more.
//
// KEEP THE INVARIANT FROM RX_GATE.md: whatever the software filter accepts,
// the hardware filter must also accept -- in particular ALL of 224.0.0.0/24,
// because dropping IGMP queries does not break anything immediately. The
// memberships just age out and the switch quietly stops forwarding our groups.
esp_err_t eth_ts_mcast_allow(uint32_t group_ip);

#endif // ETH_TS_H
