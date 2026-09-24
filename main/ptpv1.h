// PTPv1 (IEEE 1588-2002) slave, AoIP profile.
//
// SLAVE ONLY. We never transmit Sync, so we never participate in BMCA and can
// never be elected Leader -- correct for a receiver, and also why AoIP
// Controller should not offer this device as a Preferred Leader.
//
// This is what turns AoIP Controller's Sync indicator green.

#ifndef PTPV1_H
#define PTPV1_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

typedef struct {
    bool     locked;
    bool     have_master;
    uint8_t  master_uuid[6];
    uint16_t master_port_id;

    int64_t  offset_ns;
    int64_t  mean_path_delay_ns;

    uint32_t rx_sync, rx_followup, rx_delay_resp, rx_other;

    // PAIRING ANOMALIES. Every one of these used to be discarded in silence on
    // the FPGA, and they are the documented source of the +-us offset
    // excursions: a lost FollowUp or DelayResp mispairs with the wrong Sync and
    // injects microseconds. Counting them is what made the fault findable.
    uint32_t lost_followup;
    uint32_t orphan_followup;
    uint32_t mispair;
    uint32_t no_hw_ts;          // a PTP frame arrived without a hardware stamp

    uint32_t tx_delay_req;
    uint32_t servo_updates;
    int32_t  rate_ppb;          // servo integral only -- see ptp_servo.h

    // Any step moves absolute time out from under anything anchored to it.
    // media_clock.c watches this and re-anchors when it moves.
    uint32_t step_count;

    uint32_t path_delay_rejected;   // (ms + sm)/2 outside 0..10 ms
    uint32_t phase_shifts;          // small phase steps taken while locked

    int32_t  out_ppb;               // last P+I command to the PTP clock
    uint32_t rate_adj_failed;       // eth_ts_set_rate_ppb() != ESP_OK
    int32_t  rate_adj_last_err;
} ptpv1_state_t;

extern ptpv1_state_t g_ptpv1;

esp_err_t ptpv1_start(void);

#endif // PTPV1_H
