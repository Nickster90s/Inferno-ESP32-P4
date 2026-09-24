// Device identity, CMC (UDP 8800) and the info / heartbeat multicast (UDP 8700
// -> 224.0.0.231:8702 and 224.0.0.233:8708).
//
// Ported from FPGA project firmware/aoip_info.c and aoip_cmc.c, which
// were brought up against AoIP Controller, a RedNet AM2 and an A16R. Read
// that file's comments before changing any layout here -- nearly every
// constant has a bench session behind it.
//
// WHY A RECEIVER NEEDS THIS AT ALL: AoIP Controller does not poll a device it
// has merely seen in mDNS. Its first unicast is CMC 0x1001; with no answer it
// retries forever and never sends a single ARC request. And it builds its
// device list from the info multicasts, so a device that does not announce
// "shows only its name". Both were measured on this board's first bench run.

#ifndef AOIP_INFO_H
#define AOIP_INFO_H

#include <stdint.h>
#include "esp_err.h"

#define AOIP_PORT_INFO        8702   // device-info multicast, 224.0.0.231
#define AOIP_PORT_HEARTBEAT   8708   // heartbeat multicast,   224.0.0.233

// EUI-64 of the MAC. MUST be byte-identical in the mDNS id= TXT, the CMC reply
// and every info multicast header, or the device appears and then vanishes.
const uint8_t *aoip_device_id(void);

esp_err_t aoip_info_start(void);

// Tell AoIP Controller that these RX channels changed (bit n = channel n+1).
// DC re-reads a channel's subscription status only on this event; without it
// a patch shows "Unresolved" forever even after the flow is up.
void aoip_info_notify_rx_change(uint32_t mask);

#endif // AOIP_INFO_H
