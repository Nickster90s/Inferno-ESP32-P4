// Advertise this device so AoIP Controller sees it.
//
// A RECEIVE-ONLY device advertises _netaudio-arc and _netaudio-cmc and nothing
// else. The _netaudio-chan and _netaudio-bund records exist to let OTHER
// devices subscribe to our transmit channels, and we have none -- the PCM1690
// is a DAC. That removes the entire record set the FPGA project spent most of
// its mDNS effort on.

#ifndef AOIP_MDNS_H
#define AOIP_MDNS_H

#include "esp_err.h"

esp_err_t aoip_mdns_start(void);
const char *aoip_mdns_hostname(void);
esp_err_t aoip_mdns_set_name(const char *name);

#endif // AOIP_MDNS_H
