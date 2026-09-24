// ARC server, UDP 4440 -- the control plane AoIP Controller talks to.
//
// *** THIS IS THE FILE THAT WILL NEED BENCH ITERATION. ***
//
// The rest of this project is either arithmetic or a protocol captured
// byte-for-byte. The ARC reply layouts are neither: they are reverse-engineered
// structures whose unknown fields matter. The FPGA project's own experience is
// the warning -- one invented constant in a TXT record caused AoIP Controller
// to display the device and then never send it a single packet, and three
// separate symptoms all traced to that one field.
//
// So: capture what a real receiver answers, diff it against what this answers,
// and fix the difference. Do not reason about what the fields "should" be.
// tools/arc_probe.py is there for exactly this.

#ifndef AOIP_ARC_H
#define AOIP_ARC_H

#include "esp_err.h"

esp_err_t aoip_arc_start(void);

#endif // AOIP_ARC_H
