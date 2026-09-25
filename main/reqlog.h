// Request log: the last few control requests worth seeing, kept in RAM and
// fetched over the network (telemetry port 7779, query "L").
//
// For reverse-engineering what a controller sends -- e.g. the sample-rate set
// command -- on a board whose serial console cannot be held open (opening it
// can park this board in reset). Routine polling is not logged; anything with
// a payload, and anything on the info port, is.

#ifndef REQLOG_H
#define REQLOG_H

#include <stdint.h>
#include <stddef.h>

// `svc` names the port ("arc", "info", "cmc"); `ip` is the sender, network order.
void reqlog_add(const char *svc, uint32_t ip, const uint8_t *p, int n);

// Render the log as text into `out`; returns the length.
int reqlog_dump(char *out, size_t cap);

#endif // REQLOG_H
