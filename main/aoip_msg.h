// AoIP request/response framing. Ported unchanged from the FPGA project (../FPGA)/firmware.
//
// The control plane (ARC 4440, CMC 8800, flow control 4455) shares one framing:
// a 10-byte big-endian header, then content.
//
//   0  2  start_code    protocol version; echo the peer's
//   2  2  total_length  header + content
//   4  2  seqnum        echo the peer's
//   6  2  opcode1       the request; echo it
//   8  2  opcode2       0 in a request; 1 = OK in a response, else an error
//
// THE TRAP: string fields inside responses are referenced by offsets that are
// ABSOLUTE FROM THE START OF THE PACKET -- i.e. they include this 10-byte
// header (inferno arc_server.rs:96). Getting that wrong yields a device that
// AoIP Controller shows with blank names, which looks like a rendering
// glitch rather than an encoding bug.

#ifndef AOIP_MSG_H
#define AOIP_MSG_H

#include <stdint.h>

#define AOIP_HDR_LEN     10
#define AOIP_CODE_OK     1
#define AOIP_CODE_MORE   0x8112     // paginated: more items remain

// A response under construction. `buf` points at the UDP payload, so content
// starts at buf + AOIP_HDR_LEN and every offset we write is relative to buf --
// which is exactly the absolute-including-header convention.
typedef struct {
    uint8_t *buf;
    uint32_t len;        // bytes used so far, including the header
} aoip_msg_t;

// Begin a response echoing the request's start_code/seqnum/opcode1.
void aoip_msg_begin(aoip_msg_t *m, uint8_t *buf, const uint8_t *req);

// Append helpers. All big-endian.
void aoip_msg_u8 (aoip_msg_t *m, uint8_t v);
void aoip_msg_u16(aoip_msg_t *m, uint16_t v);
void aoip_msg_u32(aoip_msg_t *m, uint32_t v);
void aoip_msg_bytes(aoip_msg_t *m, const void *p, uint32_t n);
void aoip_msg_zeros(aoip_msg_t *m, uint32_t n);

// Append a NUL-terminated string, returning the ABSOLUTE offset to write into
// whatever descriptor references it.
uint16_t aoip_msg_str(aoip_msg_t *m, const char *s);

// Patch a u16 already written at absolute offset `at`.
void aoip_msg_patch_u16(aoip_msg_t *m, uint32_t at, uint16_t v);

// Finish: set total_length and opcode2. Returns the total length.
uint32_t aoip_msg_finish(aoip_msg_t *m, uint16_t opcode2);

// Request accessors.
static inline uint16_t aoip_req_u16(const uint8_t *p, uint32_t off) {
    return (uint16_t)((p[off] << 8) | p[off + 1]);
}
#define aoip_req_start_code(p)  aoip_req_u16(p, 0)
#define aoip_req_length(p)      aoip_req_u16(p, 2)
#define aoip_req_seqnum(p)      aoip_req_u16(p, 4)
#define aoip_req_opcode1(p)     aoip_req_u16(p, 6)
#define aoip_req_opcode2(p)     aoip_req_u16(p, 8)

#endif // AOIP_MSG_H
