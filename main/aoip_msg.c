// AoIP request/response framing. Ported unchanged from the FPGA project (../FPGA)/firmware. See aoip_msg.h.

#include "aoip_msg.h"
#include <string.h>

void aoip_msg_begin(aoip_msg_t *m, uint8_t *buf, const uint8_t *req)
{
    m->buf = buf;
    m->len = AOIP_HDR_LEN;
    // Echo start_code, seqnum and opcode1 straight back. start_code is a
    // protocol version -- real devices advertise dbcp1=0x1200 while inferno
    // forces 0x1102, so echoing rather than forcing keeps us compatible with
    // whatever the peer speaks.
    buf[0] = req[0]; buf[1] = req[1];        // start_code
    buf[2] = 0;      buf[3] = 0;             // total_length, patched at finish
    buf[4] = req[4]; buf[5] = req[5];        // seqnum
    buf[6] = req[6]; buf[7] = req[7];        // opcode1
    buf[8] = 0;      buf[9] = 0;             // opcode2, set at finish
}

void aoip_msg_u8(aoip_msg_t *m, uint8_t v)   { m->buf[m->len++] = v; }

void aoip_msg_u16(aoip_msg_t *m, uint16_t v)
{
    m->buf[m->len++] = (uint8_t)(v >> 8);
    m->buf[m->len++] = (uint8_t)v;
}

void aoip_msg_u32(aoip_msg_t *m, uint32_t v)
{
    m->buf[m->len++] = (uint8_t)(v >> 24);
    m->buf[m->len++] = (uint8_t)(v >> 16);
    m->buf[m->len++] = (uint8_t)(v >> 8);
    m->buf[m->len++] = (uint8_t)v;
}

void aoip_msg_bytes(aoip_msg_t *m, const void *p, uint32_t n)
{
    memcpy(m->buf + m->len, p, n);
    m->len += n;
}

void aoip_msg_zeros(aoip_msg_t *m, uint32_t n)
{
    memset(m->buf + m->len, 0, n);
    m->len += n;
}

uint16_t aoip_msg_str(aoip_msg_t *m, const char *s)
{
    uint16_t off = (uint16_t)m->len;          // absolute, includes the header
    while (*s) m->buf[m->len++] = (uint8_t)*s++;
    m->buf[m->len++] = 0;
    return off;
}

void aoip_msg_patch_u16(aoip_msg_t *m, uint32_t at, uint16_t v)
{
    m->buf[at]     = (uint8_t)(v >> 8);
    m->buf[at + 1] = (uint8_t)v;
}

uint32_t aoip_msg_finish(aoip_msg_t *m, uint16_t opcode2)
{
    aoip_msg_patch_u16(m, 2, (uint16_t)m->len);
    aoip_msg_patch_u16(m, 8, opcode2);
    return m->len;
}
