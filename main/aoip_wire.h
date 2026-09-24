// AoIP wire formats, in one place.
//
// Everything here was confirmed byte-for-byte on a bench against a Focusrite
// RedNet A16R / RedNet AM2 / virtual soundcard (DVS) by the FPGA project in
// the FPGA project (../FPGA), or read out of `inferno` (GPL-3, used as a SPECIFICATION
// ONLY -- no code copied). Citations are to those trees.
//
// NOT AFFILIATED WITH AUDINATE. AoIP is Audinate's trademark; the protocol is
// undocumented and unlicensed. Bench and research use only.

#ifndef AOIP_WIRE_H
#define AOIP_WIRE_H

#include <stdint.h>
#include <string.h>

// ---------------------------------------------------------------------------
// Byte helpers -- everything on the AoIP wire is big-endian
// ---------------------------------------------------------------------------

static inline uint16_t dw_rd16(const uint8_t *p) { return (uint16_t)((p[0] << 8) | p[1]); }
static inline uint32_t dw_rd24(const uint8_t *p) { return ((uint32_t)p[0] << 16) | ((uint32_t)p[1] << 8) | p[2]; }
static inline uint32_t dw_rd32(const uint8_t *p) {
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
}
static inline void dw_wr16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)(v >> 8); p[1] = (uint8_t)v; }
static inline void dw_wr32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)(v >> 24); p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >> 8);  p[3] = (uint8_t)v;
}

// ---------------------------------------------------------------------------
// Ports  (FPGA project firmware/aoip_dev.h)
// ---------------------------------------------------------------------------

#define AOIP_PORT_ARC      4440   // _netaudio-arc   control / routing
#define AOIP_PORT_CMC      8800   // _netaudio-cmc   device advertisement
#define AOIP_PORT_FLOWS    4455   // _netaudio-chan  SRV target: flow control
#define AOIP_PORT_MEDIA    4321   // audio
#define AOIP_PORT_INFO_REQ 8700   // info request / heartbeat

// Our own receive port for audio flows. Real receivers use 14336+; an A16R and
// an AM2 both asked our transmitter to send to :14337 (FPGA project UNICAST_FLOWS.md).
#define AOIP_RX_AUDIO_PORT 14336

// ---------------------------------------------------------------------------
// Audio packet  (FPGA project README.md, "Why AoIP-native rather than AES67")
// ---------------------------------------------------------------------------
//
//   [0]      0x02                    constant
//   [1..5]   seconds        u32 BE
//   [5..9]   subsec_samples u32 BE   0 .. sample_rate-1
//   [9..]    interleaved samples, big-endian, MSB-justified
//
//   timestamp = seconds * sample_rate + subsec_samples   (in units of samples)
//
// The timestamp labels the OLDEST sample in the packet.

#define AOIP_AUDIO_MAGIC   0x02
#define AOIP_AUDIO_HDR_LEN 9

typedef struct {
    uint32_t seconds;
    uint32_t subsec_samples;
    const uint8_t *samples;
    uint32_t sample_bytes;
} aoip_audio_hdr_t;

// Returns 1 and fills `h` on a well-formed packet, 0 otherwise.
static inline int aoip_audio_parse(const uint8_t *p, uint32_t len, aoip_audio_hdr_t *h)
{
    if (len < AOIP_AUDIO_HDR_LEN + 3) return 0;
    if (p[0] != AOIP_AUDIO_MAGIC) return 0;
    h->seconds        = dw_rd32(p + 1);
    h->subsec_samples = dw_rd32(p + 5);
    h->samples        = p + AOIP_AUDIO_HDR_LEN;
    h->sample_bytes   = len - AOIP_AUDIO_HDR_LEN;
    return 1;
}

// Absolute sample index on the AoIP timeline. 64-bit deliberately.
//
// THE FPGA RING WRAPPED AFTER 3.107 HOURS AND WEDGED THE AUDIO FOUR TIMES
// (FPGA project README.md, "ROOT CAUSE FOUND 2026-08-14"): a 32-bit pointer
// advancing at 384,000/s, an occupancy computed as a plain difference, and a
// safety clamp that hid the resulting -4,294,967,232 as a harmless 0. At 48 kHz
// a uint64_t wraps in 12 million years, so that failure mode is removed by
// construction here rather than defended against. Keep it that way: every
// sample index in this codebase is uint64_t, and occupancy is a SIGNED
// difference of two of them.
static inline uint64_t aoip_ts_to_samples(uint32_t seconds, uint32_t subsec, uint32_t rate)
{
    return (uint64_t)seconds * rate + subsec;
}

// ---------------------------------------------------------------------------
// ARC / CMC / flow-control request-response envelope
// ---------------------------------------------------------------------------
// (inferno protocol/req_resp.rs -- 10-byte big-endian header)
//
//   u16 start_code
//   u16 total_length   (header + content)
//   u16 seqnum
//   u16 opcode1
//   u16 opcode2        (0 in a request; 1 = OK in a reply, else an error code)
//   u8  content[]

#define DRR_HDR_LEN         10
#define DRR_CODE_OK         1

// Flow-control error codes seen in the wild (inferno flows_control.rs):
#define DRR_ERR_FLOW_EXPIRED    0x0103  // no keepalives -- the flow was dropped
#define DRR_ERR_TOO_MANY_FLOWS  0x0315
#define DRR_ERR_RATE_MISMATCH   0x0301

// start_code for flow control. inferno forces 0x1102 rather than echoing the
// transmitter's advertised dbcp1, on the grounds that it is a version number.
#define DRR_START_FLOWS     0x1102

// Flow-control opcodes
#define DFC_OP_REQUEST      0x0100
#define DFC_OP_STOP         0x0101
#define DFC_OP_UPDATE       0x0102   // doubles as the keepalive

// ARC opcodes (inferno protocol/proto_arc.rs)
#define ARC_OP_CHANNEL_COUNTS       0x1000
#define ARC_OP_GET_DEVICE_NAME      0x1002
#define ARC_OP_GET_DEVICE_NAMES     0x1003
#define ARC_OP_GET_TX_CHANNELS      0x2000
#define ARC_OP_GET_RX_CHANNELS      0x3000
#define ARC_OP_RENAME_RX_CHANNELS   0x3001
#define ARC_OP_SET_SUBSCRIPTIONS    0x3010
#define ARC_OP_SET_LATENCY          0x1101

// Subscription status words, as AoIP Controller reads them
// (inferno proto_arc.rs get_receive_channels::ChannelDescriptor):
#define ARC_SUB_ACTIVE      0x01010009u  // subscribed and flowing
#define ARC_SUB_PENDING     0x00000001u  // remembered / resolving / not found
#define ARC_SUB_NONE        0x00000000u

static inline uint32_t drr_put_header(uint8_t *p, uint16_t start_code, uint16_t seqnum,
                                      uint16_t opcode1, uint16_t opcode2, uint16_t total_len)
{
    dw_wr16(p + 0, start_code);
    dw_wr16(p + 2, total_len);
    dw_wr16(p + 4, seqnum);
    dw_wr16(p + 6, opcode1);
    dw_wr16(p + 8, opcode2);
    return DRR_HDR_LEN;
}

// ---------------------------------------------------------------------------
// PTPv1 (IEEE 1588-2002), AoIP profile
// ---------------------------------------------------------------------------
// Confirmed byte for byte against a RedNet A16R -- FPGA project firmware/ptpv1.c.
//
//   transport   UDP multicast 224.0.1.129, event 319, general 320
//   IP          TOS 0xE0 (DSCP CS7), TTL 1
//   header      40 bytes, big-endian
//     [0..2]    version_ptp = 1
//     [2..4]    version_network = 1
//     [4..20]   subdomain[16] = "_DFLT"
//     [20]      port_type (1 = Event for Sync/DelayReq, else General)
//     [21]      source_communication_technology = 1
//     [22..28]  source_uuid = sender MAC
//     [28..30]  source_port_id
//     [30..32]  sequence_id
//     [32]      control
//     [35]      flags; bit3 = ptp_assist (two-step -> a FollowUp will follow)
//   Sync body   originTimestamp at [40..48] = u32 seconds + u32 nanoseconds
//               (NOT PTPv2's 6+4 layout)

#define PTP1_EVENT_PORT     319
#define PTP1_GENERAL_PORT   320
#define PTP1_GROUP_IP       0xE0000181u  // 224.0.1.129
#define PTP1_TOS            0xE0         // DSCP CS7

#define PTP1_HDR_LEN            40
#define PTP1_SYNC_BODY_LEN      84
#define PTP1_FOLLOWUP_BODY_LEN  12
#define PTP1_DELAY_RESP_BODY_LEN 20

#define PTP1_CTRL_SYNC          0
#define PTP1_CTRL_DELAY_REQ     1
#define PTP1_CTRL_FOLLOWUP      2
#define PTP1_CTRL_DELAY_RESP    3

#define PTP1_PORT_TYPE_EVENT    1
#define PTP1_PORT_TYPE_GENERAL  2

#define PTP1_FLAG_PTP_ASSIST    (1u << 3)

static inline int ptp1_header_ok(const uint8_t *p, uint32_t len)
{
    if (len < PTP1_HDR_LEN) return 0;
    if (dw_rd16(p) != 1) return 0;                     // PTPv1 only
    return p[4] == '_' && p[5] == 'D' && p[6] == 'F' && p[7] == 'L' && p[8] == 'T';
}

static inline uint32_t ptp1_put_header(uint8_t *p, const uint8_t uuid[6], uint16_t port_id,
                                       uint8_t control, uint8_t port_type, uint16_t seq)
{
    memset(p, 0, PTP1_HDR_LEN);
    dw_wr16(p + 0, 1);                   // version_ptp
    dw_wr16(p + 2, 1);                   // version_network
    p[4] = '_'; p[5] = 'D'; p[6] = 'F'; p[7] = 'L'; p[8] = 'T';
    p[20] = port_type;
    p[21] = 1;                           // source_communication_technology
    memcpy(p + 22, uuid, 6);
    dw_wr16(p + 28, port_id);
    dw_wr16(p + 30, seq);
    p[32] = control;
    return PTP1_HDR_LEN;
}

// ---------------------------------------------------------------------------
// mDNS service names  (FPGA project firmware/mdns.c)
// ---------------------------------------------------------------------------

#define MDNS_SVC_ARC    "_netaudio-arc"
#define MDNS_SVC_CMC    "_netaudio-cmc"
#define MDNS_SVC_CHAN   "_netaudio-chan"
#define MDNS_SVC_BUND   "_netaudio-bund"
#define MDNS_PROTO      "_udp"

#endif // AOIP_WIRE_H
