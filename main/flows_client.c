#include "flows_client.h"
#include "rate.h"
#include "aoip_wire.h"
#include "aoip_mdns.h"
#include "lwip/sockets.h"
#include "esp_netif.h"
#include "eth_ts.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "flows_cli";

static uint16_t s_seq = 1;

// Last outcome of each control call, for telemetry: the console is not always
// readable (opening the port can hold this board in reset).
flows_client_diag_t g_flows_diag;

// ---------------------------------------------------------------------------
// Transaction
// ---------------------------------------------------------------------------

static esp_err_t transact(uint32_t ip, uint16_t port, uint16_t opcode1,
                          const uint8_t *body, uint32_t body_len,
                          uint8_t *resp, uint32_t resp_cap, uint32_t *resp_len)
{
    int s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (s < 0) return ESP_FAIL;

    struct timeval tv = { .tv_sec = 3, .tv_usec = 0 };
    setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    struct sockaddr_in dst = {
        .sin_family = AF_INET,
        .sin_addr.s_addr = htonl(ip),
        .sin_port = htons(port),
    };

    uint8_t pkt[512];
    uint32_t total = DRR_HDR_LEN + body_len;
    if (total > sizeof(pkt)) { close(s); return ESP_ERR_INVALID_SIZE; }

    uint16_t seq = s_seq++;
    drr_put_header(pkt, DRR_START_FLOWS, seq, opcode1, 0, (uint16_t)total);
    memcpy(pkt + DRR_HDR_LEN, body, body_len);

    if (sendto(s, pkt, total, 0, (struct sockaddr *)&dst, sizeof(dst)) < 0) {
        close(s);
        g_flows_diag.last_opcode = opcode1;
        g_flows_diag.last_code   = 0xFFFE;      // could not even send
        return ESP_FAIL;
    }

    // Loop rather than take the first datagram: a spurious packet on this
    // socket must not be mistaken for the reply.
    for (int attempt = 0; attempt < 4; attempt++) {
        uint8_t rb[512];
        int n = recv(s, rb, sizeof(rb), 0);
        if (n < (int)DRR_HDR_LEN) break;
        if (dw_rd16(rb + 4) != seq)     continue;
        if (dw_rd16(rb + 6) != opcode1) continue;

        uint16_t op2 = dw_rd16(rb + 8);
        close(s);
        if (op2 != DRR_CODE_OK) {
            g_flows_diag.last_opcode = opcode1;
            g_flows_diag.last_code   = op2;
            g_flows_diag.refused++;
            ESP_LOGW(TAG, "transmitter refused opcode 0x%04x: 0x%04x",
                     opcode1, op2);
            return (op2 == DRR_ERR_FLOW_EXPIRED) ? ESP_ERR_NOT_FOUND : ESP_FAIL;
        }
        uint32_t clen = (uint32_t)n - DRR_HDR_LEN;
        if (resp && resp_len) {
            *resp_len = clen < resp_cap ? clen : resp_cap;
            memcpy(resp, rb + DRR_HDR_LEN, *resp_len);
        }
        return ESP_OK;
    }

    close(s);
    g_flows_diag.last_opcode = opcode1;
    g_flows_diag.last_code   = 0xFFFF;          // no reply
    g_flows_diag.timeouts++;
    return ESP_ERR_TIMEOUT;
}

// ---------------------------------------------------------------------------
// Request body
// ---------------------------------------------------------------------------
//
// Layout, from inferno flows_control.rs request_flow(). The two string offsets
// are absolute within the packet (they include the 10-byte envelope header),
// which is why HEADER_LENGTH appears in strings_offset.

static uint32_t build_request_body(const flow_request_t *r, uint8_t *out, uint32_t cap)
{
    const char *hostname = aoip_mdns_hostname();
    uint32_t nch = r->nslots;

    uint32_t strings_offset = 0x26 + nch * 2 + DRR_HDR_LEN;

    uint8_t strings[128];
    uint32_t sp = 0;

    uint32_t hn = strlen(hostname);
    if (sp + hn + 1 > sizeof(strings)) return 0;
    memcpy(strings + sp, hostname, hn); sp += hn;
    strings[sp++] = 0;

    uint16_t rx_flow_name_offset = (uint16_t)(sp + strings_offset);
    uint32_t fn = strlen(r->rx_flow_name);
    if (sp + fn + 1 > sizeof(strings)) return 0;
    memcpy(strings + sp, r->rx_flow_name, fn); sp += fn;
    strings[sp++] = 0;

    // Pad so the trailing address blob starts 8-byte aligned in the packet.
    while ((sp + strings_offset) % 8 != 0) {
        if (sp >= sizeof(strings)) return 0;
        strings[sp++] = 0;
    }
    uint16_t addr_blob_offset = (uint16_t)(sp + strings_offset);

    esp_netif_ip_info_t ip;
    if (esp_netif_get_ip_info(eth_ts_netif(), &ip) != ESP_OK) return 0;
    uint32_t our_ip = ntohl(ip.ip.addr);

    dw_wr16(strings + sp, 0x0802);        sp += 2;
    dw_wr16(strings + sp, r->rx_port);    sp += 2;
    dw_wr32(strings + sp, our_ip);        sp += 4;

    uint32_t n = 0;
    if (cap < strings_offset - DRR_HDR_LEN + sp) return 0;

    dw_wr16(out + n, (uint16_t)strings_offset);     n += 2;
    dw_wr32(out + n, rate_hz());                    n += 4;
    dw_wr32(out + n, AP_BITS_PER_SAMPLE);           n += 4;
    dw_wr16(out + n, 1);                            n += 2;
    dw_wr16(out + n, (uint16_t)nch);                n += 2;
    dw_wr16(out + n, addr_blob_offset);             n += 2;
    for (uint32_t i = 0; i < nch; i++) {
        dw_wr16(out + n, r->tx_channel_id[i]);      n += 2;
    }
    dw_wr16(out + n, (uint16_t)(0x1c + 2 * nch));   n += 2;
    dw_wr16(out + n, 0x0a00);                       n += 2;
    dw_wr16(out + n, 0x0002);                       n += 2;
    dw_wr16(out + n, r->fpp);                       n += 2;
    dw_wr16(out + n, rx_flow_name_offset);          n += 2;
    memset(out + n, 0, 12);                         n += 12;

    // The offsets above are only correct if the body really is this long.
    // inferno asserts the same thing; a mismatch here produces a flow that the
    // transmitter accepts and then sends to the wrong place.
    if (n != strings_offset - DRR_HDR_LEN) {
        ESP_LOGE(TAG, "body length %u != expected %u -- offsets are wrong",
                 (unsigned)n, (unsigned)(strings_offset - DRR_HDR_LEN));
        return 0;
    }

    memcpy(out + n, strings, sp);
    return n + sp;
}

esp_err_t flows_client_request(const flow_request_t *req, uint8_t handle[FLOW_HANDLE_LEN])
{
    uint8_t body[384];
    uint32_t len = build_request_body(req, body, sizeof(body));
    if (!len) return ESP_ERR_INVALID_SIZE;

    uint8_t resp[64];
    uint32_t rlen = 0;
    esp_err_t err = transact(req->tx_ip, req->tx_flow_port, DFC_OP_REQUEST,
                             body, len, resp, sizeof(resp), &rlen);
    if (err != ESP_OK) return err;
    if (rlen < FLOW_HANDLE_LEN) return ESP_ERR_INVALID_RESPONSE;

    memcpy(handle, resp, FLOW_HANDLE_LEN);
    ESP_LOGI(TAG, "flow up: %u ch, fpp %u -> our port %u, handle %02x%02x%02x%02x%02x%02x",
             req->nslots, req->fpp, req->rx_port,
             handle[0], handle[1], handle[2], handle[3], handle[4], handle[5]);
    return ESP_OK;
}

esp_err_t flows_client_update(const flow_request_t *req,
                              const uint8_t handle[FLOW_HANDLE_LEN])
{
    uint8_t body[8 + AP_MAX_CH_PER_FLOW * 2];
    uint32_t n = 0;
    memcpy(body + n, handle, FLOW_HANDLE_LEN);      n += FLOW_HANDLE_LEN;
    dw_wr16(body + n, req->nslots);                 n += 2;
    for (uint32_t i = 0; i < req->nslots; i++) {
        dw_wr16(body + n, req->tx_channel_id[i]);   n += 2;
    }
    return transact(req->tx_ip, req->tx_flow_port, DFC_OP_UPDATE, body, n,
                    NULL, 0, NULL);
}

esp_err_t flows_client_stop(const flow_request_t *req,
                            const uint8_t handle[FLOW_HANDLE_LEN])
{
    return transact(req->tx_ip, req->tx_flow_port, DFC_OP_STOP,
                    handle, FLOW_HANDLE_LEN, NULL, 0, NULL);
}
