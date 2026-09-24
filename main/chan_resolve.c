#include "chan_resolve.h"
#include "aoip_wire.h"
#include "mdns.h"
#include "lwip/sockets.h"
#include "esp_log.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

static const char *TAG = "chan_res";

static uint32_t txt_u32(const mdns_result_t *r, const char *key, uint32_t dflt)
{
    for (size_t i = 0; i < r->txt_count; i++) {
        if (r->txt[i].key && strcmp(r->txt[i].key, key) == 0 && r->txt[i].value) {
            return (uint32_t)strtoul(r->txt[i].value, NULL, 0);
        }
    }
    return dflt;
}

static void parse_fpp(const mdns_result_t *r, tx_channel_t *out)
{
    // "fpp=<MAX>,<MIN>". An A16R -- a transmitter that serves an AM2 at 1 ms
    // and DVS at 4 ms simultaneously while itself set to 0.25 ms -- advertises
    // "fpp=4,2". The maximum is a real constraint, not a hint: ask for more
    // than it and the flow either fails or drags the device's latency up.
    out->fpp_max = 16;
    out->fpp_min = 2;
    for (size_t i = 0; i < r->txt_count; i++) {
        if (!r->txt[i].key || strcmp(r->txt[i].key, "fpp") != 0 || !r->txt[i].value) continue;
        unsigned mx = 0, mn = 0;
        if (sscanf(r->txt[i].value, "%u,%u", &mx, &mn) == 2) {
            out->fpp_max = (uint16_t)mx;
            out->fpp_min = (uint16_t)mn;
        }
        return;
    }
}

static void parse_bundle(const mdns_result_t *r, tx_channel_t *out)
{
    // A key of the form "b.<flowid>=<position>" means this channel is already
    // carried by a multicast bundle. The FPGA project learned the hard way that
    // this key must appear ONLY when a real group exists: advertising it
    // unconditionally forced every subscription to multicast and put 65.5 of
    // 69.6 Mbit/s of unwanted audio on the segment. We read it the same way --
    // its presence, not its value, decides the path.
    for (size_t i = 0; i < r->txt_count; i++) {
        const char *k = r->txt[i].key;
        if (k && k[0] == 'b' && k[1] == '.') {
            out->multicast  = true;
            out->bundle_id  = (uint16_t)strtoul(k + 2, NULL, 10);
            out->bundle_pos = r->txt[i].value ? (uint8_t)strtoul(r->txt[i].value, NULL, 10) : 0;
            return;
        }
    }
}

esp_err_t chan_resolve(const char *channel, const char *device, tx_channel_t *out)
{
    char instance[96];
    snprintf(instance, sizeof(instance), "%s@%s", channel, device);

    mdns_result_t *results = NULL;
    esp_err_t err = mdns_query(instance, MDNS_SVC_CHAN, MDNS_PROTO,
                               MDNS_TYPE_ANY, 3000, 4, &results);
    if (err != ESP_OK) return err;
    if (!results) {
        ESP_LOGW(TAG, "no mDNS answer for %s", instance);
        return ESP_ERR_NOT_FOUND;
    }

    memset(out, 0, sizeof(*out));
    esp_err_t rc = ESP_ERR_NOT_FOUND;

    for (mdns_result_t *r = results; r; r = r->next) {
        // The address is a LIST, and DVS advertises IPv6 alongside IPv4: the
        // first entry being v6 used to discard the whole answer. Walk it. If
        // the answer carried no A record at all, ask for the host's A record.
        uint32_t ip_be = 0;
        for (mdns_ip_addr_t *a = r->addr; a; a = a->next) {
            if (a->addr.type == ESP_IPADDR_TYPE_V4) { ip_be = a->addr.u_addr.ip4.addr; break; }
        }
        if (!ip_be && r->hostname) {
            esp_ip4_addr_t a4;
            if (mdns_query_a(r->hostname, 2000, &a4) == ESP_OK) ip_be = a4.addr;
        }
        if (!ip_be) {
            ESP_LOGW(TAG, "%s: answer has no IPv4 address (host %s)", instance,
                     r->hostname ? r->hostname : "?");
            continue;
        }

        out->ip        = ntohl(ip_be);
        out->flow_port = r->port ? r->port : AOIP_PORT_FLOWS;
        out->tx_channel_id = (uint16_t)txt_u32(r, "id", 0);
        out->dbcp1     = (uint16_t)txt_u32(r, "dbcp1", DRR_START_FLOWS);
        out->sample_rate = txt_u32(r, "rate", 48000);
        out->bits      = txt_u32(r, "enc", txt_u32(r, "en", 24));
        out->nchan     = (uint16_t)txt_u32(r, "nchan", 8);
        out->latency_ns = txt_u32(r, "latency_ns", 0);
        parse_fpp(r, out);
        parse_bundle(r, out);
        rc = ESP_OK;
        break;
    }

    mdns_query_results_free(results);

    if (rc == ESP_OK) {
        ESP_LOGI(TAG, "%s -> %u.%u.%u.%u:%u id=%u rate=%u bits=%u fpp=%u..%u "
                      "lat=%u ns%s",
                 instance,
                 (unsigned)(out->ip >> 24), (unsigned)((out->ip >> 16) & 0xFF),
                 (unsigned)((out->ip >> 8) & 0xFF), (unsigned)(out->ip & 0xFF),
                 out->flow_port, out->tx_channel_id, (unsigned)out->sample_rate,
                 (unsigned)out->bits, out->fpp_min, out->fpp_max,
                 (unsigned)out->latency_ns, out->multicast ? " [in a bundle]" : "");
    }
    return rc;
}
