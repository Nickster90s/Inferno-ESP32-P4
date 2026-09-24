#include "aoip_mdns.h"
#include "aoip_wire.h"
#include "app_config.h"
#include "eth_ts.h"
#include "aoip_info.h"
#include "mdns.h"
#include "esp_log.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "aoip_mdns";
static char s_name[32] = AP_DEFAULT_NAME;
static char s_hexid[24];

const char *aoip_mdns_hostname(void) { return s_name; }

static void build_hexid(void)
{
    // "id=" is the factory device id: the EUI-64 of the MAC, exactly as the
    // AM2 advertises (id=001dc1fffea1723c). It MUST equal the id in the CMC
    // reply and the info multicast header -- aoip_info.c owns it -- or DC
    // shows the device and then drops it.
    const uint8_t *d = aoip_device_id();
    snprintf(s_hexid, sizeof(s_hexid), "id=%02x%02x%02x%02x%02x%02x%02x%02x",
             d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7]);
}

static esp_err_t publish(void)
{
    mdns_txt_item_t arc_txt[] = {
        { "arcp_vers",   "2.8.9" },
        { "arcp_min",    "0.2.4" },
        { "router_vers", "4.4.0" },
        { "router_info", "N-Series AoIP" },   // shown as "AoIP Model" in DC
        { "mf",          "N-Series" },
        { "model",       "_00000000000000ff" },
    };

    // channels= : USE THE KNOWN-GOOD CONSTANT. DO NOT INVENT ONE.
    //
    // The FPGA project invented a plausible-looking value and AoIP Controller
    // responded by never sending the device a single packet -- no ARC, no CMC,
    // no subscription command -- across 75 s of capture including a click on
    // the device in Device View. It displayed the device and never spoke to it:
    // blank Subscription Status, 0 bandwidth, grey Latency Status, three
    // symptoms from this one field.
    //
    //     AM2   0x6000004d
    //     A16R  0x6000017f
    //     invented 0x60000130   <- shared NOT ONE bit with either
    //
    // Nobody has decoded the bits. inferno hardcodes the AM2's value with a
    // literal "// ???" rather than compute one. Do the same.
    mdns_txt_item_t cmc_txt[] = {
        { "id",          s_hexid + 3 },
        { "process",     "0" },
        { "cmcp_vers",   "1.2.0" },
        { "cmcp_min",    "1.0.0" },
        { "server_vers", "4.1.0" },
        { "channels",    "0x6000004d" },
        { "mf",          "N-Series" },
        { "model",       "_00000000000000ff" },
    };

    ESP_ERROR_CHECK(mdns_service_add(s_name, MDNS_SVC_ARC, MDNS_PROTO,
                                     AOIP_PORT_ARC, arc_txt,
                                     sizeof(arc_txt) / sizeof(arc_txt[0])));
    ESP_ERROR_CHECK(mdns_service_add(s_name, MDNS_SVC_CMC, MDNS_PROTO,
                                     AOIP_PORT_CMC, cmc_txt,
                                     sizeof(cmc_txt) / sizeof(cmc_txt[0])));
    ESP_LOGI(TAG, "advertised %s: arc:%d cmc:%d", s_name,
             AOIP_PORT_ARC, AOIP_PORT_CMC);
    return ESP_OK;
}

esp_err_t aoip_mdns_start(void)
{
    ESP_ERROR_CHECK(mdns_init());
    build_hexid();
    ESP_ERROR_CHECK(mdns_hostname_set(s_name));
    ESP_ERROR_CHECK(mdns_instance_name_set(s_name));
    return publish();
}

esp_err_t aoip_mdns_set_name(const char *name)
{
    if (!name || !*name) return ESP_ERR_INVALID_ARG;
    strncpy(s_name, name, sizeof(s_name) - 1);
    s_name[sizeof(s_name) - 1] = 0;
    mdns_service_remove_all();
    mdns_hostname_set(s_name);
    mdns_instance_name_set(s_name);
    return publish();
}
