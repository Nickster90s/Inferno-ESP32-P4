// AoIP -> 8-channel PCM1690 DAC on a Waveshare ESP32-P4-ETH.
//
// Bring-up order matters and is not arbitrary:
//
//   1. Ethernet, because everything else needs a MAC address and a link.
//   2. I2S, because the APLL must already be running at the media rate before
//      mclk_hw can read back the coefficients it is going to trim, and before
//      the PCM1690 will accept a control write.
//   3. PTP, because the media clock has no timeline without it.
//   4. The jitter buffer and audio task, which idle on silence until anchored.
//   5. The control plane last, because a subscription that arrives before
//      there is a clock has nowhere to put its audio.
//
// See README.md for what is verified, what is unverified, and the bring-up
// order to follow on the bench -- which is NOT the same as this list.

#include "app_config.h"
#include "rate.h"
#include "eth_ts.h"
#include "ptpv1.h"
#include "media_clock.h"
#include "audio_out.h"
#include "pcm1690.h"
#include "jitterbuf.h"
#include "aoip_rx.h"
#include "aoip_arc.h"
#include "aoip_mdns.h"
#include "aoip_info.h"
#include "subscriber.h"
#include "telem.h"
#include "console.h"
#include "aoip_wire.h"

#include "nvs_flash.h"
#include "lwip/sockets.h"
#include "lwip/igmp.h"
#include "lwip/netif.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "main";

static void wait_for_ip(void)
{
    for (int i = 0; i < 300; i++) {          // 30 s, then carry on regardless
        esp_netif_ip_info_t ip;
        if (eth_ts_link_up() &&
            esp_netif_get_ip_info(eth_ts_netif(), &ip) == ESP_OK && ip.ip.addr) {
            ESP_LOGI(TAG, "address " IPSTR, IP2STR(&ip.ip));
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    ESP_LOGW(TAG, "no address yet -- continuing; link-local should appear");
}

// IGMP KEEPER. A link loss (a switch reboot, a cable, our own EMAC restart)
// makes esp_netif take the interface down, and lwIP's igmp_stop() then FREES
// every group membership; nothing joins them again when the link returns. The
// sockets still think they are members, so no error shows -- but the switch
// hears no reports, stops forwarding the PTP group, and the media clock runs
// free (bench, 2026-10-10: Luminex firmware update, 0 Syncs afterwards, DAC8
// missing from Araneo's IGMP tab). This puts back any group lwIP lost.
static const uint32_t s_groups[] = {
    PTP1_GROUP_IP,   // 224.0.1.129 PTP primary
    0xE00000E7,      // 224.0.0.231 AoIP info
    0xE00000E9,      // 224.0.0.233 AoIP heartbeat
    0xEFFFFFFF,      // 239.255.255.255 (what Araneo files "Dante" under)
};

static esp_err_t igmp_rejoin_cb(void *ctx)
{
    int *rejoined = ctx;
    struct netif *n = netif_default;
    if (!n || !netif_is_up(n) || !netif_is_link_up(n) ||
        ip4_addr_isany_val(*netif_ip4_addr(n))) return ESP_OK;
    for (size_t i = 0; i < sizeof(s_groups) / sizeof(s_groups[0]); i++) {
        ip4_addr_t a = { .addr = htonl(s_groups[i]) };
        if (!igmp_lookfor_group(n, &a) && igmp_joingroup_netif(n, &a) == ERR_OK)
            (*rejoined)++;
    }
    return ESP_OK;
}

static void igmp_keeper_task(void *arg)
{
    (void)arg;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        int rejoined = 0;
        esp_netif_tcpip_exec(igmp_rejoin_cb, &rejoined);
        if (rejoined) ESP_LOGW(TAG, "IGMP: re-joined %d group(s) lost with the link", rejoined);
    }
}

void app_main(void)
{
    // FIRST, before any clock or peripheral is configured: which rate are
    // we? Everything downstream derives from it. NVS comes first only
    // because a rate chosen in the controller is stored there; the rate pin
    // is still read while it is just a pin.
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(rate_select());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // 1. Ethernet
    ESP_ERROR_CHECK(eth_ts_init());
    ESP_ERROR_CHECK(eth_ts_start());

    // Hardware multicast filter. The FPGA needed a gateware equivalent of this
    // and measured 21% of control-plane round-trips lost without it; the EMAC
    // gives it to us for free. The 224.0.0.0/24 entries are not optional --
    // dropping IGMP queries breaks nothing immediately and then the switch
    // quietly stops forwarding our groups.
    //
    // The EMAC has 8 filter slots and every add takes one, duplicate or not;
    // a 9th fails (logged, harmless for a group nobody reads). Here only what
    // is needed BEFORE there is an address: 224.0.0.1 and 224.0.0.251. mDNS
    // must stay: until the PTP join (which needs an address) it is the only
    // traffic a snooping switch sends us, and without it the RX watchdog took
    // the silence for a dead RX and restarted the EMAC over and over -- the
    // board never got an address (bench, 2026-10-10). The PTP group is added
    // by ptpv1_start(); lwIP's joins add the rest: 224.0.0.1 (its own), PTP,
    // mDNS, and aoip_info's 224.0.0.231, .233 and 239.255.255.255 -- the
    // last one is the 9th and gets no slot, which it does not need.
    eth_ts_mcast_allow(0xE0000001);   // 224.0.0.1   all-hosts / IGMP
    eth_ts_mcast_allow(0xE00000FB);   // 224.0.0.251 mDNS

    // NO IP WAIT HERE. Nothing until the control plane needs an address:
    // PTP Sync reception, stepping and the frequency estimate run on link
    // alone, and link-local addressing takes ~10 s (RFC 3927 probing) that
    // used to sit in front of PTP -- a fifth of the time to Sync green.

    // 2. Audio clock and DAC
    ESP_ERROR_CHECK(pcm1690_init());
    ESP_ERROR_CHECK(audio_out_init());

    // 3. Media clock -- reads back the APLL the I2S driver just programmed
    jb_init();
    ESP_ERROR_CHECK(mclk_init(audio_out_dma_depth_frames()));

    // 4. PTP, then audio out. The audio task emits silence until anchored.
    ESP_ERROR_CHECK(telem_start());
    ESP_ERROR_CHECK(ptpv1_start());
    ESP_ERROR_CHECK(audio_out_start());
    ESP_ERROR_CHECK(aoip_rx_start());

    // Discipline ARMED. It started disarmed for bring-up (as rx_gate did on
    // the FPGA), but feed-forward alone lets the phase drift: +2 ms in 3.7 h
    // on the bench, so the configured latency was not the real one. It only
    // acts once PTP is locked; console 'a 0' disarms it.
    mclk_arm(true);

    // 5. Control plane -- the only part that needs an address.
    wait_for_ip();

    // IGMP JOIN for the PTP group. The MAC filter above lets Sync in before
    // there is an address, but a switch with IGMP snooping forwards a group
    // only to ports that have JOINED it -- and nothing here ever had. It
    // worked while the clock leader shared an unmanaged switch with us; when a
    // leader elsewhere took over, not one Sync arrived (bench: 0 Syncs, no
    // master, every packet discarded, silence). lwIP keeps the membership and
    // answers the switch's queries for as long as this socket is open.
    {
        int s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        struct ip_mreq mr = { .imr_multiaddr.s_addr = htonl(PTP1_GROUP_IP),
                              .imr_interface.s_addr = htonl(INADDR_ANY) };
        if (s < 0 || setsockopt(s, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mr, sizeof(mr)) != 0)
            ESP_LOGW(TAG, "IGMP join of the PTP group failed");
        else
            ESP_LOGI(TAG, "joined PTP group 224.0.1.129 (IGMP)");
    }

    // Identity first: mDNS id= is read from aoip_info.
    ESP_ERROR_CHECK(aoip_info_start());
    ESP_ERROR_CHECK(aoip_mdns_start());
    ESP_ERROR_CHECK(aoip_arc_start());
    ESP_ERROR_CHECK(subscriber_start());
    ESP_ERROR_CHECK(console_start());
    xTaskCreate(igmp_keeper_task, "igmp_keep", 3072, NULL, 2, NULL);

    ESP_LOGI(TAG, "up: %d channels, %s, %d-bit, fpp %u, latency %u us",
             AP_NCH, rate_get()->label, AP_BITS_PER_SAMPLE,
             rate_get()->fpp, (unsigned)mclk_get_latency_us());
}
