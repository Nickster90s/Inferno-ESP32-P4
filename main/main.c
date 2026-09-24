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

void app_main(void)
{
    // FIRST, before any clock or peripheral is configured: which rate are
    // we? Everything downstream derives from it, and the answer comes off
    // a pin that must be read while it is still just a pin.
    ESP_ERROR_CHECK(rate_select());

    ESP_ERROR_CHECK(nvs_flash_init());
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
    eth_ts_mcast_allow(0xE0000001);   // 224.0.0.1   all-hosts / IGMP
    eth_ts_mcast_allow(0xE00000FB);   // 224.0.0.251 mDNS
    eth_ts_mcast_allow(0xE00000E7);   // 224.0.0.231 AoIP info
    eth_ts_mcast_allow(0xE00000E9);   // 224.0.0.233 AoIP heartbeat
    eth_ts_mcast_allow(PTP1_GROUP_IP);

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

    // Discipline starts DISARMED, exactly as rx_gate did on the FPGA: the
    // device behaves like an undisciplined one until something asks for more,
    // so a servo bug cannot be the reason audio never worked at all. Arm it
    // from the console with 'a' once PTP is locked, or here once you trust it.
    mclk_arm(false);

    // 5. Control plane -- the only part that needs an address.
    wait_for_ip();

    // Identity first: mDNS id= is read from aoip_info.
    ESP_ERROR_CHECK(aoip_info_start());
    ESP_ERROR_CHECK(aoip_mdns_start());
    ESP_ERROR_CHECK(aoip_arc_start());
    ESP_ERROR_CHECK(subscriber_start());
    ESP_ERROR_CHECK(console_start());

    ESP_LOGI(TAG, "up: %d channels, %s, %d-bit, fpp %u, latency %u us",
             AP_NCH, rate_get()->label, AP_BITS_PER_SAMPLE,
             rate_get()->fpp, (unsigned)mclk_get_latency_us());
}
