#include "eth_ts.h"
#include "esp_heap_caps.h"
// UART console.
//
// The escape hatch. RX_GATE.md's third safety layer was "the console is the
// escape hatch -- a MAC filter cannot lock it out"; the same reasoning applies
// to anything here that can take the device off the network. Keep every
// dangerous knob reachable from this file.

#include "console.h"
#include "rate.h"
#include "app_config.h"
#include "media_clock.h"
#include "mclk_hw.h"
#include "aoip_mdns.h"
#include "subscriber.h"
#include "pcm1690.h"
#include "ptpv1.h"
#include "jitterbuf.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

static const char *TAG = "console";

static void help(void)
{
    printf(
      "\n"
      "  s            stats summary\n"
      "  a [0|1]      media clock discipline: disarm / arm (toggle if no arg)\n"
      "  l <us>       playout latency, %u..%u us\n"
      "  r            re-anchor the playout pointer to PTP now\n"
      "  n <name>     device name (re-advertises mDNS)\n"
      "  k            force a subscription refresh\n"
      "  m [0|1]      DAC mute\n"
      "  ?            this help\n\n",
      (unsigned)rate_get()->latency_min_us, AP_LATENCY_US_MAX);
}

static void health(void)
{
    // Where receive-side stalls have to be diagnosed from: heap (IDF mallocs
    // every received frame from internal DMA-capable RAM) and the state of
    // IDF's emac_rx task.
    TaskHandle_t rx = xTaskGetHandle("emac_rx");
    static const char *states[] = { "running", "ready", "BLOCKED", "suspended", "deleted", "invalid" };
    eTaskState ts = rx ? eTaskGetState(rx) : eInvalid;
    eth_ts_mac_dump();
    uint32_t dst, mf, of; eth_ts_rx_diag(&dst, &mf, &of);
    printf("  rx dma    state %u (3 waiting, 4 SUSPENDED no-desc, 0 stopped)  missed %u  fifo overflow %u\n",
           (unsigned)dst, (unsigned)mf, (unsigned)of);
    printf("  heap      internal free %u  min %u  largest %u | dma free %u largest %u\n"
           "  emac_rx   %s  stack hwm %u   rx kicks %u  emac restarts %u  phy resets %u\n",
           (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
           (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL),
           (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL),
           (unsigned)heap_caps_get_free_size(MALLOC_CAP_DMA),
           (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DMA),
           rx ? states[ts < 6 ? ts : 5] : "not found",
           rx ? (unsigned)uxTaskGetStackHighWaterMark(rx) : 0,
           (unsigned)eth_ts_rx_kicks(), (unsigned)eth_ts_rx_restarts(), (unsigned)eth_ts_phy_resets());
}

static void stats(void)
{
    health();
    jb_stats_t jb; jb_get_stats(&jb);
    subscriber_stats_t sub; subscriber_get_stats(&sub);
    const rate_profile_t *r = rate_get();
    printf("\n"
           "  rate      %s  BCK %.3f MHz  SCKI %.3f MHz  fpp %u  %s\n"
           "  ptp       %s  offset %lld ns  path %lld ns  rate %d ppb  steps %u\n"
           "  ptp pair  sync %u  followup %u  lost %u  orphan %u  mispair %u  no_hw_ts %u\n"
           "  mclk      %s %s  err %d frames  ppb %d (target %d, LSB %u ppb)  anchors %u\n"
           "  buffer    level %d frames  underruns %u  late %u  future %u  steps %u\n"
           "  flows     %u active  req %u/%u  keepalive %u ok / %u lost\n"
           "  latency   %u us\n\n",
           r->label, r->bck_fs * (double)r->hz / 1e6,
           r->scki_fs * (double)r->hz / 1e6, r->fpp,
           rate_is_pinned() ? "pinned at build time"
                            : (rate_pin_level() < 0 ? "pin unread"
                               : (rate_pin_level() ? "pin open" : "pin closed")),
           g_ptpv1.locked ? "LOCKED" : "unlocked",
           (long long)g_ptpv1.offset_ns, (long long)g_ptpv1.mean_path_delay_ns,
           (int)g_ptpv1.rate_ppb, (unsigned)g_ptpv1.step_count,
           (unsigned)g_ptpv1.rx_sync, (unsigned)g_ptpv1.rx_followup,
           (unsigned)g_ptpv1.lost_followup, (unsigned)g_ptpv1.orphan_followup,
           (unsigned)g_ptpv1.mispair, (unsigned)g_ptpv1.no_hw_ts,
           g_mclk.armed ? "ARMED" : "disarmed",
           g_mclk.anchored ? "anchored" : "unanchored",
           (int)g_mclk.error_frames, (int)g_mclk.ppb_applied,
           (int)g_mclk.ppb_target, (unsigned)mclk_hw_lsb_ppb(),
           (unsigned)g_mclk.anchors,
           (int)jb.level_frames, (unsigned)jb.frames_underrun,
           (unsigned)jb.pkt_late, (unsigned)jb.pkt_future, (unsigned)jb.playout_steps,
           (unsigned)sub.flows_active, (unsigned)sub.requests_ok,
           (unsigned)(sub.requests_ok + sub.requests_failed),
           (unsigned)sub.keepalives_ok, (unsigned)sub.keepalives_lost,
           (unsigned)mclk_get_latency_us());

    for (uint8_t c = 0; c < AP_NCH; c++) {
        const rx_channel_t *ch = subscriber_channel(c);
        printf("  ch%u \"%s\"  <- %s%s%s  [%s]\n", c + 1, ch->friendly,
               ch->tx_channel, ch->tx_channel[0] ? "@" : "", ch->tx_device,
               ch->status == 0x01010009u ? "active"
             : ch->status ? "pending" : "-");
    }
    printf("\n");
}

static void console_task(void *arg)
{
    (void)arg;
    char line[96];
    printf("\nAoIP RX 8ch -- '?' for help\n");

    for (;;) {
        if (!fgets(line, sizeof(line), stdin)) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        char *p = line;
        while (*p == ' ') p++;
        char cmd = *p++;
        while (*p == ' ') p++;
        char *arg = p;
        arg[strcspn(arg, "\r\n")] = 0;

        switch (cmd) {
        case 's': stats(); break;
        case '?': help(); break;
        case 'a':
            mclk_arm(*arg ? (atoi(arg) != 0) : !mclk_is_armed());
            break;
        case 'l':
            if (*arg) mclk_set_latency_us((uint32_t)atoi(arg));
            printf("latency %u us\n", (unsigned)mclk_get_latency_us());
            break;
        case 'r': mclk_anchor(); break;
        case 'n':
            if (*arg) { aoip_mdns_set_name(arg); printf("name -> %s\n", arg); }
            break;
        case 'k': subscriber_force_refresh(); printf("refreshing\n"); break;
        case 'm': pcm1690_set_mute(*arg ? (atoi(arg) != 0) : true); break;
        case '\n': case '\r': case 0: break;
        default: printf("? '%c'\n", cmd); break;
        }
    }
}

esp_err_t console_start(void)
{
    setvbuf(stdin, NULL, _IONBF, 0);
    BaseType_t ok = xTaskCreatePinnedToCore(console_task, "console", 4096, NULL,
                                            AP_PRIO_CONTROL - 1, NULL, AP_CORE_CONTROL);
    (void)TAG;
    return ok == pdPASS ? ESP_OK : ESP_FAIL;
}
