#include "subscriber.h"
#include "rate.h"
#include "chan_resolve.h"
#include "flows_client.h"
#include "aoip_rx.h"
#include "aoip_wire.h"
#include "media_clock.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "nvs.h"
#include "aoip_info.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "subscriber";

// Real receivers refresh every ~5 s (FPGA project UNICAST_FLOWS.md). Refresh at
// 2 s so a single lost datagram does not expire the flow.
#define KEEPALIVE_MS    3000     // liveness window: no audio this long = flow dead
#define RETRY_MS        5000

typedef enum { F_IDLE = 0, F_ACTIVE } fstate_t;

typedef struct {
    fstate_t       state;
    char           device[32];
    flow_request_t req;
    uint8_t        handle[FLOW_HANDLE_LEN];
    int8_t         slot_to_ch[AP_MAX_CH_PER_FLOW];
    int64_t        next_action_us;
    uint32_t       last_packets;    // liveness: aoip_rx packet count at last check
} flow_slot_t;

static rx_channel_t  s_ch[AP_NCH];
static flow_slot_t   s_flow[AP_MAX_FLOWS];
static subscriber_stats_t s_st;
static SemaphoreHandle_t  s_lock;
static volatile bool s_dirty = true;

const rx_channel_t *subscriber_channel(uint8_t c)
{
    return c < AP_NCH ? &s_ch[c] : NULL;
}

void subscriber_force_refresh(void) { s_dirty = true; }

// ---------------------------------------------------------------------------
// Persistence. Real AoIP devices remember their subscriptions across a power
// cycle and re-establish them; without this a reboot (or a serial console
// open, which resets this board) silently un-patches the device.
// ---------------------------------------------------------------------------

typedef struct {
    char tx_channel[32];
    char tx_device[32];
    char friendly[32];
} saved_ch_t;

#define NVS_NS   "subs"
#define NVS_KEY  "ch"

static void save(void)
{
    saved_ch_t sv[AP_NCH];
    xSemaphoreTake(s_lock, portMAX_DELAY);
    for (int c = 0; c < AP_NCH; c++) {
        memcpy(sv[c].tx_channel, s_ch[c].tx_channel, sizeof(sv[c].tx_channel));
        memcpy(sv[c].tx_device,  s_ch[c].tx_device,  sizeof(sv[c].tx_device));
        memcpy(sv[c].friendly,   s_ch[c].friendly,   sizeof(sv[c].friendly));
    }
    xSemaphoreGive(s_lock);
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_blob(h, NVS_KEY, sv, sizeof(sv));
    nvs_commit(h);
    nvs_close(h);
}

static void load(void)
{
    saved_ch_t sv[AP_NCH];
    size_t len = sizeof(sv);
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) return;
    esp_err_t err = nvs_get_blob(h, NVS_KEY, sv, &len);
    nvs_close(h);
    if (err != ESP_OK || len != sizeof(sv)) return;
    for (int c = 0; c < AP_NCH; c++) {
        sv[c].tx_channel[31] = sv[c].tx_device[31] = sv[c].friendly[31] = 0;
        memcpy(s_ch[c].tx_channel, sv[c].tx_channel, sizeof(sv[c].tx_channel));
        memcpy(s_ch[c].tx_device,  sv[c].tx_device,  sizeof(sv[c].tx_device));
        if (sv[c].friendly[0]) memcpy(s_ch[c].friendly, sv[c].friendly, sizeof(sv[c].friendly));
        s_ch[c].status = s_ch[c].tx_channel[0] ? ARC_SUB_PENDING : ARC_SUB_NONE;
        if (s_ch[c].tx_channel[0])
            ESP_LOGI(TAG, "restored ch%d <- %s@%s", c + 1, s_ch[c].tx_channel, s_ch[c].tx_device);
    }
}

esp_err_t subscriber_set(uint8_t c, const char *tx_channel, const char *tx_device)
{
    if (c >= AP_NCH) return ESP_ERR_INVALID_ARG;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (!tx_channel || !*tx_channel) {
        s_ch[c].tx_channel[0] = 0;
        s_ch[c].tx_device[0]  = 0;
        s_ch[c].status = ARC_SUB_NONE;
    } else {
        strncpy(s_ch[c].tx_channel, tx_channel, sizeof(s_ch[c].tx_channel) - 1);
        strncpy(s_ch[c].tx_device,  tx_device ? tx_device : "", sizeof(s_ch[c].tx_device) - 1);
        s_ch[c].status = ARC_SUB_PENDING;
    }
    s_dirty = true;
    xSemaphoreGive(s_lock);
    ESP_LOGI(TAG, "ch%u <- %s@%s", c + 1, s_ch[c].tx_channel, s_ch[c].tx_device);
    save();
    aoip_info_notify_rx_change(1u << c);
    return ESP_OK;
}

esp_err_t subscriber_rename(uint8_t c, const char *friendly)
{
    if (c >= AP_NCH) return ESP_ERR_INVALID_ARG;
    strncpy(s_ch[c].friendly, friendly ? friendly : "", sizeof(s_ch[c].friendly) - 1);
    save();
    aoip_info_notify_rx_change(1u << c);
    return ESP_OK;
}

void subscriber_get_stats(subscriber_stats_t *out)
{
    uint8_t n = 0;
    for (int i = 0; i < AP_MAX_FLOWS; i++) if (s_flow[i].state == F_ACTIVE) n++;
    s_st.flows_active = n;
    *out = s_st;
}

// ---------------------------------------------------------------------------

static void tear_down(uint8_t i)
{
    flow_slot_t *f = &s_flow[i];
    if (f->state == F_ACTIVE) {
        flows_client_stop(&f->req, f->handle);
        aoip_rx_unbind_flow(i);
    }
    memset(f, 0, sizeof(*f));
    f->state = F_IDLE;
}

// Rebuild the flow table from the subscription table: one flow per distinct
// transmitter device, carrying every channel subscribed to it.
static void set_status(uint8_t c, uint32_t st)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    bool changed = s_ch[c].status != st;
    s_ch[c].status = st;
    xSemaphoreGive(s_lock);
    if (changed) aoip_info_notify_rx_change(1u << c);
}

// ---------------------------------------------------------------------------
// Resolve cache. chan_resolve() is an mDNS round trip of up to 3 s; a patch on
// ONE channel must not re-resolve the seven that did not change.
// ---------------------------------------------------------------------------

typedef struct {
    char         tx_channel[32];
    char         tx_device[32];
    tx_channel_t tx;
    bool         valid;
} resolved_t;

static resolved_t s_res[AP_NCH];

static bool resolve_cached(uint8_t c, const rx_channel_t *ch, tx_channel_t *out)
{
    resolved_t *r = &s_res[c];
    if (r->valid && strcmp(r->tx_channel, ch->tx_channel) == 0 &&
        strcmp(r->tx_device, ch->tx_device) == 0) {
        *out = r->tx;
        return true;
    }
    r->valid = false;
    if (chan_resolve(ch->tx_channel, ch->tx_device, out) != ESP_OK) return false;
    strncpy(r->tx_channel, ch->tx_channel, sizeof(r->tx_channel) - 1);
    strncpy(r->tx_device, ch->tx_device, sizeof(r->tx_device) - 1);
    r->tx = *out;
    r->valid = true;
    return true;
}

// ---------------------------------------------------------------------------
// INCREMENTAL rebuild.
//
// It used to tear every flow down, re-resolve every channel, and re-request
// everything -- so patching one channel silenced ALL of them for seconds
// (bench: "if I patch a ch every ch gets muted"). Now:
//
//   flow to a device whose channel list is unchanged   untouched
//   flow to a device whose channel list changed        MAKE-BEFORE-BREAK: a new
//                                                      flow is requested while
//                                                      the old keeps playing; the
//                                                      old is stopped only once
//                                                      the new one delivers
//   device no longer wanted                            stopped
//   new device                                         requested
//
// NOT called with s_lock held: chan_resolve() blocks, and the ARC server on the
// other core needs the lock.
// ---------------------------------------------------------------------------

typedef struct {
    char           device[32];
    flow_request_t req;                         // tx side + slots; rx_port unset
    int8_t         slot_to_ch[AP_MAX_CH_PER_FLOW];
    bool           done;
    int            replaces;                    // flow slot to retire once this is up, or -1
} want_t;

static void activate_slots(const flow_slot_t *f)
{
    for (int k = 0; k < f->req.nslots; k++) {
        int8_t c = f->slot_to_ch[k];
        if (c >= 0) set_status((uint8_t)c, ARC_SUB_ACTIVE);
    }
}

static bool same_slots(const flow_slot_t *f, const want_t *w)
{
    if (f->req.nslots != w->req.nslots) return false;
    for (int k = 0; k < w->req.nslots; k++) {
        if (f->req.tx_channel_id[k] != w->req.tx_channel_id[k]) return false;
        if (f->slot_to_ch[k] != w->slot_to_ch[k]) return false;
    }
    return true;
}

static void rebuild(void)
{
    rx_channel_t snap[AP_NCH];
    xSemaphoreTake(s_lock, portMAX_DELAY);
    memcpy(snap, s_ch, sizeof(snap));
    xSemaphoreGive(s_lock);

    // 1. What we WANT: one entry per transmitter device.
    want_t want[AP_MAX_FLOWS];
    memset(want, 0, sizeof(want));
    int nwant = 0;

    for (uint8_t c = 0; c < AP_NCH; c++) {
        if (!snap[c].tx_channel[0]) { s_res[c].valid = false; continue; }

        tx_channel_t tx;
        if (!resolve_cached(c, &snap[c], &tx)) {
            s_st.resolve_failed++;
            set_status(c, ARC_SUB_PENDING);
            continue;
        }
        if (tx.sample_rate != rate_hz()) {
            // Subscriptions do not cross sample rates, and a device at the
            // wrong one is invisible as a fault. Say which way round it is.
            ESP_LOGW(TAG, "ch%u: %s transmits at %u Hz, we are at %s",
                     c + 1, snap[c].tx_device, (unsigned)tx.sample_rate,
                     rate_get()->label);
            set_status(c, ARC_SUB_PENDING);
            continue;
        }

        int w = -1;
        for (int i = 0; i < nwant; i++)
            if (strcmp(want[i].device, snap[c].tx_device) == 0) { w = i; break; }
        if (w < 0) {
            if (nwant >= AP_MAX_FLOWS) {
                ESP_LOGW(TAG, "ch%u: no free flow (max %d transmitters)", c + 1, AP_MAX_FLOWS);
                set_status(c, ARC_SUB_PENDING);
                continue;
            }
            w = nwant++;
            want[w].replaces = -1;
            strncpy(want[w].device, snap[c].tx_device, sizeof(want[w].device) - 1);
            for (int k = 0; k < AP_MAX_CH_PER_FLOW; k++) want[w].slot_to_ch[k] = -1;
            want[w].req.tx_ip        = tx.ip;
            want[w].req.tx_flow_port = tx.flow_port;
            // fpp: ours, clamped to what this transmitter does.
            uint16_t fpp = rate_get()->fpp;
            if (fpp > tx.fpp_max) fpp = tx.fpp_max;
            if (fpp < tx.fpp_min) fpp = tx.fpp_min;
            want[w].req.fpp = fpp;
        }
        if (want[w].req.nslots >= AP_MAX_CH_PER_FLOW) {
            set_status(c, ARC_SUB_PENDING);
            continue;
        }
        uint8_t k = want[w].req.nslots++;
        want[w].req.tx_channel_id[k] = tx.tx_channel_id;
        want[w].slot_to_ch[k]        = (int8_t)c;

        // The transmitter's advertised latency is a floor on ours.
        uint32_t tx_lat_us = tx.latency_ns / 1000;
        if (tx_lat_us > mclk_get_latency_us()) {
            ESP_LOGI(TAG, "raising latency to %u us: %s asks for it",
                     (unsigned)tx_lat_us, snap[c].tx_device);
            mclk_set_latency_us(tx_lat_us);
        }
    }

    // 2. Reconcile the flows we HAVE against it.
    for (int i = 0; i < AP_MAX_FLOWS; i++) {
        flow_slot_t *f = &s_flow[i];
        if (!f->device[0]) continue;

        want_t *w = NULL;
        for (int k = 0; k < nwant; k++)
            if (!want[k].done && strcmp(want[k].device, f->device) == 0) { w = &want[k]; break; }

        if (!w || f->state != F_ACTIVE) { tear_down((uint8_t)i); continue; }

        if (same_slots(f, w)) {                       // untouched
            w->done = true;
            activate_slots(f);
            continue;
        }

        // Channel list changed. NOT 0x0102 update-in-place: on the bench the
        // virtual soundcard acknowledged it and changed the packet layout, but
        // sent zeros in the added slots. Leave this flow PLAYING and have step
        // 3 request its replacement; the old one is retired once the new one
        // delivers. Both write identical timestamped samples into the same
        // jitter buffer while they overlap, so there is nothing to hear.
        w->replaces = i;
    }

    // 3. Request whatever is still wanted.
    for (int k = 0; k < nwant; k++) {
        want_t *w = &want[k];
        if (w->done) continue;
        int i = -1;
        for (int j = 0; j < AP_MAX_FLOWS; j++) if (!s_flow[j].device[0]) { i = j; break; }
        if (i < 0) continue;

        flow_slot_t *f = &s_flow[i];
        memset(f, 0, sizeof(*f));
        strncpy(f->device, w->device, sizeof(f->device) - 1);
        f->req = w->req;
        memcpy(f->slot_to_ch, w->slot_to_ch, sizeof(f->slot_to_ch));

        uint16_t port = 0;
        if (aoip_rx_bind_flow((uint8_t)i, f->req.nslots, f->slot_to_ch,
                               f->req.fpp, &port) != ESP_OK) {
            ESP_LOGE(TAG, "flow %d: could not bind a socket", i);
            f->device[0] = 0;
            continue;
        }
        f->req.rx_port = port;
        snprintf(f->req.rx_flow_name, sizeof(f->req.rx_flow_name), "rx%d", i);

        if (flows_client_request(&f->req, f->handle) == ESP_OK) {
            f->state = F_ACTIVE;
            f->last_packets = 0;
            f->next_action_us = esp_timer_get_time() + KEEPALIVE_MS * 1000;
            s_st.requests_ok++;
            if (w->replaces >= 0) {
                // Break only after make: wait for the new flow's audio (up to
                // 1 s), then stop the old one.
                for (int t = 0; t < 100 && aoip_rx_flow_packets((uint8_t)i) == 0; t++)
                    vTaskDelay(pdMS_TO_TICKS(10));
                ESP_LOGI(TAG, "flow %d to %s replaced by flow %d (%u ch), %s", w->replaces,
                         w->device, i, f->req.nslots,
                         aoip_rx_flow_packets((uint8_t)i) ? "seamless" : "new flow silent");
                tear_down((uint8_t)w->replaces);
            }
            activate_slots(f);
        } else {
            s_st.requests_failed++;
            aoip_rx_unbind_flow((uint8_t)i);
            f->device[0] = 0;
            ESP_LOGW(TAG, "flow to %s refused; retrying%s", w->device,
                     w->replaces >= 0 ? " (old flow kept playing)" : "");
            for (int c = 0; c < w->req.nslots; c++)
                if (w->slot_to_ch[c] >= 0) set_status((uint8_t)w->slot_to_ch[c], ARC_SUB_PENDING);
        }
    }
}

// A subscription that could not be brought up (transmitter not resolved yet,
// flow refused) is retried every RETRY_MS -- before this it stayed pending
// until AoIP Controller happened to change something. Safe while other flows
// are running: rebuild() leaves unchanged flows alone.
static bool any_pending(void)
{
    bool pending = false, active = false;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    for (uint8_t c = 0; c < AP_NCH; c++) {
        if (s_ch[c].tx_channel[0] && s_ch[c].status != ARC_SUB_ACTIVE) pending = true;
    }
    xSemaphoreGive(s_lock);
    (void)active;       // rebuild is incremental now: working flows are not disturbed
    return pending;
}

static void sub_task(void *arg)
{
    (void)arg;
    int64_t next_retry = 0;
    for (;;) {
        if (s_dirty) {
            s_dirty = false;
            rebuild();
            next_retry = esp_timer_get_time() + RETRY_MS * 1000;
        } else if (esp_timer_get_time() >= next_retry && any_pending()) {
            ESP_LOGI(TAG, "retrying pending subscriptions");
            rebuild();
            next_retry = esp_timer_get_time() + RETRY_MS * 1000;
        }

        int64_t now = esp_timer_get_time();
        for (int i = 0; i < AP_MAX_FLOWS; i++) {
            flow_slot_t *f = &s_flow[i];
            if (f->state != F_ACTIVE || now < f->next_action_us) continue;

            // LIVENESS, not a control-port keepalive: the keepalive proper is
            // sent on the audio socket (aoip_rx.c). A flow whose packet count
            // has stopped moving is dead -- the transmitter dropped it, or it
            // never started -- and is re-requested from scratch.
            uint32_t pk = aoip_rx_flow_packets((uint8_t)i);
            if (pk != f->last_packets) {
                f->last_packets = pk;
                s_st.keepalives_ok++;
                f->next_action_us = now + KEEPALIVE_MS * 1000;
            } else {
                s_st.keepalives_lost++;
                ESP_LOGW(TAG, "flow %d: no audio for %d ms -- re-requesting",
                         i, KEEPALIVE_MS);
                // Drop it FIRST: rebuild() leaves a flow whose channel list is
                // unchanged alone, and a dead one looks exactly like that.
                tear_down((uint8_t)i);
                s_dirty = true;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

esp_err_t subscriber_start(void)
{
    s_lock = xSemaphoreCreateMutex();
    if (!s_lock) return ESP_ERR_NO_MEM;
    for (uint8_t c = 0; c < AP_NCH; c++) {
        snprintf(s_ch[c].friendly, sizeof(s_ch[c].friendly), "%u", c + 1);
    }
    load();
    BaseType_t ok = xTaskCreatePinnedToCore(sub_task, "subscriber", 6144, NULL,
                                            AP_PRIO_CONTROL, NULL, AP_CORE_CONTROL);
    return ok == pdPASS ? ESP_OK : ESP_FAIL;
}
