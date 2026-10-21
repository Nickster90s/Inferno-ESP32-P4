// Web page: status, output volume, firmware update. See web.h.
//
//   GET  /                 the page (web_page.html)
//   GET  /api/status       JSON for the page
//   POST /api/volume?db=N  output gain, -60..+4 dB (kept); &mute=0|1 (not kept)
//   POST /api/update       a signed .fw file (tools/fwsign.py) as the body
//   POST /api/previous     boot the other app slot again
//
// The server runs on the control core at control priority, below everything
// that moves audio. Writing flash stops both cores' caches for each sector,
// so the output is muted for the whole update; it ends in a restart anyway.

#include "web.h"
#include "app_config.h"
#include "jitterbuf.h"
#include "media_clock.h"
#include "rate.h"
#include "ptpv1.h"
#include "subscriber.h"
#include "aoip_mdns.h"
#include "eth_ts.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "esp_http_server.h"
#include "esp_ota_ops.h"
#include "esp_app_desc.h"
#include "esp_image_format.h"
#include "hal/efuse_hal.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_log.h"
#include "mbedtls/pk.h"
#include "mbedtls/sha256.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "web";

#ifndef AP_UPDATE_URL
#define AP_UPDATE_URL ""
#endif
// v1.x and v3.x ESP32-P4 chips need different images (sdkconfig.defaults), so
// the server keeps them apart as two products.
#if CONFIG_ESP_REV_MIN_FULL >= 300
#define FW_PRODUCT  "dac8v3"
#else
#define FW_PRODUCT  "dac8"
#endif

// .fw = app image + this trailer (tools/fwsign.py)
#define FW_TRAILER  128
#define FW_MAGIC    "NSFWSIG1"

extern const char page_start[] asm("_binary_web_page_html_start");
extern const char page_end[]   asm("_binary_web_page_html_end");
extern const char pub_start[]  asm("_binary_fw_sign_pub_pem_start");
extern const char pub_end[]    asm("_binary_fw_sign_pub_pem_end");

static volatile bool s_updating;

// ---------------------------------------------------------------------------

static esp_err_t page_get(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    // EMBED_TXTFILES adds a NUL; don't send it
    return httpd_resp_send(req, page_start, page_end - page_start - 1);
}

// JSON string body (no quotes): the names come from controllers on the network.
static void jstr(char *out, size_t n, const char *s)
{
    size_t o = 0;
    for (; s && *s && o + 7 < n; s++) {
        unsigned char c = (unsigned char)*s;
        if (c == '"' || c == '\\') { out[o++] = '\\'; out[o++] = c; }
        else if (c < 0x20) o += snprintf(out + o, n - o, "\\u%04x", c);
        else out[o++] = c;
    }
    out[o] = 0;
}

// The other slot's version, read once: a flash read pauses the cache of both
// cores, and the page asks for the status every 2 s.
static char s_other_version[32];

static void read_other_version(void)
{
    esp_app_desc_t d;
    const esp_partition_t *p = esp_ota_get_next_update_partition(NULL);
    if (p && esp_ota_get_partition_description(p, &d) == ESP_OK)
        strlcpy(s_other_version, d.version, sizeof(s_other_version));
    else
        s_other_version[0] = 0;
}

static esp_err_t status_get(httpd_req_t *req)
{
    const esp_app_desc_t *app = esp_app_get_description();
    const esp_partition_t *run = esp_ota_get_running_partition();
    esp_netif_ip_info_t ip = { 0 };
    esp_netif_get_ip_info(eth_ts_netif(), &ip);
    const uint8_t *m = eth_ts_mac();
    jb_stats_t jb; jb_get_stats(&jb);
    subscriber_stats_t ss; subscriber_get_stats(&ss);

    char *buf = malloc(3072);
    if (!buf) return httpd_resp_send_500(req);
    char name[80], a[80], b[80], sha[9];
    esp_app_get_elf_sha256(sha, sizeof(sha));     // the build, when the date is not

    jstr(name, sizeof(name), aoip_mdns_hostname());
    int o = snprintf(buf, 3072,
        "{\"name\":\"%s\",\"mac\":\"%02x:%02x:%02x:%02x:%02x:%02x\",\"ip\":\"" IPSTR "\","
        "\"product\":\"" FW_PRODUCT "\",\"version\":\"%s\",\"build\":\"%s %.5s · %s\",\"idf\":\"%s\","
        "\"slot\":\"%s\",\"previous\":\"%s\",\"update_url\":\"%s\",\"updating\":%d,"
        "\"uptime_s\":%llu,\"rate\":%u,\"latency_us\":%u,\"latency_set_us\":%u,"
        "\"gain_db\":%d,\"gain_min\":%d,\"gain_max\":%d,\"mute\":%d,"
        "\"ptp_locked\":%d,\"ptp_offset_ns\":%lld,\"ptp_lock_losses\":%u,"
        "\"flows\":%u,\"late\":%u,\"underrun_frames\":%u,\"channels\":[",
        name, m[0], m[1], m[2], m[3], m[4], m[5], IP2STR(&ip.ip),
        app->version, app->date, app->time, sha, app->idf_ver,
        run ? run->label : "?", s_other_version, AP_UPDATE_URL, (int)s_updating,
        (unsigned long long)(esp_timer_get_time() / 1000000), (unsigned)rate_hz(),
        (unsigned)mclk_get_latency_us(), (unsigned)mclk_get_config_latency_us(),
        jb_get_gain_db(), JB_GAIN_MIN_DB, JB_GAIN_MAX_DB, (int)jb_get_mute(),
        (int)g_ptpv1.locked, (long long)g_ptpv1.filtered_ns, (unsigned)g_ptpv1.lock_losses,
        (unsigned)ss.flows_active, (unsigned)jb.pkt_late, (unsigned)jb.frames_underrun);
    for (int i = 0; i < AP_NCH && o < 2900; i++) {
        const rx_channel_t *c = subscriber_channel(i);
        jstr(name, sizeof(name), c ? c->friendly : "");
        jstr(a, sizeof(a), c ? c->tx_channel : "");
        jstr(b, sizeof(b), c ? c->tx_device : "");
        o += snprintf(buf + o, 3072 - o, "%s{\"name\":\"%s\",\"tx_channel\":\"%s\",\"tx_device\":\"%s\"}",
                      i ? "," : "", name, a, b);
    }
    o += snprintf(buf + o, 3072 - o, "]}");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    esp_err_t r = httpd_resp_send(req, buf, o);
    free(buf);
    return r;
}

static esp_err_t volume_post(httpd_req_t *req)
{
    char q[64], v[16];
    if (httpd_req_get_url_query_str(req, q, sizeof(q)) == ESP_OK) {
        if (httpd_query_key_value(q, "db", v, sizeof(v)) == ESP_OK)
            jb_set_gain_db(atoi(v));
        if (httpd_query_key_value(q, "mute", v, sizeof(v)) == ESP_OK)
            jb_set_mute(v[0] == '1');
        ESP_LOGI(TAG, "volume %d dB%s", jb_get_gain_db(), jb_get_mute() ? ", muted" : "");
    }
    return status_get(req);
}

// ---------------------------------------------------------------------------
// Firmware update

static void restart_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(500));      // let the answer leave
    eth_ts_reboot();
}

static esp_err_t fail(httpd_req_t *req, const char *status, const char *msg)
{
    ESP_LOGW(TAG, "update refused: %s", msg);
    httpd_resp_set_status(req, status);
    httpd_resp_set_type(req, "text/plain");
    return httpd_resp_sendstr(req, msg);
}

static bool signature_ok(const uint8_t hash[32], const uint8_t *trailer)
{
    if (memcmp(trailer, FW_MAGIC, 8) != 0) return false;
    size_t n = trailer[8] | (trailer[9] << 8);
    if (n == 0 || n > FW_TRAILER - 10) return false;
    mbedtls_pk_context pk;
    mbedtls_pk_init(&pk);
    bool ok = mbedtls_pk_parse_public_key(&pk, (const unsigned char *)pub_start,
                                          pub_end - pub_start) == 0 &&
              mbedtls_pk_verify(&pk, MBEDTLS_MD_SHA256, hash, 32, trailer + 10, n) == 0;
    mbedtls_pk_free(&pk);
    return ok;
}

static esp_err_t update_post(httpd_req_t *req)
{
    const esp_partition_t *part = esp_ota_get_next_update_partition(NULL);
    size_t total = req->content_len;
    if (s_updating) return fail(req, "409 Conflict", "an update is already running");
    if (!part) return fail(req, "500 Internal Server Error", "no spare app slot");
    if (total < 4096 + FW_TRAILER || total - FW_TRAILER > part->size)
        return fail(req, "400 Bad Request", "not a DAC8 firmware file (size)");
    const size_t img_len = total - FW_TRAILER;
    const size_t desc_end = sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t)
                          + sizeof(esp_app_desc_t);

    uint8_t *buf = malloc(4096);
    uint8_t *head = malloc(desc_end);
    mbedtls_sha256_context sha;
    uint8_t trailer[FW_TRAILER], hash[32];
    esp_ota_handle_t ota = 0;
    const char *err = NULL, *status = "400 Bad Request";
    if (!buf || !head) { free(buf); free(head); return fail(req, "500 Internal Server Error", "no memory"); }

    // Still on trial (first minute after an update): it booted and took this
    // upload, so it is good enough to come back to -- and esp_ota_begin()
    // refuses to start while the running image is unconfirmed.
    esp_ota_img_states_t st;
    if (esp_ota_get_state_partition(esp_ota_get_running_partition(), &st) == ESP_OK &&
        st == ESP_OTA_IMG_PENDING_VERIFY)
        esp_ota_mark_app_valid_cancel_rollback();

    s_updating = true;
    jb_set_mute(true);
    vTaskDelay(pdMS_TO_TICKS(30));       // the DMA ring plays out to silence
    ESP_LOGW(TAG, "update: %u bytes to %s", (unsigned)total, part->label);
    mbedtls_sha256_init(&sha);
    mbedtls_sha256_starts(&sha, 0);
    if (esp_ota_begin(part, OTA_WITH_SEQUENTIAL_WRITES, &ota) != ESP_OK) {
        err = "could not open the spare app slot"; status = "500 Internal Server Error";
    }

    int timeouts = 0;
    for (size_t got = 0; !err && got < total; ) {
        int n = httpd_req_recv(req, (char *)buf, total - got < 4096 ? total - got : 4096);
        if (n == HTTPD_SOCK_ERR_TIMEOUT && ++timeouts < 3) continue;
        if (n <= 0) { err = "upload interrupted"; break; }
        timeouts = 0;
        if (got == 0 && (uint8_t)buf[0] != ESP_IMAGE_HEADER_MAGIC) { err = "not a DAC8 firmware file"; break; }
        size_t img_n = got < img_len ? (img_len - got < (size_t)n ? img_len - got : (size_t)n) : 0;
        if (got < desc_end)
            memcpy(head + got, buf, desc_end - got < (size_t)n ? desc_end - got : (size_t)n);
        if (img_n) {
            mbedtls_sha256_update(&sha, buf, img_n);
            if (esp_ota_write(ota, buf, img_n) != ESP_OK) {
                err = "flash write failed"; status = "500 Internal Server Error"; break;
            }
        }
        if ((size_t)n > img_n) memcpy(trailer + (got + img_n - img_len), buf + img_n, n - img_n);
        got += n;
        if (got >= desc_end && got - n < desc_end) {
            // Same key signs only DAC8 builds today; check the project anyway.
            const esp_app_desc_t *d = (const esp_app_desc_t *)
                (head + sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t));
            const esp_image_header_t *h = (const esp_image_header_t *)head;
            unsigned rev = efuse_hal_chip_revision();
            if (head[0] != ESP_IMAGE_HEADER_MAGIC || d->magic_word != ESP_APP_DESC_MAGIC_WORD ||
                strncmp(d->project_name, esp_app_get_description()->project_name,
                        sizeof(d->project_name)) != 0)
                err = "not a DAC8 firmware file";
            else if (rev < h->min_chip_rev_full || rev > h->max_chip_rev_full)
                err = "firmware is for another chip revision";
            else
                ESP_LOGW(TAG, "update: version %.32s", d->version);
        }
    }
    mbedtls_sha256_finish(&sha, hash);
    mbedtls_sha256_free(&sha);

    if (!err && !signature_ok(hash, trailer)) { err = "signature does not match"; status = "403 Forbidden"; }
    if (!err && esp_ota_end(ota) != ESP_OK) { err = "image check failed"; ota = 0; }
    else if (!err) ota = 0;
    if (!err && esp_ota_set_boot_partition(part) != ESP_OK) {
        err = "could not select the new image"; status = "500 Internal Server Error";
    }
    if (ota) esp_ota_abort(ota);
    // A refused image must not stay bootable: "Go back" boots whatever is in
    // the spare slot, and that has to be firmware that was verified.
    if (err && esp_partition_erase_range(part, 0, part->erase_size) != ESP_OK)
        ESP_LOGE(TAG, "could not wipe %s", part->label);
    if (err) read_other_version();
    free(buf);
    free(head);

    if (err) {
        jb_set_mute(false);
        s_updating = false;
        return fail(req, status, err);
    }
    ESP_LOGW(TAG, "update: written and verified -- restarting");
    httpd_resp_sendstr(req, "ok, restarting");
    xTaskCreatePinnedToCore(restart_task, "restart", 2048, NULL, AP_PRIO_CONTROL, NULL, AP_CORE_CONTROL);
    return ESP_OK;
}

static esp_err_t previous_post(httpd_req_t *req)
{
    const esp_partition_t *p = esp_ota_get_next_update_partition(NULL);
    esp_app_desc_t d;
    if (s_updating) return fail(req, "409 Conflict", "an update is running");
    if (!p || esp_ota_get_partition_description(p, &d) != ESP_OK)
        return fail(req, "404 Not Found", "no previous firmware");
    if (esp_ota_set_boot_partition(p) != ESP_OK)
        return fail(req, "500 Internal Server Error", "the previous firmware is not valid");
    ESP_LOGW(TAG, "booting the previous firmware %s from %s", d.version, p->label);
    s_updating = true;
    jb_set_mute(true);
    httpd_resp_sendstr(req, "ok, restarting");
    xTaskCreatePinnedToCore(restart_task, "restart", 2048, NULL, AP_PRIO_CONTROL, NULL, AP_CORE_CONTROL);
    return ESP_OK;
}

// A new image runs on trial (rollback): it keeps the slot once it has run
// for a minute. A crash or reset before that boots the previous one again.
static void confirm_task(void *arg)
{
    esp_ota_img_states_t st;
    const esp_partition_t *run = esp_ota_get_running_partition();
    if (esp_ota_get_state_partition(run, &st) == ESP_OK && st == ESP_OTA_IMG_PENDING_VERIFY) {
        ESP_LOGW(TAG, "new firmware on trial: confirmed after 60 s");
        vTaskDelay(pdMS_TO_TICKS(60000));
        esp_ota_mark_app_valid_cancel_rollback();
        ESP_LOGW(TAG, "new firmware confirmed");
    }
    vTaskDelete(NULL);
}

esp_err_t web_start(void)
{
    httpd_handle_t h = NULL;
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.core_id = AP_CORE_CONTROL;
    cfg.task_priority = AP_PRIO_CONTROL;
    cfg.stack_size = 8192;
    cfg.max_open_sockets = 5;
    cfg.lru_purge_enable = true;
    cfg.recv_wait_timeout = 10;
    cfg.send_wait_timeout = 10;
    read_other_version();
    esp_err_t e = httpd_start(&h, &cfg);
    if (e != ESP_OK) return e;
    static const httpd_uri_t uris[] = {
        { .uri = "/",             .method = HTTP_GET,  .handler = page_get },
        { .uri = "/api/status",   .method = HTTP_GET,  .handler = status_get },
        { .uri = "/api/volume",   .method = HTTP_POST, .handler = volume_post },
        { .uri = "/api/update",   .method = HTTP_POST, .handler = update_post },
        { .uri = "/api/previous", .method = HTTP_POST, .handler = previous_post },
    };
    for (size_t i = 0; i < sizeof(uris) / sizeof(uris[0]); i++)
        httpd_register_uri_handler(h, &uris[i]);
    xTaskCreatePinnedToCore(confirm_task, "fw_confirm", 3072, NULL, 1, NULL, AP_CORE_CONTROL);
    ESP_LOGI(TAG, "web page on port 80, firmware %s", esp_app_get_description()->version);
    return ESP_OK;
}
