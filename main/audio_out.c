#include "esp_timer.h"
#include "audio_out.h"
#include "app_config.h"
#include "rate.h"
#include "jitterbuf.h"
#include "media_clock.h"
#include "pcm1690.h"
#include "driver/i2s_tdm.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "audio_out";

static i2s_chan_handle_t s_tx;     // audio: BCK / WS / DOUT
static volatile uint32_t s_blocks;  // TX interrupts (descriptors played)

// The DMA plays the ring itself (jitterbuf.h): DAC position is jb_playout_idx()
// directly, so there is no depth to fold into the media clock's target. The
// DMA's lead (read-ahead + FIFO) sits in the latency minimum instead (rate.c).
uint32_t audio_out_dma_depth_frames(void) { return 0; }
uint32_t audio_out_blocks(void) { return s_blocks; }

// Every finished descriptor: the ring's position. The driver then zeroes the
// buffer (auto_clear), so it plays silence next lap unless a packet lands.
static bool IRAM_ATTR on_sent(i2s_chan_handle_t h, i2s_event_data_t *e, void *u)
{
    (void)h; (void)u;
    jb_isr_sent(e->dma_buf);
    s_blocks++;
    return false;
}

static volatile uint32_t s_peak_out[AP_NCH];    // max |sample| sent to the DAC, 32-bit

void audio_out_take_peaks(uint32_t *out)
{
    for (int c = 0; c < AP_NCH; c++) { out[c] = s_peak_out[c]; s_peak_out[c] = 0; }
}

// ---------------------------------------------------------------------------
// SCKI = BCK
// ---------------------------------------------------------------------------
//
// The PCM1690's system clock is the I2S BIT CLOCK, routed out a second time on
// the MCK pin through the GPIO matrix -- electrically the jumper between J1
// pins 5 and 9 that ../stm32/ESP32-P4/p4-uac-pcm1690 uses, verified there on
// this exact breakout, all eight jacks. Reasons, all from that project:
//
//  * IDF's TDM driver forces mclk_multiple >= 768 fs, so the P4's own MCLK
//    output can never be the 256/512 fs the PCM1690 accepts for TDM.
//  * High-speed TDM (FMTDA 1000) is SCKI = BCK = 256 fs, and the datasheet
//    wants only a ratiometric relation, not a phase one (7.4.3).
//  * SCKI needs a 40-60 % duty cycle (6.8). mclk_multiple 1024 gives an EVEN
//    BCK divider of 4 -- exactly 50 %.
//
// What it replaced: a second I2S port emitting a nominal 24.576 MHz "SCKI"
// off the audio port's APLL. On the bench it read 33 % duty (odd divider),
// and the DAC stayed silent with audio arriving at -5 dBFS on DIN1.
#include "soc/i2s_periph.h"
#include "soc/i2s_struct.h"
#include "hal/i2s_ll.h"
#include "hal/clk_tree_ll.h"
#include "esp_rom_gpio.h"

// ---------------------------------------------------------------------------
// 96 kHz: BCK DIVIDER 2, set behind the driver's back
// ---------------------------------------------------------------------------
//
// IDF's TDM master clocking is APLL -> MCLK (divider >= 2) -> BCK (divider
// >= 3, rounded to an even 4 here for SCKI's duty cycle). At 96 kHz, BCK =
// 256 fs = 24.576 MHz, and that chain needs a 196.6 MHz APLL -- over the
// P4's 125 MHz ceiling, which is why ../stm32/ESP32-P4/p4-uac-pcm1690 stopped
// at 48 kHz.
//
// The BCK >= 3 rule is the driver's, not the hardware's: i2s_tdm.c says data
// goes wrong at <= 2 "while RECEIVING multiple slots". This port only
// transmits. So the driver is configured for HALF the rate -- exactly the
// verified 48 kHz clocking, APLL 98.304 MHz, MCLK 49.152 MHz, BCK divider 4 --
// and the BCK divider is then rewritten to 2 before the channel is enabled
// (i2s_ll_tx_start latches it). BCK = 24.576 MHz, still an even divider, so
// still 50 % duty for SCKI; LRCK = 96 kHz. The APLL is at the same frequency
// as at 48 kHz, so the media-clock trim range is unchanged too.
static bool bck_halved(uint32_t hz)
{
    return 2ULL * AP_I2S_MCLK_MULTIPLE * hz > CLK_LL_APLL_MAX_HZ;
}

static void route_bck_to_scki(void)
{
    esp_rom_gpio_pad_select_gpio(AP_PIN_I2S_MCLK);
    gpio_set_direction(AP_PIN_I2S_MCLK, GPIO_MODE_OUTPUT);
    esp_rom_gpio_connect_out_signal(AP_PIN_I2S_MCLK,
                                    i2s_periph_signal[I2S_NUM_0].m_tx_bck_sig,
                                    false, false);
    ESP_LOGI(TAG, "SCKI = BCK (%u fs) on GPIO%d", AP_BCK_FS, AP_PIN_I2S_MCLK);
}

// ---------------------------------------------------------------------------

esp_err_t audio_out_init(void)
{
    const rate_profile_t *r = rate_get();

    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    // THE RING: AP_RING_FRAMES of DMA buffers in a closed loop, 4096 frames =
    // 85 ms at 48 kHz, 42 ms at 96 kHz. dma_frames only sets the interrupt
    // rate (3000/s). auto_clear zeroes each buffer after it has played.
    chan_cfg.dma_desc_num  = AP_RING_FRAMES / r->dma_frames;
    chan_cfg.dma_frame_num = r->dma_frames;
    chan_cfg.auto_clear    = true;
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &s_tx, NULL));

    i2s_tdm_config_t tdm_cfg = {
        .clk_cfg = {
            .sample_rate_hz = bck_halved(r->hz) ? r->hz / 2 : r->hz,
            // APLL, not the default clock. This is the whole media clock: with
            // I2S_CLK_SRC_DEFAULT the sample rate is whatever the SoC's PLL
            // divides to and there is nothing to trim.
            .clk_src        = I2S_CLK_SRC_APLL,
            // Internal only -- MCLK is not routed from this port. 512 keeps the
            // bclk divider at a legal 2 at both rates.
            .mclk_multiple  = AP_I2S_MCLK_MULTIPLE,
            // bclk_div only takes effect in SLAVE role (where IDF requires
            // >= 8). We are master, so BCLK comes from the slot geometry:
            // AP_NCH * AP_SLOT_BIT_WIDTH = 256 fs.
            .bclk_div       = 8,
        },
        // 32-BIT DATA, NOT 24. The ring holds 24-bit samples MSB-justified in
        // 32-bit words -- which is exactly what the AoIP wire format gives us,
        // shifted up by 8 and nothing else. Declaring 24-bit data would make
        // the driver expect a different in-memory packing and the output would
        // be quiet garbage. The PCM1690 reads the leading 24 bits of each
        // 32-bit TDM slot, so MSB-justified 32-bit is what it wants.
        .slot_cfg = I2S_TDM_PHILIPS_SLOT_DEFAULT_CONFIG(
                        I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO,
                        I2S_TDM_SLOT0 | I2S_TDM_SLOT1 | I2S_TDM_SLOT2 | I2S_TDM_SLOT3 |
                        I2S_TDM_SLOT4 | I2S_TDM_SLOT5 | I2S_TDM_SLOT6 | I2S_TDM_SLOT7),
        .gpio_cfg = {
            // MCLK not routed: SCKI is BCK, see route_bck_to_scki().
            .mclk = I2S_GPIO_UNUSED,
            .bclk = AP_PIN_I2S_BCLK,
            .ws   = AP_PIN_I2S_WS,
            .dout = AP_PIN_I2S_DOUT,
            .din  = I2S_GPIO_UNUSED,
            .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false },
        },
    };
    tdm_cfg.slot_cfg.slot_bit_width = AP_SLOT_BIT_WIDTH;
    // LRCK: IDF's Philips TDM default, unchanged. That is what the verified
    // ../stm32 UAC firmware uses on this breakout with FMTDA 1000. (An earlier
    // edit here flipped ws_pol from a reading of the datasheet figure; the
    // working reference says the default is right.)
    tdm_cfg.slot_cfg.total_slot     = AP_NCH;

    // *** IF THIS CALL FAILS, READ THIS BEFORE DEBUGGING ANYTHING ELSE ***
    //
    // ESP-IDF issue #14311 reports exactly this configuration -- ESP32-P4,
    // TDM8, 32-bit slots, I2S_CLK_SRC_APLL -- failing with "freq shouldn't be 0,
    // calibration failed". It is closed as "Resolution: NA". The workaround in
    // the thread (switch to I2S_CLK_SRC_DEFAULT) is useless here because the
    // APLL *is* the media clock.
    //
    // DO NOT try 24-bit slots as a workaround, whatever a search suggests: the
    // PCM1690's TDM formats require BCK = 256 fs, and 8 x 24 is 192 fs, which
    // the part does not accept at any sample rate. app_config.h asserts this.
    // Try a different AP_I2S_MCLK_MULTIPLE, then the external-clock backend in
    // mclk_hw.h.
    //
    // If it fails only at 96 kHz, move the rate pin to 48 kHz and confirm the
    // board is otherwise sound before concluding anything -- 96 kHz doubles
    // BCK to 24.576 MHz, which is 1.7% inside the PCM1690's 40 ns tBCY limit.
    esp_err_t err = i2s_channel_init_tdm_mode(s_tx, &tdm_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "TDM init failed at %s: %s -- see the comment above this "
                      "line, and put a scope on SCKI/BCK/LRCK before anything else.",
                 r->label, esp_err_to_name(err));
        return err;
    }

    if (bck_halved(r->hz)) {
        uint32_t was = I2S0.tx_conf.tx_bck_div_num + 1;
        if (was != 4) {
            ESP_LOGE(TAG, "%s: expected the driver's BCK divider 4, found %u",
                     r->label, (unsigned)was);
            return ESP_ERR_INVALID_STATE;
        }
        i2s_ll_tx_set_bck_div_num(&I2S0, 2);
        ESP_LOGW(TAG, "%s: BCK divider 4 -> 2 (APLL as at %u Hz)",
                 r->label, (unsigned)(r->hz / 2));
    }

    // Registered before the channel starts: the first lap of interrupts is how
    // the ring learns its buffer addresses (the DMA starts at descriptor 0).
    jb_attach_dma(chan_cfg.dma_desc_num, chan_cfg.dma_frame_num);
    const i2s_event_callbacks_t cbs = { .on_sent = on_sent };
    i2s_channel_register_event_callback(s_tx, &cbs, NULL);

    // Audio port first (it sets the APLL), then the SCKI generator.
    route_bck_to_scki();

    ESP_LOGI(TAG, "I2S TDM8 @ %s: %d-bit slots, BCK %d fs = %.3f MHz",
             r->label, AP_SLOT_BIT_WIDTH, r->bck_fs,
             r->bck_fs * (double)r->hz / 1e6);
    ESP_LOGI(TAG, "DMA plays the ring: %u x %u frames; lead %u frames / %.2f ms",
             (unsigned)chan_cfg.dma_desc_num, r->dma_frames, r->dma_depth_frames,
             r->dma_depth_frames * 1000.0 / r->hz);
    return ESP_OK;
}

// NOT an audio pump any more: the DMA plays the ring by itself. This is the
// media clock's service loop (10 Hz) and the output meter, with no deadline --
// a late wake-up costs nothing but a late servo update.
static void audio_task(void *arg)
{
    (void)arg;
    const TickType_t period = pdMS_TO_TICKS(1000 / AP_MCLK_UPDATE_HZ);
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last, period);
        mclk_tick();

        // Output meter: the buffer the DMA is playing now. Signal here and
        // silence from the speaker means the DAC side (control port, wiring,
        // power, AMUTEI).
        uint32_t pk[AP_NCH];
        jb_peek_peaks(pk);
        for (int c = 0; c < AP_NCH; c++) if (pk[c] > s_peak_out[c]) s_peak_out[c] = pk[c];
    }
}

esp_err_t audio_out_start(void)
{
    // SCKI before BCK/LRCK: the PCM1690 wants a stable system clock before it
    // will accept a control write, and it re-synchronises on the SCKI/LRCK/BCK
    // relationship when the audio clocks appear.
    ESP_ERROR_CHECK(i2s_channel_enable(s_tx));

    pcm1690_start();

    BaseType_t ok = xTaskCreatePinnedToCore(audio_task, "audio", 4096, NULL,
                                            AP_PRIO_I2S, NULL, AP_CORE_AUDIO);
    return ok == pdPASS ? ESP_OK : ESP_FAIL;
}
