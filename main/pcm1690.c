#include "pcm1690.h"
#include "rate.h"
#include "app_config.h"
#include "driver/spi_master.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "pcm1690";

// Register map, PCM1690 datasheet SBAS448B Table 11. Only what is needed to
// get 8 channels of TDM audio out at 0 dB; everything else stays at reset.
//
// An earlier draft had mute at 72 and attenuation at 70..77, which is off by
// four: unmuting wrote 0x00 into ch1's attenuator (infinite attenuation), and
// "0 dB" wrote 0xFF into register 70 -- 32 kHz de-emphasis, wide-range
// attenuation, inverted zero flags -- and into reserved register 71.
#define REG_MODE_CONTROL    64      // 0x40  MRST SRST AMUTE[3:0] SRDA[1:0]
#define REG_FORMAT          65      // 0x41  PSMDA, FMTDA[3:0]
#define REG_SOFT_MUTE       68      // 0x44  MUTDA[8:1], 1 = muted
#define REG_ATT_BASE        72      // 0x48  ATDA1..ATDA8 at 72..79, 0xFF = 0 dB

// FMTDA[3:0] in register 0x41, from the PCM1690 datasheet (SBAS448B):
//
//   0110  24-bit I2S mode TDM               48 kHz, SCKI 256/512 fs, BCK 256 fs, DIN1
//   0111  24-bit left-justified mode TDM
//   1000  24-bit high-speed I2S mode TDM    96 kHz, SCKI 256 fs,     BCK 256 fs, DIN1
//   1001  24-bit high-speed LJ mode TDM
//
// rate.c picks the right one for the selected rate.

// ---------------------------------------------------------------------------
// Control port: I2C or SPI, DETECTED, because the board decides.
//
// The part's MODE pin (24) selects the port -- VDD: SPI, GND: I2C, open:
// hardware mode -- and a DAC board straps it, usually without saying which.
// The same three wires serve both (SBAS448B pin table):
//
//   board label   PCM1690 pin        SPI role   I2C role
//   CS            22 MS/ADR0         MS (CS)    ADR0
//   SCL           21 MC/SCL          MC         SCL
//   SDA           20 MD/SDA          MD         SDA
//
// Probe I2C first WITH CS HELD HIGH. That is what makes the probe harmless on
// an SPI-strapped part: with MS high the SPI port ignores MC/MD entirely, so a
// failed probe cannot latch a half-written word. On an I2C part the same high
// level is ADR0 = 1, so the address is 1001 1 ADR1 1 = 0x4D or 0x4F (ADR1 is
// the TEST pin, whatever the board did with it).
//
// I2C is the better outcome: it can READ BACK, so the format register is
// verified instead of assumed. SPI on this part is write-only.
// ---------------------------------------------------------------------------

static enum { BUS_NONE, BUS_I2C, BUS_SPI } s_bus;
static i2c_master_bus_handle_t s_i2c_bus;
static i2c_master_dev_handle_t s_i2c;
static uint8_t                 s_i2c_addr;

static esp_err_t wr(uint8_t reg, uint8_t val)
{
    if (s_bus == BUS_I2C) {
        uint8_t b[2] = { reg, val };
        return i2c_master_transmit(s_i2c, b, 2, 50);
    }
    if (s_bus == BUS_SPI) {
        // BIT-BANGED, so the three pins can be PARKED at fixed levels between
        // writes (see park()). 16 bits MSB first: '0' (write), ADR[6:0],
        // D[7:0] (Figure 25); MD is latched on MC's rising edge, the word on
        // MS's rising edge. A few writes at boot: speed is irrelevant.
        uint16_t word = (uint16_t)((reg & 0x7F) << 8) | val;
        gpio_set_level(AP_PIN_DAC_SPI_CS, 1);           // abort any open frame
        gpio_set_level(AP_PIN_DAC_SPI_CLK, 0);
        esp_rom_delay_us(2);
        gpio_set_level(AP_PIN_DAC_SPI_CS, 0);
        esp_rom_delay_us(2);
        for (int b = 15; b >= 0; b--) {
            gpio_set_level(AP_PIN_DAC_SPI_MOSI, (word >> b) & 1);
            esp_rom_delay_us(2);
            gpio_set_level(AP_PIN_DAC_SPI_CLK, 1);
            esp_rom_delay_us(2);
            gpio_set_level(AP_PIN_DAC_SPI_CLK, 0);
        }
        esp_rom_delay_us(2);
        gpio_set_level(AP_PIN_DAC_SPI_CS, 1);           // latch
        esp_rom_delay_us(2);
        return ESP_OK;
    }
    return ESP_ERR_INVALID_STATE;
}

static esp_err_t rd(uint8_t reg, uint8_t *val)
{
    if (s_bus != BUS_I2C) return ESP_ERR_NOT_SUPPORTED;
    return i2c_master_transmit_receive(s_i2c, &reg, 1, val, 1, 50);
}

static bool try_i2c(void)
{
    gpio_config_t cs = { .pin_bit_mask = 1ULL << AP_PIN_DAC_SPI_CS, .mode = GPIO_MODE_OUTPUT };
    gpio_config(&cs);
    gpio_set_level(AP_PIN_DAC_SPI_CS, 1);             // SPI deselected / ADR0 = 1

    i2c_master_bus_config_t bus = {
        .i2c_port = -1,
        .sda_io_num = AP_PIN_DAC_SPI_MOSI,
        .scl_io_num = AP_PIN_DAC_SPI_CLK,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        // Internal pull-ups are weak (~45k); enough to probe at 100 kHz on a
        // short wire, and a board labelled SDA/SCL normally fits its own.
        .flags.enable_internal_pullup = true,
    };
    if (i2c_new_master_bus(&bus, &s_i2c_bus) != ESP_OK) return false;

    static const uint8_t addrs[] = { 0x4D, 0x4F, 0x4C, 0x4E };
    for (size_t i = 0; i < sizeof(addrs); i++) {
        if (i2c_master_probe(s_i2c_bus, addrs[i], 20) == ESP_OK) {
            s_i2c_addr = addrs[i];
            i2c_device_config_t dev = {
                .dev_addr_length = I2C_ADDR_BIT_LEN_7,
                .device_address  = s_i2c_addr,
                .scl_speed_hz    = 100000,
            };
            if (i2c_master_bus_add_device(s_i2c_bus, &dev, &s_i2c) == ESP_OK) return true;
        }
    }
    i2c_del_master_bus(s_i2c_bus);
    s_i2c_bus = NULL;
    gpio_reset_pin(AP_PIN_DAC_SPI_MOSI);
    gpio_reset_pin(AP_PIN_DAC_SPI_CLK);
    gpio_reset_pin(AP_PIN_DAC_SPI_CS);
    return false;
}

static esp_err_t init_spi(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << AP_PIN_DAC_SPI_CS) | (1ULL << AP_PIN_DAC_SPI_CLK) |
                        (1ULL << AP_PIN_DAC_SPI_MOSI),
        .mode = GPIO_MODE_OUTPUT,
    };
    esp_err_t err = gpio_config(&io);
    gpio_set_level(AP_PIN_DAC_SPI_CS, 1);
    gpio_set_level(AP_PIN_DAC_SPI_CLK, 0);
    return err;
}

// PARK the control pins so the DAC runs correctly in SPI mode OR in parallel
// HARDWARE mode (MODE pin open) -- a breakout straps MODE without saying so,
// and SPI cannot be read back to tell which. SBAS448B 7.4.5, Tables 8/9:
//
//   pin            hardware mode meaning          parked   SPI mode effect
//   SCL = MC/FMT   high: 24-bit I2S TDM           HIGH     none -- no clock edges
//   SDA = MD/DEMP  high: 44.1 kHz de-emphasis     LOW      none
//   CS  = MS/RSV   reserved, "set low"            LOW      frame opened, never clocked
//
// First bench run: DVS audio reached the I2S pins at -5 dBFS on both
// subscribed channels and the DAC was silent. With the SPI driver the clock
// idled LOW, which in hardware mode selects plain stereo I2S -- the wrong
// format for a 256-BCK TDM frame.
static void park(void)
{
    gpio_set_level(AP_PIN_DAC_SPI_MOSI, 0);
    gpio_set_level(AP_PIN_DAC_SPI_CLK, 1);
    gpio_set_level(AP_PIN_DAC_SPI_CS, 0);
}

// THE USB PADS. On the ESP32-P4, GPIO24/25 are the USB-Serial/JTAG PHY pads
// and GPIO26/27 the USB 1.1 OTG PHY pads -- exactly this DAC's CS, SCL, SDA and
// RST. USB_SERIAL_JTAG.conf0.usb_pad_enable resets to 1, so the PHY owns those
// pads and gpio_config() alone does not take them back. RST in particular:
// the PCM1690 has an internal pull-down on RST, so an undriven RST holds the
// DAC in reset -- silent, whatever arrives on DIN1.
#include "soc/usb_serial_jtag_struct.h"
#include "soc/usb_wrap_struct.h"

static void release_usb_pads(void)
{
    USB_SERIAL_JTAG.conf0.usb_pad_enable = 0;
    USB_WRAP.otg_conf.usb_pad_enable = 0;
}

// Sample each DAC pin and report the fraction of samples that read high:
// ~50% is a running clock, 0/100% a static level. A pin that reads the
// opposite of what we drive is owned or loaded by something else.
void pcm1690_pin_check(void)
{
    static const struct { int pin; const char *name; } p[] = {
        { AP_PIN_I2S_MCLK, "MCK " }, { AP_PIN_I2S_BCLK, "BCK " },
        { AP_PIN_I2S_WS,   "LRCK" }, { AP_PIN_I2S_DOUT, "DIN1" },
        { AP_PIN_DAC_SPI_CS, "CS  " }, { AP_PIN_DAC_SPI_CLK, "SCL " },
        { AP_PIN_DAC_SPI_MOSI, "SDA " }, { AP_PIN_DAC_RST, "RST " },
    };
    for (size_t i = 0; i < sizeof(p) / sizeof(p[0]); i++) {
        if (p[i].pin < 0) continue;
        gpio_input_enable(p[i].pin);
        int hi = 0;
        for (int k = 0; k < 2000; k++) hi += gpio_get_level(p[i].pin);
        ESP_LOGI(TAG, "pin check  %s GPIO%-2d  high %3d%%", p[i].name, p[i].pin, hi / 20);
    }
}

esp_err_t pcm1690_init(void)
{
    release_usb_pads();
    if (AP_PIN_DAC_RST >= 0) {
        gpio_config_t io = {
            .pin_bit_mask = 1ULL << AP_PIN_DAC_RST,
            .mode = GPIO_MODE_OUTPUT,
        };
        gpio_config(&io);
        gpio_set_level(AP_PIN_DAC_RST, 0);      // hold in reset until SCKI runs
    }
    return ESP_OK;
}

esp_err_t pcm1690_start(void)
{
    if (AP_PIN_DAC_RST >= 0) {
        vTaskDelay(pdMS_TO_TICKS(5));
        gpio_set_level(AP_PIN_DAC_RST, 1);
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // The control port only answers out of reset, so detection happens here.
    if (try_i2c()) {
        s_bus = BUS_I2C;
        ESP_LOGI(TAG, "control port: I2C at 0x%02x (SDA GPIO%d, SCL GPIO%d, ADR0 = CS high)",
                 s_i2c_addr, AP_PIN_DAC_SPI_MOSI, AP_PIN_DAC_SPI_CLK);
    } else {
        esp_err_t err = init_spi();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "no I2C answer and SPI init failed (%s)", esp_err_to_name(err));
            return err;
        }
        s_bus = BUS_SPI;
        ESP_LOGW(TAG, "control port: no I2C answer at 0x4C-0x4F -> SPI writes, then pins "
                      "parked for hardware mode too (FMT=1 TDM, DEMP=0). Cannot be read back.");
    }

    // Sampling mode is left in AUTO (SRDA = 00): the part selects single or
    // dual rate from the SCKI-to-LRCK ratio, and both of our configurations
    // land on the right one -- 512 fs -> single rate at 48 kHz, 256 fs -> dual
    // rate at 96 kHz. Forcing it would only add a way to get it wrong.
    const rate_profile_t *r = rate_get();
    // Same two writes, same order, as the verified reference driver:
    // format, then reg 64 = MRST/SRST normal, AMUTE off, SRDA auto.
    ESP_ERROR_CHECK_WITHOUT_ABORT(wr(REG_FORMAT, r->pcm1690_fmt));
    ESP_ERROR_CHECK_WITHOUT_ABORT(wr(REG_MODE_CONTROL, 0xC0));
    ESP_ERROR_CHECK_WITHOUT_ABORT(wr(REG_SOFT_MUTE, 0x00));        // all unmuted
    for (uint8_t ch = 0; ch < 8; ch++) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(wr(REG_ATT_BASE + ch, 0xFF)); // 0 dB
    }

    if (s_bus == BUS_SPI) park();
    pcm1690_pin_check();

    uint8_t fmt = 0;
    if (rd(REG_FORMAT, &fmt) == ESP_OK) {
        if ((fmt & 0x0F) == r->pcm1690_fmt) {
            ESP_LOGI(TAG, "format register reads back 0x%02x -- verified", fmt);
        } else {
            ESP_LOGE(TAG, "format register reads back 0x%02x, wrote 0x%02x", fmt, r->pcm1690_fmt);
        }
    }

    ESP_LOGI(TAG, "PCM1690: fmt 0x%02x (%s TDM), %s, 8 ch, 0 dB, SCKI %d fs",
             r->pcm1690_fmt,
             r->pcm1690_fmt == 0x08 ? "24-bit high-speed I2S" : "24-bit I2S",
             r->label, r->scki_fs);
    return ESP_OK;
}

esp_err_t pcm1690_set_mute(bool mute)
{
    esp_err_t err = wr(REG_SOFT_MUTE, mute ? 0xFF : 0x00);
    if (s_bus == BUS_SPI) park();
    return err;
}

esp_err_t pcm1690_set_attenuation(uint8_t ch, uint8_t code)
{
    if (ch >= 8) return ESP_ERR_INVALID_ARG;
    esp_err_t err = wr(REG_ATT_BASE + ch, code);
    if (s_bus == BUS_SPI) park();
    return err;
}
