// Board, stream geometry and servo tunables for the 8-channel AoIP receiver.
//
// Target: Waveshare ESP32-P4-ETH  ->  PCM1690 8-channel DAC over I2S TDM8.
//
// Every number here that was learned on the FPGA bench cites where. Do not
// "tidy" a value without reading the citation first -- several of these look
// arbitrary and are not.

#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <stdint.h>

// ---------------------------------------------------------------------------
// Stream geometry
// ---------------------------------------------------------------------------

#define AP_NCH              8           // PCM1690 = 8 channels, DAC only
#define AP_BITS_PER_SAMPLE  24          // AoIP wire format, MSB-justified BE

// ---------------------------------------------------------------------------
// SAMPLE RATE -- selected by a pin at boot
// ---------------------------------------------------------------------------
//
// One SPDT switch (or a jumper) to AP_PIN_RATE_SEL:
//
//     open / high  ->  48 kHz     (internal pull-up, so an unfitted switch
//                                  gives the conservative bring-up rate)
//     closed / low ->  96 kHz
//
// Read ONCE at boot, before any audio hardware is touched. Flipping it needs a
// power cycle, which is not a limitation worth engineering around: an AoIP
// sample-rate change tears down every subscription and re-anchors the media
// clock anyway, and devices in different clock domains cannot subscribe to
// each other at all.
//
// -DSAMPLE_RATE=96000 still works and PINS the rate, ignoring the switch.
// -DAP_PIN_RATE_SEL=-1 disables the switch and uses AP_RATE_DEFAULT.
//
// WHY THIS IS NOT A AOIP CONTROLLER DROPDOWN: the capability structure DC
// populates that menu from is carried in ARC opcodes 0x1100 / 0x1102, and
// nobody has decoded it -- inferno answers both with 110 and 94 ZERO bytes and
// a "???" comment. The set opcode is unidentified. Our own 0x1101 is device
// latency, so the rate is plausibly a neighbour, but "plausibly" is exactly
// the reasoning that produced channels=0x60000130 and a Controller that
// displayed the device and never sent it a packet. A pin is honest; a guessed
// opcode is not. Plenty of shipping AoIP hardware is fixed-rate for the same
// reason (Ultimo 2x2 is 48 kHz only).
//
// What we DO owe AoIP Controller is the truth about which rate we are at --
// see aoip_arc.c, CommonChannelsDescriptor. Without that, a subscription
// fails as a clock-domain mismatch with no useful explanation.

#define AP_RATE_48K         48000
#define AP_RATE_96K         96000

#ifndef AP_RATE_DEFAULT
# ifdef AP_SAMPLE_RATE
#  define AP_RATE_DEFAULT   AP_SAMPLE_RATE      // -DSAMPLE_RATE pins it
#  define AP_RATE_PINNED    1
# else
#  define AP_RATE_DEFAULT   AP_RATE_48K
# endif
#endif
#ifndef AP_RATE_PINNED
# define AP_RATE_PINNED     0
#endif

#ifndef AP_PIN_RATE_SEL
// GPIO19: confirmed free on the Waveshare ESP32-P4-ETH, and confirmed NOT a
// strapping pin.
//
// BOTH CHECKS MATTER, and the second is the one that bites. This pin is tied
// to GND by a switch and read at boot, which is exactly what a strapping pin
// does -- put the rate switch on one and closing it for 96 kHz would stop the
// board booting at all, presenting as "96 kHz mode is dead" rather than as a
// pin conflict.
//
// ESP32-P4 strapping pins are GPIO34, 35, 36, 37, 38. GPIO35 low at reset
// enters the serial bootloader; GPIO36 = 0 with GPIO35 = 0 is an invalid
// combination with undefined behaviour. Keep the rate switch off all five.
//
// The board's Ethernet occupies 28/29/30/31/34/35/49/50/52 (RMII + MDIO), so
// avoid those too if you move this pin.
#define AP_PIN_RATE_SEL     19      // to GND = 96 kHz; open = 48 kHz
#endif

// ---------------------------------------------------------------------------
// What each rate demands of the PCM1690  (datasheet SBAS448B, Table 5,
// software control, 24-bit)
// ---------------------------------------------------------------------------
//
//   format                        max fS    SCKI        BCK      pins
//   I2S TDM                       48 kHz    256/512 fs  256 fs   DIN1
//   I2S TDM                       96 kHz    128/256 fs  128 fs   DIN1/2  <- two wires
//   high-speed I2S TDM            96 kHz    256 fs      256 fs   DIN1    <- what we use
//   high-speed I2S TDM           192 kHz    128 fs      128 fs   DIN1/2  <- out of scope
//
// THE COINCIDENCE THAT MAKES A BOOT PIN ENOUGH:
//
//     48 kHz: SCKI = 512 fs = 24.576 MHz
//     96 kHz: SCKI = 256 fs = 24.576 MHz
//
// SCKI IS THE SAME FREQUENCY AT BOTH RATES. Only BCK changes (12.288 ->
// 24.576 MHz). So the DAC's system clock is a FIXED 24.576 MHz oscillator that
// never needs reconfiguring, and the PCM1690's automatic sampling-mode
// detection -- which keys on the SCKI:LRCK ratio -- lands on single rate at
// 48 kHz and dual rate at 96 kHz by itself. Nothing to set, nothing to rewire.
//
// That fixed 24.576 MHz comes from a SECOND I2S port used as nothing but an
// MCLK generator (AP_PIN_I2S_MCLK). It cannot come from the audio port's own
// MCLK, because at 96 kHz that would make the bclk divider 1 and the hardware
// will not go below 2. Both ports divide the same APLL, so the SCKI:LRCK ratio
// is exact and constant -- which is the only thing the datasheet demands:
//
//   "does not need a specific phase relationship between the audio interface
//    clocks (LRCK, BCK) and the system clock (SCKI), but does require a
//    specific frequency relationship (ratiometric)"
//
// Keep it that way. If the SCKI-to-LRCK relationship ever shifts by more than
// +-2 BCK the DAC stops, forces its outputs to VCOM and needs 38/fS to
// recover. One APLL cannot drift against itself; two oscillators will.

#define AP_SCKI_HZ          24576000    // fixed, both rates
#define AP_I2S_MCLK_MULTIPLE 1024       // audio port, internal: BCK divider 4, even -> 50 % duty (BCK is also SCKI)

// Slot width is NOT a free choice. The PCM1690's TDM formats require
// BCK = 256 fs, and 8 slots x 32 bits is exactly 256 fs. 24-bit slots give
// 192 fs, which the part rejects at every rate -- so it is not the workaround
// for ESP-IDF issue #14311, however much it looks like one.
#define AP_SLOT_BIT_WIDTH   32
#define AP_BCK_FS           (AP_NCH * AP_SLOT_BIT_WIDTH)    // 256 fs

// ---------------------------------------------------------------------------
// Buffers are sized for the FASTEST rate and under-filled at the slower one
// ---------------------------------------------------------------------------

#define AP_RATE_MAX         AP_RATE_96K

// Blocks per second, held constant across rates so the audio task wakes at the
// same cadence either way: 16 frames at 48 kHz, 32 at 96 kHz.
//
// 16 IS THE FLOOR, not a round number -- the Espressif AES67 work on this chip
// found DMA descriptors below 16 frames unreliable under load. 48 kHz sits
// exactly on that limit.
#define AP_BLOCKS_PER_S     3000
#define AP_DMA_FRAMES_MAX   (AP_RATE_MAX / AP_BLOCKS_PER_S)
#define AP_DMA_DESC_NUM     3

// Frames per packet we REQUEST from the transmitter.
//
// We are the requester on the unicast path, so unlike the FPGA transmitter we
// choose this. It sets both the packet rate and the transmitter's own latency
// floor: a packet cannot exist until fpp samples after its own timestamp
// (FPGA project README.md, "Latency is per-flow, not per-device").
//
// fpp counts FRAMES, so holding the packetisation TIME constant means doubling
// it at 96 kHz. 3000 packets/s either way:
//
//   48 kHz, fpp 16 -> 0.333 ms, 459 B on the wire, 11.0 Mbit/s for 8 ch
//   96 kHz, fpp 32 -> 0.333 ms, 843 B on the wire, 20.2 Mbit/s for 8 ch
//
// Both are a fifth of a 100 Mbit/s link or less. Raise the packet rate only
// after the servo is measured stable.
// (runtime: see rate.h, rate_fpp())

// Channels carried in one flow. AoIP's limit, and what real transmitters
// advertise (nchan=8, FPGA project firmware/mdns.c build_txt_chan).
#define AP_MAX_CH_PER_FLOW  8
#define AP_MAX_FLOWS        4

// ---------------------------------------------------------------------------
// Latency
// ---------------------------------------------------------------------------

// Device playout latency: how far behind the PTP timeline we converge the DAC.
//
// THIS MUST EXCEED THE DMA DEPTH, and by enough to cover network transit.
//
// The playout pointer is placed at (now - latency + dma_depth), so the ring is
// only ever read for samples that have had (latency - dma_depth) to arrive. A
// latency at or below the DMA depth asks the ring for audio from the future
// and underruns on every block, with PTP locked and every other counter
// healthy. AP_LATENCY_US_MIN is therefore derived, not chosen -- an earlier
// version of this file hardcoded 1000 us against 1500 us of DMA, which could
// never have worked.
//
// 2 ms is the target. 1 ms is reachable only by dropping AP_DMA_DESC_NUM to 2,
// and 0.25 ms is a Brooklyn-3-class capability the FPGA earned with a hardware
// packetiser -- this is a software receiver on FreeRTOS and its floor is task
// scheduling, not gate delay.
#define AP_LATENCY_US_DEFAULT   2000
// AP_LATENCY_US_MIN is runtime -- rate_latency_min_us() in rate.h
#define AP_LATENCY_US_MAX       10000

// ---------------------------------------------------------------------------
// Jitter buffer
// ---------------------------------------------------------------------------

// Ring depth in frames. MUST be a power of two.
//
// Fixed at 4096 for both rates -- 85 ms at 48 kHz, 42 ms at 96 kHz, 128 KiB of
// internal SRAM either way. Sizing it for the faster rate and letting it be
// generous at the slower one costs 64 KiB and removes a rate-dependent array
// bound, which is the kind of thing that goes wrong quietly.
//
// It must stay in internal SRAM. PSRAM access latency on the DMA feed path is
// exactly the kind of jitter this design exists to avoid.
#define AP_RING_FRAMES      4096
#define AP_RING_MASK        (AP_RING_FRAMES - 1)

// A packet whose timestamp is further ahead than this is rejected rather than
// written. Without it a single corrupt timestamp scribbles over the whole ring.
#define AP_FUTURE_GUARD_FRAMES  (AP_RING_FRAMES / 2)

// ---------------------------------------------------------------------------
// I2S / DMA
// ---------------------------------------------------------------------------

// Frames per DMA descriptor, and descriptor count.
//
// Sized by TIME so the audio task wakes at 3000 Hz at either sample rate:
// 16 frames at 48 kHz, 32 at 96 kHz, 1/3 ms each way. (16 at 96 kHz, for a
// 1 ms latency minimum, was tried and failed -- rate.c.)
//
// 16 IS THE FLOOR, not a round number. The Espressif AES67 work on this same
// chip found DMA descriptors smaller than 16 frames unreliable under load, so
// 48 kHz sits exactly on that limit and 44.1 kHz would fall below it.
//
// The depth is a CONSTANT offset ahead of the converter, folded into the media
// clock's phase target (media_clock.c) and into AP_LATENCY_US_MIN above.
#define AP_DMA_FRAMES       (AP_SAMPLE_RATE / 3000)
#define AP_DMA_DESC_NUM     3
#define AP_DMA_DEPTH_FRAMES (AP_DMA_FRAMES * AP_DMA_DESC_NUM)
#define AP_DMA_DEPTH_US     ((AP_DMA_DEPTH_FRAMES * 1000000) / AP_SAMPLE_RATE)

// Clock rates that result, for reference when you put a scope on it:
//
//            fS        BCK = 256 fs      SCKI            MCLK pin
//   48 kHz             12.288 MHz        24.576 MHz      yes, 512 fs
//   96 kHz             24.576 MHz        24.576 MHz      NO -- SCKI <- BCK
//
// tBCY (BCK cycle time) minimum is 40 ns. At 96 kHz the period is 40.69 ns --
// legal, with 1.7% of margin, against 100% at 48 kHz. Keep BCK and DIN traces
// short and consider series termination before blaming the software.

// ---------------------------------------------------------------------------
// Pins -- Waveshare ESP32-P4-ETH, 40-pin header
// ---------------------------------------------------------------------------
// The onboard ES8311 sits on GPIO9..13. It is a mono codec and useless for 8
// channels; we leave it alone and put the PCM1690 on header pins. Adjust to
// match how the DAC board is actually wired.

// SCKI is a fixed 24.576 MHz from the auxiliary I2S port at BOTH rates
// (512 fs at 48 kHz, 256 fs at 96 kHz) -- audio_out.c mclk_gen_init().
#define AP_PIN_I2S_MCLK     20      // -> PCM1690 SCKI (pin 14), both rates
#define AP_PIN_I2S_BCLK     21      // -> PCM1690 BCK
#define AP_PIN_I2S_WS       22      // -> PCM1690 LRCK
#define AP_PIN_I2S_DOUT     23      // -> PCM1690 DIN1

// Control port: SPI or I2C, detected at start (pcm1690.c). Board labels in
// quotes are the common PCM1690 breakout's.
#define AP_PIN_DAC_SPI_CS   24      // "CS"  -> MS/ADR0 (pin 22)
#define AP_PIN_DAC_SPI_CLK  25      // "SCL" -> MC/SCL  (pin 21)
#define AP_PIN_DAC_SPI_MOSI 26      // "SDA" -> MD/SDA  (pin 20)
#define AP_PIN_DAC_RST      27      // "RST" -> RST     (pin 15), active low

// Ethernet -- see sdkconfig.defaults. IP101 on RMII, MDC/MDIO below.
#define AP_PIN_ETH_MDC      31
#define AP_PIN_ETH_MDIO     52
#define AP_PIN_ETH_REF_CLK  50      // 50 MHz IN from the board's oscillator
#define AP_PIN_ETH_PHY_RST  -1      // not wired on this board
#define AP_ETH_PHY_ADDR     1

// ---------------------------------------------------------------------------
// Media clock servo
// ---------------------------------------------------------------------------

// Rate authority. DRIFT_HANDOFF.md ran the FPGA phase term against a 2000 ppb
// clamp and it never needed more than 65 ppb once the actuator quantisation
// was removed. Same clamp here.
#define AP_MCLK_PPB_CLAMP       2000

// FEED-FORWARD: the crystal's own error, taken from the PTP servo.
//
// The APLL and the EMAC's PTP clock run from the SAME 40 MHz crystal, so the
// rate the PTP servo settles on (ptp_servo integral, g_ptpv1.rate_ppb) is that
// crystal's error against the Leader -- exactly the correction the audio clock
// needs too. First P4 bench run: +43.5 ppm. Against the 2000 ppb phase clamp
// above that was unreachable, and the ramp at 100 ppb/s would have taken
// seven minutes, so the phase loop alone could never hold the buffer.
//
// Applied directly, NOT slewed: it moves only as fast as the PTP integral does
// (tens of ppb per second once locked), and the slew limit exists for the
// phase term that caused the FPGA's underrun storms. It is not a second
// controller on the buffer -- it never looks at the buffer.
//
// Bounded at 200 ppm: a crystal further off than that is broken, not trimmed.
#define AP_MCLK_FF_PPB_MAX      200000
#define AP_MCLK_TOTAL_PPB_MAX   (AP_MCLK_FF_PPB_MAX + AP_MCLK_PPB_CLAMP)

// Slew limit, ppb per second.
//
// THIS IS THE ONE THAT COST TWO BENCH SESSIONS. MCR_REPLACEMENT.md: both
// earlier attempts stepped the clock at main-loop rate and produced underrun
// storms. The missing constraint was slew rate, not the choice of rate
// estimate. 100 ppb/s is what worked.
#define AP_MCLK_SLEW_PPB_PER_S  100

// Servo update rate (Hz) and gains. PI on playout phase error in frames.
#define AP_MCLK_UPDATE_HZ       10
#define AP_MCLK_KP_NUM          40      // ppb per frame of error
#define AP_MCLK_KI_NUM          3       // ppb per frame per update

// Error beyond this means the buffer is nowhere near where it belongs and the
// servo cannot pull it back in reasonable time -- step the playout pointer
// instead and count it. Every step is an audible discontinuity; if this fires
// in steady state, something upstream is wrong.
#define AP_MCLK_RESET_FRAMES    480     // 10 ms

// ---------------------------------------------------------------------------
// Identity
// ---------------------------------------------------------------------------

#define AP_MANUFACTURER     "bench"
#define AP_MODEL_NAME       "ESP32P4-AOIP-RX-8"
#define AP_DEFAULT_NAME     "N-Series_DAC8"

// ---------------------------------------------------------------------------
// Task layout
// ---------------------------------------------------------------------------
// Core 0: audio (I2S write + jitter buffer) and PTP. Nothing else.
// Core 1: lwIP, mDNS, ARC, flow keepalives, telemetry, console.
//
// This split IS the replacement for the FPGA's "the CPU is never in the
// per-sample path" invariant. It is the only thing standing between the DAC
// and mDNS parsing latency. Do not move a control-plane task to core 0.

#define AP_CORE_AUDIO       0
#define AP_CORE_CONTROL     1
// NOT core 1 for aoip_rx / ptpv1: above lwIP's tcpip thread (18) on the SAME
// core, select() reported a socket readable that tcpip had not finished
// delivering, recvfrom() found nothing, and the loop spun -- tcpip starved,
// task watchdog on IDLE1, audio gone. On core 0 they run alongside tcpip.

#define AP_PRIO_I2S         23
#define AP_PRIO_RX          22
#define AP_PRIO_PTP         21
#define AP_PRIO_CONTROL     5


// ---------------------------------------------------------------------------
// Compile-time sanity
// ---------------------------------------------------------------------------
// The rate-dependent checks moved to rate.c, where the rate is known. These
// are the ones that hold whatever the pin says.

#include <assert.h>

// jitterbuf.c masks with AP_RING_MASK; a non-power-of-two silently corrupts.
static_assert((AP_RING_FRAMES & (AP_RING_FRAMES - 1)) == 0,
              "AP_RING_FRAMES must be a power of two");

// The PCM1690's TDM formats require BCK = 256 fs. 8 x 32 is the only way there.
static_assert(AP_BCK_FS == 256,
              "PCM1690 TDM requires BCK = 256 fs: AP_NCH * AP_SLOT_BIT_WIDTH must be 256");

// The AES67 work on this chip found sub-16-frame DMA descriptors unreliable.
static_assert(AP_RATE_48K / AP_BLOCKS_PER_S >= 16,
              "DMA descriptors below 16 frames are unreliable under load on this chip");

// SCKI is the same at both rates only if these ratios hold. If this fails, the
// whole "one fixed SCKI oscillator" argument above has stopped being true.
static_assert(AP_SCKI_HZ == 512 * AP_RATE_48K && AP_SCKI_HZ == 256 * AP_RATE_96K,
              "SCKI must be 512 fs at 48 kHz and 256 fs at 96 kHz -- the same frequency");

// PCM1690 absolute maximum system clock is 36.864 MHz; tBCY min 40 ns caps BCK
// at 25 MHz.
static_assert(AP_SCKI_HZ <= 36864000, "SCKI exceeds the PCM1690 maximum");
static_assert((uint64_t)AP_BCK_FS * AP_RATE_MAX <= 25000000ULL,
              "BCK exceeds the PCM1690's 40 ns minimum cycle time");

#endif // APP_CONFIG_H
