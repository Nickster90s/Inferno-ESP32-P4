# AoIP → 8-channel DAC on an ESP32-P4

A native AoIP **receiver** for the Waveshare ESP32-P4-ETH, playing out to the
PCM1690 breakout (`../../stm32/pcm1690/pcm1690-breakout`) over one-wire I2S
TDM. It shows up in AoIP Controller as an 8-channel receive-only device, locks
to the network's PTPv1 leader, requests unicast flows from real transmitters,
and disciplines its conversion clock to the network.

**Board:** Waveshare ESP32-P4-ETH (ESP32-P4, IP101 PHY, 100 Mbit/s, CH343 USB-UART)
**DAC:** PCM1690 breakout, 24-bit, 8 line outputs on 4 × 3.5 mm jacks
**Rate:** 48 kHz (see [Why 48 kHz](#why-48-khz))
**Toolchain:** ESP-IDF v5.5

## Status: working end to end

Verified on the bench on 2026-09-24, against a RedNet AM2 (PTP leader) and a
virtual soundcard (DVS) on a Mac as the transmitter:

| | |
|---|---|
| discovery | appears in AoIP Controller with name, model, IP, 8 RX channels |
| clock | PTPv1 locked, Sync **green about 15 s after power-on** |
| subscription | patch goes **green**; restored from flash after a reboot |
| patching | adding or removing a channel causes **0 underruns** on channels already playing |
| audio | DVS → P4 → PCM1690 → jack, heard on J5 |
| long run | see [Measured](#measured); a soak test after the transmit fix below is in progress |

Open issues are listed under [Known issues](#known-issues).

## Thanks to Inferno

**This project would not exist without [Inferno](https://github.com/teodly/inferno)**
by Teo ([teodly](https://github.com/teodly), also on
[GitLab](https://gitlab.com/lumifaza/inferno)), the open-source AoIP
implementation for Linux. Almost everything a device has to say on this network
was learned from reading it:

- the request/response framing and the flow-control protocol: how to request a
  flow, and the `13 37` keepalive that keeps it alive
- the ARC opcodes and reply layouts: channel counts, device names, receive
  channels, subscriptions
- the CMC device advertisement, the info multicasts and the heartbeat
- the mDNS records a device has to publish, and the channel-change event that
  lets a patch turn green

Years of careful reverse-engineering are collected in that codebase and
generously published in the open. Inferno was used here as a specification, to
read and learn from; the firmware in this repository is an independent
implementation for a microcontroller. **A huge thank you to Teo and everyone who
contributed to Inferno.**

## Legal

Not affiliated with, authorized by or approved by any AoIP protocol vendor. The
protocol is undocumented; everything here comes from public reverse-engineering
work and from captures on this bench. `inferno` is used only as a
**specification to read**; no code from it is copied. Bench and research use.

---

## Wiring

The breakout's J1 header, straight through except for the pin choice on the
ESP32 side. **No jumper between BCK and MCK is needed**: the firmware routes
the bit clock out a second time on GPIO20 (see [Clocking](#clocking)).

| J1 pin | DAC signal | ESP32-P4 |
|--:|---|---|
| 9 | MCK (SCKI) | GPIO20 — **the BCK signal, routed twice** |
| 5 | BCK | GPIO21 |
| 3 | LRCK | GPIO22 |
| 7 | DIN1 | GPIO23 — all 8 channels |
| 14 | CS / MS | GPIO24 |
| 13 | SCL / MC | GPIO25 |
| 12 | SDA / MD | GPIO26 |
| 11 | RST | GPIO27 |
| 2, 16 | +5 V | 5 V (the breakout makes its own 3.3 V) |
| 1, 4, 6, 8, 10, 15 | GND | GND |

Outputs: `CH1/CH2 → J5`, `CH3/CH4 → J6`, `CH5/CH6 → J7`, `CH7/CH8 → J8`
(odd = tip, even = ring). They are **line level** (~1.4 Vrms, ~150 Ω), not
headphone outputs.

Other pins: **GPIO19** selects the rate at boot (open = 48 kHz). The board's
Ethernet uses 28–31, 34, 35, 49, 50, 52.

**GPIO24–27 are the P4's USB PHY pads.** `USB_SERIAL_JTAG.conf0.usb_pad_enable`
resets to 1, so the PHY owns them until it is switched off; `pcm1690.c` clears
it first. Without that, RST never reaches the DAC and its internal pull-down
holds it in reset.

## Build, flash, watch

```bash
. ~/esp/esp-idf/export.sh
idf.py build
idf.py -p /dev/ttyACM0 flash
```

Console is UART at 115200. On this board, **opening the serial port can reset
the chip** (the CH343's DTR/RTS drive EN and the boot strap), so read logs with
a tool that leaves those lines alone. For anything running, prefer the network
telemetry:

```bash
tools/stats.py 169.254.3.29            # one snapshot, key=value
tools/stats.py 169.254.3.29 watch 120  # follow, and report drift at the end
```

Subscriptions are stored in NVS and come back on their own after a reboot.

---

## Architecture

```
 network                  ESP32-P4                                     PCM1690
 --------  +------------------- core 1 -------------------+
 mDNS      |  mDNS · ARC 4440 · CMC 8800 · info 8700      |
 ARC/CMC ->|  info multicast + 1 Hz heartbeat · telemetry |
 info      |  subscriber: resolve -> flow request         |
           +----------------------+-----------------------+
                                  | binds flows
 audio     +------------------- core 0 -------------------+
 4321+  -->|  aoip_rx --> jitterbuf --> audio_out ---------|-- TDM8 --> 8 ch
           |  (UDP, keepalive)   (timestamp-    (blocking |   BCK = SCKI
 PTPv1  -->|  ptpv1 (HW stamps)   indexed)   I2S write)   |   12.288 MHz
 319/320   |        |                ^            |       |
           |        +-- rate ff --> mclk <--------+       |
           +--------------------------|-------------------+
                                APLL trim (sigma-delta)
```

Three properties hold the design together:

- **The CPU is not in the per-sample path; DMA is.** The audio task blocks in
  `i2s_channel_write()`, and that block *is* the pacing.
- **Core 0 is audio and PTP only.** Control-plane parsing never delays a DMA block.
- **One controller per buffer.** The media clock's phase loop is the only thing
  that acts on the buffer. The feed-forward term (below) is a rate, taken from
  the PTP servo, and never looks at the buffer.

### Transmit: one descriptor ring, two cores

**`CONFIG_ETH_TRANSMIT_MUTEX=y` is required** (`sdkconfig.defaults`). The PTP
task on core 0 sends Delay_Req with `esp_eth_transmit_ctrl_vargs()` to get a
hardware TX timestamp. lwIP transmits from its own thread on core 1. IDF
leaves the transmit mutex **off** by default, so without it both cores write
the EMAC's TX descriptor ring at the same time.

On the bench that failed slowly rather than at once. Each collision spoiled
one of the 10 TX descriptors, so the board lost sending capacity over about
70 minutes:

| since boot | symptom |
|---|---|
| minutes | stats replies slow (> 0.4 s) and about half time out; unicast ARP probes unanswered |
| ~0–41 min | 7 audio flow drops of ~3 s each (keepalives not getting out) |
| ~68 min | nothing can be sent: no ping, no heartbeat, flow requests fail in 1 ms, **audio gone**. Receive still works (PTP Sync keeps counting) |

With the mutex on, stats replies come back in ≤ 0.02 s with no timeouts, and
the keepalive counters (`rx_ka_sent`, `rx_ka_failed`) show 0 failures.

### Receive: timestamp only what PTP needs

IDF's PTP setup stamps only PTPv2-over-Ethernet frames, so PTPv1 needs its own
classifier setting. The first version set **`en_ts4all`** (timestamp every
frame, 1,600+ audio packets a second included). Under the bursts of a patch
change, the MAC's RX FIFO read controller then **hung while transferring a
frame's timestamp**:

| `EMACDEBUG` | healthy | dead |
|---|---|---|
| RX FIFO read controller | idle | stuck in "reading frame status / timestamp" |
| RX FIFO fill level | empty | **full** |

From then on every frame was dropped before the DMA. Receive was dead for good,
transmit still worked, and the DMA looked healthy ("waiting for packets", free
descriptors). No EMAC restart and no PHY reset recovered it.

`eth_ts.c` now stamps **only PTPv1 event messages over UDP/IPv4**
(`en_proc_ptp_ipv4_udp`, version-1 format, event messages, slave role): a few
frames a second. Measured afterwards: 18 subscribe/unsubscribe changes in a row
with 0 failed requests, correct status after every step, and PTP locked
throughout with `no_hw_ts = 0`.

Two related settings, both kept:

- **802.3x flow control is off.** With the MAC's hardware flow control, a
  receive stall keeps the switch paused, which turns a hiccup into a deadlock.
- **Receive watchdog** (`eth_ts.c`): if the link is up and no frame arrives for
  1 s (PTP alone brings ~8/s), it wakes IDF's `emac_rx` task and pokes the RX
  poll demand; then restarts the EMAC; then soft-resets the PHY. Counters:
  `eth_rx_kicks`, `eth_rx_restarts`, `eth_phy_resets`. It is a safety net, and
  it did **not** recover the timestamp hang; the fix above is what matters.

## Clocking

**SCKI = BCK = 256 fS = 12.288 MHz, format `FMTDA = 1000`** (24-bit high-speed
I2S TDM), eight 32-bit slots on DIN1. This is the configuration verified on
this breakout by `../../stm32/ESP32-P4/p4-uac-pcm1690`:

1. IDF's TDM driver forces `mclk_multiple >= 768 fS`, so the P4's own MCLK
   output can never be the 256/512 fS the PCM1690 accepts for TDM.
2. High-speed TDM wants SCKI = BCK = 256 fS, and the datasheet only asks for a
   frequency relationship between SCKI and BCK/LRCK, not a phase one (§7.4.3).
   So the bit clock is routed to the MCK pin too (`route_bck_to_scki()`).
3. SCKI needs a 40–60 % duty cycle (§6.8). `mclk_multiple = 1024` makes the
   BCK divider 4, which is even, so the duty cycle is exactly 50 %.

What this replaced: a second I2S port emitting a nominal 24.576 MHz "SCKI".
It measured 33 % duty (an odd divider), and the DAC stayed silent while audio
reached DIN1 at −5 dBFS.

### The media clock

The APLL is the conversion clock, and it is trimmed through its sigma-delta
fraction (`mclk_hw.c`). One LSB is ~1.55 ppm, so the fraction is dithered at
1 kHz to make the mean exact. Only the DSDM bytes that changed are written:
rewriting the whole APLL config from a 1 kHz ISR once ran the DAC 0.84 % slow
and cost a third of the incoming packets.

The trim is **feed-forward plus phase**:

- **Feed-forward**: the APLL and the PTP clock share one 40 MHz crystal, so the
  rate the PTP servo settles on *is* the crystal error (+41 ppm on this board).
  It is applied directly once PTP locks.
- **Phase loop**: a slewed PI on playout phase error. It is **off by default**
  (console `a 1` arms it). Disarmed, playout is rate-matched but carries a
  constant phase offset from where it was anchored.

## PTP

PTPv1, slave-only, hardware timestamps from the EMAC. IDF's PTP setup stamps
only PTPv2-over-Ethernet, so `eth_ts.c` sets `en_ts4all`, or no PTPv1 frame
would ever be stamped.

Acquisition is staged, each stage as early as it can run:

1. **Starts on link-up, not on IP.** Sync reception, stepping and the frequency
   estimate need no address. Link-local addressing takes ~10 s and used to
   delay all of it.
2. **Step** on the first Sync pair.
3. **Frequency estimate**: a least-squares slope of the raw `t2 − t1` over 4 s.
   The path delay is constant, so it drops out of the slope.
4. **Path delay**: Delay_Req at 4 Hz until known. The Leader answers by
   multicast and identifies us by clock ID, so no IP is needed. Samples taken
   before the frequency fix are discarded: they were biased by up to 7 µs.
5. **Phase snap**: one small step removes whatever phase built up meanwhile.
6. **Servo**: acquisition gains until locked, then locked gains. Those are
   stronger than the FPGA's settled-crystal tuning, because this board's
   crystal warms up after power-on and walks several ppm during the first
   minute.

On a step, the P term is dropped and the servo falls back to its rate estimate.
Keeping the saturated P term once deadlocked the servo at −200 ppm.

### Outliers and phase shifts (the old "71 s step")

When an audio flow starts, the Leader-to-us delay drops by **~36 µs and stays
there**; the path is faster with traffic on the port than when it is idle. It
is not EEE (the switch does not advertise it, and EEE is off on our side
anyway) and not a second clock (every Sync comes from the Leader). The physical
cause is unknown; the servo has to handle it either way.

The old servo mishandled it twice:

1. Outliers went **into** the 7-sample median, so four of them flipped it, and
   the shift was then accepted as truth.
2. It answered a **phase** shift with a **frequency** kick. That overshot for
   ~8 s and dropped lock. Worse, one such kick could leave the output ~20 ppm
   off, and every following sample then "looked like an outlier" against the
   lagging median. The servo froze on the bad rate, the offset ramped 20 µs/s,
   and 50 s later it crossed 1 ms and stepped. That was the 1 ms step **~71 s
   after every boot**.

Now, while locked:

| samples far (> 10 µs) from the last accepted value | action |
|---|---|
| 4 in a row that agree within 5 µs | a real **phase shift**: one small step, rate and lock kept, path delay re-measured at 4 Hz |
| up to 8 in a row, scattered | spikes: held out of the median, the rate is held |
| more than 8 | real movement: reseed the median and servo on it |

A step under 100 µs (< 5 samples) does **not** re-anchor the audio; it is
absorbed as phase error instead of making a click. Measured after the fix: lock
at 14.5 s, the flow-start shift corrected in ~1 s with lock kept, no lock
losses and no steps after boot (`ptp_phase_shifts` counts these).

## Control plane

What a receiver has to answer before the controller will even show it:

| port | what | notes |
|---|---|---|
| mDNS | `_netaudio-arc`, `_netaudio-cmc` | `id=` is the EUI-64 of the MAC and must match everywhere else |
| 8800 | CMC `0x1001` | the controller's **first** unicast; unanswered, it never sends ARC |
| 8700 → 224.0.0.231:8702 | board, product, network, clock, caps | announced every 3 s; network info carries the IP and gates routing; clock info makes us a v1 Follower of the right Leader |
| → 224.0.0.233:8708 | 1 Hz heartbeat | clock, sync quality, meters, latency blocks |
| 4440 | ARC | channel counts, names, RX channels (paged, 20-byte items), property tables, subscriptions, latency |
| 224.0.0.231:8702 | `0x0102` channel change | sent whenever a channel's status changes; without it a patch stays orange |

Receiving a flow:

- **Resolve** `<channel>@<device>` over mDNS, walking all addresses for IPv4.
  Results are **cached per channel**, so a patch looks up only the channel that
  changed (a lookup can take up to 3 s). Pending subscriptions are retried
  every 5 s, even while other flows play.
- **Request** with flow-control opcode `0x0100` to the transmitter's 4455:
  one flow per transmitter, carrying every channel subscribed to it.
- **Keepalive**: `13 37`, every 250 ms, **from the audio socket to the audio
  source address**. Opcode `0x0102` is not a keepalive. A transmitter drops a
  flow after ~4 s without the real one. Every `sendto()` is checked and
  counted (`rx_ka_sent` / `rx_ka_failed` / `rx_ka_errno`); a keepalive that
  silently fails to send is invisible otherwise.
- **Liveness**: a flow whose packet count stops for 3 s is torn down and
  re-requested.
- **Status must be current**: a rebuild works on a snapshot and can take
  seconds. It only marks a channel active or pending if that channel's
  subscription is still the one it worked on; otherwise an unsubscribe made
  mid-rebuild "came back" in the controller.
- **If a transmitter refuses the replacement flow** (the virtual soundcard
  sometimes answers `0x0301` to a second, overlapping flow), the old flow is
  stopped first and the request repeated: a short gap instead of a stuck patch
  (`flow_fallbacks`).
- **Exact packet size**: a packet must be exactly `fpp × channels × 3` bytes,
  or it is dropped (`rx_wrong_len`). Checking only divisibility would let a
  packet in a flow's previous layout be misread during a change.

### Patching without dropouts

`subscriber.c` reconciles the flows it **has** against the flows it **wants**
instead of rebuilding everything:

| situation | action |
|---|---|
| a flow's channel list is unchanged | left alone |
| a flow's channel list changed | **make-before-break**: request a new flow with the new list while the old one keeps playing; stop the old one only once the new one delivers packets |
| a transmitter is no longer wanted | its flow is stopped |
| a new transmitter | a new flow is requested |

While both flows run they write identical, timestamped samples into the same
jitter buffer, so the overlap is inaudible. Measured: patching and un-patching
a third channel caused 0 underruns on the two channels already playing.

The obvious alternative, opcode `0x0102` "update flow" (change the channel
list in place), was tried first. The virtual soundcard acknowledged it and
changed the packet layout, but sent **silence in the added slots**, so it is
not used. `flows_client_update()` stays in the code for transmitters where it
may work.

The old behaviour tore down every flow, re-resolved every channel and
re-requested everything, so each patch silenced all channels for seconds.

Each flow slot is needed only briefly during a change, so with
`AP_MAX_FLOWS = 4` a change still needs one free slot. If all four are busy,
the old flow keeps playing and the new channel stays pending.

Most of the control-plane layouts come from the FPGA project (`../FPGA`),
which was brought up against the same controller, an AM2, an A16R and DVS.
The AM2's replies were replayed and diffed on this bench.

## Why 48 kHz

With SCKI = BCK, the APLL has to run at 2 × 1024 × fS: 98.3 MHz at 48 kHz, but
196.6 MHz at 96 kHz, against a 125 MHz ceiling. The verified reference
firmware documents the same limit. `rate.c` refuses 96 kHz at boot with a
message rather than producing a wrong clock.

## Measured

Bench, 2026-09-24, board up for 100 minutes with audio playing, then a 5-minute
logged window (telemetry every 5 s):

| | result |
|---|---|
| playout phase drift | error **250–254 frames for the whole 100 min**: under 4 frames, i.e. **< ~15 ppb** net drift against the network clock |
| conversion rate | 48 001.9 frames/s by the local crystal = +40.5 ppm, matching the +40.9 ppm feed-forward (the crystal runs ~41 ppm slow) |
| buffer | **0 underruns, 0 late packets**, 0 playout steps in the window |
| flow | 1 500.06 packets/s (fpp 32), liveness 1 933 ok / 0 lost |
| PTP offset | typically ±0.5 µs; outliers to −6.5 µs and +17.4 µs, one brief lock drop |
| PTP steps | 4 in 100 min (boot step, phase snap, and the ~71 s event, since fixed; see [Outliers and phase shifts](#outliers-and-phase-shifts-the-old-71-s-step)) |

The PTP outliers do not reach the audio. The DAC clock follows the servo's
integral (the rate estimate), not the offset, and the playout phase stayed
within ±2 frames through them. They are consistent with switch queueing, which
PTPv1 cannot correct because it has no correctionField.

## Telemetry

`tools/stats.py` asks UDP 7779 for a `key=value` snapshot; parse it by name.
The fields that matter most:

| key | meaning |
|---|---|
| `ptp_locked`, `ptp_offset_ns`, `ptp_path_delay_ns` | PTP state |
| `ptp_rate_ppb` | servo integral, i.e. the crystal error |
| `ptp_steps`, `ptp_no_hw_ts`, `ptp_path_rejected` | should stay flat after boot |
| `ptp_phase_shifts` | small phase corrections while locked (one per flow start is normal) |
| `mclk_ppb_ff` | feed-forward applied to the APLL |
| `mclk_error_frames`, `mclk_reset_steps` | playout phase; steps are audible |
| `jb_underrun_frames`, `jb_pkt_late` | buffer health (underruns also count silence before a flow exists) |
| `rx_packets`, `flow_ka_ok/lost` | flow health (the `ka` counters are the liveness checks) |
| `rx_ka_sent`, `rx_ka_failed`, `rx_ka_errno` | flow keepalives sent / refused by the stack; failures mean transmit trouble |
| `fc_last_op`, `fc_last_code`, `fc_refused`, `fc_timeouts`, `flow_fallbacks` | flow-control requests: last opcode and the transmitter's answer (`0xFFFF` no reply, `0xFFFE` could not send) |
| `eth_rx_kicks`, `eth_rx_restarts`, `eth_phy_resets` | receive watchdog actions (should stay 0) |
| `reset_reason` | why the board last started: 1 power-on, 3 software, 4 panic, 5–7 watchdog, 9 brown-out |
| `peak_in_dbfs`, `peak_out_dbfs` | per-channel level from the network and to the DAC, since last read |

The telemetry **stream** (UDP 7778, sent to whoever last queried 7779) carries
one binary record per Sync (`a` filtered offset, `b` raw offset, `c` ppb,
`d` path delay), plus media-clock and flow events. It is the non-blocking way
to see what PTP is doing sample by sample.

`peak_in` against `peak_out` is the quickest fault split: signal in both and
silence at the jack means the DAC side.

The console's `s` status also prints heap (internal / DMA), the `emac_rx`
task state, the RX DMA state with missed / FIFO-overflow counts, and the raw MAC
config, frame-filter, debug and address-filter registers. That is how the
receive hang above was found. **Opening the serial port can hold this board in
reset**; send `s`, read, and close again rather than leaving it open.

Console (`?` for help): `s` status, `a 0/1` phase loop, `l <us>` latency,
`r` re-anchor, `n <name>` rename, `k` refresh subscriptions, `m 0/1` mute.

## Files

| file | role |
|---|---|
| `app_config.h` | pins and every tunable, with the reason for each |
| `rate.[ch]` | boot-pin rate profile |
| `aoip_wire.h` | wire formats |
| `eth_ts.[ch]` | EMAC bring-up, hardware timestamps, multicast filter |
| `ptpv1.[ch]`, `ptp_servo.[ch]` | PTPv1 slave, staged acquisition, PI servo |
| `media_clock.[ch]`, `mclk_hw.[ch]` | playout phase, feed-forward, APLL trim |
| `jitterbuf.[ch]` | timestamp-indexed ring, 64-bit indices |
| `aoip_rx.[ch]` | audio sockets, keepalive, input meters |
| `audio_out.[ch]` | I2S TDM8, SCKI = BCK routing, output meters |
| `pcm1690.[ch]` | DAC control (SPI, I2C detected), USB pad release, pin check |
| `chan_resolve.[ch]`, `flows_client.[ch]`, `subscriber.[ch]` | subscriptions → flows, persisted in NVS |
| `aoip_arc.[ch]` | ARC server |
| `aoip_info.[ch]` | device id, CMC, info multicast, heartbeat, channel-change events |
| `aoip_mdns.[ch]` | our advertisement |
| `aoip_msg.[ch]` | request/response builder (absolute string offsets) |
| `telem.[ch]`, `console.[ch]` | telemetry and console |
| `tools/stats.py`, `tools/arc_probe.py` | snapshot/drift, raw ARC probe |

## Known issues

1. **Long-run confirmation of the transmit fix is pending.** Before the fix the
   board lost all transmit after ~68 minutes; a 90-minute soak is running.
2. **Sync can blink red for ~5 s after a phase-shift correction** (seen at 35 s
   and 201 s after boot). The audio is unaffected, since the DAC clock follows
   the rate, not the offset. Likely the re-measured path delay moving the
   offset past the 5 µs unlock threshold; not yet fixed.
3. **The ~36 µs shift in path delay when a flow starts is unexplained.** The
   servo now absorbs it. A side effect: PTPv1 assumes a symmetric path, so if
   the change is only in the Leader-to-us direction, our clock sits a
   constant ~18 µs off the Leader. That is harmless for playout, but it is a
   real offset against other devices.
4. **The phase loop is disarmed by default**, so playout sits a constant few
   milliseconds away from the target latency.
5. **Flow drops under heavy console logging.** Each log line blocks a core-0
   task for ~10 ms of UART time. With per-sample PTP tracing on, the flow was
   lost six times in two minutes, so tracing is compiled out
   (`AP_PTP_TRACE=0`). Use the telemetry stream (UDP 7778) for per-sample
   data: it does not block.
6. **The heartbeat meters are zeros**, so the controller shows no levels. The
   board now has real per-channel peaks to report.
7. **48 kHz only** (see above).

## Lessons carried across

- 64-bit sample indices everywhere; the FPGA's 32-bit ring pointer wrapped
  after 3.1 hours.
- Sigma-delta the actuator fraction; a servo cannot see how coarsely its own
  output is rendered.
- Telemetry first, parsed by name.
- Never invent a protocol constant that looks plausible; replay a real device
  and diff instead.
- When a working reference exists for the same hardware, match it before
  reasoning from the datasheet: the DAC stayed silent until the clocking
  matched `p4-uac-pcm1690` exactly.
- Anything that calls into a driver from a second task needs that driver's
  locking checked, not assumed. The transmit race took 70 minutes to kill
  the board and looked like network trouble, ARP trouble and a flaky
  transmitter along the way.
- A counter on every "can't fail" call (`sendto()` for a keepalive) turns an
  invisible fault into a number.
