#!/usr/bin/env python3
"""Query the device's stats reply and, with `watch`, follow the servo.

The reply is newline-separated key=value text and is PARSED BY NAME, never by
offset. That is not stylistic: TELEMETRY_AND_PTP.md records two wrong
conclusions that came from hand-indexed offsets that had gone stale, and one
occasion where growing a packed reply from 200 to 208 bytes killed the port
outright for reasons never found.

  stats.py <ip>                 one snapshot
  stats.py <ip> watch [secs]    follow, and report DRIFT at the end
"""
import socket
import sys
import time

PORT = 7779


def query(ip, timeout=1.0):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(timeout)
    s.sendto(b"?", (ip, PORT))
    data, _ = s.recvfrom(4096)
    s.close()
    out = {}
    for line in data.decode(errors="replace").splitlines():
        if "=" in line:
            k, v = line.split("=", 1)
            try:
                out[k] = int(v)
            except ValueError:
                out[k] = v
    return out


def snapshot(ip):
    d = query(ip)
    for k in sorted(d):
        print(f"{k:24s} {d[k]}")


def watch(ip, secs):
    print(f"{'t':>6} {'lock':>4} {'offs_ns':>9} {'err':>6} {'ppb':>6} "
          f"{'level':>6} {'under':>7} {'late':>6} {'anch':>5}")
    t0 = time.time()
    first = last = None
    while time.time() - t0 < secs:
        try:
            d = query(ip)
        except socket.timeout:
            print("  timeout")
            time.sleep(1)
            continue
        t = time.time() - t0
        print(f"{t:6.1f} {d.get('ptp_locked',0):>4} {d.get('ptp_offset_ns',0):>9} "
              f"{d.get('mclk_error_frames',0):>6} {d.get('mclk_ppb_applied',0):>6} "
              f"{d.get('jb_level_frames',0):>6} {d.get('jb_underrun_frames',0):>7} "
              f"{d.get('jb_pkt_late',0):>6} {d.get('mclk_anchors',0):>5}")
        if first is None:
            first = (t, d)
        last = (t, d)
        time.sleep(1)

    if first and last and last[0] > first[0]:
        # Residual drift, the quantity the FPGA project spent two sessions on.
        # A phase error that WALKS cannot be fixed by biasing it -- the bias
        # only chooses where in the walk you start. If this reads non-zero and
        # steady, look at mclk_lsb_ppb before touching the servo gains.
        de = last[1].get("mclk_error_frames", 0) - first[1].get("mclk_error_frames", 0)
        dt = last[0] - first[0]
        ppm = (de / dt) / 48000.0 * 1e6
        print(f"\n  drift  {de:+d} frames / {dt:.0f} s = {ppm:+.3f} ppm "
              f"({ppm * 3.6:+.2f} ms/hour)")
        print(f"  actuator LSB = {last[1].get('mclk_lsb_ppb', 0)} ppb  "
              f"-- if |drift| is near this, it is quantisation, not the servo")
        u = last[1].get("jb_underrun_frames", 0) - first[1].get("jb_underrun_frames", 0)
        print(f"  underruns over the window: {u}")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    if len(sys.argv) > 2 and sys.argv[2] == "watch":
        watch(sys.argv[1], int(sys.argv[3]) if len(sys.argv) > 3 else 60)
    else:
        snapshot(sys.argv[1])
