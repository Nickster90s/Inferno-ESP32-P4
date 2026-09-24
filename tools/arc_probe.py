#!/usr/bin/env python3
"""Send a raw ARC request and dump the reply.

aoip_arc.c is the one file in this project whose wire layout is inferred
rather than captured, so it is the one that needs a diff against real hardware.
Point this at a real AoIP receiver AND at this device, and compare.

  arc_probe.py <ip> <opcode-hex> [hex-body]
  arc_probe.py 169.254.1.5 3000
"""
import socket
import sys

PORT = 4440
START_CODE = 0x1200


def main():
    ip = sys.argv[1]
    opcode = int(sys.argv[2], 16)
    body = bytes.fromhex(sys.argv[3]) if len(sys.argv) > 3 else b""

    total = 10 + len(body)
    pkt = (START_CODE.to_bytes(2, "big") + total.to_bytes(2, "big") +
           (1).to_bytes(2, "big") + opcode.to_bytes(2, "big") +
           (0).to_bytes(2, "big") + body)

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(3)
    s.sendto(pkt, (ip, PORT))
    try:
        data, _ = s.recvfrom(2048)
    except socket.timeout:
        print("no reply")
        return

    print(f"{len(data)} bytes")
    print(f"  start_code   0x{int.from_bytes(data[0:2],'big'):04x}")
    print(f"  total_length {int.from_bytes(data[2:4],'big')}")
    print(f"  seqnum       {int.from_bytes(data[4:6],'big')}")
    print(f"  opcode1      0x{int.from_bytes(data[6:8],'big'):04x}")
    print(f"  opcode2      0x{int.from_bytes(data[8:10],'big'):04x}")
    for off in range(10, len(data), 16):
        chunk = data[off:off + 16]
        hexs = " ".join(f"{b:02x}" for b in chunk)
        text = "".join(chr(b) if 32 <= b < 127 else "." for b in chunk)
        print(f"  {off:04x}  {hexs:<47}  {text}")


if __name__ == "__main__":
    main()
