#!/usr/bin/env python3
"""Signed firmware files for the DAC8's web update (main/web.c).

A .fw file is the app image (build/aoip-rx-8ch.bin) followed by a 128-byte
trailer: "NSFWSIG1", the signature's length (u16 little-endian), then the
ECDSA P-256 / SHA-256 signature of the image (DER), zero padded. The DAC8
writes the image to its spare OTA slot, checks the signature against
main/fw_sign_pub.pem and only then boots it.

  fwsign.py keygen                 new key: .secrets/fw-sign-key.pem (private,
                                   never committed) and main/fw_sign_pub.pem
  fwsign.py sign IMAGE OUT.fw      sign with .secrets/fw-sign-key.pem
  fwsign.py verify FILE.fw         check a .fw against main/fw_sign_pub.pem

Needs only the openssl command.
"""
import os
import struct
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
KEY = os.path.join(ROOT, ".secrets", "fw-sign-key.pem")
PUB = os.path.join(ROOT, "main", "fw_sign_pub.pem")
MAGIC = b"NSFWSIG1"
TRAILER = 128


def openssl(*args, data=None):
    return subprocess.run(["openssl", *args], input=data, capture_output=True, check=True).stdout


def keygen():
    if os.path.exists(KEY):
        sys.exit("%s exists; delete it first if you really want a new key "
                 "(boards with the old key then need one USB flash)" % KEY)
    os.makedirs(os.path.dirname(KEY), exist_ok=True)
    pem = openssl("ecparam", "-name", "prime256v1", "-genkey", "-noout")
    fd = os.open(KEY, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    with os.fdopen(fd, "wb") as f:
        f.write(pem)
    with open(PUB, "wb") as f:
        f.write(openssl("ec", "-in", KEY, "-pubout"))
    print("private key", KEY, "\npublic key ", PUB)


def sign(image_path, out_path):
    image = open(image_path, "rb").read()
    if image[:1] != b"\xe9":
        sys.exit("%s is not an ESP app image" % image_path)
    sig = openssl("dgst", "-sha256", "-sign", KEY, data=image)
    trailer = MAGIC + struct.pack("<H", len(sig)) + sig
    trailer += b"\0" * (TRAILER - len(trailer))
    with open(out_path, "wb") as f:
        f.write(image + trailer)
    verify(out_path)


def verify(path):
    data = open(path, "rb").read()
    image, trailer = data[:-TRAILER], data[-TRAILER:]
    if trailer[:8] != MAGIC:
        sys.exit("%s: no signature trailer" % path)
    (n,) = struct.unpack("<H", trailer[8:10])
    with tempfile.NamedTemporaryFile() as s:
        s.write(trailer[10:10 + n])
        s.flush()
        r = subprocess.run(["openssl", "dgst", "-sha256", "-verify", PUB, "-signature", s.name],
                           input=image, capture_output=True)
    if r.returncode != 0:
        sys.exit("%s: signature does NOT match %s" % (path, PUB))
    print("%s: signature OK (%d bytes image)" % (path, len(image)))


if __name__ == "__main__":
    cmd = sys.argv[1:2]
    if cmd == ["keygen"] and len(sys.argv) == 2:
        keygen()
    elif cmd == ["sign"] and len(sys.argv) == 4:
        sign(sys.argv[2], sys.argv[3])
    elif cmd == ["verify"] and len(sys.argv) == 3:
        verify(sys.argv[2])
    else:
        sys.exit(__doc__)
