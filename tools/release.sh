#!/usr/bin/env bash
# Builds and signs a DAC8 firmware release for the update server, once per
# ESP32-P4 chip family (see sdkconfig.defaults):
#
#   tools/release.sh ["release notes"]
#
# Version: version.txt. Output in dist/dac8/ (v1.x chips) and dist/dac8v3/
# (v3.x chips): N-Series-DAC8-<version>.fw and manifest.json each. Copy both
# folders to the server's data/firmware/ (on the QNAP:
# /share/Container/N-Serie-AoIP-Server/data/firmware/); each DAC8's web page
# asks for its own product. Needs ESP-IDF v5.5.6 in the environment and
# .secrets/fw-sign-key.pem (tools/fwsign.py keygen).
set -euo pipefail
cd "$(dirname "$0")/.."
version=$(head -n1 version.txt | tr -d '[:space:]')
notes=${1:-}
[ -f .secrets/fw-sign-key.pem ] || { echo "no .secrets/fw-sign-key.pem" >&2; exit 1; }
[ -f .secrets/update-url ] || echo "warning: no .secrets/update-url -- this build's page cannot check for updates" >&2

release() {  # product build-dir idf.py-args...
    local product=$1 dir=$2; shift 2
    idf.py -B "$dir" "$@" build >/dev/null
    local built
    built=$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["project_version"])' \
            "$dir/project_description.json")
    [ "$built" = "$version" ] || { echo "$product: build says $built, version.txt says $version" >&2; exit 1; }

    local out=dist/$product fw="N-Series-DAC8-$version.fw"
    mkdir -p "$out"
    python3 tools/fwsign.py sign "$dir/aoip-rx-8ch.bin" "$out/$fw"
    PRODUCT="$product" VERSION="$version" FILE="$fw" NOTES="$notes" OUT="$out" python3 - <<'EOF2'
import hashlib, json, os, time
path = os.path.join(os.environ["OUT"], os.environ["FILE"])
data = open(path, "rb").read()
manifest = {"product": os.environ["PRODUCT"], "version": os.environ["VERSION"], "file": os.environ["FILE"],
            "size": len(data), "sha256": hashlib.sha256(data).hexdigest(),
            "date": time.strftime("%Y-%m-%d"), "notes": os.environ["NOTES"]}
json.dump(manifest, open(os.path.join(os.environ["OUT"], "manifest.json"), "w"), indent=2)
print(json.dumps(manifest, indent=2))
EOF2
}

release dac8 build
release dac8v3 build-p4v3 -D SDKCONFIG=sdkconfig.p4v3 \
        -D SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.defaults.p4v3"
echo "copy dist/dac8/ and dist/dac8v3/ to the server's data/firmware/"
