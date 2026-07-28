#!/usr/bin/env python3
# Serves build-web over HTTPS with a self-signed certificate, for headsets
# that cannot reach the dev box through `adb reverse`.
#
# This exists because WebXR requires a SECURE CONTEXT and `http://<LAN-IP>`
# is not one — only https:// and localhost are. `python3 -m http.server` is
# still the right tool over adb (see README); this is the wireless fallback,
# and it is a script rather than a one-liner only because generating a cert
# and wrapping the socket cannot be expressed as one.
#
# The certificate carries the IP in subjectAltName. A CN-only certificate is
# rejected outright by Chromium rather than offered as a click-through
# warning, so the SAN is not cosmetic.
#
# You still get a warning page — the certificate is self-signed and nothing
# on the headset trusts it. Accept it once per origin. UNVERIFIED on Pico
# Browser: the bypass-then-secure-context behaviour is Chromium's documented
# behaviour and is how self-signed WebXR development is normally done, but
# nobody here has a headset to confirm it. If the browser refuses to enter
# XR after the bypass, use adb (README, "On a headset") — that path needs no
# certificate at all.
#
# Usage:   scripts/serve-web-tls.py [dir] [port]
# Default: build-web on 8443. Certificate is written to <dir>/.tls/ and
#          regenerated when the address changes.

import http.server
import ipaddress
import socket
import ssl
import subprocess
import sys
from pathlib import Path


def lan_address() -> str:
    """The address a headset on the same network would dial."""
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.connect(("10.255.255.255", 1))  # no packet leaves; picks a route
        return probe.getsockname()[0]
    except OSError:
        return "127.0.0.1"
    finally:
        probe.close()


def ensure_cert(tls_dir: Path, address: str) -> tuple[Path, Path]:
    cert, key = tls_dir / "cert.pem", tls_dir / "key.pem"
    stamp = tls_dir / "address"
    if cert.exists() and key.exists() and stamp.read_text().strip() == address:
        return cert, key
    tls_dir.mkdir(parents=True, exist_ok=True)
    san = ("IP:" if _is_ip(address) else "DNS:") + address
    subprocess.run(
        ["openssl", "req", "-x509", "-newkey", "rsa:2048", "-nodes",
         "-keyout", str(key), "-out", str(cert), "-days", "365",
         "-subj", f"/CN={address}", "-addext", f"subjectAltName={san}"],
        check=True, capture_output=True)
    stamp.write_text(address + "\n")
    print(f"generated a self-signed certificate for {address}")
    return cert, key


def _is_ip(value: str) -> bool:
    try:
        ipaddress.ip_address(value)
        return True
    except ValueError:
        return False


def main() -> int:
    root = Path(sys.argv[1] if len(sys.argv) > 1 else "build-web").resolve()
    port = int(sys.argv[2]) if len(sys.argv) > 2 else 8443
    if not (root / "index.html").exists():
        print(f"{root} has no index.html — build the web target first:\n"
              "  emcmake cmake -S . -B build-web -DCMAKE_BUILD_TYPE=Release\n"
              "  cmake --build build-web --parallel --target mxr", file=sys.stderr)
        return 1

    address = lan_address()
    cert, key = ensure_cert(root / ".tls", address)
    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    ctx.load_cert_chain(cert, key)

    handler = lambda *a, **k: http.server.SimpleHTTPRequestHandler(
        *a, directory=str(root), **k)
    server = http.server.ThreadingHTTPServer(("0.0.0.0", port), handler)
    server.socket = ctx.wrap_socket(server.socket, server_side=True)
    print(f"serving {root} at https://{address}:{port}/")
    print("the certificate is self-signed: accept the warning once per origin")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
