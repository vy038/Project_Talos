#!/usr/bin/env python3
"""
test_red_ball_serial.py

Reads [RED_BALL] log lines from ESP-CAM serial port,
broadcasts parsed data as JSON over WebSocket for test_red_ball.html.

Usage:
    python3 test_red_ball_serial.py --port /dev/ttyUSB0 --baud 115200

Install deps:
    pip install pyserial websockets
"""

import argparse
import asyncio
import json
import os
import re
import sys
import threading
import webbrowser
import serial
import websockets
from http.server import HTTPServer, SimpleHTTPRequestHandler

# all connected HTML clients
clients: set = set()

def parse_line(line: str) -> dict | None:
    """Parse a [RED_BALL] log line into a dict. Returns None if not a RED_BALL line."""
    if "[RED_BALL]" not in line:
        return None

    fields = {}
    # match key=value pairs (int, float, or hex string)
    for match in re.finditer(r'(\w+)=([-\d.]+|[0-9A-Fa-f]{22})', line):
        key, val = match.group(1), match.group(2)
        if key == "pkt":
            fields[key] = val
        else:
            try:
                fields[key] = int(val) if "." not in val else float(val)
            except ValueError:
                fields[key] = val

    if not fields:
        return None

    # decode UART packet bytes into labeled fields for display
    pkt_hex = fields.get("pkt", "")
    if len(pkt_hex) == 22:
        pkt_bytes = bytes.fromhex(pkt_hex)
        fields["pkt_decoded"] = {
            "start":    f"0x{pkt_bytes[0]:02X} 0x{pkt_bytes[1]:02X}",
            "type":     f"0x{pkt_bytes[2]:02X}",
            "detected": pkt_bytes[3],
            "x":        (pkt_bytes[4] << 8) | pkt_bytes[5],
            "y":        (pkt_bytes[6] << 8) | pkt_bytes[7],
            "r":        (pkt_bytes[8] << 8) | pkt_bytes[9],
            "checksum": f"0x{pkt_bytes[10]:02X}",
        }

    return fields


async def serial_reader(port: str, baud: int):
    """Read serial lines and broadcast parsed RED_BALL data to all WS clients."""
    print(f"Opening serial port {port} @ {baud}")
    try:
        ser = serial.Serial(port, baud, timeout=1)
    except serial.SerialException as e:
        print(f"ERROR: Could not open {port}: {e}")
        sys.exit(1)

    print(f"Serial open. Waiting for [RED_BALL] lines...")

    loop = asyncio.get_event_loop()

    while True:
        # read is blocking — run in executor so WS server stays alive
        line = await loop.run_in_executor(None, lambda: ser.readline().decode("utf-8", errors="replace").strip())
        if not line:
            continue

        # print all serial output to console for debugging
        print(line)

        data = parse_line(line)
        if data and clients:
            msg = json.dumps(data)
            await asyncio.gather(*[c.send(msg) for c in clients], return_exceptions=True)


async def ws_handler(websocket, *args):
    """Handle a new WebSocket connection.
    *args absorbs the 'path' argument that older websockets versions pass."""
    clients.add(websocket)
    print(f"HTML client connected ({len(clients)} total)")
    try:
        await websocket.wait_closed()
    finally:
        clients.discard(websocket)
        print(f"HTML client disconnected ({len(clients)} total)")


def start_http_server(http_port: int):
    """Serve the tests directory over HTTP in a background thread."""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    os.chdir(script_dir)
    server = HTTPServer(("localhost", http_port), SimpleHTTPRequestHandler)
    t = threading.Thread(target=server.serve_forever, daemon=True)
    t.start()
    return server


async def main(port: str, baud: int, ws_port: int, http_port: int):
    print(f"\n{'='*50}")
    print(f"  Red Ball Detection Bridge")
    print(f"{'='*50}")
    print(f"  Serial : {port} @ {baud} baud")
    print(f"  WS     : ws://localhost:{ws_port}")
    print(f"  HTTP   : http://localhost:{http_port}/test_red_ball.html")
    print(f"{'='*50}\n")

    start_http_server(http_port)
    url = f"http://localhost:{http_port}/test_red_ball.html"
    print(f"Opening {url} ...")
    webbrowser.open(url)

    async with websockets.serve(ws_handler, "localhost", ws_port):
        await serial_reader(port, baud)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="ESP-CAM red ball serial -> WebSocket bridge")
    parser.add_argument("--port",    default="/dev/ttyUSB0", help="Serial port")
    parser.add_argument("--baud",    default=115200, type=int, help="Baud rate")
    parser.add_argument("--ws-port",   default=8765, type=int, help="WebSocket port")
    parser.add_argument("--http-port", default=8000, type=int, help="HTTP server port")
    args = parser.parse_args()

    asyncio.run(main(args.port, args.baud, args.ws_port, args.http_port))
