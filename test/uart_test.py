#!/usr/bin/env python3
"""
uart_test.py — Interactive UART terminal for the Elevator Door Controller

Features:
  • Computes CRC-8 (Dallas/Maxim) and formats correct protocol frames
  • Shorthand commands: type  open / close / stop / emg / reset / obs / clr
  • Raw mode: prefix line with '!' to send without CRC (tests rejection)
  • Auto-increments sequence number; type 'seq <n>' to override
  • Parallel reader thread prints all ESP32 output in real time

Usage:
  python uart_test.py [PORT] [BAUD]
  python uart_test.py COM5 115200
"""

import serial
import threading
import sys
import time

# -------------------------------------------------------
# Configuration
# -------------------------------------------------------
DEFAULT_PORT = "COM5"
DEFAULT_BAUD = 115200

# -------------------------------------------------------
# CRC-8 Dallas/Maxim (poly = 0x8C) — must match firmware
# -------------------------------------------------------
def crc8(data: bytes) -> int:
    crc = 0x00
    for byte in data:
        for _ in range(8):
            if (crc ^ byte) & 0x01:
                crc = (crc >> 1) ^ 0x8C
            else:
                crc >>= 1
            byte >>= 1
    return crc

def crc8_str(s: str) -> int:
    return crc8(s.encode("ascii"))

# -------------------------------------------------------
# Build a valid protocol frame
# Format: $CMD,TYPE=<cmd>,SEQ=<n>,CRC=<XX>
# -------------------------------------------------------
def make_frame(cmd: str, seq: int) -> str:
    base = f"$CMD,TYPE={cmd},SEQ={seq}"
    c = crc8_str(base)
    return f"{base},CRC={c:02X}"

# -------------------------------------------------------
# Shorthand aliases
# -------------------------------------------------------
ALIASES = {
    "open"  : "OPEN",
    "close" : "CLOSE",
    "stop"  : "STOP",
    "emg"   : "EMERGENCY_OPEN",
    "reset" : "RESET",
    "obs"   : "OBS_ON",
    "clr"   : "OBS_OFF",
}

# -------------------------------------------------------
# Serial reader thread
# -------------------------------------------------------
def reader(ser: serial.Serial, stop_event: threading.Event):
    while not stop_event.is_set():
        try:
            if ser.in_waiting:
                line = ser.readline().decode("ascii", errors="replace").rstrip()
                if line:
                    print(f"\r[ESP32] {line}")
                    print("cmd> ", end="", flush=True)
        except serial.SerialException:
            break
        time.sleep(0.005)

# -------------------------------------------------------
# Main
# -------------------------------------------------------
def main():
    port = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PORT
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_BAUD

    print(f"Connecting to {port} @ {baud} baud …")
    try:
        ser = serial.Serial(port, baud, timeout=0.1)
    except serial.SerialException as e:
        print(f"ERROR: {e}")
        sys.exit(1)

    time.sleep(1.5)   # Let ESP32 boot
    print("Connected.  Type 'help' for commands.\n")

    stop_evt = threading.Event()
    t = threading.Thread(target=reader, args=(ser, stop_evt), daemon=True)
    t.start()

    seq = 1

    print("Shortcuts: open | close | stop | emg | reset | obs | clr")
    print("Override seq:  seq <n>")
    print("Raw (no CRC):  !<raw text>")
    print("Quit:          q / exit\n")

    while True:
        try:
            raw = input("cmd> ").strip()
        except (EOFError, KeyboardInterrupt):
            break

        if not raw:
            continue

        if raw.lower() in ("q", "exit", "quit"):
            break

        # seq override
        if raw.lower().startswith("seq "):
            try:
                seq = int(raw.split()[1])
                print(f"  Sequence reset to {seq}")
            except ValueError:
                print("  Usage: seq <integer>")
            continue

        if raw.lower() == "help":
            print("  Commands (case-insensitive shortcuts):")
            for alias, cmd in ALIASES.items():
                frame = make_frame(cmd, seq)
                print(f"    {alias:8s} → {frame}")
            print("  !<raw>  send raw string (tests rejection)")
            print("  seq <n> override next sequence number")
            continue

        # Raw mode (bypasses CRC — should trigger NACK)
        if raw.startswith("!"):
            line = raw[1:] + "\n"
            ser.write(line.encode("ascii"))
            print(f"  [RAW] {line.rstrip()}")
            continue

        # Resolve alias or use as-is
        cmd = ALIASES.get(raw.lower(), raw.upper())
        frame = make_frame(cmd, seq)
        ser.write((frame + "\n").encode("ascii"))
        print(f"  [TX]  {frame}")
        seq += 1

    stop_evt.set()
    ser.close()
    print("Disconnected.")

if __name__ == "__main__":
    main()
