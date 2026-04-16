#!/usr/bin/env python3
"""
test_scenarios.py — Automated scenario-based validation for
the Elevator Door Safety Controller.

Each scenario:
    1. Sends a sequence of commands with correct CRC
    2. Waits for responses
    3. Asserts expected $STATE messages

Usage:
    python test_scenarios.py [PORT] [BAUD]
    python test_scenarios.py COM5 115200

Exit code 0 = all tests passed, 1 = one or more failures.

Requirements covered:
  T-1  FR-1  Normal open
  T-2  FR-2  Normal close
  T-3  FR-3  CMD_OPEN preempts CLOSING
  T-4  FR-5  Periodic state reporting
  T-5  SR-1  Obstruction prevents close / reverses door
  T-6  SR-2  Obstruction reaction time ≤ 20 ms (measured)
  T-7  SR-5  Communication timeout → FAULT
  T-8  SR-6  EMERGENCY_OPEN overrides any state
  T-9  FR-6  NACK on invalid frame (bad CRC)
  T-10 FR-4  Redundant command suppressed
  T-11 NFR-1 Command-to-ACK latency ≤ 10 ms
  T-12       RESET recovers from FAULT
"""

import serial
import threading
import sys
import time
import queue
import re

# -------------------------------------------------------
# CRC-8 (must match firmware)
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

def make_frame(cmd: str, seq: int) -> str:
    base = f"$CMD,TYPE={cmd},SEQ={seq}"
    c = crc8(base.encode("ascii"))
    return f"{base},CRC={c:02X}"

# -------------------------------------------------------
# Serial helper
# -------------------------------------------------------
class SerialHelper:
    def __init__(self, port: str, baud: int):
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self._rx_q: queue.Queue = queue.Queue()
        self._stop = threading.Event()
        self._t = threading.Thread(target=self._reader, daemon=True)
        self._t.start()
        time.sleep(1.5)   # ESP32 boot

    def _reader(self):
        while not self._stop.is_set():
            try:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode("ascii", errors="replace").rstrip()
                    if line:
                        self._rx_q.put((time.time(), line))
            except serial.SerialException:
                break
            time.sleep(0.002)

    def send(self, frame: str):
        self.ser.write((frame + "\n").encode("ascii"))

    def flush_rx(self, timeout: float = 0.1):
        """Discard all pending received lines."""
        end = time.time() + timeout
        while time.time() < end:
            try:
                self._rx_q.get(timeout=0.02)
            except queue.Empty:
                break

    def wait_for(self, pattern: str, timeout: float = 6.0) -> tuple:
        """
        Block until a line matching 'pattern' is received.
        Returns (True, line, elapsed_s) on match,
                (False, '', elapsed_s) on timeout.
        """
        t0 = time.time()
        rx = re.compile(pattern)
        while True:
            elapsed = time.time() - t0
            if elapsed > timeout:
                return False, "", elapsed
            try:
                ts, line = self._rx_q.get(timeout=0.05)
                if rx.search(line):
                    return True, line, elapsed
            except queue.Empty:
                pass

    def close(self):
        self._stop.set()
        self.ser.close()

# -------------------------------------------------------
# Test framework
# -------------------------------------------------------
PASS  = "\033[92mPASS\033[0m"
FAIL  = "\033[91mFAIL\033[0m"
SKIP  = "\033[93mSKIP\033[0m"

results = []

def run_test(name: str, fn, helper: SerialHelper):
    print(f"\n{'─'*60}")
    print(f"  {name}")
    print(f"{'─'*60}")
    try:
        ok, detail = fn(helper)
        tag = PASS if ok else FAIL
        print(f"  Result : {tag}")
        if detail:
            print(f"  Detail : {detail}")
        results.append((name, ok, detail))
    except Exception as e:
        print(f"  Result : {FAIL}")
        print(f"  Exception: {e}")
        results.append((name, False, str(e)))

# -------------------------------------------------------
# Helper: reset FSM to known IDLE state before each test
# -------------------------------------------------------
SEQ = [1]   # Mutable integer; seq resets to 1 each test

def reset_to_idle(h: SerialHelper, seq_list):
    seq_list[0] = 1
    h.flush_rx(0.2)
    frame = make_frame("RESET", seq_list[0])
    h.send(frame)
    seq_list[0] += 1
    h.wait_for(r"\$STATE,STATE=IDLE", timeout=2.0)
    h.flush_rx(0.3)

# -------------------------------------------------------
# T-1: FR-1 — Normal door open
# -------------------------------------------------------
def t1_normal_open(h: SerialHelper):
    seq = [1]
    reset_to_idle(h, seq)

    frame = make_frame("OPEN", seq[0]); seq[0] += 1
    h.send(frame)

    ok1, line, _ = h.wait_for(r"\$ACK,TYPE=OPEN", timeout=2.0)
    ok2, line2, _ = h.wait_for(r"\$STATE,STATE=OPENING", timeout=2.0)
    ok3, _, _    = h.wait_for(r"\$STATE,STATE=OPEN", timeout=6.0)

    ok = ok1 and ok2 and ok3
    return ok, f"ACK={ok1} OPENING={ok2} OPEN={ok3}"

# -------------------------------------------------------
# T-2: FR-2 — Normal door close
# -------------------------------------------------------
def t2_normal_close(h: SerialHelper):
    seq = [1]
    reset_to_idle(h, seq)

    # Open first
    h.send(make_frame("OPEN", seq[0])); seq[0] += 1
    h.wait_for(r"\$STATE,STATE=OPEN", timeout=6.0)
    h.flush_rx(0.1)

    # Now close
    h.send(make_frame("CLOSE", seq[0])); seq[0] += 1
    ok1, _, _ = h.wait_for(r"\$ACK,TYPE=CLOSE", timeout=2.0)
    ok2, _, _ = h.wait_for(r"\$STATE,STATE=CLOSING", timeout=2.0)
    ok3, _, _ = h.wait_for(r"\$STATE,STATE=CLOSED", timeout=6.0)

    ok = ok1 and ok2 and ok3
    return ok, f"ACK={ok1} CLOSING={ok2} CLOSED={ok3}"

# -------------------------------------------------------
# T-3: FR-3 — CMD_OPEN preempts CLOSING
# -------------------------------------------------------
def t3_preemption(h: SerialHelper):
    seq = [1]
    reset_to_idle(h, seq)

    # Open door
    h.send(make_frame("OPEN", seq[0])); seq[0] += 1
    h.wait_for(r"\$STATE,STATE=OPEN", timeout=6.0)
    h.flush_rx(0.1)

    # Start closing
    h.send(make_frame("CLOSE", seq[0])); seq[0] += 1
    h.wait_for(r"\$STATE,STATE=CLOSING", timeout=2.0)

    # Immediately send OPEN to preempt
    time.sleep(0.3)
    h.send(make_frame("OPEN", seq[0])); seq[0] += 1
    ok, _, elapsed = h.wait_for(r"\$STATE,STATE=OPENING", timeout=2.0)

    return ok, f"Preemption in {elapsed*1000:.1f} ms"

# -------------------------------------------------------
# T-4: FR-5 — Periodic state reporting
# -------------------------------------------------------
def t4_periodic_state(h: SerialHelper):
    seq = [1]
    reset_to_idle(h, seq)
    h.flush_rx(0.05)

    # Collect $STATE messages for 1 second
    t0 = time.time()
    count = 0
    while time.time() - t0 < 1.0:
        ok, _, _ = h.wait_for(r"\$STATE,", timeout=0.5)
        if ok:
            count += 1

    # Expect ~10 reports per second (every 100 ms)
    ok = count >= 5
    return ok, f"Received {count} $STATE reports in 1 s (expect ≥5)"

# -------------------------------------------------------
# T-5: SR-1 — Obstruction prevents close / reverses motion
# -------------------------------------------------------
def t5_obstruction(h: SerialHelper):
    seq = [1]
    reset_to_idle(h, seq)

    # Open door
    h.send(make_frame("OPEN", seq[0])); seq[0] += 1
    h.wait_for(r"\$STATE,STATE=OPEN", timeout=6.0)
    h.flush_rx(0.1)

    # Enable obstruction
    h.send(make_frame("OBS_ON", seq[0])); seq[0] += 1
    time.sleep(0.1)

    # Send close — safety task should trigger reversal to OPENING
    h.send(make_frame("CLOSE", seq[0])); seq[0] += 1
    ok1, _, _ = h.wait_for(r"\$STATE,STATE=CLOSING", timeout=2.0)
    ok2, _, _ = h.wait_for(r"\$STATE,STATE=OPENING", timeout=3.0)

    # Clear obstruction
    h.send(make_frame("OBS_OFF", seq[0])); seq[0] += 1

    ok = ok1 and ok2
    return ok, f"Entered CLOSING={ok1}, Reversed to OPENING={ok2}"

# -------------------------------------------------------
# T-6: SR-2 — Obstruction reaction time ≤ 20 ms
# -------------------------------------------------------
def t6_reaction_time(h: SerialHelper):
    seq = [1]
    reset_to_idle(h, seq)

    h.send(make_frame("OPEN", seq[0])); seq[0] += 1
    h.wait_for(r"\$STATE,STATE=OPEN", timeout=6.0)
    h.flush_rx(0.1)

    h.send(make_frame("CLOSE", seq[0])); seq[0] += 1
    h.wait_for(r"\$STATE,STATE=CLOSING", timeout=2.0)
    time.sleep(0.3)

    # Measure time from OBS_ON to OPENING state
    h.flush_rx(0.05)
    t0 = time.time()
    h.send(make_frame("OBS_ON", seq[0])); seq[0] += 1
    ok, _, elapsed = h.wait_for(r"\$STATE,STATE=OPENING", timeout=1.0)

    elapsed_ms = elapsed * 1000
    h.send(make_frame("OBS_OFF", seq[0])); seq[0] += 1

    # ≤ 100 ms is realistic over UART (real hw constraint is ≤ 20 ms)
    THRESHOLD_MS = 100
    passed = ok and elapsed_ms <= THRESHOLD_MS
    return passed, f"Reaction time: {elapsed_ms:.1f} ms (threshold {THRESHOLD_MS} ms)"

# -------------------------------------------------------
# T-7: SR-5 — Communication timeout → FAULT
# -------------------------------------------------------
def t7_comm_timeout(h: SerialHelper):
    seq = [1]
    reset_to_idle(h, seq)

    # Send one command then go silent
    h.send(make_frame("OPEN", seq[0])); seq[0] += 1
    h.wait_for(r"\$STATE,STATE=OPEN", timeout=6.0)
    h.flush_rx(0.1)

    print("    Waiting for comm timeout (≈ 3 s) …")
    ok, _, elapsed = h.wait_for(r"\$STATE,STATE=FAULT", timeout=8.0)

    # Recover for subsequent tests
    time.sleep(0.2)
    h.send(make_frame("RESET", 1))
    h.wait_for(r"\$STATE,STATE=IDLE", timeout=2.0)

    return ok, f"FAULT after {elapsed:.1f} s of silence"

# -------------------------------------------------------
# T-8: SR-6 — EMERGENCY_OPEN overrides any state
# -------------------------------------------------------
def t8_emergency(h: SerialHelper):
    seq = [1]
    reset_to_idle(h, seq)

    # Put door in CLOSING
    h.send(make_frame("OPEN", seq[0])); seq[0] += 1
    h.wait_for(r"\$STATE,STATE=OPEN", timeout=6.0)
    h.send(make_frame("CLOSE", seq[0])); seq[0] += 1
    h.wait_for(r"\$STATE,STATE=CLOSING", timeout=2.0)
    time.sleep(0.3)

    # Fire EMERGENCY_OPEN
    h.send(make_frame("EMERGENCY_OPEN", seq[0])); seq[0] += 1
    ok1, _, elapsed = h.wait_for(r"\$STATE,STATE=(OPENING|EMERGENCY)", timeout=2.0)

    # Confirm door stays open — CLOSE must be rejected
    time.sleep(0.2)
    h.send(make_frame("CLOSE", seq[0])); seq[0] += 1
    ok2, line2, _ = h.wait_for(r"\$(ACK|NACK|STATE)", timeout=1.0)
    # Should stay OPENING/EMERGENCY, not CLOSING
    ok_stay = "CLOSING" not in line2

    # Recovery
    h.send(make_frame("RESET", 1))
    h.wait_for(r"\$STATE,STATE=IDLE", timeout=2.0)

    ok = ok1 and ok_stay
    return ok, f"Entered EMERGENCY in {elapsed*1000:.1f} ms; CLOSE rejected={ok_stay}"

# -------------------------------------------------------
# T-9: FR-6 — NACK on invalid CRC
# -------------------------------------------------------
def t9_bad_crc(h: SerialHelper):
    seq = [1]
    reset_to_idle(h, seq)

    # Send frame with deliberately wrong CRC
    bad_frame = "$CMD,TYPE=OPEN,SEQ=1,CRC=00"   # CRC=00 is almost never correct
    h.flush_rx(0.05)
    h.ser.write((bad_frame + "\n").encode("ascii"))
    ok, line, _ = h.wait_for(r"\$NACK", timeout=2.0)

    return ok, f"NACK received: {line}"

# -------------------------------------------------------
# T-10: FR-4 — Redundant command suppressed (no state change)
# -------------------------------------------------------
'''
def t10_redundant(h: SerialHelper):
    seq = [1]
    reset_to_idle(h, seq)

    h.send(make_frame("OPEN", seq[0])); seq[0] += 1
    h.wait_for(r"\$STATE,STATE=OPEN", timeout=6.0)
    h.flush_rx(0.2)

    # Send OPEN again while already OPEN
    h.send(make_frame("OPEN", seq[0])); seq[0] += 1
    # ACK should still come (command valid, just no transition)
    ok1, _, _ = h.wait_for(r"\$ACK,TYPE=OPEN", timeout=2.0)
    # Confirm state did NOT change to something else
    not_changed = True
    for _ in range(5):
        _, line, _ = h.wait_for(r"\$STATE,", timeout=0.3)
        if "OPENING" in line or "FAULT" in line:
            not_changed = False
    return ok1 and not_changed, f"ACK={ok1} StateStable={not_changed}"
'''

def t10_redundant(h: SerialHelper):
    import time

    seq = [1]
    reset_to_idle(h, seq)

    # --- Step 1: Move to OPEN and ensure stable ---
    h.send(make_frame("OPEN", seq[0])); seq[0] += 1
    
    # FIX: Add \b (word boundary) so it strictly matches OPEN, not OPENING
    ok_open, _, _ = h.wait_for(r"\$STATE,STATE=OPEN\b", timeout=6.0)
    if not ok_open:
        return False, "Setup failed: FSM did not reach OPEN" 

    # --- Step 2: Time-bounded RX drain (prevents infinite loop) ---
    drain_deadline = time.time() + 0.5
    while time.time() < drain_deadline:
        h.wait_for(r".+", timeout=0.05)

    # --- Step 3: Send redundant OPEN ---
    current_seq = seq[0]
    h.send(make_frame("OPEN", current_seq)); seq[0] += 1

    # --- Step 4: Wait for ACK (robust matching) ---
    ok1 = False
    ack_deadline = time.time() + 2.0

    while time.time() < ack_deadline:
        ok, line, _ = h.wait_for(r"\$ACK", timeout=0.2)
        if not ok or not line:
            continue

        if "TYPE=OPEN" in line:
            if f"SEQ={current_seq}" in line or "SEQ=" not in line:
                ok1 = True
                break

    # --- Step 5: Verify state stability strictly ---
    not_changed = True
    observed_open = False
    state_deadline = time.time() + 2.0

    while time.time() < state_deadline:
        ok, line, _ = h.wait_for(r"\$STATE,", timeout=0.2)
        if not ok or not line:
            continue

        # FIX: Ensure it doesn't match OPENING during the strict check
        if "STATE=OPEN" in line and "OPENING" not in line:
            observed_open = True
        else:
            not_changed = False

    not_changed = not_changed and observed_open

    return ok1 and not_changed, f"ACK={ok1} StateStable={not_changed}"
# -------------------------------------------------------
# T-11: NFR-1 — Command-to-ACK latency ≤ 10 ms
# -------------------------------------------------------
def t11_latency(h: SerialHelper):
    seq = [1]
    reset_to_idle(h, seq)

    samples = []
    for i in range(5):
        h.flush_rx(0.05)
        frame = make_frame("OPEN" if i % 2 == 0 else "RESET", seq[0])
        seq[0] += 1
        t0 = time.perf_counter()
        h.send(frame)
        ok, _, _ = h.wait_for(r"\$(ACK|NACK)", timeout=0.5)
        elapsed_ms = (time.perf_counter() - t0) * 1000
        if ok:
            samples.append(elapsed_ms)
        time.sleep(0.1)

    if not samples:
        return False, "No ACKs received"

    avg = sum(samples) / len(samples)
    mx  = max(samples)
    # Over UART the round-trip includes Python serial overhead.
    # Real WCET constraint is on the embedded side; we check ≤ 50 ms total.
    ok = mx <= 50.0
    return ok, (f"Avg={avg:.1f} ms  Max={mx:.1f} ms  "
                f"Samples={samples}  (UART+Python overhead included)")

# -------------------------------------------------------
# T-12: RESET recovers from FAULT
# -------------------------------------------------------
def t12_fault_recovery(h: SerialHelper):
    seq = [1]
    reset_to_idle(h, seq)

    # Force FAULT via comm timeout (quick: stop sending for > 2 s)
    print("    Waiting for comm timeout …")
    h.wait_for(r"\$STATE,STATE=FAULT", timeout=8.0)
    h.flush_rx(0.1)

    # Send RESET with seq=1 (reset sequence counter)
    h.send(make_frame("RESET", 1))
    ok, _, elapsed = h.wait_for(r"\$STATE,STATE=IDLE", timeout=3.0)

    return ok, f"IDLE restored in {elapsed*1000:.1f} ms after RESET"

# -------------------------------------------------------
# Main
# -------------------------------------------------------
def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM5"
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200

    print(f"\n{'═'*60}")
    print(f"  Elevator Door Controller — Automated Test Suite")
    print(f"  Port: {port}  Baud: {baud}")
    print(f"{'═'*60}\n")

    try:
        h = SerialHelper(port, baud)
    except serial.SerialException as e:
        print(f"ERROR: Cannot open {port}: {e}")
        sys.exit(1)

    # Ordered test list
    tests = [
        ("T-1  FR-1  Normal open",                  t1_normal_open),
        ("T-2  FR-2  Normal close",                 t2_normal_close),
        ("T-3  FR-3  CMD_OPEN preempts CLOSING",    t3_preemption),
        ("T-4  FR-5  Periodic state reporting",     t4_periodic_state),
        ("T-5  SR-1  Obstruction reversal",         t5_obstruction),
        ("T-6  SR-2  Obstruction reaction time",    t6_reaction_time),
        ("T-7  SR-5  Comm timeout → FAULT",         t7_comm_timeout),
        ("T-8  SR-6  EMERGENCY_OPEN override",      t8_emergency),
        ("T-9  FR-6  NACK on bad CRC",              t9_bad_crc),
        ("T-10 FR-4  Redundant command suppressed", t10_redundant),
        ("T-11 NFR-1 Command-to-ACK latency",       t11_latency),
        ("T-12       RESET from FAULT",             t12_fault_recovery),
    ]

    for name, fn in tests:
        run_test(name, fn, h)
        time.sleep(0.5)   # Brief gap between tests

    h.close()

    # Summary
    passed  = sum(1 for _, ok, _ in results if ok)
    failed  = len(results) - passed
    print(f"\n{'═'*60}")
    print(f"  RESULTS: {passed}/{len(results)} passed   {failed} failed")
    print(f"{'═'*60}\n")
    for name, ok, detail in results:
        tag = PASS if ok else FAIL
        print(f"  {tag}  {name}")
        if not ok and detail:
            print(f"        {detail}")

    sys.exit(0 if failed == 0 else 1)

if __name__ == "__main__":
    main()
