"""
elevator_door_test.py
=====================
UART Test Automation for ESP32 Elevator Door Safety Controller
CS G523 – Software for Embedded Systems

Architecture:
  ┌─────────────────┐
  │  UARTInterface  │  open/close serial, send_command, read_line
  └────────┬────────┘
           │
  ┌────────▼────────┐
  │   LogParser     │  extract STATE, ACK/NACK, event tokens
  └────────┬────────┘
           │
  ┌────────▼────────┐
  │  DeviceMonitor  │  background thread – continuous RX, state tracking
  └────────┬────────┘
           │
  ┌────────▼────────┐
  │   TestRunner    │  run_test_case(), wait_for_state(), assertions
  └────────┬────────┘
           │
  ┌────────▼────────┐
  │    Reporter     │  PASS/FAIL per TC, final summary
  └─────────────────┘

Usage:
  pip install pyserial
  python elevator_door_test.py [--port COM5] [--baud 115200] [--tc 1-30]

HOW TO RUN:
  1. Install dependency:       pip install pyserial
  2. Connect ESP32 via USB, identify COM port (e.g. COM5 on Windows, /dev/ttyUSB0 on Linux)
  3. Flash firmware with FSM_UNIT_TEST disabled (production firmware)
  4. Run:  python elevator_door_test.py --port COM5
  5. Optionally run a single TC:  python elevator_door_test.py --port COM5 --tc 3
"""

import serial
import threading
import time
import re
import queue
import argparse
import sys
from dataclasses import dataclass, field
from typing import Optional, List, Tuple
from enum import Enum

# ─────────────────────────────────────────────────────────────────────────────
# CONFIGURABLE CONSTANTS
# ─────────────────────────────────────────────────────────────────────────────
DEFAULT_PORT          = "COM5"
DEFAULT_BAUD          = 115200
DEFAULT_SERIAL_TIMEOUT= 0.1          # seconds – read_line blocking timeout
HOMING_TIMEOUT_S      = 20.0         # max time to wait for IDLE after reset/boot
MOVE_TIMEOUT_S        = 15.0         # max time for OPENING/CLOSING to complete
ACK_TIMEOUT_S         = 3.0          # max time to wait for $ACK
STATE_SETTLE_S        = 0.5          # brief settle after a confirmed state
COMM_TIMEOUT_S        = 2000.0       # device COMM_TIMEOUT_MS = 2 000 000; add margin
COMM_TIMEOUT_MARGIN_S = 400.0        # acceptable timing slop (±) for timeout TCs
STALL_TIMEOUT_S       = 12.0         # MOTOR_STALL_TIMEOUT_MS = 10 000 ms + margin
INTER_CMD_DELAY_S     = 0.1          # brief pause between back-to-back commands
BOOT_SETTLE_S         = 3.0          # wait after serial open for device to boot

# ─────────────────────────────────────────────────────────────────────────────
# FSM STATES (mirrors firmware enum)
# ─────────────────────────────────────────────────────────────────────────────
class FsmState(str, Enum):
    INIT    = "INIT"
    HOMING  = "HOMING"
    IDLE    = "IDLE"
    OPENING = "OPENING"
    OPEN    = "OPEN"
    CLOSING = "CLOSING"
    CLOSED  = "CLOSED"
    FAULT   = "FAULT"
    UNKNOWN = "UNKNOWN"

# ─────────────────────────────────────────────────────────────────────────────
# LOG PARSER
# ─────────────────────────────────────────────────────────────────────────────
class LogParser:
    """
    Stateless parser for individual log lines from the device.
    All methods return None when the line doesn't match the query.
    """

    # $STATE,STATE=HOMING  or  $STATE,STATE=IDLE,TS=1234
    _STATE_RE   = re.compile(r'\$STATE,STATE=(\w+)')
    _ACK_RE     = re.compile(r'\$ACK\b')
    _NACK_RE    = re.compile(r'\$NACK\b')
    # FSM: State: 3 → 4  (firmware uses → U+2192)
    _TRANS_RE   = re.compile(r'State:\s*\d+\s*[→>]+\s*\d+')
    # LOGGER: [EVT  ] ts=...  CMD_OPEN ...
    _EVT_RE     = re.compile(r'\[EVT\s*\]\s+.*?(\bCMD_\w+|\bSENSOR_\w+|\bOBSTRUCTION_\w+|\bCOMM_TIMEOUT\b|\bMOTOR_STALL\b|\bSPOF_DETECTED\b|\bFAULT_PERSIST\b)')
    # LOGGER: [TRANS] ts=... HOMING → IDLE
    _LOG_TRANS_RE = re.compile(r'\[TRANS\]\s+.*?(\w+)\s*[→>]+\s*(\w+)')

    @classmethod
    def extract_state(cls, line: str) -> Optional[FsmState]:
        m = cls._STATE_RE.search(line)
        if m:
            try:
                return FsmState(m.group(1))
            except ValueError:
                return FsmState.UNKNOWN
        return None

    @classmethod
    def is_ack(cls, line: str) -> bool:
        return bool(cls._ACK_RE.search(line))

    @classmethod
    def is_nack(cls, line: str) -> bool:
        return bool(cls._NACK_RE.search(line))

    @classmethod
    def extract_log_event(cls, line: str) -> Optional[str]:
        m = cls._EVT_RE.search(line)
        return m.group(1) if m else None

    @classmethod
    def extract_log_transition(cls, line: str) -> Optional[Tuple[str, str]]:
        m = cls._LOG_TRANS_RE.search(line)
        if m:
            return (m.group(1).upper(), m.group(2).upper())
        return None

    @classmethod
    def is_fault_line(cls, line: str) -> bool:
        return "FAULT" in line and ("entered" in line or "STATE=FAULT" in line)

# ─────────────────────────────────────────────────────────────────────────────
# UART INTERFACE
# ─────────────────────────────────────────────────────────────────────────────
class UARTInterface:
    """Low-level UART wrapper. Thread-safe send; read is used by DeviceMonitor."""

    def __init__(self, port: str, baud: int):
        self.port  = port
        self.baud  = baud
        self._ser: Optional[serial.Serial] = None
        self._lock = threading.Lock()

    def open(self):
        self._ser = serial.Serial(
            port      = self.port,
            baudrate  = self.baud,
            bytesize  = serial.EIGHTBITS,
            parity    = serial.PARITY_NONE,
            stopbits  = serial.STOPBITS_ONE,
            timeout   = DEFAULT_SERIAL_TIMEOUT,
        )
        print(f"[UART] Opened {self.port} @ {self.baud} baud")

    def close(self):
        if self._ser and self._ser.is_open:
            self._ser.close()
            print("[UART] Port closed")

    def send_command(self, cmd: str):
        """
        Sends a command string, appending \\n if missing.
        Thread-safe via internal lock.
        """
        if not self._ser or not self._ser.is_open:
            raise IOError("Serial port not open")
        cmd = cmd.strip()
        if not cmd:
            return
        if not cmd.endswith('\n'):
            cmd += '\n'
        with self._lock:
            self._ser.write(cmd.encode('ascii'))
        print(f"  [TX] {cmd.strip()}")

    def read_line(self) -> Optional[str]:
        """
        Reads one line from UART. Returns None on timeout or empty read.
        Not called directly by tests – DeviceMonitor uses this.
        """
        if not self._ser or not self._ser.is_open:
            return None
        try:
            raw = self._ser.readline()
            if raw:
                return raw.decode('ascii', errors='replace').strip()
        except serial.SerialException as e:
            print(f"[UART] Read error: {e}")
        return None

    @property
    def is_open(self) -> bool:
        return self._ser is not None and self._ser.is_open

# ─────────────────────────────────────────────────────────────────────────────
# DEVICE MONITOR (background thread)
# ─────────────────────────────────────────────────────────────────────────────
class DeviceMonitor:
    """
    Background thread that continuously reads UART and maintains:
      - current_state: latest confirmed FsmState
      - log_queue:     all raw lines for test assertions
      - ack_event:     set whenever $ACK is seen
      - nack_event:    set whenever $NACK is seen
      - state_event:   set whenever a new STATE is parsed
    """

    def __init__(self, uart: UARTInterface):
        self._uart          = uart
        self.current_state  = FsmState.UNKNOWN
        self.log_queue: queue.Queue[str] = queue.Queue(maxsize=500)
        self.all_lines: List[str]        = []          # full audit log
        self.ack_event    = threading.Event()
        self.nack_event   = threading.Event()
        self.state_event  = threading.Event()          # fires on any state change
        self._stop_flag   = threading.Event()
        self._thread      = threading.Thread(target=self._run, daemon=True)
        self._state_lock  = threading.Lock()

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop_flag.set()
        self._thread.join(timeout=2.0)

    def reset_events(self):
        """Call before sending a command so old ACK/NACK/state don't bleed through."""
        self.ack_event.clear()
        self.nack_event.clear()
        self.state_event.clear()
        # Drain log queue
        while not self.log_queue.empty():
            try:
                self.log_queue.get_nowait()
            except queue.Empty:
                break

    def _run(self):
        while not self._stop_flag.is_set():
            line = self._uart.read_line()
            if not line:
                continue
            # Store for audit
            self.all_lines.append(line)
            try:
                self.log_queue.put_nowait(line)
            except queue.Full:
                pass  # drop oldest implicitly; test will still see events

            print(f"  [RX] {line}")

            # Parse state
            state = LogParser.extract_state(line)
            if state:
                with self._state_lock:
                    self.current_state = state
                self.state_event.set()

            # Parse ACK / NACK
            if LogParser.is_ack(line):
                self.ack_event.set()
            if LogParser.is_nack(line):
                self.nack_event.set()

    def get_state(self) -> FsmState:
        with self._state_lock:
            return self.current_state

# ─────────────────────────────────────────────────────────────────────────────
# TEST RUNNER
# ─────────────────────────────────────────────────────────────────────────────
@dataclass
class TestResult:
    tc_id: int
    name: str
    passed: bool
    reason: str = ""
    duration_s: float = 0.0

class TestRunner:
    """
    Provides event-driven wait helpers and assertion methods.
    Each TC method returns (passed: bool, reason: str).
    """

    def __init__(self, uart: UARTInterface, monitor: DeviceMonitor):
        self.uart    = uart
        self.monitor = monitor

    # ── Synchronisation Helpers ───────────────────────────────────────────────

    def wait_for_state(self, target: FsmState, timeout: float) -> bool:
        """
        Blocks until the device reports target state or timeout expires.
        Uses event-driven polling – does NOT rely solely on sleep.
        """
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.monitor.get_state() == target:
                return True
            # Wait for next state update, then re-check
            remaining = deadline - time.monotonic()
            self.monitor.state_event.wait(timeout=min(0.5, remaining))
            self.monitor.state_event.clear()
        return self.monitor.get_state() == target

    def wait_for_log(self, pattern: str, timeout: float) -> Optional[str]:
        """
        Blocks until a log line matching `pattern` is seen, or timeout expires.
        Returns the matching line or None.
        """
        regex    = re.compile(pattern, re.IGNORECASE)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            try:
                line = self.monitor.log_queue.get(timeout=min(0.3, remaining))
                if regex.search(line):
                    return line
            except queue.Empty:
                pass
        return None

    def wait_for_ack(self, timeout: float = ACK_TIMEOUT_S) -> bool:
        """Returns True if $ACK is received within timeout."""
        return self.monitor.ack_event.wait(timeout=timeout)

    def wait_for_nack(self, timeout: float = ACK_TIMEOUT_S) -> bool:
        """Returns True if $NACK is received within timeout."""
        return self.monitor.nack_event.wait(timeout=timeout)

    def send(self, cmd: str):
        """Send command with brief inter-command delay."""
        self.uart.send_command(cmd)
        time.sleep(INTER_CMD_DELAY_S)

    def reset_and_home(self) -> bool:
        """
        Utility: ensure the FSM reaches IDLE before a test case.
        Uses RESET/HOMING semantics and, if needed, emergency-open to force
        a recoverable state from CLOSED/OPEN/CLOSING/FAULT.
        Returns True on success.
        """
        current_state = self.monitor.get_state()
        if current_state == FsmState.IDLE:
            return True

        if current_state == FsmState.HOMING:
            self.monitor.reset_events()
            self.send("$SENSOR,FULLY_CLOSED=1")
            return self.wait_for_state(FsmState.IDLE, HOMING_TIMEOUT_S)

        self.monitor.reset_events()
        self.send("$CMD,TYPE=EMERGENCY_OPEN")
        if not self.wait_for_state(FsmState.OPENING, MOVE_TIMEOUT_S):
            return False

        self.monitor.reset_events()
        self.send("$CMD,TYPE=RESET")
        if not self.wait_for_state(FsmState.HOMING, HOMING_TIMEOUT_S):
            return False

        self.send("$SENSOR,FULLY_CLOSED=1")
        return self.wait_for_state(FsmState.IDLE, HOMING_TIMEOUT_S)

    def open_door(self) -> bool:
        """CMD_OPEN → wait for OPENING → send FULLY_OPEN → wait for OPEN."""
        self.monitor.reset_events()
        self.send("$CMD,TYPE=OPEN")
        if not self.wait_for_state(FsmState.OPENING, MOVE_TIMEOUT_S):
            return False
        self.send("$SENSOR,FULLY_OPEN=1")
        return self.wait_for_state(FsmState.OPEN, MOVE_TIMEOUT_S)

    def close_door(self) -> bool:
        """CMD_CLOSE → wait for CLOSING → send FULLY_CLOSED → wait for CLOSED."""
        self.monitor.reset_events()
        self.send("$CMD,TYPE=CLOSE")
        if not self.wait_for_state(FsmState.CLOSING, MOVE_TIMEOUT_S):
            return False
        self.send("$SENSOR,FULLY_CLOSED=1")
        return self.wait_for_state(FsmState.CLOSED, MOVE_TIMEOUT_S)

    # ── Individual Test Cases ─────────────────────────────────────────────────

    def tc01_boot_state_is_homing(self):
        """TC-01: After boot/reset, FSM must be in HOMING."""
        self.monitor.reset_events()
        self.send("$CMD,TYPE=RESET")
        ok = self.wait_for_state(FsmState.HOMING, HOMING_TIMEOUT_S)
        if not ok:
            return False, f"Expected HOMING, got {self.monitor.get_state()}"
        return True, ""

    def tc02_homing_completes_to_idle(self):
        """TC-02: FULLY_CLOSED during HOMING → IDLE."""
        self.monitor.reset_events()
        self.send("$CMD,TYPE=RESET")
        if not self.wait_for_state(FsmState.HOMING, HOMING_TIMEOUT_S):
            return False, "Device did not enter HOMING"
        self.monitor.reset_events()
        self.send("$SENSOR,FULLY_CLOSED=1")
        ok = self.wait_for_state(FsmState.IDLE, HOMING_TIMEOUT_S)
        if not ok:
            return False, f"Expected IDLE after homing, got {self.monitor.get_state()}"
        return True, ""

    def tc03_open_from_idle(self):
        """TC-03: CMD_OPEN from IDLE → OPENING."""
        if not self.reset_and_home():
            return False, "Preamble failed (could not reach IDLE)"
        self.monitor.reset_events()
        self.send("$CMD,TYPE=OPEN")
        ok = self.wait_for_state(FsmState.OPENING, MOVE_TIMEOUT_S)
        if not ok:
            return False, f"Expected OPENING, got {self.monitor.get_state()}"
        return True, ""

    def tc04_fully_open_sensor_to_open(self):
        """TC-04: FULLY_OPEN sensor during OPENING → OPEN."""
        if not self.reset_and_home():
            return False, "Preamble failed"
        self.send("$CMD,TYPE=OPEN")
        if not self.wait_for_state(FsmState.OPENING, MOVE_TIMEOUT_S):
            return False, "Could not reach OPENING"
        self.monitor.reset_events()
        self.send("$SENSOR,FULLY_OPEN=1")
        ok = self.wait_for_state(FsmState.OPEN, MOVE_TIMEOUT_S)
        if not ok:
            return False, f"Expected OPEN, got {self.monitor.get_state()}"
        return True, ""

    def tc05_close_from_open(self):
        """TC-05: CMD_CLOSE from OPEN → CLOSING."""
        if not self.reset_and_home():
            return False, "Preamble failed"
        if not self.open_door():
            return False, "Could not reach OPEN"
        self.monitor.reset_events()
        self.send("$CMD,TYPE=CLOSE")
        ok = self.wait_for_state(FsmState.CLOSING, MOVE_TIMEOUT_S)
        if not ok:
            return False, f"Expected CLOSING, got {self.monitor.get_state()}"
        return True, ""

    def tc06_fully_closed_to_closed(self):
        """TC-06: FULLY_CLOSED during CLOSING → CLOSED."""
        if not self.reset_and_home():
            return False, "Preamble failed"
        if not self.open_door():
            return False, "Could not reach OPEN"
        self.send("$CMD,TYPE=CLOSE")
        if not self.wait_for_state(FsmState.CLOSING, MOVE_TIMEOUT_S):
            return False, "Could not reach CLOSING"
        self.monitor.reset_events()
        self.send("$SENSOR,FULLY_CLOSED=1")
        ok = self.wait_for_state(FsmState.CLOSED, MOVE_TIMEOUT_S)
        if not ok:
            return False, f"Expected CLOSED, got {self.monitor.get_state()}"
        return True, ""

    def tc07_open_from_closed(self):
        """TC-07: CMD_OPEN from CLOSED → OPENING (full round-trip check)."""
        if not self.reset_and_home():
            return False, "Preamble failed"
        if not self.close_door():
            # close_door goes OPEN→CLOSING→CLOSED; but we start from IDLE
            # so do open first
            if not self.open_door():
                return False, "Could not open door"
            if not self.close_door():
                return False, "Could not close door"
        self.monitor.reset_events()
        self.send("$CMD,TYPE=OPEN")
        ok = self.wait_for_state(FsmState.OPENING, MOVE_TIMEOUT_S)
        if not ok:
            return False, f"Expected OPENING from CLOSED, got {self.monitor.get_state()}"
        return True, ""

    def tc08_obstruction_during_closing_reverses(self):
        """TC-08: Obstruction during CLOSING → immediate reversal to OPENING."""
        if not self.reset_and_home():
            return False, "Preamble failed"
        if not self.open_door():
            return False, "Could not open door"
        self.send("$CMD,TYPE=CLOSE")
        if not self.wait_for_state(FsmState.CLOSING, MOVE_TIMEOUT_S):
            return False, "Could not reach CLOSING"
        self.monitor.reset_events()
        self.send("$SENSOR,OBSTRUCTION=1")
        ok = self.wait_for_state(FsmState.OPENING, MOVE_TIMEOUT_S)
        if not ok:
            return False, f"Expected OPENING after obstruction, got {self.monitor.get_state()}"
        return True, ""

    def tc09_close_rejected_during_obstruction(self):
        """TC-09: CMD_CLOSE must be rejected while obstruction is active."""
        if not self.reset_and_home():
            return False, "Preamble failed"
        if not self.open_door():
            return False, "Could not reach OPEN"
        self.send("$SENSOR,OBSTRUCTION=1")
        time.sleep(0.3)  # allow obstruction flag to be set
        self.monitor.reset_events()
        self.send("$CMD,TYPE=CLOSE")
        # Give device 2 s; state must NOT change to CLOSING
        time.sleep(2.0)
        state = self.monitor.get_state()
        if state == FsmState.CLOSING:
            return False, "CMD_CLOSE was accepted during active obstruction (should be rejected)"
        return True, ""

    def tc10_obstruction_clear_reenables_close(self):
        """TC-10: After obstruction cleared, CMD_CLOSE is accepted."""
        if not self.reset_and_home():
            return False, "Preamble failed"
        if not self.open_door():
            return False, "Could not reach OPEN"
        self.send("$SENSOR,OBSTRUCTION=1")
        time.sleep(0.3)
        self.send("$SENSOR,OBSTRUCTION=0")
        time.sleep(0.3)
        self.monitor.reset_events()
        self.send("$CMD,TYPE=CLOSE")
        ok = self.wait_for_state(FsmState.CLOSING, MOVE_TIMEOUT_S)
        if not ok:
            return False, f"Expected CLOSING after obstruction clear, got {self.monitor.get_state()}"
        return True, ""

    def tc11_close_during_opening_rejected(self):
        """TC-11: CMD_CLOSE during OPENING must be rejected."""
        if not self.reset_and_home():
            return False, "Preamble failed"
        self.send("$CMD,TYPE=OPEN")
        if not self.wait_for_state(FsmState.OPENING, MOVE_TIMEOUT_S):
            return False, "Could not reach OPENING"
        self.monitor.reset_events()
        self.send("$CMD,TYPE=CLOSE")
        time.sleep(1.5)
        state = self.monitor.get_state()
        if state != FsmState.OPENING:
            return False, f"Expected OPENING to persist; got {state}"
        return True, ""

    def tc12_open_preempts_closing(self):
        """TC-12: CMD_OPEN during CLOSING → OPENING (preemption)."""
        if not self.reset_and_home():
            return False, "Preamble failed"
        if not self.open_door():
            return False, "Could not open door"
        self.send("$CMD,TYPE=CLOSE")
        if not self.wait_for_state(FsmState.CLOSING, MOVE_TIMEOUT_S):
            return False, "Could not reach CLOSING"
        self.monitor.reset_events()
        self.send("$CMD,TYPE=OPEN")
        ok = self.wait_for_state(FsmState.OPENING, MOVE_TIMEOUT_S)
        if not ok:
            return False, f"CMD_OPEN did not preempt CLOSING; state={self.monitor.get_state()}"
        return True, ""

    def tc13_emergency_open_from_closing(self):
        """TC-13: EMERGENCY_OPEN from CLOSING → OPENING + emergency lock."""
        if not self.reset_and_home():
            return False, "Preamble failed"
        if not self.open_door():
            return False, "Could not open door"
        self.send("$CMD,TYPE=CLOSE")
        if not self.wait_for_state(FsmState.CLOSING, MOVE_TIMEOUT_S):
            return False, "Could not reach CLOSING"
        self.monitor.reset_events()
        self.send("$CMD,TYPE=EMERGENCY_OPEN")
        ok = self.wait_for_state(FsmState.OPENING, MOVE_TIMEOUT_S)
        if not ok:
            return False, f"Expected OPENING after EMERGENCY_OPEN, got {self.monitor.get_state()}"
        return True, ""

    def tc14_close_rejected_under_emergency_lock(self):
        """TC-14: CMD_CLOSE must be rejected while emergency lock is active."""
        if not self.reset_and_home():
            return False, "Preamble failed"
        if not self.open_door():
            return False, "Could not open door"
        # Engage emergency lock
        self.send("$CMD,TYPE=EMERGENCY_OPEN")
        if not self.wait_for_state(FsmState.OPENING, MOVE_TIMEOUT_S):
            return False, "EMERGENCY_OPEN did not trigger OPENING"
        self.send("$SENSOR,FULLY_OPEN=1")
        self.wait_for_state(FsmState.OPEN, MOVE_TIMEOUT_S)
        self.monitor.reset_events()
        self.send("$CMD,TYPE=CLOSE")
        time.sleep(2.0)
        state = self.monitor.get_state()
        if state == FsmState.CLOSING:
            return False, "CMD_CLOSE accepted under emergency lock (should be rejected)"
        return True, ""

    def tc15_reset_clears_emergency_lock(self):
        """TC-15: RESET clears emergency lock and restarts homing."""
        if not self.reset_and_home():
            return False, "Initial preamble failed"
        if not self.open_door():
            return False, "Could not open door"
        self.send("$CMD,TYPE=EMERGENCY_OPEN")
        self.wait_for_state(FsmState.OPENING, MOVE_TIMEOUT_S)
        self.monitor.reset_events()
        self.send("$CMD,TYPE=RESET")
        ok = self.wait_for_state(FsmState.HOMING, HOMING_TIMEOUT_S)
        if not ok:
            return False, f"Expected HOMING after RESET, got {self.monitor.get_state()}"
        return True, ""

    def tc16_motor_stall_during_opening(self):
        """TC-16: Motor stall during OPENING → FAULT."""
        if not self.reset_and_home():
            return False, "Preamble failed"
        self.send("$CMD,TYPE=OPEN")
        if not self.wait_for_state(FsmState.OPENING, MOVE_TIMEOUT_S):
            return False, "Could not reach OPENING"
        # Simulate stall: do NOT send FULLY_OPEN; wait for stall timeout
        # The stall timer fires after MOTOR_STALL_TIMEOUT_MS (10 000 ms) + margin
        ok = self.wait_for_state(FsmState.FAULT, STALL_TIMEOUT_S)
        if not ok:
            return False, f"Expected FAULT from stall; got {self.monitor.get_state()}"
        return True, ""

    def tc17_motor_stall_during_closing(self):
        """TC-17: Motor stall during CLOSING → FAULT."""
        if not self.reset_and_home():
            return False, "Preamble failed"
        if not self.open_door():
            return False, "Could not open door"
        self.send("$CMD,TYPE=CLOSE")
        if not self.wait_for_state(FsmState.CLOSING, MOVE_TIMEOUT_S):
            return False, "Could not reach CLOSING"
        ok = self.wait_for_state(FsmState.FAULT, STALL_TIMEOUT_S)
        if not ok:
            return False, f"Expected FAULT from stall; got {self.monitor.get_state()}"
        return True, ""

    def tc18_motor_stall_during_homing(self):
        """TC-18: Motor stall during HOMING → FAULT."""
        self.monitor.reset_events()
        self.send("$CMD,TYPE=RESET")
        if not self.wait_for_state(FsmState.HOMING, HOMING_TIMEOUT_S):
            return False, "Device did not enter HOMING"
        # Do NOT send FULLY_CLOSED; wait for stall
        ok = self.wait_for_state(FsmState.FAULT, STALL_TIMEOUT_S)
        if not ok:
            return False, f"Expected FAULT from stall during homing; got {self.monitor.get_state()}"
        return True, ""

    def tc19_spof_causes_fault(self):
        """TC-19: SPOF (both fully-open and fully-closed simultaneously) → FAULT."""
        if not self.reset_and_home():
            return False, "Preamble failed"
        self.monitor.reset_events()
        # Send contradictory sensor state (SPOF)
        self.send("$SENSOR,FULLY_OPEN=1")
        self.send("$SENSOR,FULLY_CLOSED=1")
        ok = self.wait_for_state(FsmState.FAULT, MOVE_TIMEOUT_S)
        if not ok:
            return False, f"Expected FAULT on SPOF; got {self.monitor.get_state()}"
        return True, ""

    def tc20_comm_timeout_causes_fault(self):
        """TC-20: No communication for COMM_TIMEOUT_MS → FAULT. Validates timing."""
        if not self.reset_and_home():
            return False, "Preamble failed"
        self.monitor.reset_events()
        t_start = time.monotonic()
        # Stop sending; device will fire comm timeout
        ok = self.wait_for_state(FsmState.FAULT, COMM_TIMEOUT_S + COMM_TIMEOUT_MARGIN_S)
        elapsed = time.monotonic() - t_start
        if not ok:
            return False, f"Expected FAULT after comm timeout; got {self.monitor.get_state()}"
        # Validate timing: should be within [COMM_TIMEOUT_S-margin, COMM_TIMEOUT_S+margin]
        expected_lo = (COMM_TIMEOUT_S - COMM_TIMEOUT_MARGIN_S - STATE_SETTLE_S)
        expected_hi = (COMM_TIMEOUT_S + COMM_TIMEOUT_MARGIN_S)
        if not (expected_lo <= elapsed <= expected_hi):
            return False, (f"Timing out of range: elapsed={elapsed:.2f}s, "
                           f"expected [{expected_lo:.1f}, {expected_hi:.1f}]s")
        return True, ""

    def tc21_open_rejected_in_fault(self):
        """TC-21: CMD_OPEN in FAULT must be rejected."""
        # Drive to FAULT via stall
        if not self.reset_and_home():
            return False, "Preamble failed"
        self.send("$CMD,TYPE=OPEN")
        if not self.wait_for_state(FsmState.OPENING, MOVE_TIMEOUT_S):
            return False, "Could not reach OPENING"
        if not self.wait_for_state(FsmState.FAULT, STALL_TIMEOUT_S):
            return False, "Did not reach FAULT"
        self.monitor.reset_events()
        self.send("$CMD,TYPE=OPEN")
        time.sleep(2.0)
        state = self.monitor.get_state()
        if state != FsmState.FAULT:
            return False, f"CMD_OPEN in FAULT changed state to {state}"
        return True, ""

    def tc22_close_rejected_in_fault(self):
        """TC-22: CMD_CLOSE in FAULT must be rejected."""
        if not self.reset_and_home():
            return False, "Preamble failed"
        self.send("$CMD,TYPE=OPEN")
        if not self.wait_for_state(FsmState.OPENING, MOVE_TIMEOUT_S):
            return False, "Could not reach OPENING"
        if not self.wait_for_state(FsmState.FAULT, STALL_TIMEOUT_S):
            return False, "Did not reach FAULT"
        self.monitor.reset_events()
        self.send("$CMD,TYPE=CLOSE")
        time.sleep(2.0)
        state = self.monitor.get_state()
        if state != FsmState.FAULT:
            return False, f"CMD_CLOSE in FAULT changed state to {state}"
        return True, ""

    def tc23_reset_from_fault_restores_homing(self):
        """TC-23: RESET from FAULT → HOMING (safe recovery)."""
        # Enter FAULT via SPOF shortcut (faster than waiting for stall)
        if not self.reset_and_home():
            return False, "Preamble failed"
        self.send("$SENSOR,FULLY_OPEN=1")
        self.send("$SENSOR,FULLY_CLOSED=1")
        if not self.wait_for_state(FsmState.FAULT, MOVE_TIMEOUT_S):
            return False, "Could not reach FAULT for TC-23"
        self.monitor.reset_events()
        self.send("$CMD,TYPE=RESET")
        ok = self.wait_for_state(FsmState.HOMING, HOMING_TIMEOUT_S)
        if not ok:
            return False, f"Expected HOMING after RESET; got {self.monitor.get_state()}"
        return True, ""

    def tc24_nvs_boot_fault(self):
        """
        TC-24: Persistent NVS fault on boot → FSM boots into FAULT.
        Strategy: force a fault, power-cycle by reset without clearing NVS,
        then verify FAULT is entered before normal homing completes.
        NOTE: This TC is best validated via the unit test (TC-24 in test_door_fsm.c).
        In hardware integration we trigger fault then observe EVT_FAULT_PERSIST log.
        """
        # Enter fault (stall) then reset without clearing – observe FAULT_PERSIST log
        if not self.reset_and_home():
            return False, "Preamble failed"
        self.send("$SENSOR,FULLY_OPEN=1")
        self.send("$SENSOR,FULLY_CLOSED=1")
        if not self.wait_for_state(FsmState.FAULT, MOVE_TIMEOUT_S):
            return False, "Could not reach FAULT"
        # Now reset – firmware should detect NVS fault on next boot cycle
        self.monitor.reset_events()
        self.send("$CMD,TYPE=RESET")
        # Check for FAULT_PERSIST log or direct FAULT state (before IDLE)
        line = self.wait_for_log(r"FAULT_PERSIST|fault.*boot|NVS fault", HOMING_TIMEOUT_S)
        if line:
            return True, ""
        # Fallback: if we landed in FAULT before IDLE, also acceptable
        state = self.monitor.get_state()
        if state == FsmState.FAULT:
            return True, ""
        return False, "FAULT_PERSIST not detected after reset with persisted fault"

    def tc25_redundant_open_no_transition(self):
        """TC-25: CMD_OPEN in OPEN → no state change."""
        if not self.reset_and_home():
            return False, "Preamble failed"
        if not self.open_door():
            return False, "Could not reach OPEN"
        self.monitor.reset_events()
        self.send("$CMD,TYPE=OPEN")
        time.sleep(1.5)
        state = self.monitor.get_state()
        if state != FsmState.OPEN:
            return False, f"Redundant CMD_OPEN changed state to {state}"
        return True, ""

    def tc26_redundant_close_no_transition(self):
        """TC-26: CMD_CLOSE in CLOSED → no state change."""
        if not self.reset_and_home():
            return False, "Preamble failed"
        if not self.open_door():
            return False, "Could not open door"
        if not self.close_door():
            return False, "Could not reach CLOSED"
        self.monitor.reset_events()
        self.send("$CMD,TYPE=CLOSE")
        time.sleep(1.5)
        state = self.monitor.get_state()
        if state != FsmState.CLOSED:
            return False, f"Redundant CMD_CLOSE changed state to {state}"
        return True, ""

    def tc27_emergency_open_overrides_fault(self):
        """TC-27: EMERGENCY_OPEN from FAULT overrides fault → OPENING."""
        if not self.reset_and_home():
            return False, "Preamble failed"
        self.send("$SENSOR,FULLY_OPEN=1")
        self.send("$SENSOR,FULLY_CLOSED=1")
        if not self.wait_for_state(FsmState.FAULT, MOVE_TIMEOUT_S):
            return False, "Could not reach FAULT"
        self.monitor.reset_events()
        self.send("$CMD,TYPE=EMERGENCY_OPEN")
        ok = self.wait_for_state(FsmState.OPENING, MOVE_TIMEOUT_S)
        if not ok:
            return False, f"EMERGENCY_OPEN did not override FAULT; got {self.monitor.get_state()}"
        return True, ""

    def tc28_emergency_open_during_closing(self):
        """TC-28: EMERGENCY_OPEN during CLOSING → immediate OPENING."""
        if not self.reset_and_home():
            return False, "Preamble failed"
        if not self.open_door():
            return False, "Could not open door"
        self.send("$CMD,TYPE=CLOSE")
        if not self.wait_for_state(FsmState.CLOSING, MOVE_TIMEOUT_S):
            return False, "Could not reach CLOSING"
        self.monitor.reset_events()
        self.send("$CMD,TYPE=EMERGENCY_OPEN")
        ok = self.wait_for_state(FsmState.OPENING, MOVE_TIMEOUT_S)
        if not ok:
            return False, f"EMERGENCY_OPEN during CLOSING did not yield OPENING; got {self.monitor.get_state()}"
        return True, ""

    def tc29_open_during_homing_rejected(self):
        """TC-29: CMD_OPEN during HOMING must be rejected."""
        self.monitor.reset_events()
        self.send("$CMD,TYPE=RESET")
        if not self.wait_for_state(FsmState.HOMING, HOMING_TIMEOUT_S):
            return False, "Device did not enter HOMING"
        self.monitor.reset_events()
        self.send("$CMD,TYPE=OPEN")
        time.sleep(1.5)
        state = self.monitor.get_state()
        if state != FsmState.HOMING:
            return False, f"CMD_OPEN during HOMING changed state to {state}"
        # Send FULLY_CLOSED to exit homing cleanly for next TC
        self.send("$SENSOR,FULLY_CLOSED=1")
        return True, ""

    def tc30_reset_noop_in_idle(self):
        """TC-30: RESET in IDLE (non-fault, no emergency) → no homing triggered."""
        if not self.reset_and_home():
            return False, "Preamble failed"
        # Verify we're in IDLE
        if self.monitor.get_state() != FsmState.IDLE:
            return False, f"Expected IDLE for TC-30, got {self.monitor.get_state()}"
        self.monitor.reset_events()
        self.send("$CMD,TYPE=RESET")
        # Should remain IDLE – not re-enter HOMING
        time.sleep(2.0)
        state = self.monitor.get_state()
        if state == FsmState.HOMING:
            return False, "RESET triggered spurious HOMING in IDLE (should be no-op)"
        if state != FsmState.IDLE:
            return False, f"Unexpected state after RESET in IDLE: {state}"
        return True, ""

    # ── Test Case Registry ────────────────────────────────────────────────────

    def get_all_test_cases(self):
        """
        Returns ordered list of (tc_id, name, method) tuples.
        Add new TCs here; no other changes needed.
        """
        return [
            (1,  "Boot state is HOMING",                      self.tc01_boot_state_is_homing),
            (2,  "Homing completes → IDLE on FULLY_CLOSED",   self.tc02_homing_completes_to_idle),
            (3,  "CMD_OPEN from IDLE → OPENING",              self.tc03_open_from_idle),
            (4,  "FULLY_OPEN sensor → OPEN",                  self.tc04_fully_open_sensor_to_open),
            (5,  "CMD_CLOSE from OPEN → CLOSING",             self.tc05_close_from_open),
            (6,  "FULLY_CLOSED sensor → CLOSED",              self.tc06_fully_closed_to_closed),
            (7,  "CMD_OPEN from CLOSED → OPENING",            self.tc07_open_from_closed),
            (8,  "Obstruction during CLOSING → OPENING",      self.tc08_obstruction_during_closing_reverses),
            (9,  "CMD_CLOSE rejected during obstruction",     self.tc09_close_rejected_during_obstruction),
            (10, "Obstruction clear re-enables CLOSE",        self.tc10_obstruction_clear_reenables_close),
            (11, "CMD_CLOSE during OPENING rejected",         self.tc11_close_during_opening_rejected),
            (12, "CMD_OPEN preempts CLOSING",                 self.tc12_open_preempts_closing),
            (13, "EMERGENCY_OPEN from CLOSING",               self.tc13_emergency_open_from_closing),
            (14, "CMD_CLOSE rejected under emergency lock",   self.tc14_close_rejected_under_emergency_lock),
            (15, "RESET clears emergency lock",               self.tc15_reset_clears_emergency_lock),
            (16, "Motor stall during OPENING → FAULT",        self.tc16_motor_stall_during_opening),
            (17, "Motor stall during CLOSING → FAULT",        self.tc17_motor_stall_during_closing),
            (18, "Motor stall during HOMING → FAULT",         self.tc18_motor_stall_during_homing),
            (19, "SPOF → FAULT",                              self.tc19_spof_causes_fault),
            (20, "COMM_TIMEOUT → FAULT",                      self.tc20_comm_timeout_causes_fault),
            (21, "CMD_OPEN rejected in FAULT",                self.tc21_open_rejected_in_fault),
            (22, "CMD_CLOSE rejected in FAULT",               self.tc22_close_rejected_in_fault),
            (23, "RESET from FAULT → HOMING",                 self.tc23_reset_from_fault_restores_homing),
            (24, "Persistent NVS fault → FAULT on boot",      self.tc24_nvs_boot_fault),
            (25, "Redundant CMD_OPEN → no transition",        self.tc25_redundant_open_no_transition),
            (26, "Redundant CMD_CLOSE → no transition",       self.tc26_redundant_close_no_transition),
            (27, "EMERGENCY_OPEN overrides FAULT",            self.tc27_emergency_open_overrides_fault),
            (28, "EMERGENCY_OPEN during CLOSING",             self.tc28_emergency_open_during_closing),
            (29, "CMD_OPEN during HOMING rejected",           self.tc29_open_during_homing_rejected),
            (30, "RESET no-op in IDLE",                       self.tc30_reset_noop_in_idle),
        ]

# ─────────────────────────────────────────────────────────────────────────────
# REPORTER
# ─────────────────────────────────────────────────────────────────────────────
class Reporter:
    def __init__(self):
        self.results: List[TestResult] = []

    def record(self, result: TestResult):
        self.results.append(result)
        status = "PASS" if result.passed else "FAIL"
        suffix = f" → {result.reason}" if not result.passed else ""
        print(f"[TC-{result.tc_id:02d}] {status}{suffix}  ({result.duration_s:.1f}s)")

    def print_summary(self):
        total  = len(self.results)
        passed = sum(1 for r in self.results if r.passed)
        failed = total - passed
        print("\n" + "═" * 55)
        print("SUMMARY")
        print("═" * 55)
        for r in self.results:
            flag = "✓" if r.passed else "✗"
            print(f"  {flag} TC-{r.tc_id:02d}  {r.name}")
        print("─" * 55)
        print(f"  {passed} PASS, {failed} FAIL  ({total} total)")
        print("═" * 55)
        return failed == 0

# ─────────────────────────────────────────────────────────────────────────────
# MAIN ENTRY POINT
# ─────────────────────────────────────────────────────────────────────────────
def parse_args():
    ap = argparse.ArgumentParser(description="ESP32 Elevator Door FSM UART Test Suite")
    ap.add_argument("--port", default=DEFAULT_PORT,  help="Serial port (e.g. COM5 or /dev/ttyUSB0)")
    ap.add_argument("--baud", default=DEFAULT_BAUD,  type=int, help="Baud rate")
    ap.add_argument("--tc",   default=None,          type=int, nargs="+",
                    help="Run specific TC(s) only. E.g. --tc 1 3 5")
    return ap.parse_args()


def main():
    args   = parse_args()
    uart   = UARTInterface(args.port, args.baud)
    reporter = Reporter()

    try:
        uart.open()
    except serial.SerialException as e:
        print(f"[ERROR] Cannot open serial port: {e}")
        sys.exit(1)

    monitor = DeviceMonitor(uart)
    monitor.start()

    # Allow device to boot / settle before first command
    print(f"[INIT] Waiting {BOOT_SETTLE_S}s for device to boot...")
    time.sleep(BOOT_SETTLE_S)

    runner   = TestRunner(uart, monitor)
    all_tcs  = runner.get_all_test_cases()

    # Filter to requested TCs
    if args.tc:
        all_tcs = [(tc_id, name, fn) for (tc_id, name, fn) in all_tcs if tc_id in args.tc]
        if not all_tcs:
            print("[ERROR] No matching test cases found for given --tc values")
            sys.exit(1)

    print(f"\nRunning {len(all_tcs)} test case(s) on {args.port} @ {args.baud}\n")
    print("─" * 55)

    for (tc_id, name, fn) in all_tcs:
        print(f"\n>>> TC-{tc_id:02d}: {name}")
        t0 = time.monotonic()
        try:
            passed, reason = fn()
        except Exception as exc:
            passed = False
            reason = f"Exception: {exc}"
        duration = time.monotonic() - t0
        reporter.record(TestResult(tc_id, name, passed, reason, duration))
        # Brief settle between TCs
        time.sleep(STATE_SETTLE_S)

    all_passed = reporter.print_summary()
    monitor.stop()
    uart.close()
    sys.exit(0 if all_passed else 1)


if __name__ == "__main__":
    main()