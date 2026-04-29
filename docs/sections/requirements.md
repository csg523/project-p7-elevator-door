# Elevator Door Control System — Requirements Specification

---

## 1. Stakeholder Requirements

1. **SRK-1 — Bounded Real-Time Response**
   The system responds to external door-related events within bounded and verifiable time limits, independent of system load and scheduling variations.

2. **SRK-2 — Safety-Critical Operation**
   The system prevents unsafe door motion under all operating and fault conditions, including normal operation, communication loss, sensor faults, emergency conditions, and scheduling delays or failures.

3. **SRK-3 — Fail-Safe Behavior**
   The system detects abnormal, missing, delayed, or inconsistent inputs and behaves conservatively by transitioning to a safe state within a bounded and verifiable time.

4. **SRK-4 — Safe Recovery**
   The system does not initiate or resume door motion after reset, power loss, or fault recovery without an explicit valid command from the supervisor.

5. **SRK-5 — Observability and Testability**
   The system provides sufficient state visibility, event logging, and traceability to support real-time monitoring, offline debugging, and deterministic replay of scenarios.

6. **SRK-6 — Distributed System Integrity**
   The system maintains safe and correct operation under loss, delay, duplication, or corruption of UART communication, ensuring no unsafe behavior occurs due to communication faults.

---

## 2. Functional Requirements

### FR-1 — Command-Based Door Opening

The system shall transition to door opening only upon receiving a valid `CMD=OPEN` or `CMD=EMERGENCY_OPEN` from the supervisor, provided all safety constraints are satisfied.

**Test Intent:**
Violation is detected if door opening occurs without a valid command or while any safety constraint is active.

---

### FR-2 — Command-Based Door Closing

The system shall initiate door closing only upon receiving a valid `CMD=CLOSE`, and only when the door is not already in a closing or closed condition and no safety constraints are active.

**Test Intent:**
Violation is detected if a CLOSE command is issued while the door-closed condition is already present, while a safety constraint is active, or while the door is actively opening (see FR-3).

---

### FR-3 — Command Preemption and Priority

The system shall enforce the following command priority order:

```
EMERGENCY_OPEN  >  OPEN  >  CLOSE
```

- `CMD=EMERGENCY_OPEN` shall immediately override **any** active state.
- `CMD=OPEN` shall immediately preempt an active `CLOSING` operation.
- `CMD=CLOSE` shall be **rejected** (no state change) if received while the door is actively `OPENING`. The Supervisor must wait for the `OPEN` state before issuing `CMD=CLOSE`.

**Rationale:**
Allowing CLOSE to interrupt OPENING creates a race condition and contradicts the OPEN > CLOSE priority rule. The system must complete the higher-priority OPEN motion before accepting a CLOSE.

**Test Intent:**
Violation is detected if:
- A higher-priority command does not immediately override a lower-priority operation in progress.
- `CMD=CLOSE` during `OPENING` causes a state transition to `CLOSING` (priority inversion).

---

### FR-4 — Redundant Command Suppression

The system shall ignore commands that do not result in a valid state transition (e.g., `CMD=OPEN` while already in `OPEN`), acknowledging them with an `ACK` but producing no state change.

**Test Intent:**
Violation is detected if a redundant or inapplicable command triggers an unintended state transition or generates a spurious motor output.

---

### FR-5 — State Reporting

The system shall periodically transmit its current state to the supervisor at an interval of ≤ 200 ms (nominal: every 100 ms).

Example transmission:
```
$STATE,STATE=OPENING,TS=12345
```

**Test Intent:**
Violation is detected if state reports are absent, delayed beyond 200 ms, or contain incorrect state information.

---

### FR-6 — Command Acknowledgment

The system shall acknowledge each received command with an `ACK` or `NACK` based on the command's validity and CRC integrity at the time of receipt.

- `ACK` is sent for structurally valid, CRC-correct frames (regardless of whether a state transition occurs).
- `NACK` is sent for frames with CRC errors, malformed fields, or unknown command types.

**Test Intent:**
Violation is detected if a CRC-invalid command receives an `ACK`, or if a valid command receives no acknowledgment.

---

### FR-7 — Task-Based Execution (RTOS-Specific)

The system shall implement functional behavior using independent FreeRTOS tasks:

- Control task / FSM (Priority 4)
- Safety monitoring task (Priority 5 — highest)
- UART RX task (Priority 3)
- UART TX task (Priority 2)
- Logging task (Priority 1)
- Watchdog task (Priority 3 — FSM liveness only)

Each task shall operate independently and communicate exclusively via FreeRTOS queues and atomically-written shared flags.

**Test Intent:**
Violation is detected if any task directly accesses another task's internal state without queue-based synchronization, or if a task failure causes another task to block or behave incorrectly.

---

## 3. Safety Requirements

### SR-1 — Obstruction Prevention

The system shall not continue door closing while an obstruction signal is present.

**Test Intent:**
Violation is detected if door-closing motion continues while the obstruction signal is active.

---

### SR-2 — Obstruction Reaction Time

The system shall stop or reverse door motion within a bounded time (≤ 20 ms at the hardware level; ≤ 100 ms over the UART simulation interface) after obstruction detection during a closing operation.

**Test Intent:**
Violation is detected if door motion continues beyond the specified response time after obstruction detection.

---

### SR-3 — Sensor Fault Handling

If sensor data is missing, delayed, or inconsistent, the system shall:

- Stop all door motion immediately
- Transition to the `FAULT` state

**Test Intent:**
Violation is detected if door motion continues after sensor data becomes missing, invalid, or inconsistent.

---

### SR-4 — Power-On Safety

After reset or power restoration, the system shall:

- Initialize to the `IDLE` state
- Prohibit all door motion until a valid explicit command is received

**Test Intent:**
Violation is detected if door motion resumes automatically after power restoration or reset without a new valid command.

---

### SR-5 — Communication Loss Safety

If no valid command is received within `COMM_TIMEOUT_MS` (configurable; production target: 100 ms), the system shall:

- Stop all door motion
- Transition to the `FAULT` state

This requirement applies in **all** non-terminal states (i.e., all states except `FAULT` and `EMERGENCY`), not only during active motion.

**Test Intent:**
Violation is detected if door motion continues or the system does not transition to `FAULT` within the defined timeout period following communication loss, regardless of the current state.

---

### SR-6 — Emergency Override Guarantee

Upon receiving `CMD=EMERGENCY_OPEN`, the system shall:

- Immediately interrupt all active motion
- Transition to `EMERGENCY` state (motor opens) within a bounded time
- Remain in `EMERGENCY` state with the door open until an explicit `RESET` command is received
- **Reject** `CMD=CLOSE` while in `EMERGENCY` state

**Test Intent:**
Violation is detected if the system fails to interrupt active motion, does not transition to `EMERGENCY` within the bounded time, closes automatically without an explicit `RESET`, or accepts `CMD=CLOSE` while in `EMERGENCY`.

---

### SR-7 — Task Failure Containment (RTOS-Specific)

Failure or delay of non-critical tasks (e.g., logging or UART TX) shall not affect control task execution, delay safety responses, or violate any timing guarantees.

**Test Intent:**
Violation is detected if a simulated failure or artificial delay in a non-critical task causes the control or safety task to miss its deadline or produce an incorrect output.

---

## 4. Non-Functional Requirements

### NFR-1 — Hard Real-Time Response

The system shall process and respond to all input events within a worst-case execution time (WCET) of ≤ 10 ms.

**Test Intent:**
Violation is detected if the measured response time exceeds 10 ms under any operating condition.

---

### NFR-2 — Deterministic Execution

The system shall produce identical outputs for identical input sequences, regardless of task scheduling order or timing jitter.

**Test Intent:**
Violation is detected if repeated test runs with identical input sequences produce different outputs.

---

### NFR-3 — Resource Boundedness

The system shall operate within the following predefined limits:

- CPU utilization: ≤ 70%
- Stack usage: bounded per task (defined in `config.h`)
- Dynamic memory allocation: prohibited during runtime

**Test Intent:**
Violation is detected if CPU utilization or memory usage exceeds predefined limits during normal operation or stress testing.

---

### NFR-4 — Event Logging

The system shall log state transitions, commands received, and fault events with timestamps for offline analysis.

**Test Intent:**
Violation is detected if a required state transition, received command, or fault event occurs without a corresponding timestamped log entry.

---

### NFR-5 — Communication Determinism

All UART communication shall be framed, CRC-8 validated (Dallas/Maxim polynomial), and sequence-controlled to ensure reliable and deterministic message handling.

**Test Intent:**
Violation is detected if a message is accepted without a valid CRC, if out-of-sequence messages are processed without detection, or if framing errors go unreported.

---

### NFR-6 — Timing Isolation

Critical control and safety tasks shall not be affected by logging operations, UART transmission delays, or debug instrumentation.

**Test Intent:**
Violation is detected if enabling logging or debug output causes a control or safety task to miss its deadline.

---

### NFR-7 — Scheduling Determinism (RTOS-Specific)

The system shall enforce fixed-priority preemptive scheduling with bounded task execution times and no unbounded priority inversion.

Task priorities (highest to lowest): Safety (5) > FSM (4) > UART_RX = Watchdog (3) > UART_TX (2) > Logger = Display (1).

**Test Intent:**
Violation is detected if a lower-priority task preempts a higher-priority task, or if priority inversion is observed to extend a higher-priority task's blocking time beyond a bounded limit.

---

### NFR-8 — Memory Safety

The system shall avoid memory leaks, dynamic allocation failures, and stack overflows across all tasks.

**Test Intent:**
Violation is detected if any task exceeds its allocated stack size, if a dynamic allocation fails at runtime, or if a memory leak is observed during extended operation.
