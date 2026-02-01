Stakeholder Requirements-
1. Responds to external door-related events within bounded and verifiable time limits
2. Prevents unsafe door motion under all operating and fault conditions
3. Detects abnormal or missing inputs and behaves conservatively
4. Does not initiate unexpected door movement after power loss or reset
5. Supports systematic testing and post-operation analysis

Functional Requirements-
1. FR-1 Door Opening Control
    The system shall command the elevator door to open only when a valid door-open request is received and all safety conditions are satisfied.
Test Intent:
    Violation is detected if an OPEN command is issued without a valid open request or while any safety constraint is active.

2. FR-2 Door Closing Control
    The system shall accept a door-close request only when the door is not already fully closed and no safety constraints are violated.
Test Intent:
    Violation is detected if a CLOSE command is issued while the door-closed condition is present or while a safety constraint is active.

3. FR-3 Command Interruption
    The system shall allow a valid door-open request to interrupt an ongoing door-closing operation.
Test Intent:
    Violation is detected if an OPEN request during closing does not cause closing to stop and opening to begin.

4. FR-4 Redundant Command Handling
    The system shall ignore door-open requests when the door is already fully open.
Test Intent:
    Violation is detected if additional OPEN commands are generated while the door-open condition is present.

Safety Requirements
1. SR-1 Obstruction Protection
    The system shall not command the elevator door to close while an obstruction signal is present.
Test Intent:
    Violation is detected if a CLOSE command is issued while the obstruction signal is active.

2. SR-2 Obstruction Response
    The system shall stop door motion within a bounded time after an obstruction is detected during door closing.
Test Intent:
    Violation is detected if door motion continues beyond the specified response time after obstruction detection.

3. SR-3 Fault-Induced Safe State
    The system shall transition to a safe, non-moving state when door sensor data is missing, invalid, delayed, or inconsistent.
Test Intent:
    Violation is detected if door motion continues after sensor data becomes missing or invalid.

4. SR-4 Power Loss Safety
    The system shall not automatically resume door motion after a power loss or system reset without an explicit command.
Test Intent:
    Violation is detected if door motion resumes automatically after power restoration without a new command.

Non-Functional Requirements
1. NFR-1 Timing Determinism
    The system shall respond to valid external events within 10 ms under defined operating conditions.
Test Intent:
    Violation is detected if measured response time exceeds 10 ms.

2. NFR-2 Deterministic Behavior
    The system shall exhibit deterministic behavior for identical sequences of inputs.
Test Intent:
    Violation is detected if repeated test runs with identical input sequences produce different outputs.

3. NFR-3 Resource Boundedness
    The system shall operate within bounded memory and CPU usage during normal operation.
Test Intent:
    Violation is detected if memory or CPU usage exceeds predefined limits during stress testing.

4. NFR-4 Event Logging
    The system shall log door state transitions and detected fault events for offline analysis.
Test Intent:
    Violation is detected if a required state transition or fault occurs without a corresponding log entry.