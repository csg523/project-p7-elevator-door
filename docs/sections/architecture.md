# Architecture

Architecture description.
System --> UART["UART INTERFACE
Responsibility:
- Parse external commands
Encapsulates:
- RXBuffer
Interface:
- parse_frame()
Owner:
- Ankit
"]
System --> Logger["LOGGER
Responsibility:
- Record transitions
Encapsulates:
- LogBuffer
Interface:
- log_event()
Owner:
- Ankit
"]
