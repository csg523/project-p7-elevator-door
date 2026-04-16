// =============================================================
// protocol/protocol.h — UART framing, CRC-8, parse/format
//
// Wire format (ASCII, newline-terminated):
//   Incoming: $CMD,TYPE=<CMD>,SEQ=<N>,CRC=<XX>\n
//   Outgoing: $ACK,TYPE=<CMD>,SEQ=<N>,TS=<MS>\n
//             $NACK,REASON=<R>,SEQ=<N>\n
//             $STATE,STATE=<S>,TS=<MS>\n
//             $LOG,MSG=<M>,TS=<MS>\n
//
// CRC is computed over all bytes from '$' up to (not including)
// ',CRC=', expressed as two uppercase hex digits.
// =============================================================
#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "fsm/fsm.h"

// -------------------------------------------------------
// Parsed incoming message
// -------------------------------------------------------
#define CMD_STR_MAX 24

typedef struct {
    bool    valid;            // Frame structurally correct + CRC OK
    char    cmd[CMD_STR_MAX]; // e.g. "OPEN", "CLOSE", "EMERGENCY_OPEN"
    int32_t seq;              // Sequence number (>=1)
    uint8_t crc_recv;         // CRC byte from frame
    uint8_t crc_calc;         // Computed CRC (for diagnostics)
} ParsedMsg;

// -------------------------------------------------------
// CRC-8 (Dallas/Maxim 1-Wire, poly=0x8C)
// -------------------------------------------------------
uint8_t crc8_compute(const uint8_t *data, size_t len);
uint8_t crc8_str(const char *str);

// -------------------------------------------------------
// Parsing
// -------------------------------------------------------
// Parse a null-terminated frame (leading '$', no trailing '\n').
// Returns filled ParsedMsg; valid=false on any error.
ParsedMsg proto_parse(const char *frame);

// -------------------------------------------------------
// Formatting outgoing messages into caller-supplied buffer
// All functions NUL-terminate and return bytes written (excl. NUL).
// -------------------------------------------------------
int proto_format_ack  (char *buf, size_t sz,
                       const char *cmdType, int32_t seq);
int proto_format_nack (char *buf, size_t sz,
                       const char *reason,  int32_t seq);
int proto_format_state(char *buf, size_t sz, DoorState state);
