// =============================================================
// protocol/protocol.cpp — CRC-8, frame parser, message formatters
// =============================================================
#include "protocol.h"
#include "config.h"
#include "fsm/fsm.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <Arduino.h>   // millis(), snprintf on ESP32

// -------------------------------------------------------
// CRC-8 Dallas/Maxim (1-Wire) — polynomial 0x8C
// -------------------------------------------------------
uint8_t crc8_compute(const uint8_t *data, size_t len)
{
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) {
        uint8_t b = data[i];
        for (int j = 0; j < 8; j++) {
            if ((crc ^ b) & 0x01) {
                crc = (crc >> 1) ^ CRC8_POLY;
            } else {
                crc >>= 1;
            }
            b >>= 1;
        }
    }
    return crc;
}

uint8_t crc8_str(const char *str)
{
    return crc8_compute((const uint8_t *)str, strlen(str));
}

// -------------------------------------------------------
// Helper: find value after "KEY=" up to next ',' or '\0'
// Returns pointer to start of value, length via *outLen.
// Returns NULL if key not found.
// -------------------------------------------------------
static const char *find_field(const char *frame,
                               const char *key, size_t *outLen)
{
    const char *p = strstr(frame, key);
    if (!p) return NULL;
    p += strlen(key);
    const char *end = strchr(p, ',');
    *outLen = end ? (size_t)(end - p) : strlen(p);
    return p;
}

// -------------------------------------------------------
// proto_parse
// Frame example: "$CMD,TYPE=OPEN,SEQ=3,CRC=A7"
// -------------------------------------------------------
ParsedMsg proto_parse(const char *frame)
{
    ParsedMsg msg;
    memset(&msg, 0, sizeof(msg));

    // Must start with '$'
    if (!frame || frame[0] != PROTO_START_CHAR) return msg;

    // Locate CRC field
    const char *crcTag = strstr(frame, ",CRC=");
    if (!crcTag) return msg;

    // Compute CRC over everything before ",CRC="
    size_t dataLen = (size_t)(crcTag - frame);
    msg.crc_calc = crc8_compute((const uint8_t *)frame, dataLen);

    // Parse received CRC (hex string after ",CRC=")
    const char *crcHex = crcTag + 5;  // skip ",CRC="
    msg.crc_recv = (uint8_t)strtol(crcHex, NULL, 16);

    if (msg.crc_recv != msg.crc_calc) {
        // CRC mismatch — frame corrupted
        return msg;  // valid stays false
    }

    // Parse TYPE field
    size_t typeLen = 0;
    const char *typeVal = find_field(frame, "TYPE=", &typeLen);
    if (!typeVal || typeLen == 0 || typeLen >= CMD_STR_MAX) return msg;
    strncpy(msg.cmd, typeVal, typeLen);
    msg.cmd[typeLen] = '\0';

    // Parse SEQ field
    size_t seqLen = 0;
    const char *seqVal = find_field(frame, "SEQ=", &seqLen);
    if (!seqVal || seqLen == 0) return msg;
    msg.seq = atoi(seqVal);
    if (msg.seq < 1) return msg;

    msg.valid = true;
    return msg;
}

// -------------------------------------------------------
// Formatters
// -------------------------------------------------------
int proto_format_ack(char *buf, size_t sz,
                     const char *cmdType, int32_t seq)
{
    return snprintf(buf, sz, "$ACK,TYPE=%s,SEQ=%d,TS=%lu\n",
                    cmdType, (int)seq, (unsigned long)millis());
}

int proto_format_nack(char *buf, size_t sz,
                      const char *reason, int32_t seq)
{
    return snprintf(buf, sz, "$NACK,REASON=%s,SEQ=%d\n",
                    reason, (int)seq);
}

int proto_format_state(char *buf, size_t sz, DoorState state)
{
    return snprintf(buf, sz, "$STATE,STATE=%s,TS=%lu\n",
                    fsm_state_name(state),
                    (unsigned long)millis());
}
