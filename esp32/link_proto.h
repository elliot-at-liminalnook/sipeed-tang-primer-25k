/*
 * link_proto -- host/ESP32 <-> FPGA frame codec.
 *
 *     AA 55 | LEN | SEQ | OP | payload[LEN] | CRC8
 *
 * Deliberately free of any platform dependency: no ESP-IDF, no sockets, no
 * malloc. The SAME file compiles into the ESP32 firmware and into the
 * Verilator harness that drives the simulated FPGA.
 *
 * That is the point. The C encoder and the Verilog parser are two
 * independent implementations of one wire format, and the closed-loop test
 * runs them against each other -- so a disagreement about CRC order or
 * endianness fails in simulation rather than on a robot.
 *
 * Mirrors src/link_rx.v and src/link_tx.v. Change one, change all three.
 */
#ifndef LINK_PROTO_H
#define LINK_PROTO_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LINK_SYNC0       0xAA
#define LINK_SYNC1       0x55
#define LINK_MAX_PAYLOAD 64
#define LINK_MAX_FRAME   (6 + LINK_MAX_PAYLOAD)

/* host -> FPGA */
#define OP_SET_TARGETS   0x01
#define OP_SET_TGT_SPEED 0x02
#define OP_SPIN          0x03
#define OP_STOP          0x04
#define OP_PWM           0x05
#define OP_RAW           0x10
#define OP_CONFIG        0x20
#define OP_KEEPALIVE     0x7F

/* FPGA -> host */
#define OP_OBS           0x81
#define OP_FAULT         0x82

/* CRC-8/ATM: poly 0x07, init 0x00, no reflection, no final xor. */
uint8_t link_crc8(const uint8_t *data, size_t len);

/*
 * Encode one frame into `out` (needs LINK_MAX_FRAME bytes).
 * Returns the byte count, or 0 if len exceeds LINK_MAX_PAYLOAD.
 */
size_t link_encode(uint8_t *out, uint8_t seq, uint8_t op,
                   const uint8_t *payload, uint8_t len);

/* Incremental byte-at-a-time decoder, matching the RTL state machine. */
typedef struct {
    int      state;
    uint8_t  len, seq, op, crc;
    uint8_t  idx;
    uint8_t  payload[LINK_MAX_PAYLOAD];
    uint32_t crc_errors;
    uint32_t frames;
} link_parser_t;

void link_parser_init(link_parser_t *p);

/* Returns 1 when a CRC-valid frame has just completed, else 0. */
int link_parse_byte(link_parser_t *p, uint8_t b);

/* ---- payload helpers ------------------------------------------------- */

/* SET_TARGETS payload: n, then (id, pos_lo, pos_hi) x n. */
size_t link_build_targets(uint8_t *payload, const uint8_t *ids,
                          const uint16_t *positions, uint8_t n);

static inline uint16_t link_rd16(const uint8_t *p) {
    return (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
}
static inline uint32_t link_rd32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

#ifdef __cplusplus
}
#endif
#endif /* LINK_PROTO_H */
