/* link_proto -- see link_proto.h. Platform-free by design. */

#include "link_proto.h"

uint8_t link_crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07)
                               : (uint8_t)(crc << 1);
    }
    return crc;
}

size_t link_encode(uint8_t *out, uint8_t seq, uint8_t op,
                   const uint8_t *payload, uint8_t len)
{
    if (len > LINK_MAX_PAYLOAD)
        return 0;

    size_t n = 0;
    out[n++] = LINK_SYNC0;
    out[n++] = LINK_SYNC1;

    /* The sync bytes are outside the CRC: they carry no information, and
     * including them would only dilute the check. Matches link_rx.v. */
    size_t crc_start = n;
    out[n++] = len;
    out[n++] = seq;
    out[n++] = op;
    for (uint8_t i = 0; i < len; i++)
        out[n++] = payload[i];

    out[n] = link_crc8(&out[crc_start], n - crc_start);
    n++;
    return n;
}

enum { S_A5 = 0, S_55, S_LEN, S_SEQ, S_OP, S_PAY, S_CRC };

void link_parser_init(link_parser_t *p)
{
    p->state = S_A5;
    p->len = p->seq = p->op = p->crc = 0;
    p->idx = 0;
    p->crc_errors = 0;
    p->frames = 0;
}

static uint8_t crc_step(uint8_t crc, uint8_t d)
{
    crc ^= d;
    for (int b = 0; b < 8; b++)
        crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    return crc;
}

int link_parse_byte(link_parser_t *p, uint8_t b)
{
    switch (p->state) {
    case S_A5:
        if (b == LINK_SYNC0) p->state = S_55;
        break;

    case S_55:
        /* A second 0xAA is still a plausible frame start -- hold position
         * rather than dropping back and losing it. Matches link_rx.v. */
        if (b == LINK_SYNC1)      p->state = S_LEN;
        else if (b == LINK_SYNC0) p->state = S_55;
        else                      p->state = S_A5;
        break;

    case S_LEN:
        if (b > LINK_MAX_PAYLOAD) { p->state = S_A5; break; }
        p->len   = b;
        p->crc   = crc_step(0x00, b);
        p->idx   = 0;
        p->state = S_SEQ;
        break;

    case S_SEQ:
        p->seq   = b;
        p->crc   = crc_step(p->crc, b);
        p->state = S_OP;
        break;

    case S_OP:
        p->op    = b;
        p->crc   = crc_step(p->crc, b);
        p->state = (p->len == 0) ? S_CRC : S_PAY;
        break;

    case S_PAY:
        p->payload[p->idx++] = b;
        p->crc = crc_step(p->crc, b);
        if (p->idx == p->len) p->state = S_CRC;
        break;

    case S_CRC:
        p->state = S_A5;
        if (b == p->crc) { p->frames++; return 1; }
        p->crc_errors++;
        break;

    default:
        p->state = S_A5;
        break;
    }
    return 0;
}

size_t link_build_targets(uint8_t *payload, const uint8_t *ids,
                          const uint16_t *positions, uint8_t n)
{
    size_t k = 0;
    payload[k++] = n;
    for (uint8_t i = 0; i < n; i++) {
        payload[k++] = ids[i];
        payload[k++] = (uint8_t)(positions[i] & 0xFF);
        payload[k++] = (uint8_t)(positions[i] >> 8);
    }
    return k;
}
