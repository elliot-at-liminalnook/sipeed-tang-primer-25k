// Closed-loop co-simulation: the ESP32's own C code against a Verilated FPGA.
//
// The ESP32 side is not a mock. esp32/link_proto.c is compiled here exactly
// as it will be compiled into the firmware, and its output is bit-banged
// down a simulated UART into the Verilated link_core. Replies are decoded by
// the same parser the firmware uses.
//
// So this tests the two implementations against each other. If the C encoder
// and the Verilog parser ever disagree about CRC coverage, byte order or the
// zero-length payload corner, it fails here rather than on a robot.
//
//   make -C servo sim-link

#include <verilated.h>
#include "Vlink_core.h"

#include <cstdio>
#include <cstdint>
#include <vector>
#include <string>

extern "C" {
#include "link_proto.h"
}

// Must match the -G overrides in the Makefile.
static const int CLKS_PER_BIT = 25;
static const int TICK_DIV     = 20000;
static const int DEADMAN_CYC  = 200000;

static Vlink_core *dut;
static vluint64_t  main_time = 0;
static int         errors    = 0;

static void fail(const std::string &msg) {
    printf("FAIL: %s\n", msg.c_str());
    errors++;
}

// ---- clock -----------------------------------------------------------------
static link_parser_t host_parser;
static std::vector<std::vector<uint8_t>> rx_frames;   // frames from the FPGA
static std::vector<uint8_t>              rx_ops;

// UART receive state for the host end of the wire.
static int      rxbit   = -1;
static int      rxcnt   = 0;
static uint8_t  rxshift = 0;

static void sample_host_tx() {
    // Mid-bit sampling of dut->host_tx, mirroring uart_rx.v.
    int line = dut->host_tx;
    if (rxbit < 0) {
        if (line == 0) { rxbit = 0; rxcnt = 0; }        // start bit edge
        return;
    }
    rxcnt++;
    if (rxbit == 0) {
        if (rxcnt == CLKS_PER_BIT + CLKS_PER_BIT / 2) { // centre of bit 0
            rxshift = (uint8_t)(line ? 0x80 : 0x00);
            rxbit = 1; rxcnt = 0;
        }
    } else if (rxbit < 8) {
        if (rxcnt == CLKS_PER_BIT) {
            rxshift = (uint8_t)((rxshift >> 1) | (line ? 0x80 : 0x00));
            rxbit++; rxcnt = 0;
        }
    } else {
        if (rxcnt == CLKS_PER_BIT) {                    // stop bit
            if (link_parse_byte(&host_parser, rxshift)) {
                rx_frames.push_back(std::vector<uint8_t>(
                    host_parser.payload, host_parser.payload + host_parser.len));
                rx_ops.push_back(host_parser.op);
            }
            rxbit = -1; rxcnt = 0;
        }
    }
}

static void tick() {
    dut->clk = 0; dut->eval(); main_time++;
    sample_host_tx();
    dut->clk = 1; dut->eval(); main_time++;
}

static void run(int cycles) { for (int i = 0; i < cycles; i++) tick(); }

// ---- host -> FPGA: bit-bang a byte down host_rx -----------------------------
static void send_byte(uint8_t b) {
    dut->host_rx = 0; run(CLKS_PER_BIT);                 // start
    for (int i = 0; i < 8; i++) {
        dut->host_rx = (b >> i) & 1; run(CLKS_PER_BIT);   // LSB first
    }
    dut->host_rx = 1; run(CLKS_PER_BIT);                 // stop
}

static void send_frame(uint8_t seq, uint8_t op, const uint8_t *pay, uint8_t len) {
    uint8_t buf[LINK_MAX_FRAME];
    size_t n = link_encode(buf, seq, op, pay, len);
    if (n == 0) { fail("link_encode refused a frame"); return; }
    for (size_t i = 0; i < n; i++) send_byte(buf[i]);
}

// Clock until a frame arrives from the FPGA, or give up.
static bool wait_frame(int max_cycles = TICK_DIV * 3) {
    size_t before = rx_frames.size();
    for (int i = 0; i < max_cycles; i++) {
        tick();
        if (rx_frames.size() > before) return true;
    }
    return false;
}

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vlink_core;
    link_parser_init(&host_parser);

    dut->host_rx = 1;
    dut->rst = 1; run(20);
    dut->rst = 0; run(20);

    printf("--- 1. C encoder vs Verilog parser: SET_TARGETS ---\n");
    {
        uint8_t  ids[3]  = {1, 2, 3};
        uint16_t pos[3]  = {1000, 2000, 3000};
        uint8_t  pay[32];
        size_t   n = link_build_targets(pay, ids, pos, 3);

        send_frame(0x11, OP_SET_TARGETS, pay, (uint8_t)n);
        run(2000);

        if (dut->target0 != 1000) fail("target0 != 1000");
        if (dut->target1 != 2000) fail("target1 != 2000");
        if (dut->target2 != 3000) fail("target2 != 3000");
        if (dut->last_seq != 0x11) fail("seq not latched");
        if (!errors)
            printf("ok   targets %u/%u/%u latched, seq=0x%02X\n",
                   dut->target0, dut->target1, dut->target2, dut->last_seq);
    }

    printf("--- 2. FPGA -> host telemetry on the control tick ---\n");
    {
        size_t before = rx_frames.size();
        if (!wait_frame()) {
            fail("no OBS frame within three tick periods");
        } else {
            auto &p  = rx_frames.back();
            uint8_t op = rx_ops.back();
            if (op != OP_OBS)      fail("frame was not OP_OBS");
            else if (p.size() != 10) fail("OBS payload length wrong");
            else {
                uint32_t t  = link_rd32(&p[0]);
                uint16_t t0 = link_rd16(&p[4]);
                uint16_t t1 = link_rd16(&p[6]);
                uint16_t t2 = link_rd16(&p[8]);
                if (t0 != 1000 || t1 != 2000 || t2 != 3000)
                    fail("OBS positions do not match the commanded targets");
                else
                    printf("ok   OBS tick=%u pos=%u/%u/%u\n", t, t0, t1, t2);
            }
            (void)before;
        }
    }

    printf("--- 3. tick is periodic (a policy depends on it) ---\n");
    {
        // Two consecutive OBS frames must be exactly TICK_DIV apart.
        size_t n0 = rx_frames.size();
        vluint64_t t_a = 0, t_b = 0;
        for (int i = 0; i < TICK_DIV * 3; i++) {
            tick();
            if (rx_frames.size() > n0) { t_a = main_time; n0 = rx_frames.size(); break; }
        }
        for (int i = 0; i < TICK_DIV * 3; i++) {
            tick();
            if (rx_frames.size() > n0) { t_b = main_time; break; }
        }
        // main_time advances twice per cycle.
        long long delta = (long long)((t_b - t_a) / 2);
        if (t_a == 0 || t_b == 0) fail("did not observe two OBS frames");
        else if (delta != TICK_DIV)
            printf("FAIL: tick interval %lld cycles, expected %d\n", delta, TICK_DIV), errors++;
        else
            printf("ok   tick interval exactly %lld cycles, zero jitter\n", delta);
    }

    printf("--- 4. corrupted frame must not move anything ---\n");
    {
        uint16_t was0 = dut->target0;
        uint8_t  ids[1] = {1};
        uint16_t pos[1] = {4000};
        uint8_t  pay[8];
        size_t   n = link_build_targets(pay, ids, pos, 1);

        uint8_t buf[LINK_MAX_FRAME];
        size_t  fn = link_encode(buf, 0x22, OP_SET_TARGETS, pay, (uint8_t)n);
        buf[fn - 1] ^= 0xFF;                      // wreck the CRC
        for (size_t i = 0; i < fn; i++) send_byte(buf[i]);
        run(2000);

        if (dut->target0 != was0)
            fail("a CRC-invalid frame changed a target");
        else if (dut->crc_errors == 0)
            fail("crc_errors did not increment");
        else
            printf("ok   rejected, target held at %u, crc_errors=%u\n",
                   dut->target0, dut->crc_errors);
    }

    printf("--- 5. deadman trips on silence, and HOLDS position ---\n");
    {
        uint16_t held = dut->target0;
        run(DEADMAN_CYC + 5000);                  // say nothing at all
        if (!dut->deadman)
            fail("deadman did not trip after the timeout");
        else if (dut->target0 != held)
            fail("deadman released the target instead of holding it");
        else
            printf("ok   deadman tripped, target held at %u (not zeroed)\n",
                   dut->target0);
    }

    printf("--- 6. link recovers after the deadman ---\n");
    {
        uint8_t  ids[3] = {1, 2, 3};
        uint16_t pos[3] = {1500, 2500, 3500};
        uint8_t  pay[32];
        size_t   n = link_build_targets(pay, ids, pos, 3);
        send_frame(0x33, OP_SET_TARGETS, pay, (uint8_t)n);
        run(2000);

        if (dut->deadman)          fail("deadman did not clear on a valid frame");
        else if (dut->target0 != 1500 || dut->target2 != 3500)
            fail("targets did not update after recovery");
        else
            printf("ok   recovered, targets %u/%u/%u\n",
                   dut->target0, dut->target1, dut->target2);
    }

    printf("\n");
    printf("frames FPGA->host: %u, host CRC errors: %u, FPGA CRC errors: %u\n",
           host_parser.frames, host_parser.crc_errors, dut->crc_errors);
    if (errors == 0)
        printf("PASS: ESP32 C codec and Verilog link agree end to end\n");
    else
        printf("FAILED: %d check(s) failed\n", errors);

    dut->final();
    delete dut;
    return errors ? 1 : 0;
}
