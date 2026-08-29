/*
 * linkctl -- drive the FPGA link from the Mac, over the tether or the radio.
 *
 *   cc -I../../esp32 -o linkctl linkctl.c ../../esp32/link_proto.c
 *
 *   ./linkctl -s /dev/cu.usbserial-XXXX targets 1000 2000 3000
 *   ./linkctl -s /dev/cu.usbserial-XXXX monitor
 *   ./linkctl -u 192.168.4.1:9000 targets 1000 2000 3000
 *   ./linkctl -u 192.168.4.1:9000 stream 50
 *
 * Same link_proto.c as the ESP32 firmware and the Verilator harness, so
 * there is one C implementation of the wire format and three users of it.
 * The Verilog is the independent second implementation, which is the whole
 * point -- they are checked against each other in servo/sim.
 *
 * Serial and UDP are interchangeable by design: develop over the tether,
 * deploy over the radio, without the protocol changing underneath you.
 */

#define _DARWIN_C_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/time.h>
#include <math.h>
#ifdef __APPLE__
#include <IOKit/serial/ioss.h>
#include <sys/ioctl.h>
#endif

#include "link_proto.h"

#define LINK_BAUD 2000000

static int fd = -1;              /* serial fd or UDP socket */
static int is_udp = 0;
static struct sockaddr_in peer;

/* ---- transports --------------------------------------------------------- */
static int open_serial(const char *dev)
{
    int f = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (f < 0) { perror("open"); return -1; }

    struct termios t;
    if (tcgetattr(f, &t)) { perror("tcgetattr"); return -1; }
    cfmakeraw(&t);
    t.c_cflag |= (CLOCAL | CREAD);
    t.c_cflag &= ~CRTSCTS;
    t.c_cc[VMIN]  = 0;
    t.c_cc[VTIME] = 0;
    if (tcsetattr(f, TCSANOW, &t)) { perror("tcsetattr"); return -1; }

#ifdef __APPLE__
    /* 2 Mbaud is beyond the termios speed constants; macOS needs this ioctl
     * and it must come AFTER tcsetattr or it gets clobbered. */
    speed_t sp = LINK_BAUD;
    if (ioctl(f, IOSSIOSPEED, &sp) == -1) {
        fprintf(stderr, "warning: could not set %d baud (%s)\n",
                LINK_BAUD, strerror(errno));
    }
#endif
    return f;
}

static int open_udp(const char *hostport)
{
    char host[64]; int port = 9000;
    const char *c = strchr(hostport, ':');
    if (c) { size_t n = (size_t)(c - hostport);
             if (n >= sizeof(host)) n = sizeof(host) - 1;
             memcpy(host, hostport, n); host[n] = 0; port = atoi(c + 1); }
    else   { snprintf(host, sizeof(host), "%s", hostport); }

    int s = socket(AF_INET, SOCK_DGRAM, 0);
    if (s < 0) { perror("socket"); return -1; }

    memset(&peer, 0, sizeof(peer));
    peer.sin_family = AF_INET;
    peer.sin_port   = htons((uint16_t)port);
    if (inet_pton(AF_INET, host, &peer.sin_addr) != 1) {
        fprintf(stderr, "bad address: %s\n", host); return -1;
    }
    struct timeval tv = { .tv_sec = 0, .tv_usec = 200000 };
    setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    is_udp = 1;
    return s;
}

static void tx(const uint8_t *buf, size_t n)
{
    if (is_udp) sendto(fd, buf, n, 0, (struct sockaddr *)&peer, sizeof(peer));
    else        write(fd, buf, n);
}

static int rx(uint8_t *buf, size_t n)
{
    if (is_udp) return (int)recvfrom(fd, buf, n, 0, NULL, NULL);
    return (int)read(fd, buf, n);
}

/* ---- commands ----------------------------------------------------------- */
static uint8_t seq = 0;

static void send_targets(const uint16_t *pos, uint8_t n)
{
    uint8_t ids[8], pay[64], frame[LINK_MAX_FRAME];
    for (uint8_t i = 0; i < n; i++) ids[i] = (uint8_t)(i + 1);
    size_t pl = link_build_targets(pay, ids, pos, n);
    size_t fl = link_encode(frame, seq++, OP_SET_TARGETS, pay, (uint8_t)pl);
    tx(frame, fl);
}

static void send_keepalive(void)
{
    uint8_t frame[LINK_MAX_FRAME];
    size_t fl = link_encode(frame, seq++, OP_KEEPALIVE, NULL, 0);
    tx(frame, fl);
}

static double now_ms(void)
{
    struct timeval tv; gettimeofday(&tv, NULL);
    return tv.tv_sec * 1000.0 + tv.tv_usec / 1000.0;
}

static void monitor(int seconds)
{
    link_parser_t p; link_parser_init(&p);
    uint8_t buf[512];
    double  t0 = now_ms(), last = 0;
    int     frames = 0;

    printf("listening for OBS frames (%ds)...\n", seconds);
    while (now_ms() - t0 < seconds * 1000.0) {
        int n = rx(buf, sizeof(buf));
        if (n <= 0) { usleep(1000); continue; }
        for (int i = 0; i < n; i++) {
            if (!link_parse_byte(&p, buf[i])) continue;
            if (p.op != OP_OBS) continue;
            double t  = now_ms();
            uint32_t tick = link_rd32(&p.payload[0]);
            printf("tick=%-8u pos=%5u %5u %5u   dt=%6.2f ms\n",
                   tick, link_rd16(&p.payload[4]),
                   link_rd16(&p.payload[6]), link_rd16(&p.payload[8]),
                   last ? t - last : 0.0);
            last = t; frames++;
        }
    }
    printf("%d OBS frames, %u CRC errors\n", frames, p.crc_errors);
}

/* Continuous target stream at `hz`, which is what a policy loop looks like.
 * Also keeps the deadman fed simply by talking. */
static void stream(int hz, int seconds)
{
    double period = 1000.0 / hz, t0 = now_ms(), next = t0;
    int    sent = 0;
    printf("streaming targets at %d Hz for %ds\n", hz, seconds);
    while (now_ms() - t0 < seconds * 1000.0) {
        double t = now_ms();
        if (t >= next) {
            double  ph  = (t - t0) / 1000.0;
            uint16_t a  = (uint16_t)(2048 + 800 * sin(ph * 2.0));
            uint16_t b  = (uint16_t)(2048 + 800 * sin(ph * 2.0 + 2.09));
            uint16_t cc = (uint16_t)(2048 + 800 * sin(ph * 2.0 + 4.19));
            uint16_t pos[3] = { a, b, cc };
            send_targets(pos, 3);
            sent++; next += period;
        }
        usleep(200);
    }
    printf("sent %d target frames\n", sent);
}

int main(int argc, char **argv)
{
    if (argc < 4) {
        fprintf(stderr,
            "usage: linkctl -s <serial>|-u <host:port> <command>\n"
            "  targets A B C     set three positions\n"
            "  keepalive         feed the deadman once\n"
            "  monitor [secs]    print incoming OBS frames\n"
            "  stream [hz]       continuous target stream (policy-shaped)\n");
        return 2;
    }

    if      (!strcmp(argv[1], "-s")) fd = open_serial(argv[2]);
    else if (!strcmp(argv[1], "-u")) fd = open_udp(argv[2]);
    else { fprintf(stderr, "first arg must be -s or -u\n"); return 2; }
    if (fd < 0) return 1;

    const char *cmd = argv[3];
    if (!strcmp(cmd, "targets") && argc >= 7) {
        uint16_t pos[3] = { (uint16_t)atoi(argv[4]),
                            (uint16_t)atoi(argv[5]),
                            (uint16_t)atoi(argv[6]) };
        send_targets(pos, 3);
        printf("sent targets %u %u %u\n", pos[0], pos[1], pos[2]);
        monitor(1);
    } else if (!strcmp(cmd, "keepalive")) {
        send_keepalive(); printf("keepalive sent\n");
    } else if (!strcmp(cmd, "monitor")) {
        monitor(argc > 4 ? atoi(argv[4]) : 5);
    } else if (!strcmp(cmd, "stream")) {
        stream(argc > 4 ? atoi(argv[4]) : 50, argc > 5 ? atoi(argv[5]) : 10);
    } else {
        fprintf(stderr, "unknown command: %s\n", cmd);
        return 2;
    }
    return 0;
}
