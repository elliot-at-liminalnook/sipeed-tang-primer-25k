/*
 * servo-link -- ESP32-S3 firmware. A radio, and deliberately nothing more.
 *
 *     host (UDP) <--WiFi--> ESP32 <--UART 2 Mbaud--> FPGA <--> servos
 *
 * Every promise the system makes -- the control tick, the deadman, the
 * safety clamps -- lives in the FPGA, because the FPGA is the only layer
 * that cannot be taken out by a scheduler, a WiFi dropout or a reboot. This
 * firmware moves bytes and does not interpret them beyond framing.
 *
 * Two rules make that work:
 *
 *   1. One UDP datagram carries exactly one link frame. UART is a byte
 *      stream with no boundaries, so the uplink parses frames out of it
 *      before packetising; the downlink verifies a frame is well formed
 *      before letting it reach the FPGA.
 *
 *   2. WiFi power-save is OFF. Modem sleep alone turns a 2 ms hop into
 *      100 ms spikes, and jitter is what makes a policy behave badly --
 *      far more than mean latency does.
 *
 * SoftAP by default: the host associates directly, so there is no router in
 * the path adding hops and contention. Set USE_STA to join an existing
 * network instead.
 */

#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "driver/uart.h"

#include "link_proto.h"

static const char *TAG = "servo-link";

/* ---- configuration ------------------------------------------------------ */
#define USE_STA        0                 /* 0 = SoftAP (lowest jitter)      */
#define WIFI_SSID      "servo-link"
#define WIFI_PASS      "robotrobot"      /* >= 8 chars, or the AP is open   */
#define UDP_PORT       9000

/*
 * UART to the FPGA. Any GPIO works on the S3 thanks to the GPIO matrix, but
 * avoid 0/45/46 (strapping), 19/20 (native USB), 26-32 (flash) and 33-37
 * (octal PSRAM -- this module has 8 MB, so treat them as taken).
 */
#define LINK_UART      UART_NUM_1
#define PIN_TX         17               /* -> FPGA host_rx */
#define PIN_RX         18               /* <- FPGA host_tx */
#define LINK_BAUD      2000000          /* 50 MHz / 25, exact on the FPGA   */
#define UART_BUF       2048

/* Learned from the first datagram, so the host needs no fixed address here. */
static struct sockaddr_in peer;
static volatile int       peer_known = 0;
static int                sock = -1;

/* ---- WiFi --------------------------------------------------------------- */
static void wifi_start(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

#if USE_STA
    esp_netif_create_default_wifi_sta();
#else
    esp_netif_create_default_wifi_ap();
#endif

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

#if USE_STA
    wifi_config_t wc = { 0 };
    strncpy((char *)wc.sta.ssid, WIFI_SSID, sizeof(wc.sta.ssid) - 1);
    strncpy((char *)wc.sta.password, WIFI_PASS, sizeof(wc.sta.password) - 1);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
#else
    wifi_config_t wc = { 0 };
    strncpy((char *)wc.ap.ssid, WIFI_SSID, sizeof(wc.ap.ssid) - 1);
    wc.ap.ssid_len       = strlen(WIFI_SSID);
    strncpy((char *)wc.ap.password, WIFI_PASS, sizeof(wc.ap.password) - 1);
    wc.ap.max_connection = 2;
    wc.ap.authmode       = strlen(WIFI_PASS) >= 8 ? WIFI_AUTH_WPA2_PSK
                                                  : WIFI_AUTH_OPEN;
    wc.ap.channel        = 6;
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wc));
#endif

    ESP_ERROR_CHECK(esp_wifi_start());

    /* The single most valuable line in this file for motion quality. */
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

#if USE_STA
    ESP_ERROR_CHECK(esp_wifi_connect());
    ESP_LOGI(TAG, "joining \"%s\"", WIFI_SSID);
#else
    ESP_LOGI(TAG, "AP \"%s\" up, this device is 192.168.4.1:%d",
             WIFI_SSID, UDP_PORT);
#endif
}

/* ---- UART --------------------------------------------------------------- */
static void uart_start(void)
{
    uart_config_t uc = {
        .baud_rate  = LINK_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(LINK_UART, UART_BUF, UART_BUF, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(LINK_UART, &uc));
    ESP_ERROR_CHECK(uart_set_pin(LINK_UART, PIN_TX, PIN_RX,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_LOGI(TAG, "UART%d up at %d baud (tx=%d rx=%d)",
             LINK_UART, LINK_BAUD, PIN_TX, PIN_RX);
}

/* ---- uplink: FPGA -> host ----------------------------------------------- */
/*
 * The FPGA emits telemetry on a fixed tick. UART carries no frame
 * boundaries, so parse them out here and send one datagram per frame --
 * a policy wants whole observations, not a byte stream it has to reassemble.
 */
static void uart_to_udp_task(void *arg)
{
    link_parser_t parser;
    link_parser_init(&parser);

    uint8_t chunk[256];
    uint8_t frame[LINK_MAX_FRAME];

    for (;;) {
        int n = uart_read_bytes(LINK_UART, chunk, sizeof(chunk), pdMS_TO_TICKS(20));
        for (int i = 0; i < n; i++) {
            if (!link_parse_byte(&parser, chunk[i]))
                continue;

            /* Re-encode rather than forwarding raw bytes: the parser already
             * holds the verified fields, and this guarantees the host sees
             * exactly one well-formed frame per datagram. */
            size_t len = link_encode(frame, parser.seq, parser.op,
                                     parser.payload, parser.len);
            if (len && peer_known)
                sendto(sock, frame, len, 0,
                       (struct sockaddr *)&peer, sizeof(peer));
        }
    }
}

/* ---- downlink: host -> FPGA --------------------------------------------- */
static void udp_to_uart_task(void *arg)
{
    uint8_t buf[LINK_MAX_FRAME * 2];
    struct sockaddr_in from;
    socklen_t          fromlen = sizeof(from);

    for (;;) {
        int n = recvfrom(sock, buf, sizeof(buf), 0,
                         (struct sockaddr *)&from, &fromlen);
        if (n <= 0)
            continue;

        if (!peer_known || peer.sin_addr.s_addr != from.sin_addr.s_addr ||
            peer.sin_port != from.sin_port) {
            peer = from;
            peer_known = 1;
            ESP_LOGI(TAG, "peer %s:%d",
                     inet_ntoa(from.sin_addr), ntohs(from.sin_port));
        }

        /* Validate before forwarding. UDP has its own checksum, so this is
         * cheap insurance rather than a necessity -- but it means malformed
         * bytes never reach the FPGA's parser at all. */
        link_parser_t p;
        link_parser_init(&p);
        int complete = 0;
        for (int i = 0; i < n; i++)
            if (link_parse_byte(&p, buf[i])) { complete = 1; break; }

        if (complete)
            uart_write_bytes(LINK_UART, (const char *)buf, n);
    }
}

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    wifi_start();
    uart_start();

    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    struct sockaddr_in addr = {
        .sin_family      = AF_INET,
        .sin_port        = htons(UDP_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "bind failed");
        return;
    }
    ESP_LOGI(TAG, "listening on UDP %d", UDP_PORT);

    /* Both directions pinned to core 1, away from the WiFi stack on core 0,
     * so radio work cannot delay the UART hand-off. */
    xTaskCreatePinnedToCore(uart_to_udp_task, "uart2udp", 4096, NULL, 10, NULL, 1);
    xTaskCreatePinnedToCore(udp_to_uart_task, "udp2uart", 4096, NULL, 10, NULL, 1);
}
