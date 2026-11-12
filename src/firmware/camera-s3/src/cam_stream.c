#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <errno.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_camera.h"

#include "secrets.h"
#include "cam_wifi.h"
#include "cam_stream.h"

static const char *TAG = "CAM-STREAM";

// Frame: ['C','A','M','0'][uint32 len BE][JPEG]
#define HDR_SIZE        8
#define RECONNECT_MS    2000
#define SEND_TIMEOUT_S  5

static volatile bool s_run       = false;
static volatile int  s_sock      = -1;
static volatile int64_t s_last_hb = 0;

static uint32_t s_frames = 0;
static uint64_t s_bytes  = 0;
static int64_t  s_t0     = 0;

static cam_stream_cmd_cb_t s_cmd_cb = NULL;

void cam_stream_set_cmd_cb(cam_stream_cmd_cb_t cb) { s_cmd_cb = cb; }
static bool send_all(const uint8_t *data, size_t len);

static void poll_rx(void)
{
    static uint8_t rx[64];
    static size_t  rx_len = 0;

    if (rx_len >= sizeof(rx)) rx_len = 0;

    int n = recv(s_sock, rx + rx_len, sizeof(rx) - rx_len, MSG_DONTWAIT);
    if (n <= 0) return;

    s_last_hb = esp_timer_get_time();
    rx_len += n;

    while (rx_len >= 5) {
        if (rx[0] != 'C' || rx[1] != 'M' || rx[2] != 'D') {
            memmove(rx, rx + 1, --rx_len);
            continue;
        }
        uint8_t code = rx[3], len = rx[4];
        if (rx_len < 5u + len) break;
        ESP_LOGI(TAG, "TCP cmd 0x%02X, %u bytes", code, len);
        if (s_cmd_cb) s_cmd_cb(code, rx + 5, len);
        memmove(rx, rx + 5 + len, rx_len - 5 - len);
        rx_len -= 5 + len;
    }
}

// Telemetry: ['C','A','M','T'][uint32 len BE][16 bytes]
void cam_stream_send_telemetry(const void *t, size_t len)
{
    if (s_sock < 0) return;
    uint8_t hdr[HDR_SIZE] = { 'C','A','M','T',
        (uint8_t)(len>>24),(uint8_t)(len>>16),(uint8_t)(len>>8),(uint8_t)len };
    send_all(hdr, HDR_SIZE);
    send_all((const uint8_t *)t, len);
}

static bool stream_connect(void)
{

    struct sockaddr_in dest = {
        .sin_family = AF_INET,
        .sin_port   = htons(CAM_VPS_PORT),
    };
    dest.sin_addr.s_addr = inet_addr(CAM_VPS_HOST);

    int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "socket: errno %d", errno);
        return false;
    }

    int ka = 1, idle = 5, intvl = 3, cnt = 3;
    setsockopt(sock, SOL_SOCKET,  SO_KEEPALIVE, &ka,    sizeof(ka));
    setsockopt(sock, IPPROTO_TCP, TCP_KEEPIDLE, &idle,  sizeof(idle));
    setsockopt(sock, IPPROTO_TCP, TCP_KEEPINTVL,&intvl, sizeof(intvl));
    setsockopt(sock, IPPROTO_TCP, TCP_KEEPCNT,  &cnt,   sizeof(cnt));

    struct timeval tv = { .tv_sec = SEND_TIMEOUT_S, .tv_usec = 0 };
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

    int one = 1;
    setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));

    if (connect(sock, (struct sockaddr *)&dest, sizeof(dest)) != 0) {
        ESP_LOGW(TAG, "connect %s:%d: errno %d",
                 CAM_VPS_HOST, CAM_VPS_PORT, errno);
        close(sock);
        return false;
    }

    // Short recv timeout, used only for heartbeat
    struct timeval rv = { .tv_sec = 0, .tv_usec = 1000 };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &rv, sizeof(rv));

    s_sock    = sock;
    s_last_hb = esp_timer_get_time();
    s_frames  = 0;
    s_bytes   = 0;
    s_t0      = esp_timer_get_time();

    ESP_LOGI(TAG, "Connected to %s:%d", CAM_VPS_HOST, CAM_VPS_PORT);
    return true;
}

static void stream_close(void)
{
    int sock = s_sock;
    s_sock = -1;
    if (sock >= 0) {
        shutdown(sock, SHUT_RDWR);
        close(sock);
        ESP_LOGW(TAG, "Connection closed");
    }
}

static bool send_all(const uint8_t *data, size_t len)
{
    size_t sent = 0;
    while (sent < len) {
        int n = send(s_sock, data + sent, len - sent, 0);
        if (n <= 0) {
            ESP_LOGW(TAG, "send: errno %d", errno);
            return false;
        }
        sent += n;
    }
    return true;
}

static void stream_task(void *arg)
{
    while (1) {
        if (!s_run) {
            if (s_sock >= 0) stream_close();
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        if (!cam_wifi_is_connected()) {
            if (s_sock >= 0) stream_close();
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        if (s_sock < 0) {
            if (!stream_connect()) {
                vTaskDelay(pdMS_TO_TICKS(RECONNECT_MS));
                continue;
            }
        }
        poll_rx();
        camera_fb_t *fb = esp_camera_fb_get();
        if (fb == NULL) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        uint8_t hdr[HDR_SIZE] = {
            'C', 'A', 'M', '0',
            (uint8_t)(fb->len >> 24), (uint8_t)(fb->len >> 16),
            (uint8_t)(fb->len >> 8),  (uint8_t)(fb->len)
        };

        bool ok = send_all(hdr, HDR_SIZE) && send_all(fb->buf, fb->len);

        if (ok) {
            s_frames++;
            s_bytes += fb->len;
        }
        esp_camera_fb_return(fb);

        if (!ok) {
            stream_close();
            vTaskDelay(pdMS_TO_TICKS(RECONNECT_MS));
            continue;
        }
    }
}

esp_err_t cam_stream_init(void)
{
    xTaskCreatePinnedToCore(stream_task, "cam_stream", 5120, NULL, 5, NULL, 0);
    ESP_LOGI(TAG, "Stream task created");
    return ESP_OK;
}

void cam_stream_start(void)
{
    s_last_hb = esp_timer_get_time();
    s_run = true;
    ESP_LOGI(TAG, "Streaming on");
}

void cam_stream_stop(void)
{
    s_run = false;
    cam_stream_log_stats();
    ESP_LOGI(TAG, "Streaming off");
}

bool cam_stream_is_connected(void)
{
    return s_sock >= 0;
}

int64_t cam_stream_since_hb_ms(void)
{
    if (s_last_hb == 0) return -1;
    return (esp_timer_get_time() - s_last_hb) / 1000;
}

void cam_stream_log_stats(void)
{
    if (s_frames == 0 || s_t0 == 0) return;

    int64_t dt_ms = (esp_timer_get_time() - s_t0) / 1000;
    if (dt_ms <= 0) return;

    float fps    = s_frames * 1000.0f / dt_ms;
    float avg_kb = s_bytes / (float)s_frames / 1024.0f;
    float mbit   = s_bytes * 8.0f / dt_ms / 1000.0f;

    ESP_LOGI(TAG, "FPS=%.1f | frame=%.1f KB | %.2f Mbit/s | RSSI=%d | frames=%lu",
             fps, avg_kb, mbit, cam_wifi_rssi(), (unsigned long)s_frames);
}