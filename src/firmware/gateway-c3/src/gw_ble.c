#include <string.h>
#include "esp_log.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"

#include "BLE_protocol.h"
#include "gw_ble.h"

static const char *TAG = "GW-BLE";

static const ble_uuid128_t s_svc_uuid     = CAM_SVC_UUID128;
static const ble_uuid128_t s_chr_cmd_uuid = CAM_CHR_CMD_UUID128;
static const ble_uuid128_t s_chr_evt_uuid = CAM_CHR_EVT_UUID128;

static uint8_t  s_own_addr_type;
static uint16_t s_conn_handle   = BLE_HS_CONN_HANDLE_NONE;
static uint16_t s_cmd_val_handle = 0;
static uint16_t s_evt_val_handle = 0;
static uint16_t s_evt_cccd_handle = 0;
static bool     s_ready = false;

static gw_evt_handler_t  s_evt_cb  = NULL;
static gw_link_handler_t s_link_cb = NULL;

static void gw_start_scan(void);
static int  gw_gap_event(struct ble_gap_event *event, void *arg);

// Discovery step 3: CCCD found -> subscribe
static int on_dsc_disc(uint16_t conn_handle,
                       const struct ble_gatt_error *error,
                       uint16_t chr_val_handle,
                       const struct ble_gatt_dsc *dsc,
                       void *arg)
{
    if (error->status == BLE_HS_EDONE) {
        if (s_evt_cccd_handle == 0) {
            ESP_LOGE(TAG, "CCCD not found");
            return 0;
        }
        // 0x0001 in CCCD enables notify
        uint16_t value = 1;
        int rc = ble_gattc_write_flat(conn_handle, s_evt_cccd_handle,
                                      &value, sizeof(value), NULL, NULL);
        if (rc == 0) {
            s_ready = true;
            ESP_LOGI(TAG, "Subscribed, link ready");

            gw_ble_log_phy();
            gw_ble_log_conn_params();
            gw_ble_request_slow_params();

            if (s_link_cb) s_link_cb(true);
        } else {
            ESP_LOGE(TAG, "Subscribe failed: %d", rc);
        }
        return 0;
    }

    if (error->status != 0) {
        ESP_LOGE(TAG, "Descriptor discovery failed: %d", error->status);
        return 0;
    }

    if (ble_uuid_u16(&dsc->uuid.u) == BLE_GATT_DSC_CLT_CFG_UUID16) {
        s_evt_cccd_handle = dsc->handle;
        ESP_LOGI(TAG, "CCCD found, handle=%d", s_evt_cccd_handle);
    }
    return 0;
}

// Discovery step 2: characteristics
static int on_chr_disc(uint16_t conn_handle,
                       const struct ble_gatt_error *error,
                       const struct ble_gatt_chr *chr,
                       void *arg)
{
    if (error->status == BLE_HS_EDONE) {
        ESP_LOGI(TAG, "Characteristics: cmd=%d, evt=%d",
                 s_cmd_val_handle, s_evt_val_handle);

        if (s_cmd_val_handle == 0 || s_evt_val_handle == 0) {
            ESP_LOGE(TAG, "Missing characteristics");
            ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
            return 0;
        }

        ble_gattc_disc_all_dscs(conn_handle, s_evt_val_handle,
                                0xFFFF, on_dsc_disc, NULL);
        return 0;
    }

    if (error->status != 0) {
        ESP_LOGE(TAG, "Characteristic discovery failed: %d", error->status);
        return 0;
    }

    if (ble_uuid_cmp(&chr->uuid.u, &s_chr_cmd_uuid.u) == 0) {
        s_cmd_val_handle = chr->val_handle;
    } else if (ble_uuid_cmp(&chr->uuid.u, &s_chr_evt_uuid.u) == 0) {
        s_evt_val_handle = chr->val_handle;
    }
    return 0;
}

// Discovery step 1: service
static int on_svc_disc(uint16_t conn_handle,
                       const struct ble_gatt_error *error,
                       const struct ble_gatt_svc *service,
                       void *arg)
{
    if (error->status == BLE_HS_EDONE) {
        return 0;
    }
    if (error->status != 0) {
        ESP_LOGE(TAG, "Service not found: %d", error->status);
        ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        return 0;
    }

    ESP_LOGI(TAG, "Service found: handles %d..%d",
             service->start_handle, service->end_handle);

    ble_gattc_disc_all_chrs(conn_handle,
                            service->start_handle,
                            service->end_handle,
                            on_chr_disc, NULL);
    return 0;
}

// Match by advertised name
static bool gw_is_target(const uint8_t *data, uint8_t len)
{
    struct ble_hs_adv_fields fields;
    if (ble_hs_adv_parse_fields(&fields, data, len) != 0) {
        return false;
    }
    if (fields.name == NULL || fields.name_len == 0) {
        return false;
    }
    return (fields.name_len == strlen(CAM_DEVICE_NAME)) &&
           (memcmp(fields.name, CAM_DEVICE_NAME, fields.name_len) == 0);
}

static void gw_connect(const ble_addr_t *addr)
{
    ble_gap_disc_cancel();

    struct ble_gap_conn_params coded_params = {
        .scan_itvl = 0x0060,
        .scan_window = 0x0060,
        .itvl_min = 24, // 30 ms
        .itvl_max = 40, // 50 ms
        .latency = 0,
        .supervision_timeout = 400, // 4 s
        .min_ce_len = 0,
        .max_ce_len = 0,
    };

    int rc = ble_gap_ext_connect(s_own_addr_type, addr, 10000,
                                 BLE_GAP_LE_PHY_CODED_MASK,
                                 NULL,           // 1M unused
                                 NULL,           // 2M unused
                                 &coded_params,
                                 gw_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ext_connect failed: %d", rc);
        gw_start_scan();
    }
}

static void gw_start_scan(void)
{
    struct ble_gap_ext_disc_params coded_params = {
        .itvl   = 200,     // 125 ms
        .window = 100,     // 62.5 ms
        .passive = 0,
    };

    int rc = ble_gap_ext_disc(s_own_addr_type,
                              0,      // duration: 0 = forever
                              0,      // period
                              1,      // filter_duplicates
                              BLE_HCI_SCAN_FILT_NO_WL,
                              0,      // limited
                              NULL,           // uncoded disabled
                              &coded_params,
                              gw_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ext_disc failed: %d", rc);
    } else {
        ESP_LOGI(TAG, "Scanning on Coded PHY");
    }
}

static int gw_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {

    case BLE_GAP_EVENT_EXT_DISC:
        if (gw_is_target(event->ext_disc.data, event->ext_disc.length_data)) {
            ESP_LOGI(TAG, "Found %s, RSSI=%d",
                     CAM_DEVICE_NAME, event->ext_disc.rssi);
            gw_connect(&event->ext_disc.addr);
        }
        return 0;

    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            s_conn_handle = event->connect.conn_handle;
            ESP_LOGI(TAG, "Connected, handle=%d", s_conn_handle);

            ble_gattc_exchange_mtu(s_conn_handle, NULL, NULL);

            ble_gattc_disc_svc_by_uuid(s_conn_handle, &s_svc_uuid.u,
                                       on_svc_disc, NULL);
        } else {
            ESP_LOGW(TAG, "Connection failed: %d", event->connect.status);
            gw_start_scan();
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGW(TAG, "Disconnected, reason=%d", event->disconnect.reason);
        s_conn_handle     = BLE_HS_CONN_HANDLE_NONE;
        s_cmd_val_handle  = 0;
        s_evt_val_handle  = 0;
        s_evt_cccd_handle = 0;
        s_ready = false;
        if (s_link_cb) s_link_cb(false);
        gw_start_scan();
        return 0;

    case BLE_GAP_EVENT_NOTIFY_RX: {
        uint16_t len = OS_MBUF_PKTLEN(event->notify_rx.om);
        uint8_t buf[CAM_MAX_PACKET_SIZE];
        uint16_t out_len = 0;

        if (len > sizeof(buf)) {
            ESP_LOGW(TAG, "Packet too long: %u", len);
            return 0;
        }
        ble_hs_mbuf_to_flat(event->notify_rx.om, buf, sizeof(buf), &out_len);

        if (out_len >= 1 && s_evt_cb) {
            s_evt_cb(buf[0], buf + 1, out_len - 1);
        }
        return 0;
    }

    case BLE_GAP_EVENT_CONN_UPDATE:
        struct ble_gap_conn_desc desc;
        if (ble_gap_conn_find(event->conn_update.conn_handle, &desc) == 0) {
            ESP_LOGI(TAG, "Conn params updated: itvl=%d (%.1f ms), latency=%d, timeout=%d",
                     desc.conn_itvl, desc.conn_itvl * 1.25,
                     desc.conn_latency, desc.supervision_timeout);
        }
        return 0;

    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "MTU = %d", event->mtu.value);
        return 0;

    case BLE_GAP_EVENT_PHY_UPDATE_COMPLETE:
        ESP_LOGI(TAG, "PHY: tx=%d rx=%d",
                 event->phy_updated.tx_phy, event->phy_updated.rx_phy);
        return 0;

    default:
        return 0;
    }
}

static void gw_on_sync(void)
{
    ble_hs_util_ensure_addr(0);
    ble_hs_id_infer_auto(0, &s_own_addr_type);
    gw_start_scan();
}

static void gw_on_reset(int reason)
{
    ESP_LOGE(TAG, "Stack reset, reason=%d", reason);
    s_ready = false;
}

static void gw_host_task(void *param)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void gw_ble_init(gw_evt_handler_t evt_cb, gw_link_handler_t link_cb)
{
    s_evt_cb  = evt_cb;
    s_link_cb = link_cb;

    ESP_ERROR_CHECK(nimble_port_init());

    ble_hs_cfg.sync_cb  = gw_on_sync;
    ble_hs_cfg.reset_cb = gw_on_reset;

    nimble_port_freertos_init(gw_host_task);
    ESP_LOGI(TAG, "BLE central initialized");
}

bool gw_ble_is_ready(void)
{
    return s_ready;
}

int gw_ble_send_cmd(uint8_t cmd, const void *payload, size_t len)
{
    if (!s_ready) return -1;
    if (len > CAM_MAX_PAYLOAD_SIZE) return -2;

    uint8_t buf[CAM_MAX_PACKET_SIZE];
    buf[0] = cmd;
    if (payload && len) memcpy(buf + 1, payload, len);

    return ble_gattc_write_no_rsp_flat(s_conn_handle, s_cmd_val_handle,
                                       buf, len + 1);
}

int gw_ble_get_rssi(int8_t *out_rssi)
{
    if (s_conn_handle == BLE_HS_CONN_HANDLE_NONE) return -1;
    return ble_gap_conn_rssi(s_conn_handle, out_rssi);
}

void gw_ble_log_phy(void)
{
    if (s_conn_handle == BLE_HS_CONN_HANDLE_NONE) {
        ESP_LOGW(TAG, "PHY: not connected");
        return;
    }

    uint8_t tx_phy = 0, rx_phy = 0;
    int rc = ble_gap_read_le_phy(s_conn_handle, &tx_phy, &rx_phy);
    if (rc != 0) {
        ESP_LOGE(TAG, "read_le_phy failed: %d", rc);
        return;
    }

    const char *names[] = { "?", "1M", "2M", "CODED" };
    ESP_LOGI(TAG, "PHY: tx=%s rx=%s",
             names[tx_phy <= 3 ? tx_phy : 0],
             names[rx_phy <= 3 ? rx_phy : 0]);
}

void gw_ble_log_conn_params(void)
{
    struct ble_gap_conn_desc desc;
    if (s_conn_handle == BLE_HS_CONN_HANDLE_NONE) return;
    if (ble_gap_conn_find(s_conn_handle, &desc) != 0) return;

    ESP_LOGI(TAG, "Conn params: itvl=%d (%d ms), latency=%d, timeout=%d (%d ms)",
             desc.conn_itvl, desc.conn_itvl * 125 / 100,
             desc.conn_latency,
             desc.supervision_timeout, desc.supervision_timeout * 10);
}

int gw_ble_request_slow_params(void)
{
    if (s_conn_handle == BLE_HS_CONN_HANDLE_NONE) return -1;

    struct ble_gap_upd_params slow = {
        .itvl_min            = CAM_CONN_ITVL_MIN,
        .itvl_max            = CAM_CONN_ITVL_MAX,
        .latency             = CAM_CONN_LATENCY,
        .supervision_timeout = CAM_CONN_TIMEOUT,
    };

    int rc = ble_gap_update_params(s_conn_handle, &slow);
    ESP_LOGI(TAG, "Requested low-power params, rc=%d", rc);
    return rc;
}

int gw_ble_set_stream_mode(bool streaming)
{
    if (s_conn_handle == BLE_HS_CONN_HANDLE_NONE) return -1;

    struct ble_gap_upd_params p = {
        .itvl_min = streaming ? CAM_CONN_ITVL_STREAM : CAM_CONN_ITVL_MIN,
        .itvl_max = streaming ? CAM_CONN_ITVL_STREAM : CAM_CONN_ITVL_MAX,
        .latency  = streaming ? 0 : CAM_CONN_LATENCY,
        .supervision_timeout = streaming
                             ? CAM_CONN_TIMEOUT_STREAM : CAM_CONN_TIMEOUT,
    };

    ESP_LOGI(TAG, "Link mode: %s", streaming ? "streaming" : "idle");
    return ble_gap_update_params(s_conn_handle, &p);
}