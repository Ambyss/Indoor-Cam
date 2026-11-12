#include <string.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include "BLE_protocol.h"
#include "cam_ble.h"

static const char *TAG = "CAM-BLE";

static uint8_t  s_own_addr_type;
static uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t s_evt_val_handle;
static cam_cmd_handler_t s_cmd_handler = NULL;

static void cam_ble_start_advertising(void);

static int cam_gatt_cmd_access(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) {
        return BLE_ATT_ERR_UNLIKELY;
    }

    uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
    if (len < CAM_CMD_HEADER_SIZE || len > CAM_MAX_PACKET_SIZE) {
        ESP_LOGW(TAG, "Invalid packet length: %u", len);
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    uint8_t buf[CAM_MAX_PACKET_SIZE];
    uint16_t out_len = 0;
    int rc = ble_hs_mbuf_to_flat(ctxt->om, buf, sizeof(buf), &out_len);
    if (rc != 0) {
        return BLE_ATT_ERR_UNLIKELY;
    }

    uint8_t cmd = buf[0];
    ESP_LOGI(TAG, "RX cmd 0x%02X, payload %u bytes", cmd, out_len - 1);

    if (s_cmd_handler) {
        s_cmd_handler(cmd, buf + 1, out_len - 1);
    }
    return 0;
}

static const struct ble_gatt_svc_def s_gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &(ble_uuid128_t)CAM_SVC_UUID128.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid       = &(ble_uuid128_t)CAM_CHR_CMD_UUID128.u,
                .access_cb  = cam_gatt_cmd_access,
                .flags      = BLE_GATT_CHR_F_WRITE,
            },
            {
                .uuid       = &(ble_uuid128_t)CAM_CHR_EVT_UUID128.u,
                .access_cb  = cam_gatt_cmd_access,
                .val_handle = &s_evt_val_handle,
                .flags      = BLE_GATT_CHR_F_NOTIFY,
            },
            { 0 }
        },
    },
    { 0 }
};

static int cam_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {

    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            s_conn_handle = event->connect.conn_handle;
            ESP_LOGI(TAG, "Connected, handle=%d", s_conn_handle);

            struct ble_gap_upd_params params = {
                .itvl_min            = CAM_CONN_ITVL_MIN,
                .itvl_max            = CAM_CONN_ITVL_MAX,
                .latency             = CAM_CONN_LATENCY,
                .supervision_timeout = CAM_CONN_TIMEOUT,
            };
            ble_gap_update_params(s_conn_handle, &params);

            // Force Coded PHY
            ble_gap_set_prefered_le_phy(s_conn_handle,
                                        BLE_GAP_LE_PHY_CODED_MASK,
                                        BLE_GAP_LE_PHY_CODED_MASK,
                                        BLE_GAP_LE_PHY_CODED_S8);
        } else {
            ESP_LOGW(TAG, "Connection failed: %d", event->connect.status);
            cam_ble_start_advertising();
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGW(TAG, "Disconnected, reason=%d", event->disconnect.reason);
        s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        cam_ble_start_advertising();
        return 0;

    case BLE_GAP_EVENT_CONN_UPDATE:
        ESP_LOGI(TAG, "Conn params updated, status=%d",
                 event->conn_update.status);
        return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        cam_ble_start_advertising();
        return 0;

    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "MTU = %d", event->mtu.value);
        return 0;

    default:
        return 0;
    }
}

// Extended advertising on Coded PHY
static void cam_ble_start_advertising(void)
{
    struct ble_gap_ext_adv_params params = { 0 };
    params.connectable   = 1;
    params.scannable     = 0;
    params.legacy_pdu    = 0;               // required for Coded PHY
    params.own_addr_type = s_own_addr_type;
    params.primary_phy   = BLE_HCI_LE_PHY_CODED;
    params.secondary_phy = BLE_HCI_LE_PHY_CODED;
    params.tx_power      = 9;               // +9 dBm
    params.sid           = 0;
    params.itvl_min      = 300;             // 0.625 ms units, 187.5 ms
    params.itvl_max      = 400;

    int rc = ble_gap_ext_adv_configure(0, &params, NULL, cam_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ext_adv_configure failed: %d", rc);
        return;
    }

    const char *name = CAM_DEVICE_NAME;
    uint8_t adv_data[32];
    uint8_t len = 0;
    adv_data[len++] = 1 + strlen(name);
    adv_data[len++] = BLE_HS_ADV_TYPE_COMP_NAME;
    memcpy(&adv_data[len], name, strlen(name));
    len += strlen(name);

    struct os_mbuf *data = os_msys_get_pkthdr(len, 0);
    os_mbuf_append(data, adv_data, len);
    ble_gap_ext_adv_set_data(0, data);

    rc = ble_gap_ext_adv_start(0, 0, 0);
    if (rc != 0) {
        ESP_LOGE(TAG, "ext_adv_start failed: %d", rc);
    } else {
        ESP_LOGI(TAG, "Advertising on Coded PHY started");
    }
}

static void cam_ble_on_sync(void)
{
    ble_hs_util_ensure_addr(0);
    ble_hs_id_infer_auto(0, &s_own_addr_type);
    cam_ble_start_advertising();
}

static void cam_ble_on_reset(int reason)
{
    ESP_LOGE(TAG, "Stack reset, reason=%d", reason);
}

static void cam_ble_host_task(void *param)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void cam_ble_init(cam_cmd_handler_t handler)
{
    s_cmd_handler = handler;

    ESP_ERROR_CHECK(nimble_port_init());

    ble_hs_cfg.sync_cb  = cam_ble_on_sync;
    ble_hs_cfg.reset_cb = cam_ble_on_reset;

    ble_svc_gap_init();
    ble_svc_gatt_init();
    ESP_ERROR_CHECK(ble_gatts_count_cfg(s_gatt_svcs));
    ESP_ERROR_CHECK(ble_gatts_add_svcs(s_gatt_svcs));
    ESP_ERROR_CHECK(ble_svc_gap_device_name_set(CAM_DEVICE_NAME));

    nimble_port_freertos_init(cam_ble_host_task);
    ESP_LOGI(TAG, "BLE initialized");
}

bool cam_ble_is_connected(void)
{
    return s_conn_handle != BLE_HS_CONN_HANDLE_NONE;
}

int cam_ble_send_event(uint8_t evt_code, const void *payload, size_t len)
{
    if (!cam_ble_is_connected()) return -1;
    if (len > CAM_MAX_PAYLOAD_SIZE) return -2;

    uint8_t buf[CAM_MAX_PACKET_SIZE];
    buf[0] = evt_code;
    if (payload && len) memcpy(buf + 1, payload, len);

    struct os_mbuf *om = ble_hs_mbuf_from_flat(buf, len + 1);
    if (!om) return -3;

    return ble_gatts_notify_custom(s_conn_handle, s_evt_val_handle, om);
}