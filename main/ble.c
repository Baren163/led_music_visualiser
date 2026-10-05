#include <string.h>

#include "esp_log.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include "ble.h"
#include "control.h"


static const char *TAG = "ble";


// ============================================================
// UUIDS
// ============================================================
//
// Base: a3c875XX-8ed3-4bdf-8a39-a01bebede295
// BLE_UUID128_INIT takes the bytes least-significant first.

#define VIS_UUID(x) BLE_UUID128_INIT(                       \
    0x95, 0xe2, 0xed, 0xeb, 0x1b, 0xa0, 0x39, 0x8a,         \
    0xdf, 0x4b, 0xd3, 0x8e, (x), 0x75, 0xc8, 0xa3)

static const ble_uuid128_t s_service_uuid = VIS_UUID(0x00);
static const ble_uuid128_t s_state_uuid   = VIS_UUID(0x01);
static const ble_uuid128_t s_file_uuid    = VIS_UUID(0x02);
static const ble_uuid128_t s_status_uuid  = VIS_UUID(0x03);


typedef enum
{
    CHR_STATE,
    CHR_FILE,
    CHR_STATUS,

} chr_id_t;


static uint16_t s_status_handle;
static uint8_t s_own_addr_type;

static volatile uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static volatile bool s_notify_enabled = false;


static void start_advertising(void);


// ============================================================
// GATT ACCESS
// ============================================================

static int gatt_access(
    uint16_t conn_handle,
    uint16_t attr_handle,
    struct ble_gatt_access_ctxt *ctxt,
    void *arg
)
{
    chr_id_t chr = (chr_id_t)(uintptr_t)arg;

    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR && chr == CHR_STATUS)
    {
        status_msg_t status;
        control_get_status(&status);

        int rc = os_mbuf_append(ctxt->om, &status, sizeof(status));
        return (rc == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }

    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR &&
        (chr == CHR_STATE || chr == CHR_FILE))
    {
        uint16_t len = OS_MBUF_PKTLEN(ctxt->om);

        if (len == 0 || len > PROTO_MAX_WRITE_LEN)
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;

        uint8_t buf[PROTO_MAX_WRITE_LEN];
        uint16_t copied = 0;

        if (ble_hs_mbuf_to_flat(ctxt->om, buf, sizeof(buf), &copied) != 0)
            return BLE_ATT_ERR_UNLIKELY;

        // Only parse and queue here; the control task does the work.
        ctrl_kind_t kind = (chr == CHR_STATE) ? CTRL_SRC_STATE_CHR : CTRL_SRC_FILE_CHR;

        switch (control_post_write(kind, buf, copied))
        {
        case CTRL_POST_OK:      return 0;
        case CTRL_POST_INVALID: return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        case CTRL_POST_BUSY:    return BLE_ATT_ERR_INSUFFICIENT_RES;
        }
    }

    return BLE_ATT_ERR_UNLIKELY;
}


static const struct ble_gatt_svc_def s_gatt_services[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &s_service_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &s_state_uuid.u,
                .access_cb = gatt_access,
                .arg = (void *)CHR_STATE,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            {
                .uuid = &s_file_uuid.u,
                .access_cb = gatt_access,
                .arg = (void *)CHR_FILE,
                .flags = BLE_GATT_CHR_F_WRITE,
            },
            {
                .uuid = &s_status_uuid.u,
                .access_cb = gatt_access,
                .arg = (void *)CHR_STATUS,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &s_status_handle,
            },
            { 0 },
        },
    },
    { 0 },
};


// ============================================================
// GAP
// ============================================================

static void request_fast_connection(uint16_t conn_handle)
{
    // Short connection interval speeds up write-with-response file
    // transfers. Windows may reject it, which is harmless.
    struct ble_gap_upd_params params = {
        .itvl_min = 6,                  // 7.5 ms (units of 1.25 ms)
        .itvl_max = 12,                 // 15 ms
        .latency = 0,
        .supervision_timeout = 400,     // 4 s (units of 10 ms)
        .min_ce_len = 0,
        .max_ce_len = 0,
    };

    int rc = ble_gap_update_params(conn_handle, &params);

    if (rc != 0)
        ESP_LOGW(TAG, "Connection parameter update request failed: %d", rc);
}


static int gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type)
    {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0)
        {
            ESP_LOGI(TAG, "Connected");

            s_conn_handle = event->connect.conn_handle;
            s_notify_enabled = false;

            control_post_event(CTRL_EVT_CONNECTED);
            request_fast_connection(event->connect.conn_handle);
        }
        else
        {
            start_advertising();
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "Disconnected, reason 0x%x", event->disconnect.reason);

        s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        s_notify_enabled = false;

        control_post_event(CTRL_EVT_DISCONNECTED);
        start_advertising();
        return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        start_advertising();
        return 0;

    case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.attr_handle == s_status_handle)
            s_notify_enabled = event->subscribe.cur_notify;
        return 0;

    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "MTU is now %d", event->mtu.value);
        return 0;

    case BLE_GAP_EVENT_CONN_UPDATE:
        {
            struct ble_gap_conn_desc desc;

            if (ble_gap_conn_find(event->conn_update.conn_handle, &desc) == 0)
            {
                ESP_LOGI(
                    TAG,
                    "Connection interval %.2f ms",
                    desc.conn_itvl * 1.25f
                );
            }
        }
        return 0;

    default:
        return 0;
    }
}


static void start_advertising(void)
{
    // Advertising packet: flags + service UUID (so the laptop can
    // filter by service). Scan response: device name.
    struct ble_hs_adv_fields fields = {0};

    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.uuids128 = &s_service_uuid;
    fields.num_uuids128 = 1;
    fields.uuids128_is_complete = 1;

    int rc = ble_gap_adv_set_fields(&fields);

    if (rc != 0)
    {
        ESP_LOGE(TAG, "Setting advertising data failed: %d", rc);
        return;
    }

    struct ble_hs_adv_fields response = {0};

    response.name = (const uint8_t *)BLE_DEVICE_NAME;
    response.name_len = strlen(BLE_DEVICE_NAME);
    response.name_is_complete = 1;

    rc = ble_gap_adv_rsp_set_fields(&response);

    if (rc != 0)
    {
        ESP_LOGE(TAG, "Setting scan response failed: %d", rc);
        return;
    }

    struct ble_gap_adv_params params = {0};

    params.conn_mode = BLE_GAP_CONN_MODE_UND;
    params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(
        s_own_addr_type,
        NULL,
        BLE_HS_FOREVER,
        &params,
        gap_event,
        NULL
    );

    if (rc != 0 && rc != BLE_HS_EALREADY)
        ESP_LOGE(TAG, "Starting advertising failed: %d", rc);
    else
        ESP_LOGI(TAG, "Advertising as \"%s\"", BLE_DEVICE_NAME);
}


// ============================================================
// HOST
// ============================================================

static void on_sync(void)
{
    ESP_ERROR_CHECK(ble_hs_util_ensure_addr(0));
    ESP_ERROR_CHECK(ble_hs_id_infer_auto(0, &s_own_addr_type));

    start_advertising();
}


static void on_reset(int reason)
{
    ESP_LOGW(TAG, "NimBLE host reset, reason %d", reason);
}


static void host_task(void *param)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}


void ble_init(void)
{
    ESP_ERROR_CHECK(nimble_port_init());

    ble_hs_cfg.sync_cb = on_sync;
    ble_hs_cfg.reset_cb = on_reset;

    ble_svc_gap_init();
    ble_svc_gatt_init();

    ESP_ERROR_CHECK(ble_gatts_count_cfg(s_gatt_services));
    ESP_ERROR_CHECK(ble_gatts_add_svcs(s_gatt_services));

    ESP_ERROR_CHECK(ble_svc_gap_device_name_set(BLE_DEVICE_NAME));
    ESP_ERROR_CHECK(ble_att_set_preferred_mtu(BLE_PREFERRED_MTU));

    nimble_port_freertos_init(host_task);
}


void ble_notify_status(const status_msg_t *status)
{
    uint16_t conn_handle = s_conn_handle;

    if (conn_handle == BLE_HS_CONN_HANDLE_NONE || !s_notify_enabled)
        return;

    struct os_mbuf *om = ble_hs_mbuf_from_flat(status, sizeof(*status));

    if (om == NULL)
        return;

    // Consumes `om` whether or not it succeeds.
    int rc = ble_gatts_notify_custom(conn_handle, s_status_handle, om);

    if (rc != 0)
        ESP_LOGW(TAG, "STATUS notify failed: %d", rc);
}
