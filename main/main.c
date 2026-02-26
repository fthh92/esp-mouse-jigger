#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_random.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/ble_uuid.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "host/util/util.h"

/* NimBLE bond storage (not in public header, declare manually as ESP-IDF examples do) */
void ble_store_config_init(void);

#define TAG "MOUSE"

static bool connected = false;
static bool encrypted = false;
static uint16_t conn_hdl = 0;
static uint16_t input_handle = 0;

/* ---- HID Report Descriptor ---- */

static const uint8_t hid_report_map[] = {
    0x05, 0x01,  // Usage Page (Generic Desktop)
    0x09, 0x02,  // Usage (Mouse)
    0xA1, 0x01,  // Collection (Application)
    0x09, 0x01,  //   Usage (Pointer)
    0xA1, 0x00,  //   Collection (Physical)
    0x05, 0x09,  //     Usage Page (Button)
    0x19, 0x01,  //     Usage Minimum (1)
    0x29, 0x03,  //     Usage Maximum (3)
    0x15, 0x00,  //     Logical Minimum (0)
    0x25, 0x01,  //     Logical Maximum (1)
    0x95, 0x03,  //     Report Count (3)
    0x75, 0x01,  //     Report Size (1)
    0x81, 0x02,  //     Input (Data, Var, Abs)
    0x95, 0x01,  //     Report Count (1)
    0x75, 0x05,  //     Report Size (5)
    0x81, 0x03,  //     Input (Const) - padding
    0x05, 0x01,  //     Usage Page (Generic Desktop)
    0x09, 0x30,  //     Usage (X)
    0x09, 0x31,  //     Usage (Y)
    0x09, 0x38,  //     Usage (Wheel)
    0x15, 0x81,  //     Logical Minimum (-127)
    0x25, 0x7F,  //     Logical Maximum (127)
    0x75, 0x08,  //     Report Size (8)
    0x95, 0x03,  //     Report Count (3)
    0x81, 0x06,  //     Input (Data, Var, Rel)
    0xC0,        //   End Collection
    0xC0         // End Collection
};

/* HID Information: version 1.11, country 0, flags=0x02 (normally connectable) */
static const uint8_t hid_info[] = { 0x11, 0x01, 0x00, 0x02 };

/* Report Reference descriptor: report_id=0, type=input(1) */
static const uint8_t report_ref_input[] = { 0x00, 0x01 };

static uint8_t protocol_mode = 0x01; /* Report Protocol Mode */
static uint8_t hid_ctrl_point = 0;

/* ---- GATT Access Callbacks ---- */

static int hid_svc_access(uint16_t conn_handle, uint16_t attr_handle,
                           struct ble_gatt_access_ctxt *ctxt, void *arg);

static int bat_svc_access(uint16_t conn_handle, uint16_t attr_handle,
                           struct ble_gatt_access_ctxt *ctxt, void *arg);

static int devinfo_svc_access(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg);

/* ---- GATT Service Definitions ---- */

static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    /* HID Service (0x1812) */
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(0x1812),
        .characteristics = (struct ble_gatt_chr_def[]) {
            /* Protocol Mode */
            {
                .uuid = BLE_UUID16_DECLARE(0x2A4E),
                .access_cb = hid_svc_access,
                .flags = BLE_GATT_CHR_F_READ |
                         BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            /* Report Map */
            {
                .uuid = BLE_UUID16_DECLARE(0x2A4B),
                .access_cb = hid_svc_access,
                .flags = BLE_GATT_CHR_F_READ,
            },
            /* HID Information */
            {
                .uuid = BLE_UUID16_DECLARE(0x2A4A),
                .access_cb = hid_svc_access,
                .flags = BLE_GATT_CHR_F_READ,
            },
            /* HID Control Point */
            {
                .uuid = BLE_UUID16_DECLARE(0x2A4C),
                .access_cb = hid_svc_access,
                .flags = BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            /* Input Report (Mouse) */
            {
                .uuid = BLE_UUID16_DECLARE(0x2A4D),
                .access_cb = hid_svc_access,
                .val_handle = &input_handle,
                .flags = BLE_GATT_CHR_F_READ |
                         BLE_GATT_CHR_F_NOTIFY,
                .descriptors = (struct ble_gatt_dsc_def[]) {
                    /* Report Reference Descriptor */
                    {
                        .uuid = BLE_UUID16_DECLARE(0x2908),
                        .access_cb = hid_svc_access,
                        .att_flags = BLE_ATT_F_READ,
                    },
                    { 0 }
                },
            },
            { 0 } /* End of characteristics */
        },
    },
    /* Battery Service (0x180F) */
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(0x180F),
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = BLE_UUID16_DECLARE(0x2A19), /* Battery Level */
                .access_cb = bat_svc_access,
                .flags = BLE_GATT_CHR_F_READ,
            },
            { 0 }
        },
    },
    /* Device Information Service (0x180A) */
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(0x180A),
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = BLE_UUID16_DECLARE(0x2A29), /* Manufacturer Name */
                .access_cb = devinfo_svc_access,
                .flags = BLE_GATT_CHR_F_READ,
            },
            {
                .uuid = BLE_UUID16_DECLARE(0x2A50), /* PnP ID */
                .access_cb = devinfo_svc_access,
                .flags = BLE_GATT_CHR_F_READ,
            },
            { 0 }
        },
    },
    { 0 } /* End of services */
};

/* ---- Access Callback Implementations ---- */

static int hid_svc_access(uint16_t conn_handle, uint16_t attr_handle,
                           struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    uint16_t uuid16;

    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_DSC) {
        uuid16 = ble_uuid_u16(ctxt->dsc->uuid);
        if (uuid16 == 0x2908) { /* Report Reference */
            os_mbuf_append(ctxt->om, report_ref_input, sizeof(report_ref_input));
            return 0;
        }
        return BLE_ATT_ERR_UNLIKELY;
    }

    uuid16 = ble_uuid_u16(ctxt->chr->uuid);

    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        switch (uuid16) {
        case 0x2A4E: /* Protocol Mode */
            os_mbuf_append(ctxt->om, &protocol_mode, 1);
            return 0;
        case 0x2A4B: /* Report Map */
            os_mbuf_append(ctxt->om, hid_report_map, sizeof(hid_report_map));
            return 0;
        case 0x2A4A: /* HID Information */
            os_mbuf_append(ctxt->om, hid_info, sizeof(hid_info));
            return 0;
        case 0x2A4D: /* Input Report */
            return 0; /* empty report on read */
        }
    } else if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        switch (uuid16) {
        case 0x2A4E: /* Protocol Mode */
            ble_hs_mbuf_to_flat(ctxt->om, &protocol_mode, 1, NULL);
            return 0;
        case 0x2A4C: /* HID Control Point */
            ble_hs_mbuf_to_flat(ctxt->om, &hid_ctrl_point, 1, NULL);
            return 0;
        }
    }
    return BLE_ATT_ERR_UNLIKELY;
}

static int bat_svc_access(uint16_t conn_handle, uint16_t attr_handle,
                           struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    uint8_t battery = 100;
    os_mbuf_append(ctxt->om, &battery, 1);
    return 0;
}

static int devinfo_svc_access(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    uint16_t uuid16 = ble_uuid_u16(ctxt->chr->uuid);
    if (uuid16 == 0x2A29) { /* Manufacturer Name */
        os_mbuf_append(ctxt->om, "Espressif", 9);
    } else if (uuid16 == 0x2A50) { /* PnP ID */
        uint8_t pnp[] = { 0x02, 0xE5, 0x02, 0xDF, 0x05, 0x00, 0x01 };
        os_mbuf_append(ctxt->om, pnp, sizeof(pnp));
    }
    return 0;
}

/* ---- BLE Advertising ---- */

static int ble_gap_event_cb(struct ble_gap_event *event, void *arg);

static void start_advertising(void)
{
    ble_uuid16_t hid_uuid = BLE_UUID16_INIT(0x1812);

    struct ble_hs_adv_fields fields = {0};
    fields.flags                 = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.uuids16               = &hid_uuid;
    fields.num_uuids16           = 1;
    fields.uuids16_is_complete   = 1;
    fields.appearance            = 0x03C2;
    fields.appearance_is_present = 1;

    ble_gap_adv_set_fields(&fields);

    struct ble_hs_adv_fields rsp = {0};
    const char *name     = ble_svc_gap_device_name();
    rsp.name             = (uint8_t *)name;
    rsp.name_len         = strlen(name);
    rsp.name_is_complete = 1;

    ble_gap_adv_rsp_set_fields(&rsp);

    struct ble_gap_adv_params adv_params = {0};
    adv_params.conn_mode   = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode   = BLE_GAP_DISC_MODE_GEN;
    adv_params.itvl_min    = BLE_GAP_ADV_ITVL_MS(20);
    adv_params.itvl_max    = BLE_GAP_ADV_ITVL_MS(40);
    adv_params.channel_map = BLE_GAP_ADV_DFLT_CHANNEL_MAP;

    /* If we have bonded peers, add them to whitelist and use filter policy */
    ble_addr_t peer_addr;
    int num_peers = 1;
    int rc = ble_store_util_bonded_peers(&peer_addr, &num_peers, 1);
    if (rc == 0 && num_peers > 0) {
        /* Clear and add bonded peer to whitelist */
        ble_gap_wl_set(&peer_addr, 1);
        /* Filter: allow scan from any, connect only from whitelist */
        adv_params.filter_policy = BLE_HCI_ADV_FILT_CONN;
        ESP_LOGI(TAG, "Advertising with whitelist (bonded peer)");
    }

    rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                           &adv_params, ble_gap_event_cb, NULL);
    if (rc == BLE_HS_EINVAL && num_peers > 0) {
        /* Whitelist filter failed, try without filter */
        adv_params.filter_policy = 0;
        rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                               &adv_params, ble_gap_event_cb, NULL);
    }
    ESP_LOGI(TAG, "Advertising started (rc=%d)", rc);
}

/* ---- GAP Event Handler ---- */

static int ble_gap_event_cb(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            conn_hdl = event->connect.conn_handle;
            connected = true;
            ESP_LOGI(TAG, "Connected (handle=%d)", conn_hdl);

            /* Initiate security - macOS requires encryption for HID */
            int rc = ble_gap_security_initiate(conn_hdl);
            ESP_LOGI(TAG, "Security initiate: rc=%d", rc);

            /* Check if already encrypted (bonded reconnection) */
            struct ble_gap_conn_desc desc;
            if (ble_gap_conn_find(conn_hdl, &desc) == 0 && desc.sec_state.encrypted) {
                encrypted = true;
                ESP_LOGI(TAG, "Already encrypted (bonded)");
            }
        } else {
            ESP_LOGE(TAG, "Connection failed: %d", event->connect.status);
            start_advertising();
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "Disconnected: reason=%d", event->disconnect.reason);
        connected = false;
        encrypted = false;
        start_advertising();
        break;

    case BLE_GAP_EVENT_ENC_CHANGE:
        ESP_LOGI(TAG, "Encryption change: status=%d", event->enc_change.status);
        if (event->enc_change.status == 0) {
            ESP_LOGI(TAG, "Link encrypted - HID active");
            encrypted = true;
        }
        break;

    case BLE_GAP_EVENT_SUBSCRIBE:
        ESP_LOGI(TAG, "Subscribe: handle=%d, cur_notify=%d",
                 event->subscribe.attr_handle, event->subscribe.cur_notify);
        break;

    case BLE_GAP_EVENT_REPEAT_PAIRING: {
        struct ble_gap_conn_desc desc;
        ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc);
        ble_store_util_delete_peer(&desc.peer_id_addr);
        return BLE_GAP_REPEAT_PAIRING_RETRY;
    }

    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "MTU update: conn=%d, mtu=%d",
                 event->mtu.conn_handle, event->mtu.value);
        break;

    case BLE_GAP_EVENT_NOTIFY_TX:
        if (event->notify_tx.status != 0) {
            ESP_LOGW(TAG, "Notify TX failed: status=%d", event->notify_tx.status);
        }
        break;

    default:
        ESP_LOGI(TAG, "GAP event: %d", event->type);
        break;
    }
    return 0;
}

/* ---- Human-like Mouse Movement ---- */

static void send_report(uint8_t buttons, int8_t dx, int8_t dy, int8_t wheel)
{
    uint8_t report[4] = { buttons, (uint8_t)dx, (uint8_t)dy, (uint8_t)wheel };
    struct os_mbuf *om = ble_hs_mbuf_from_flat(report, sizeof(report));
    if (om) {
        ble_gatts_notify_custom(conn_hdl, input_handle, om);
    }
}

static int8_t clamp8(int v)
{
    if (v > 127) return 127;
    if (v < -127) return -127;
    return (int8_t)v;
}

static void do_smooth_move(float target_x, float target_y, int steps)
{
    /* Simulate moving toward a target with acceleration/deceleration (ease-in-out) */
    float cx = 0, cy = 0;
    for (int i = 0; i < steps; i++) {
        if (!connected) return;

        /* Ease-in-out: slow start, fast middle, slow end */
        float t = (float)(i + 1) / steps;
        float ease = t < 0.5f ? 2 * t * t : 1 - (-2 * t + 2) * (-2 * t + 2) / 2;

        float nx = target_x * ease;
        float ny = target_y * ease;
        int8_t dx = clamp8((int)(nx - cx));
        int8_t dy = clamp8((int)(ny - cy));
        cx += dx;
        cy += dy;

        /* Add subtle jitter like a real hand */
        if (esp_random() % 3 == 0) {
            dx += (esp_random() % 3) - 1;
            dy += (esp_random() % 3) - 1;
        }

        send_report(0, dx, dy, 0);
        vTaskDelay(pdMS_TO_TICKS(8 + (esp_random() % 12)));
    }
}

static void do_micro_adjust(void)
{
    /* Small corrections like aiming at a button */
    int n = 3 + (esp_random() % 5);
    for (int i = 0; i < n; i++) {
        if (!connected) return;
        int8_t dx = (esp_random() % 7) - 3;
        int8_t dy = (esp_random() % 7) - 3;
        send_report(0, dx, dy, 0);
        vTaskDelay(pdMS_TO_TICKS(20 + (esp_random() % 40)));
    }
}

static void do_scroll(void)
{
    int n = 2 + (esp_random() % 5);
    int8_t dir = (esp_random() % 2) ? 1 : -1;
    for (int i = 0; i < n; i++) {
        if (!connected) return;
        send_report(0, 0, 0, dir);
        vTaskDelay(pdMS_TO_TICKS(60 + (esp_random() % 100)));
    }
}

static void mouse_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Mouse task started, input_handle=%d", input_handle);
    while (1) {
        if (connected && input_handle) {
            uint32_t action = esp_random() % 100;

            if (action < 45) {
                /* Smooth move to a random point */
                float tx = (float)((int)(esp_random() % 301) - 150);
                float ty = (float)((int)(esp_random() % 201) - 100);
                int steps = 15 + (esp_random() % 30);
                do_smooth_move(tx, ty, steps);
            } else if (action < 70) {
                /* Move then micro-adjust */
                float tx = (float)((int)(esp_random() % 201) - 100);
                float ty = (float)((int)(esp_random() % 141) - 70);
                int steps = 12 + (esp_random() % 20);
                do_smooth_move(tx, ty, steps);
                do_micro_adjust();
            } else if (action < 85) {
                /* Scroll */
                do_scroll();
            } else {
                /* Micro adjustments */
                do_micro_adjust();
            }

            /* Short pause between actions - stay active */
            vTaskDelay(pdMS_TO_TICKS(200 + (esp_random() % 600)));
        } else {
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }
}

/* ---- NimBLE Host ---- */

static void nimble_host_task(void *param)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

static void ble_on_sync(void)
{
    /* Use best available address */
    int rc = ble_hs_util_ensure_addr(0);
    assert(rc == 0);

    ESP_LOGI(TAG, "BLE synced, input_handle=%d", input_handle);
    start_advertising();
}

static void ble_on_reset(int reason)
{
    ESP_LOGE(TAG, "BLE host reset: reason=%d", reason);
}

/* ---- Entry Point ---- */

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    ESP_ERROR_CHECK(nimble_port_init());

    /* Security: "Just Works" */
    ble_hs_cfg.sm_io_cap         = BLE_HS_IO_NO_INPUT_OUTPUT;
    ble_hs_cfg.sm_bonding        = 1;
    ble_hs_cfg.sm_mitm           = 0;
    ble_hs_cfg.sm_sc             = 1;
    ble_hs_cfg.sm_our_key_dist   = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.sync_cb           = ble_on_sync;
    ble_hs_cfg.reset_cb          = ble_on_reset;
    ble_hs_cfg.store_status_cb   = ble_store_util_status_rr;

    /* Initialize bond storage in NVS */
    ble_store_config_init();

    ble_svc_gap_device_name_set("ESP32-C6 Mouse");
    ble_svc_gap_init();
    ble_svc_gatt_init();

    /* Register HID + Battery + DevInfo GATT services */
    int rc = ble_gatts_count_cfg(gatt_svr_svcs);
    ESP_LOGI(TAG, "gatts_count_cfg: rc=%d", rc);
    assert(rc == 0);
    rc = ble_gatts_add_svcs(gatt_svr_svcs);
    ESP_LOGI(TAG, "gatts_add_svcs: rc=%d", rc);
    assert(rc == 0);

    ESP_LOGI(TAG, "GATT services registered, input_handle=%d", input_handle);

    nimble_port_freertos_init(nimble_host_task);
    xTaskCreate(mouse_task, "mouse_task", 4096, NULL, 5, NULL);
}
