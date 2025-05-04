#include "ble_tools.h"

#include <furi.h>
#include <furi_hal_bt.h>

#include <bt/bt_service/bt.h>

#include <furi.h>
#include <gui/gui.h>
#include <input/input.h>

#include <gap.h>
#include <furi_ble/event_dispatcher.h>

#include <ble/ble.h>
#include <furi_hal_bt.h>

#include <stdint.h>
#include <core/log.h>

#define TAG "BleTool =^_^="

// AN5289: 4.7, in order to use flash controller interval must be at least 25ms + advertisement, which is 30 ms
// Since we don't use flash controller anymore interval can be lowered to 7.5ms
#define CONNECTION_INTERVAL_MIN (0x06)
// Up to 45 ms
#define CONNECTION_INTERVAL_MAX (0x24)

typedef PACKED_STRUCT
{
  uint8_t type;
  uint8_t data[1];
} hci_uart_pckt;

typedef PACKED_STRUCT
{
  uint8_t         evt;
  uint8_t         plen;
  uint8_t         data[1];
} hci_event_pckt;

static GapConfig scan_template_config = {
    .adv_service =
        {
            .UUID_Type = UUID_TYPE_16,
            .Service_UUID_16 = 0xa6a6, // doesn't matter
        },
    .role = GAP_CENTRAL_ROLE,
    .appearance_char = 0x8600, // doesn't matter
    .bonding_mode = false, // we're only scanning, don't need to remember anything
    .pairing_method = GapPairingPinCodeShow,
    .conn_param = {
        .conn_int_min = CONNECTION_INTERVAL_MIN,
        .conn_int_max = CONNECTION_INTERVAL_MAX,
        .slave_latency = 0,
        .supervisor_timeout = 0,
    }};

static void ble_tool_input_callback(InputEvent* input_event, void* context) {
    FuriThreadId thread_id = context;

    if(input_event->key == InputKeyBack && input_event->type == InputTypePress) {
        FURI_LOG_I(TAG, "sending flag to exit ble tool");
        furi_thread_flags_set(thread_id, 1);
    }
}

static void ble_tool_render_callback(Canvas* const canvas, void* context) {
    UNUSED(context);

    canvas_set_font(canvas, FontPrimary);
    canvas_draw_str(canvas, 20, 20, "BLE Test");
}

static BleEventAckStatus ble_tools_event_handler(void* event, void* context) {
    UNUSED(context);

    hci_uart_pckt* pUartPckt = (hci_uart_pckt*) &event;
    hci_event_pckt* event_pckt = (hci_event_pckt*) (pUartPckt->data);
    evt_blecore_aci* blecore_evt = (evt_blecore_aci*)event_pckt->data;

    FURI_LOG_D(
        TAG,
        "received event in ble tool: opcode: 0x%x, event type: 0x%x",
        event_pckt->evt,
        pUartPckt->type);

    if(event_pckt->evt != HCI_LE_META_EVT_CODE) {
        FURI_LOG_W(TAG, "received non LE meta event");
        return BleEventNotAck;
    }

    if (blecore_evt->ecode == HCI_LE_EXTENDED_ADVERTISING_REPORT_SUBEVT_CODE) {
        FURI_LOG_D(TAG, "received extended advertising response");
        return BleEventAckFlowEnable;
    }

    if(blecore_evt->ecode != HCI_LE_ADVERTISING_REPORT_SUBEVT_CODE) {
        hci_le_advertising_report_event_rp0 *rp0 = (void*) blecore_evt->data;
        int report_index;
        for (report_index = 0; report_index < rp0->Num_Reports; report_index++) {
            Advertising_Report_t report = rp0->Advertising_Report[report_index];
            FURI_LOG_D(TAG, "received advertising response from %s", report.Address);
            int i = 0;
            do {
                int length = report.Data[i++];
                int type = report.Data[i++];
                switch (type) {
                    case 0x01:
                        FURI_LOG_D(TAG, "flags");
                        break;
                    case 0x02:
                    case 0x03:
                    case 0x04:
                    case 0x05:
                    case 0x06:
                    case 0x07:
                        FURI_LOG_D(TAG, "service ids");
                        break;
                    case 0x08:
                        FURI_LOG_D(TAG, "name (short)");
                        break;
                    case 0x09:
                        FURI_LOG_D(TAG, "name (complete)");
                        break;
                    case 0x0A:
                        FURI_LOG_D(TAG, "TX power");
                        break;
                    case 0x0D:
                        FURI_LOG_D(TAG, "Device class");
                        break;
                    case 0x0E:
                        FURI_LOG_D(TAG, "Pairing Hash");
                        break;
                    case 0x0F:
                        FURI_LOG_D(TAG, "Pairing Randomiser");
                        break;
                    case 0x10:
                        FURI_LOG_D(TAG, "Device ID");
                        break;
                    default:
                        FURI_LOG_D(TAG, "Unhandled data type %x", type);
                        break;
                }

                i += length - 1;
            }
            while (i < report.Length_Data);
        }

        return BleEventAckFlowEnable;
    }

    return BleEventNotAck;
}

static void ble_command_scan_start(void) {
    struct hci_request rq;
    tBleStatus status = 0;

    uint8_t cmd_buffer[6];
    cmd_buffer[0] = 0x00; // scan type (passive)
    cmd_buffer[1] = 0x04; // interval
    cmd_buffer[2] = 0x00;
    cmd_buffer[3] = 0x04; // window
    cmd_buffer[4] = 0x01;
    cmd_buffer[5] = 0x00; // own address type (random)

    // opcode group field
    rq.ogf = 0x3f;
    // opcode command field
    rq.ocf = 0x97;
    rq.event = 0xFF; // unused?
    rq.cparam = cmd_buffer;
    rq.clen = 6;
    rq.rparam = &status;
    rq.rlen = 1;

    FURI_LOG_I(TAG, "sending command to start scan");

    if (hci_send_req(&rq, 0) < 0) {
        FURI_LOG_E(TAG, "timed out waiting for response to starting scan");
        return;
    }
    if (status) {
        FURI_LOG_E(TAG, "status response from starting scan was error: %i", status);
        return;
    }

    FURI_LOG_I(TAG, "command sent");
}

static void ble_command_scan_stop(void) {
    struct hci_request rq;
    tBleStatus status = 0;

    uint8_t cmd_buffer[2];
    cmd_buffer[0] = 0x01; // limited discovery procedure

    // opcode group field
    rq.ogf = 0x3f;
    // opcode command field
    rq.ocf = 0x93;
    rq.event = 0x0F; // unused?
    rq.cparam = cmd_buffer;
    rq.clen = 1;
    rq.rparam = &status;
    rq.rlen = 1;

    FURI_LOG_I(TAG, "sending command to stop scan");
    if (hci_send_req(&rq, 0) < 0) {
        FURI_LOG_E(TAG, "timed out waiting for response to stop command");
        return;
    }
    if (status) {
        FURI_LOG_E(TAG, "status response from stopping scan was error: %i", status);
        return;
    }

    FURI_LOG_I(TAG, "command sent");
}

typedef struct {
    FuriHalBleProfileBase base;
} BleToolProfile;

static FuriHalBleProfileBase* ble_tool_profile_start(FuriHalBleProfileParams profile_params) {
    UNUSED(profile_params);

    BleToolProfile* profile = malloc(sizeof(FuriHalBleProfileBase));
    profile->base.config = ble_tool_profile;

    return &profile->base;
}

static void ble_tool_profile_stop(FuriHalBleProfileBase* profile) {
    UNUSED(profile);

    // do nothing, we're not storing anything in the profile and freeing will be handled in caller
}

static void ble_tool_profile_get_config(GapConfig* config, FuriHalBleProfileParams profile_params) {
    UNUSED(profile_params);

    furi_check(config);
    memcpy(config, &scan_template_config, sizeof(GapConfig));
    // Set mac address
    memcpy(config->mac_address, furi_hal_version_get_ble_mac(), sizeof(config->mac_address));

    // Change MAC address for HID profile
    config->mac_address[0] ^= 0x0002;
    config->mac_address[1] ^= 0x0002 >> 8;
    config->mac_address[2] += 2;

    // Set advertise name
    memset(config->adv_name, 0, sizeof(config->adv_name));
    FuriString* name = furi_string_alloc_set(furi_hal_version_get_ble_local_device_name_ptr());

    const char* scanner_str = "Scanner";
    furi_string_replace_str(name, "Flipper", scanner_str);
    if(furi_string_size(name) >= sizeof(config->adv_name)) {
        furi_string_left(name, sizeof(config->adv_name) - 1);
    }
    memcpy(config->adv_name, furi_string_get_cstr(name), furi_string_size(name));
    furi_string_free(name);
}

static const FuriHalBleProfileTemplate profile_callbacks = {
    .start = ble_tool_profile_start,
    .stop = ble_tool_profile_stop,
    .get_gap_config = ble_tool_profile_get_config,
};

const FuriHalBleProfileTemplate* ble_tool_profile = &profile_callbacks;

int32_t ble_tools_app(void* p) {
    UNUSED(p);
    FURI_LOG_I(TAG, "v13");

    FURI_LOG_I(TAG, "allocating view port");
    ViewPort* view_port = view_port_alloc();
    FURI_LOG_I(TAG, "setting render callback");
    view_port_draw_callback_set(view_port, ble_tool_render_callback, NULL);
    FURI_LOG_I(TAG, "setting input callback");
    view_port_input_callback_set(view_port, ble_tool_input_callback, furi_thread_get_current_id());

    FURI_LOG_I(TAG, "opening gui");
    Gui* gui = furi_record_open(RECORD_GUI);
    FURI_LOG_I(TAG, "binding view port to gui");
    gui_add_view_port(gui, view_port, GuiLayerFullscreen);

    FURI_LOG_I(TAG, "updating view port");
    view_port_update(view_port);

    FURI_LOG_I(TAG, "initing ble handlers");
    Bt* bt = furi_record_open(RECORD_BT);
    // doesn't need cleaning up??
    FuriHalBleProfileBase* tool_profile = bt_profile_start(bt, ble_tool_profile, NULL);
    FURI_LOG_I(TAG, "started profile?");
    furi_check(tool_profile);
    FURI_LOG_I(TAG, "yes!");

    furi_delay_ms(200);

    FURI_LOG_I(TAG, "binding to ble handler");
    GapSvcEventHandler* event_handler = ble_event_dispatcher_register_svc_handler(ble_tools_event_handler, tool_profile);

    FURI_LOG_I(TAG, "starting scan");
    ble_command_scan_start();
    FURI_LOG_I(TAG, "scan started (maybe)");
   
    FURI_LOG_I(TAG, "ble tool waiting");
    furi_thread_flags_wait(1, FuriFlagWaitAny, FuriWaitForever);

    FURI_LOG_I(TAG, "ble tool finished waiting, finishing up");
    ble_command_scan_stop();

    view_port_enabled_set(view_port, false);
    gui_remove_view_port(gui, view_port);
    furi_record_close(RECORD_GUI);

    ble_event_dispatcher_unregister_svc_handler(event_handler);
    bt_profile_restore_default(bt);
    bt = NULL;
    furi_record_close(RECORD_BT);
    view_port_free(view_port);

    return 0;
}
