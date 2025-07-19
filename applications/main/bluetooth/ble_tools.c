#include "ble_tools.h"

#include <furi.h>
#include <furi_hal_bt.h>
#include <m-array.h>

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

typedef enum {
    BleToolsViewIdMenu,
    BleToolsViewIdScan,
    BleToolsViewIdVariableItemList,
} BleToolsViewId;

typedef struct {
    View* view;
} BleToolScanner;

typedef struct {
    uint8_t ecode;
} LeMetaItem;

ARRAY_DEF(LeMetaEvents, LeMetaItem, M_POD_OPLIST);

typedef struct {
    ViewDispatcher* view_dispatcher;
    BleToolScanner* scanner;
    VariableItemList* variable_item_list;

    uint16_t events;
    uint16_t non_le_meta_events;
    LeMetaEvents_t le_meta_events;
} BleTools;

static GapConfig scan_template_config = {
    .adv_service =
        {
            .UUID_Type = UUID_TYPE_16,
            .Service_UUID_16 = 0xa6a6, // doesn't matter
        },
    .masks =
        {
            .event = DEFAULT_EVENT_MASK,
            //.event = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0b0001'0000}, // only meta events = 61
            .le_event = DEFAULT_LE_EVENT_MASK,
            //.le_event = {
            //    0b0000'0010, // le advertising report event = 1
            //    0b0001'0000, // le extended advertising report event = 12
            //    0b0000'0001, // le scan timeout event = 16
            //    0x00, 0x00, 0x00, 0x00, 0x00},
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

static void ble_tool_render_callback(Canvas* const canvas, void* context) {
    BleTools** ble_tools_ptr = context;
    BleTools* ble_tools = *ble_tools_ptr;

    FURI_LOG_D(
        TAG, "drawing (events: %d, non-le meta events: %d, le meta events: %d)",
        ble_tools->events,
        ble_tools->non_le_meta_events,
        LeMetaEvents_size(ble_tools->le_meta_events));

    canvas_set_font(canvas, FontPrimary);
    canvas_draw_str(canvas, 0, 8, "BLE Test");

    canvas_set_font(canvas, FontSecondary);

    char buffer[64];
    snprintf(buffer, sizeof(buffer), "Total Events: %d", ble_tools->events);
    canvas_draw_str(canvas, 0, 31, buffer);

    snprintf(buffer, sizeof(buffer), "Non-LE Meta Events: %d", ble_tools->non_le_meta_events);
    canvas_draw_str(canvas, 0, 42, buffer);

    snprintf(buffer, sizeof(buffer), "LE Meta Events: %d", LeMetaEvents_size(ble_tools->le_meta_events));
    canvas_draw_str(canvas, 0, 53, buffer);
}

static BleEventAckStatus ble_tools_event_handler(void* event, void* context) {
    BleTools* ble_tools = context;

    hci_event_pckt* event_pckt = (hci_event_pckt*)(((hci_uart_pckt*)event)->data);

    FURI_LOG_I(
        TAG,
        "received event in ble tool: event code: 0x%x",
        event_pckt->evt);

    ble_tools->events++;

    if(event_pckt->evt == HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE) {
        evt_blecore_aci* vendor_specific_event = (evt_blecore_aci*)event_pckt->data;
        FURI_LOG_W(TAG, "received vendor specific event: 0x%02X", vendor_specific_event->ecode);
        if (vendor_specific_event->ecode == ACI_GAP_PROC_COMPLETE_VSEVT_CODE)
            // TODO: return to menu in this case
            return BleEventAckFlowEnable;
        return BleEventNotAck;
    } else if(event_pckt->evt != HCI_LE_META_EVT_CODE) {
        FURI_LOG_W(TAG, "received non LE meta event");
        ble_tools->non_le_meta_events++;
        return BleEventNotAck;
    }

    evt_le_meta_event* meta_event = (evt_le_meta_event*)event_pckt->data;

    LeMetaItem* le_meta_event = LeMetaEvents_push_new(ble_tools->le_meta_events);
    uint8_t subevent_code = meta_event->subevent;
    le_meta_event->ecode = subevent_code;

    if (subevent_code == HCI_LE_EXTENDED_ADVERTISING_REPORT_SUBEVT_CODE) {
        FURI_LOG_I(TAG, "received extended advertising response");
        return BleEventAckFlowEnable;
    }

    if(subevent_code == HCI_LE_ADVERTISING_REPORT_SUBEVT_CODE) {
        FURI_LOG_I(TAG, "received advertising response");

        hci_le_advertising_report_event_rp0 *rp0 = (void*) meta_event->data;
        int report_index;
        for (report_index = 0; report_index < rp0->Num_Reports; report_index++) {
            Advertising_Report_t report = rp0->Advertising_Report[report_index];
            uint8_t event_data_size = report.Length_Data;

            FURI_LOG_I(
                TAG,
                "received advertising response from %02X-%02X-%02X-%02X-%02X-%02X",
                report.Address[0],
                report.Address[1],
                report.Address[2],
                report.Address[3],
                report.Address[4],
                report.Address[5] );

            int k = 0;
            uint8_t *adv_report_data;
            adv_report_data = (uint8_t*)(&report.Length_Data) + 1;

            while(k < event_data_size)
            {
                uint8_t adlength = adv_report_data[k];
                uint8_t adtype = adv_report_data[k + 1];
                switch (adtype)
                {
                    case AD_TYPE_FLAGS: /* now get flags */
                        /* USER CODE BEGIN AD_TYPE_FLAGS */
                        FURI_LOG_I(TAG, "flags");

                        /* USER CODE END AD_TYPE_FLAGS */
                        break;

                    case AD_TYPE_TX_POWER_LEVEL: /* Tx power level */
                        /* USER CODE BEGIN AD_TYPE_TX_POWER_LEVEL */
                        FURI_LOG_I(TAG, "transmit power level");

                        /* USER CODE END AD_TYPE_TX_POWER_LEVEL */
                        break;

                    case AD_TYPE_SERVICE_DATA: /* service data 16 bits */
                        /* USER CODE BEGIN AD_TYPE_SERVICE_DATA */
                        FURI_LOG_I(TAG, "service data");

                        /* USER CODE END AD_TYPE_SERVICE_DATA */
                        break;
                    case AD_TYPE_SHORTENED_LOCAL_NAME:
                        FURI_LOG_I(TAG, "name");
                        break;
                    case AD_TYPE_COMPLETE_LOCAL_NAME:
                        FURI_LOG_I(TAG, "complete name");
                        break;
                    case AD_TYPE_APPEARANCE:
                        FURI_LOG_I(TAG, "appearance");
                        break;

                    default:
                        /* USER CODE BEGIN adtype_default */
                        FURI_LOG_I(TAG, "not sure %d", adtype);

                        /* USER CODE END adtype_default */
                        break;
                } /* end switch adtype */
                k += adlength + 1;
            }
        }

        FURI_LOG_I(TAG, "Ack");
        return BleEventAckFlowEnable;
    }

    FURI_LOG_I(TAG, "ack (disable)");
    return BleEventAckFlowDisable;
}

static void ble_command_scan_start(void* context) {
    UNUSED(context);

    // https://github.com/STMicroelectronics/STM32CubeWB/blob/master/Projects/P-NUCLEO-WB55.Nucleo/Applications/BLE/BLE_p2pClient/STM32_WPAN/App/ble_conf.h

    struct hci_request rq;
    tBleStatus status = 0;

    uint8_t cmd_buffer[6];
    cmd_buffer[0] = 0x20; // interval
    cmd_buffer[1] = 0x03; 
    cmd_buffer[2] = 0x20; // window
    cmd_buffer[3] = 0x03; 
    cmd_buffer[4] = 0x01; // own address type (random)
    cmd_buffer[5] = 0x00; // disable filtering

    // opcode group field
    rq.ogf = 0x3f;
    // opcode command field
    rq.ocf = 0x97; // aci_gap_start_general_discovery_proc
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

static void ble_command_scan_stop(void* context) {
    UNUSED(context);

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

BleToolScanner* ble_tool_scan_view_alloc() {
    BleToolScanner* scanner = malloc(sizeof(BleToolScanner));
    View* view = view_alloc();
    view_set_draw_callback(view, ble_tool_render_callback);
    view_set_enter_callback(view, ble_command_scan_start);
    view_set_exit_callback(view, ble_command_scan_stop);

    view_allocate_model(view, ViewModelTypeLockFree, sizeof(BleTools**));

    scanner->view = view;
    return scanner;
}

View* ble_tool_view_scan_get_view(BleToolScanner* scanner) {
    return scanner->view;
}

void submenu_navigation_event(void* context, uint32_t index) {
    BleTools* ble_tools = context;
    FURI_LOG_D(TAG, "submenu nav event");

    // NOTE: I am making an intentional choice not to use the scene_manager, since I want to
    //       properly understand how the fundamentals work, with the goal of creating an idomatic
    //       rust wrapper around views and the view dispatcher
    if (index == BleToolsViewIdScan) {
        // we don't need to do any additional "on_enter" work here, since we can initialise the view
        // on startup, and then not touch it
        FURI_LOG_D(TAG, "navigating to scanner");
        view_dispatcher_switch_to_view(ble_tools->view_dispatcher, BleToolsViewIdScan);
    } else if (index == BleToolsViewIdVariableItemList) {
        // TODO: populate variable_item_list
        FURI_LOG_D(TAG, "navigating to reports");
        view_dispatcher_switch_to_view(ble_tools->view_dispatcher, BleToolsViewIdVariableItemList);
    }
}

uint32_t ble_tool_switch_to_start(void* context) {
    UNUSED(context);
    FURI_LOG_D(TAG, "view nav event");
    return BleToolsViewIdMenu;
}

bool ble_tool_navigation_event_callback(void* context) {
    UNUSED(context);
    FURI_LOG_D(TAG, "navigation event");
    return false;
}

int32_t ble_tools_app(void* p) {
    UNUSED(p);
    FURI_LOG_I(TAG, "v2.1");

    Gui* gui = furi_record_open(RECORD_GUI);
    ViewDispatcher* view_dispatcher = view_dispatcher_alloc();

    // view_dispatcher_set_event_callback_context(
    // input events are, by default, handled in the following order:
    // 1. view_dispatcher_handle_input
    // 2. the current view's input callback, if one exists
    // 3. if the event is a back event
    //   3a. attempt to return to the previous view, using the view's previous_callback. note: this isn't usually set
    //   3b. run the navigation_event_callback
    //   3c. if there is a navigation_event_callback, but it doesn't consume the event, then exit
    view_dispatcher_set_navigation_event_callback(view_dispatcher, ble_tool_navigation_event_callback);

    Submenu* submenu = submenu_alloc();
    view_dispatcher_add_view(view_dispatcher, BleToolsViewIdMenu, submenu_get_view(submenu));

    BleToolScanner* scanner = ble_tool_scan_view_alloc();
    view_dispatcher_add_view(
        view_dispatcher,
        BleToolsViewIdScan,
        ble_tool_view_scan_get_view(scanner));

    VariableItemList* variable_item_list = variable_item_list_alloc();
    view_dispatcher_add_view(
        view_dispatcher,
        BleToolsViewIdVariableItemList,
        variable_item_list_get_view(variable_item_list));

    // because we're not using the scene_manager, we don't have a view stack to handle back events gracefully
    // as such, we need to set the usually-unused previous_callback to return to this view
    view_set_previous_callback(scanner->view, ble_tool_switch_to_start);
    view_set_previous_callback(variable_item_list_get_view(variable_item_list), ble_tool_switch_to_start);

    BleTools* ble_tools = malloc(sizeof(BleTools));
    ble_tools->view_dispatcher = view_dispatcher;
    ble_tools->scanner = scanner;

    BleTools** view_model = view_get_model(scanner->view);
    *view_model = ble_tools;

    ble_tools->variable_item_list = variable_item_list;

    submenu_add_item(
        submenu,
        "Scan",
        BleToolsViewIdScan,
        submenu_navigation_event,
        ble_tools);
    submenu_add_item(
        submenu,
        "Advertising Reports",
        BleToolsViewIdVariableItemList,
        submenu_navigation_event,
        ble_tools);

    variable_item_list_add(variable_item_list, "Test", 0, NULL, NULL);

    FURI_LOG_I(TAG, "attach dispatcher to view");
    view_dispatcher_attach_to_gui(view_dispatcher, gui, ViewDispatcherTypeFullscreen);

    FURI_LOG_I(TAG, "initing ble handlers");
    Bt* bt = furi_record_open(RECORD_BT);
    // doesn't need cleaning up??
    FuriHalBleProfileBase* tool_profile = bt_profile_start(bt, ble_tool_profile, NULL);
    FURI_LOG_I(TAG, "started profile?");
    furi_check(tool_profile);
    FURI_LOG_I(TAG, "yes!");

    FURI_LOG_I(TAG, "binding to ble handler");
    GapSvcEventHandler* event_handler = ble_event_dispatcher_register_svc_handler(ble_tools_event_handler, tool_profile);

    view_dispatcher_switch_to_view(view_dispatcher, BleToolsViewIdMenu);
   
    FURI_LOG_I(TAG, "ble tool running");
    view_dispatcher_run(view_dispatcher);

    FURI_LOG_I(TAG, "ble tool finished running, finishing up");

    FURI_LOG_I(TAG, "resetting BLE");
    ble_event_dispatcher_unregister_svc_handler(event_handler);
    FURI_LOG_I(TAG, "BLE reset, restoring serial service");
    bt_profile_restore_default(bt);
    FURI_LOG_I(TAG, "restored serial service");

    bt = NULL;
    furi_record_close(RECORD_BT);

    FURI_LOG_I(TAG, "cleaning view and freeing");

    view_dispatcher_remove_view(view_dispatcher, BleToolsViewIdMenu);
    submenu_free(submenu);

    view_dispatcher_remove_view(view_dispatcher, BleToolsViewIdVariableItemList);
    variable_item_list_free(variable_item_list);

    view_dispatcher_remove_view(view_dispatcher, BleToolsViewIdScan);
    view_free(scanner->view);

    free(ble_tools);
    view_dispatcher_free(view_dispatcher);

    furi_record_close(RECORD_GUI);

    FURI_LOG_I(TAG, "complete");
    return 0;
}
