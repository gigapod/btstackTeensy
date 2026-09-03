/**
 * Arduino Wrapper for BTstack -- Teensy 4.x + Murata Type 1YN (CYW43439) port
 *
 * BLE (GAP/GATT) section is adapted from BlueKitchen's original Arduino port
 * (port/arduino/BTstack.cpp) with the EM9301/SPI bring-up swapped for the
 * Murata 1YN/BCM 4-wire UART bring-up (see hal_teensy_uart.cpp,
 * teensy_bt_control.cpp). Classic (A2DP Sink + AVRCP, HFP Hands-Free) section
 * is adapted from example/a2dp_sink_demo.c and example/hfp_hf_demo.c the same
 * way port/teensy41-murata-1yn/src/app_a2dp_hfp.cpp did, wired to
 * hal_audio_teensy.cpp (A2DP, I2S1) and hal_teensy_pcm.cpp (HFP voice,
 * PCM/I2S2). See README.md for wiring and feature notes.
 */

#include <Arduino.h>

#include "BTstackTeensy.h"

#include "btstack_memory.h"
#include "hal_cpu.h"
#include "hal_time_ms.h"
#include "hci_cmd.h"
#include "btstack_util.h"
#include "btstack_run_loop.h"
#include "btstack_event.h"
#include "btstack_run_loop_embedded.h"
#include "hci_transport.h"
#include "hci_transport_h4.h"

#include "ad_parser.h"
#include "btstack_chipset_bcm.h"
#include "btstack_debug.h"
#include "gap.h"
#include "hci.h"
#include "hci_dump.h"
#include "hci_dump_embedded_stdout.h"
#include "l2cap.h"
#include "ble/att_db.h"
#include "ble/att_server.h"
#include "ble/att_db_util.h"
#include "ble/le_device_db.h"
#include "ble/sm.h"

// Classic (A2DP Sink + AVRCP, HFP Hands-Free)
#include "classic/a2dp_sink.h"
#include "classic/avdtp_util.h"
#include "classic/avrcp.h"
#include "classic/avrcp_controller.h"
#include "classic/avrcp_target.h"
#include "classic/sdp_util.h"
#include "classic/device_id_server.h"
#include "classic/hfp.h"
#include "classic/hfp_hf.h"
#include "classic/rfcomm.h"
#include "classic/btstack_sbc_bluedroid.h"
#include "btstack_resample.h"
#include "btstack_ring_buffer.h"
#include "btstack_audio.h"
#include "bluetooth_company_id.h"

#include "hal_teensy_uart.h"
#include "teensy_bt_control.h"
#include "hal_teensy_pcm.h"

// Declared in platform/embedded/btstack_uart_block_embedded.c -- no header
// ships with it upstream.
extern "C" const btstack_uart_block_t *btstack_uart_block_embedded_instance(void);

// prototypes
extern "C" void hal_teensy_uart_poll(void);

enum {
    SET_ADVERTISEMENT_PARAMS  = 1 << 0,
    SET_ADVERTISEMENT_DATA    = 1 << 1,
    SET_ADVERTISEMENT_ENABLED = 1 << 2,
};

typedef enum gattAction {
    gattActionWrite,
    gattActionSubscribe,
    gattActionUnsubscribe,
    gattActionServiceQuery,
    gattActionCharacteristicQuery,
    gattActionRead,
} gattAction_t;

static gattAction_t gattAction;

// btstack state
static int btstack_state;

static const uint8_t iBeaconAdvertisement01[] = { 0x02, 0x01 };
static const uint8_t iBeaconAdvertisement38[] = { 0x1a, 0xff, 0x4c, 0x00, 0x02, 0x15 };
static uint8_t   adv_data[31];
static uint16_t  adv_data_len = 0;

// adv_data is 31 bytes total; 3 go to the flags field and 2 to the name
// field's own length+type header, leaving this many for the name itself.
#define BTSTACK_TEENSY_MAX_DEVICE_NAME_LEN (31 - 3 - 2)
static char device_name[BTSTACK_TEENSY_MAX_DEVICE_NAME_LEN + 1] = "BTstack Teensy";

static bool have_custom_addr;
static bd_addr_t public_bd_addr;

static bool classic_sdp_initialized;
static bool rfcomm_initialized;

static btstack_timer_source_t connection_timer;

static void (*bleAdvertismentCallback)(BTBLEAdvertisement * bleAdvertisement) = NULL;
static void (*bleDeviceConnectedCallback)(BLEStatus status, BTBLEDevice * device)= NULL;
static void (*bleDeviceDisconnectedCallback)(BTBLEDevice * device) = NULL;
static void (*gattServiceDiscoveredCallback)(BLEStatus status, BTBLEDevice * device, BTBLEService * bleService) = NULL;
static void (*gattCharacteristicDiscoveredCallback)(BLEStatus status, BTBLEDevice * device, BTBLECharacteristic * characteristic) = NULL;
static void (*gattCharacteristicNotificationCallback)(BTBLEDevice * device, uint16_t value_handle, uint8_t* value, uint16_t length) = NULL;
static void (*gattCharacteristicIndicationCallback)(BTBLEDevice * device, uint16_t value_handle, uint8_t* value, uint16_t length) = NULL;
static void (*gattCharacteristicReadCallback)(BLEStatus status, BTBLEDevice * device, uint8_t * value, uint16_t length) = NULL;
static void (*gattCharacteristicWrittenCallback)(BLEStatus status, BTBLEDevice * device) = NULL;
static void (*gattCharacteristicSubscribedCallback)(BLEStatus status, BTBLEDevice * device) = NULL;
static void (*gattCharacteristicUnsubscribedCallback)(BLEStatus status, BTBLEDevice * device) = NULL;

static void (*a2dpStreamStateCallback)(A2DPStreamState state) = NULL;
static void (*hfpEventCallback)(HFPEvent event, hci_con_handle_t handle) = NULL;

// retarget printf/log output to USB Serial
static int serial_stdio_write(void * cookie, const char * buffer, int size){
    (void) cookie;
    return Serial.write((const uint8_t *) buffer, size);
}

static void setup_stdio_over_usb_serial(void){
    static FILE * serial_stream = fwopen(NULL, &serial_stdio_write);
    if (serial_stream != NULL){
        setvbuf(serial_stream, NULL, _IONBF, 0);
        stdout = serial_stream;
        stderr = serial_stream;
    }
}

// HAL CPU Implementation lives in hal_teensy_cpu_time.cpp

//
static void packet_handler (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){

    hci_con_handle_t con_handle;

    switch (packet_type) {

        case HCI_EVENT_PACKET:
            switch (packet[0]) {

                case BTSTACK_EVENT_STATE:
                    btstack_state = packet[2];
                    switch (btstack_event_state_get_state(packet)){
                        case HCI_STATE_INITIALIZING:
                            printf("cyw43439: HCI initializing (reset, firmware download, feature/buffer queries)...\n");
                            break;
                        case HCI_STATE_WORKING: {
                            bd_addr_t addr;
                            gap_local_bd_addr(addr);
                            printf("BTstack up and running at %s\n",  bd_addr_to_str(addr));
                            break;
                        }
                        case HCI_STATE_HALTING:
                            printf("cyw43439: HCI halting\n");
                            break;
                        case HCI_STATE_OFF:
                            printf("cyw43439: HCI off\n");
                            break;
                        default:
                            break;
                    }
                    break;

                case HCI_EVENT_DISCONNECTION_COMPLETE:
                    if (bleDeviceDisconnectedCallback) {
                        con_handle = little_endian_read_16(packet, 3);
                        BTBLEDevice device(con_handle);
                        (*bleDeviceDisconnectedCallback)(&device);
                    }
                    break;

                case GAP_EVENT_ADVERTISING_REPORT: {
                    if (bleAdvertismentCallback) {
                        BTBLEAdvertisement advertisement(packet);
                        (*bleAdvertismentCallback)(&advertisement);
                    }
                    break;
                }

                case HCI_EVENT_LE_META:
                    switch (packet[2]) {
                        case HCI_SUBEVENT_LE_CONNECTION_COMPLETE:
                            con_handle = little_endian_read_16(packet, 4);
                            printf("Connection complete, con_handle 0x%04x\n", con_handle);
                            btstack_run_loop_remove_timer(&connection_timer);
                            if (!bleDeviceConnectedCallback) break;
                            if (packet[3]){
                                (*bleDeviceConnectedCallback)(BLE_STATUS_CONNECTION_ERROR, NULL);
                            } else {
                                BTBLEDevice device(con_handle);
                                (*bleDeviceConnectedCallback)(BLE_STATUS_OK, &device);
                            }
                            break;
                        default:
                            break;
                    }
                    break;
            }
    }
}

static void extract_service(gatt_client_service_t * service, uint8_t * packet){
    service->start_group_handle = little_endian_read_16(packet, 4);
    service->end_group_handle   = little_endian_read_16(packet, 6);
    service->uuid16 = 0;
    reverse_128(&packet[8], service->uuid128);
    if (uuid_has_bluetooth_prefix(service->uuid128)){
        service->uuid16 = big_endian_read_32(service->uuid128, 0);
    }
}

static void extract_characteristic(gatt_client_characteristic_t * characteristic, uint8_t * packet){
    characteristic->start_handle = little_endian_read_16(packet, 4);
    characteristic->value_handle = little_endian_read_16(packet, 6);
    characteristic->end_handle =   little_endian_read_16(packet, 8);
    characteristic->properties =   little_endian_read_16(packet, 10);
    characteristic->uuid16 = 0;
    reverse_128(&packet[12], characteristic->uuid128);
    if (uuid_has_bluetooth_prefix(characteristic->uuid128)){
        characteristic->uuid16 = big_endian_read_32(characteristic->uuid128, 0);
    }
}

static void gatt_client_callback(uint8_t packet_type, uint16_t channel, uint8_t * packet, uint16_t size){
    (void) channel;

    // if (hci) event is not 4-byte aligned, event->handle causes crash
    // workaround: check event type, assuming GATT event types are contagious
    if (packet[0] < GATT_EVENT_QUERY_COMPLETE) return;
    if (packet[0] > GATT_EVENT_MTU) return;

    hci_con_handle_t con_handle = little_endian_read_16(packet, 2);
    uint8_t   status;
    uint8_t * value;
    uint16_t  value_handle;
    uint16_t  value_length;

    BTBLEDevice device(con_handle);
    switch(hci_event_packet_get_type(packet)){
        case GATT_EVENT_SERVICE_QUERY_RESULT:
            if (gattServiceDiscoveredCallback) {
                gatt_client_service_t service;
                extract_service(&service, packet);
                BTBLEService bleService(service);
                (*gattServiceDiscoveredCallback)(BLE_STATUS_OK, &device, &bleService);
            }
            break;
        case GATT_EVENT_CHARACTERISTIC_QUERY_RESULT:
            if (gattCharacteristicDiscoveredCallback){
                gatt_client_characteristic_t characteristic;
                extract_characteristic(&characteristic, packet);
                BTBLECharacteristic bleCharacteristic(characteristic);
               (*gattCharacteristicDiscoveredCallback)(BLE_STATUS_OK, &device, &bleCharacteristic);
            }
            break;
        case GATT_EVENT_QUERY_COMPLETE:
            status = little_endian_read_16(packet, 4);
            switch (gattAction){
                case gattActionWrite:
                    if (gattCharacteristicWrittenCallback) gattCharacteristicWrittenCallback(status ? BLE_STATUS_OTHER_ERROR : BLE_STATUS_OK, &device);
                    break;
                case gattActionSubscribe:
                    if (gattCharacteristicSubscribedCallback) gattCharacteristicSubscribedCallback(status ? BLE_STATUS_OTHER_ERROR : BLE_STATUS_OK, &device);
                    break;
                case gattActionUnsubscribe:
                    if (gattCharacteristicUnsubscribedCallback) gattCharacteristicUnsubscribedCallback(status ? BLE_STATUS_OTHER_ERROR : BLE_STATUS_OK, &device);
                    break;
                case gattActionServiceQuery:
                    if (gattServiceDiscoveredCallback) gattServiceDiscoveredCallback(BLE_STATUS_DONE, &device, NULL);
                    break;
                case gattActionCharacteristicQuery:
                    if (gattCharacteristicDiscoveredCallback) gattCharacteristicDiscoveredCallback(BLE_STATUS_DONE, &device, NULL);
                    break;
                default:
                    break;
            };
            break;
        case GATT_EVENT_NOTIFICATION:
            if (gattCharacteristicNotificationCallback) {
                value_handle = little_endian_read_16(packet, 4);
                value_length = little_endian_read_16(packet, 6);
                value = &packet[8];
                (*gattCharacteristicNotificationCallback)(&device, value_handle, value, value_length);
            }
            break;
        case GATT_EVENT_INDICATION:
            if (gattCharacteristicIndicationCallback) {
                value_handle = little_endian_read_16(packet, 4);
                value_length = little_endian_read_16(packet, 6);
                value = &packet[8];
                (*gattCharacteristicIndicationCallback)(&device, value_handle, value, value_length);
            }
            break;
        case GATT_EVENT_CHARACTERISTIC_VALUE_QUERY_RESULT:
            if (gattCharacteristicReadCallback) {
                value_handle = little_endian_read_16(packet, 4);
                value_length = little_endian_read_16(packet, 6);
                value = &packet[8];
                (*gattCharacteristicReadCallback)(BLE_STATUS_OK, &device, value, value_length);
            }
            break;
        default:
            break;
    }
}

static void connection_timeout_handler(btstack_timer_source_t * timer){
    gap_connect_cancel();
    if (!bleDeviceConnectedCallback) return;
    (*bleDeviceConnectedCallback)(BLE_STATUS_CONNECTION_TIMEOUT, NULL);  // page timeout 0x04
}

//

/// BTUUID class
BTUUID::BTUUID(void){
    memset(uuid, 0, 16);
}

BTUUID::BTUUID(const uint8_t uuid[16]){
    memcpy(this->uuid, uuid, 16);
}

BTUUID::BTUUID(const char * uuidStr){
    memset(uuid, 0, 16);
    int len = strlen(uuidStr);
    if (len <= 4){
        // Handle 4 Bytes HEX
        uint16_t uuid16;
        int result = sscanf( (char *) uuidStr, "%x", &uuid16);
        if (result == 1){
            uuid_add_bluetooth_prefix(uuid, uuid16);
        }
        return;
    }

    // quick BTUUID parser, ignoring dashes
    int i = 0;
    int data = 0;
    int have_nibble = 0;
    while(*uuidStr && i < 16){
        const char c = *uuidStr++;
        if (c == '-') continue;
        data = data << 4 | nibble_for_char(c);
        if (!have_nibble) {
            have_nibble = 1;
            continue;
        }
        uuid[i++] = data;
        data = 0;
        have_nibble = 0;
    }
}

const uint8_t * BTUUID::getUuid(void) const {
    return uuid;
}

static char uuid16_buffer[5];
const char * BTUUID::getUuidString() const {
    if (uuid_has_bluetooth_prefix((uint8_t*)uuid)){
        sprintf(uuid16_buffer, "%04x", (uint16_t) big_endian_read_32(uuid, 0));
        return uuid16_buffer;
    }  else {
        return uuid128_to_str((uint8_t*)uuid);
    }
}

const char * BTUUID::getUuid128String() const {
    return uuid128_to_str((uint8_t*)uuid);
}

bool BTUUID::matches(BTUUID *other)        const {
    return memcmp(this->uuid, other->uuid, 16) == 0;
}


// BTBD_ADDR class
BTBD_ADDR::BTBD_ADDR(void){
}

BTBD_ADDR::BTBD_ADDR(const char * address_string, BTBD_ADDR_TYPE address_type ) : address_type(address_type) {
    // TODO: implement
}

BTBD_ADDR::BTBD_ADDR(const uint8_t address[6], BTBD_ADDR_TYPE address_type) : address_type(address_type){
    memcpy(this->address, address, 6);
}

const uint8_t * BTBD_ADDR::getAddress(void){
    return address;
}

const char * BTBD_ADDR::getAddressString(void){
    return bd_addr_to_str(address);
}

BTBD_ADDR_TYPE BTBD_ADDR::getAddressType(void){
    return address_type;
}


BTBLEAdvertisement::BTBLEAdvertisement(uint8_t * event_packet) :
advertising_event_type(event_packet[2]),
rssi(event_packet[10]),
data_length(event_packet[11]),
iBeacon_UUID(NULL)
{
    bd_addr_t addr;
    reverse_bd_addr(&event_packet[4], addr);
    bd_addr = BTBD_ADDR(addr, (BTBD_ADDR_TYPE)event_packet[3]);
    memcpy(data, &event_packet[12], LE_ADVERTISING_DATA_SIZE);
}

BTBLEAdvertisement::~BTBLEAdvertisement(){
    if (iBeacon_UUID) delete(iBeacon_UUID);
}

const uint8_t * BTBLEAdvertisement::getAdvData(void){
    return data;
}

BTBD_ADDR * BTBLEAdvertisement::getBdAddr(void){
    return &bd_addr;
}

int BTBLEAdvertisement::getRssi(void){
    return rssi > 127 ? rssi - 256 : rssi;
}


bool BTBLEAdvertisement::containsService(BTUUID * service){
    return ad_data_contains_uuid128(data_length, data, (uint8_t*) service->getUuid());
}

bool BTBLEAdvertisement::nameHasPrefix(const char * name_prefix){
    ad_context_t context;
    int name_prefix_len = strlen(name_prefix);
    for (ad_iterator_init(&context, data_length, data) ; ad_iterator_has_more(&context) ; ad_iterator_next(&context)){
        uint8_t data_type = ad_iterator_get_data_type(&context);
        uint8_t data_len  = ad_iterator_get_data_len(&context);
        uint8_t * data    = ad_iterator_get_data(&context);
        int compare_len = name_prefix_len;
        switch(data_type){
            case 8: // shortented local name
            case 9: // complete local name
                if (compare_len > data_len) compare_len = data_len;
                if (strncmp(name_prefix, (const char*) data, compare_len) == 0) return true;
                break;
            default:
                break;
        }
    }
    return false;
};

const char * BTBLEAdvertisement::getName(void){
    ad_context_t context;
    uint8_t * shortened_name = NULL;
    uint8_t   shortened_name_len = 0;
    for (ad_iterator_init(&context, data_length, data) ; ad_iterator_has_more(&context) ; ad_iterator_next(&context)){
        uint8_t data_type = ad_iterator_get_data_type(&context);
        uint8_t data_len  = ad_iterator_get_data_len(&context);
        uint8_t * data    = ad_iterator_get_data(&context);
        switch (data_type){
            case 9: // complete local name -- prefer this, return immediately
                if (data_len > LE_ADVERTISING_DATA_SIZE) data_len = LE_ADVERTISING_DATA_SIZE;
                memcpy(name_buffer, data, data_len);
                name_buffer[data_len] = '\0';
                return name_buffer;
            case 8: // shortened local name -- remember, keep looking for a complete one
                shortened_name = data;
                shortened_name_len = data_len;
                break;
            default:
                break;
        }
    }
    if (!shortened_name) return NULL;
    if (shortened_name_len > LE_ADVERTISING_DATA_SIZE) shortened_name_len = LE_ADVERTISING_DATA_SIZE;
    memcpy(name_buffer, shortened_name, shortened_name_len);
    name_buffer[shortened_name_len] = '\0';
    return name_buffer;
}

bool BTBLEAdvertisement::isIBeacon(void){
    return ((memcmp(iBeaconAdvertisement01,  data,    sizeof(iBeaconAdvertisement01)) == 0)
      &&    (memcmp(iBeaconAdvertisement38, &data[3], sizeof(iBeaconAdvertisement38)) == 0));
}

const BTUUID * BTBLEAdvertisement::getIBeaconUUID(void){
    if (!iBeacon_UUID){
        iBeacon_UUID = new BTUUID(&data[9]);
    }
    return iBeacon_UUID;
};
uint16_t BTBLEAdvertisement::getIBeaconMajorID(void){
    return big_endian_read_16(data, 25);
};
uint16_t BTBLEAdvertisement::getIBecaonMinorID(void){
    return big_endian_read_16(data, 27);
};
uint8_t BTBLEAdvertisement::getiBeaconMeasuredPower(void){
    return data[29];
}


BTBLECharacteristic::BTBLECharacteristic(void){
}

BTBLECharacteristic::BTBLECharacteristic(gatt_client_characteristic_t characteristic)
: characteristic(characteristic), uuid(characteristic.uuid128) {
}

const BTUUID * BTBLECharacteristic::getUUID(void){
    return &uuid;
}

bool BTBLECharacteristic::matches(BTUUID * uuid){
    return this->uuid.matches(uuid);
}

bool BTBLECharacteristic::isValueHandle(uint16_t value_handle){
    return characteristic.value_handle == value_handle;
}

const gatt_client_characteristic_t * BTBLECharacteristic::getCharacteristic(void){
    return &characteristic;
}


BTBLEService::BTBLEService(void){
}

BTBLEService::BTBLEService(gatt_client_service_t service)
: service(service), uuid(service.uuid128){
}

const BTUUID * BTBLEService::getUUID(void){
    return &uuid;
}

bool BTBLEService::matches(BTUUID * uuid){
    return this->uuid.matches(uuid);
}

const gatt_client_service_t * BTBLEService::getService(void){
    return &service;
}

// discovery of services and characteristics
BTBLEDevice::BTBLEDevice(void){
}
BTBLEDevice::BTBLEDevice(hci_con_handle_t handle)
: handle(handle){
}
uint16_t BTBLEDevice::getHandle(void){
    return handle;
}
int BTBLEDevice::discoverGATTServices(void){
    return BTstackTeensy.discoverGATTServices(this);
}
int BTBLEDevice::discoverCharacteristicsForService(BTBLEService * service){
    return BTstackTeensy.discoverCharacteristicsForService(this, service);
}
int BTBLEDevice::readCharacteristic(BTBLECharacteristic * characteristic){
    return BTstackTeensy.readCharacteristic(this, characteristic);
}
int BTBLEDevice::writeCharacteristic(BTBLECharacteristic * characteristic, uint8_t * data, uint16_t size){
    return BTstackTeensy.writeCharacteristic(this, characteristic, data, size);
}
int BTBLEDevice::writeCharacteristicWithoutResponse(BTBLECharacteristic * characteristic, uint8_t * data, uint16_t size){
    return BTstackTeensy.writeCharacteristicWithoutResponse(this, characteristic, data, size);
}
int BTBLEDevice::subscribeForNotifications(BTBLECharacteristic * characteristic){
    return BTstackTeensy.subscribeForNotifications(this, characteristic);
}
int BTBLEDevice::unsubscribeFromNotifications(BTBLECharacteristic * characteristic){
    return BTstackTeensy.unsubscribeFromNotifications(this, characteristic);
}
int BTBLEDevice::subscribeForIndications(BTBLECharacteristic * characteristic){
    return BTstackTeensy.subscribeForIndications(this, characteristic);
}
int BTBLEDevice::unsubscribeFromIndications(BTBLECharacteristic * characteristic){
    return BTstackTeensy.unsubscribeFromIndications(this, characteristic);
}



static uint16_t (*gattReadCallback)(uint16_t characteristic_id, uint8_t * buffer, uint16_t buffer_size);
static int (*gattWriteCallback)(uint16_t characteristic_id, uint8_t *buffer, uint16_t buffer_size);

// ATT Client Read Callback for Dynamic Data
static uint16_t att_read_callback(hci_con_handle_t con_handle, uint16_t att_handle, uint16_t offset, uint8_t * buffer, uint16_t buffer_size){
    if (gattReadCallback){
        return gattReadCallback(att_handle, buffer, buffer_size);
    }
    return 0;
}

static int att_write_callback(hci_con_handle_t con_handle, uint16_t att_handle, uint16_t transaction_mode, uint16_t offset, uint8_t *buffer, uint16_t buffer_size){
    // Only forward calls that carry real data. For queued/prepared writes,
    // att_server also calls back with ATT_TRANSACTION_MODE_VALIDATE/EXECUTE/
    // CANCEL to finalize or cancel the transaction -- buffer/buffer_size
    // aren't meaningful there, and this simplified API doesn't expose
    // transaction_mode to the app callback for it to tell the difference.
    if ((transaction_mode != ATT_TRANSACTION_MODE_NONE) && (transaction_mode != ATT_TRANSACTION_MODE_ACTIVE)){
        return 0;
    }
    if (gattWriteCallback && buffer){
        gattWriteCallback(att_handle, buffer, buffer_size);
    }
    return 0;
}

BTstackTeensyManager::BTstackTeensyManager(void){
    have_custom_addr = false;
    classic_sdp_initialized = false;
    rfcomm_initialized = false;

    bleAdvertismentCallback = NULL;
    bleDeviceConnectedCallback = NULL;
    bleDeviceDisconnectedCallback = NULL;
    gattServiceDiscoveredCallback = NULL;
    gattCharacteristicDiscoveredCallback = NULL;
    gattCharacteristicNotificationCallback = NULL;
    a2dpStreamStateCallback = NULL;
    hfpEventCallback = NULL;

    att_db_util_init();

    // disable LOG_INFO messages
    hci_dump_enable_log_level(HCI_DUMP_LOG_LEVEL_INFO, 0);
}

void BTstackTeensyManager::setBLEAdvertisementCallback(void (*callback)(BTBLEAdvertisement * bleAdvertisement)){
    bleAdvertismentCallback = callback;
}
void BTstackTeensyManager::setBLEDeviceConnectedCallback(void (*callback)(BLEStatus status, BTBLEDevice * device)){
    bleDeviceConnectedCallback = callback;
}
void BTstackTeensyManager::setBLEDeviceDisconnectedCallback(void (*callback)(BTBLEDevice * device)){
    bleDeviceDisconnectedCallback = callback;
}
void BTstackTeensyManager::setGATTServiceDiscoveredCallback(void (*callback)(BLEStatus status, BTBLEDevice * device, BTBLEService * bleService)){
    gattServiceDiscoveredCallback = callback;
}
void BTstackTeensyManager::setGATTCharacteristicDiscoveredCallback(void (*callback)(BLEStatus status, BTBLEDevice * device, BTBLECharacteristic * characteristic)){
    gattCharacteristicDiscoveredCallback = callback;
}
void BTstackTeensyManager::setGATTCharacteristicNotificationCallback(void (*callback)(BTBLEDevice * device, uint16_t value_handle, uint8_t* value, uint16_t length)){
    gattCharacteristicNotificationCallback = callback;
}
void BTstackTeensyManager::setGATTCharacteristicIndicationCallback(void (*callback)(BTBLEDevice * device, uint16_t value_handle, uint8_t* value, uint16_t length)){
    gattCharacteristicIndicationCallback = callback;
}
void BTstackTeensyManager::setGATTCharacteristicReadCallback(void (*callback)(BLEStatus status, BTBLEDevice * device, uint8_t * value, uint16_t length)){
    gattCharacteristicReadCallback = callback;
}
void BTstackTeensyManager::setGATTCharacteristicWrittenCallback(void (*callback)(BLEStatus status, BTBLEDevice * device)){
    gattCharacteristicWrittenCallback = callback;
}
void BTstackTeensyManager::setGATTCharacteristicSubscribedCallback(void (*callback)(BLEStatus status, BTBLEDevice * device)){
    gattCharacteristicSubscribedCallback = callback;
}
void BTstackTeensyManager::setGATTCharacteristicUnsubscribedCallback(void (*callback)(BLEStatus status, BTBLEDevice * device)){
    gattCharacteristicUnsubscribedCallback = callback;
}

int BTstackTeensyManager::discoverGATTServices(BTBLEDevice * device){
    gattAction = gattActionServiceQuery;
    return gatt_client_discover_primary_services(gatt_client_callback, device->getHandle());
}
int BTstackTeensyManager::discoverCharacteristicsForService(BTBLEDevice * device, BTBLEService * service){
    gattAction = gattActionCharacteristicQuery;
    return gatt_client_discover_characteristics_for_service(gatt_client_callback, device->getHandle(), (gatt_client_service_t*) service->getService());
}
int  BTstackTeensyManager::readCharacteristic(BTBLEDevice * device, BTBLECharacteristic * characteristic){
    return gatt_client_read_value_of_characteristic(gatt_client_callback, device->getHandle(), (gatt_client_characteristic_t*) characteristic->getCharacteristic());
}
int  BTstackTeensyManager::writeCharacteristic(BTBLEDevice * device, BTBLECharacteristic * characteristic, uint8_t * data, uint16_t size){
    gattAction = gattActionWrite;
    return gatt_client_write_value_of_characteristic(gatt_client_callback, device->getHandle(), characteristic->getCharacteristic()->value_handle,
        size, data);
}
int  BTstackTeensyManager::writeCharacteristicWithoutResponse(BTBLEDevice * device, BTBLECharacteristic * characteristic, uint8_t * data, uint16_t size){
    return gatt_client_write_value_of_characteristic_without_response(device->getHandle(), characteristic->getCharacteristic()->value_handle,
         size, data);
}
int BTstackTeensyManager::subscribeForNotifications(BTBLEDevice * device, BTBLECharacteristic * characteristic){
    gattAction = gattActionSubscribe;
    return gatt_client_write_client_characteristic_configuration(gatt_client_callback, device->getHandle(), (gatt_client_characteristic_t*) characteristic->getCharacteristic(),
     GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION);
}
int BTstackTeensyManager::subscribeForIndications(BTBLEDevice * device, BTBLECharacteristic * characteristic){
    gattAction = gattActionSubscribe;
    return gatt_client_write_client_characteristic_configuration(gatt_client_callback, device->getHandle(), (gatt_client_characteristic_t*) characteristic->getCharacteristic(),
     GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_INDICATION);
}
int BTstackTeensyManager::unsubscribeFromNotifications(BTBLEDevice * device, BTBLECharacteristic * characteristic){
    gattAction = gattActionUnsubscribe;
    return gatt_client_write_client_characteristic_configuration(gatt_client_callback, device->getHandle(), (gatt_client_characteristic_t*) characteristic->getCharacteristic(),
     GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NONE);
}
int BTstackTeensyManager::unsubscribeFromIndications(BTBLEDevice * device, BTBLECharacteristic * characteristic){
    gattAction = gattActionUnsubscribe;
    return gatt_client_write_client_characteristic_configuration(gatt_client_callback, device->getHandle(), (gatt_client_characteristic_t*) characteristic->getCharacteristic(),
     GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NONE);
}
void BTstackTeensyManager::bleConnect(BTBLEAdvertisement * advertisement, int timeout_ms){
    bleConnect(advertisement->getBdAddr(), timeout_ms);
}
void BTstackTeensyManager::bleConnect(BTBD_ADDR * address, int timeout_ms){
    bleConnect(address->getAddressType(), address->getAddress(), timeout_ms);
}
void BTstackTeensyManager::bleConnect(BTBD_ADDR_TYPE address_type, const char * address, int timeout_ms){
    // TODO: implement
}
void BTstackTeensyManager::bleConnect(BTBD_ADDR_TYPE address_type, const uint8_t address[6], int timeout_ms){
    gap_connect((uint8_t*)address, (bd_addr_type_t) address_type);
    if (!timeout_ms) return;
    btstack_run_loop_set_timer(&connection_timer, timeout_ms);
    btstack_run_loop_set_timer_handler(&connection_timer, connection_timeout_handler);
    btstack_run_loop_add_timer(&connection_timer);
}

void BTstackTeensyManager::bleDisconnect(BTBLEDevice * device){
    btstack_run_loop_remove_timer(&connection_timer);
    gap_disconnect(device->getHandle());
}

void BTstackTeensyManager::setPublicBdAddr(bd_addr_t addr){
    have_custom_addr = true;
    memcpy(public_bd_addr, addr ,6);
}

// Call before setup() -- the name is baked into the advertisement data setup()
// builds. Truncated to BTSTACK_TEENSY_MAX_DEVICE_NAME_LEN if longer, since
// advertisement data is capped at 31 bytes total.
void BTstackTeensyManager::setDeviceName(const char * name){
    strncpy(device_name, name, BTSTACK_TEENSY_MAX_DEVICE_NAME_LEN);
    device_name[BTSTACK_TEENSY_MAX_DEVICE_NAME_LEN] = '\0';
}

static void bluetooth_hardware_error(uint8_t error){
    printf("Bluetooth Hardware Error event 0x%02x. Restarting...\n\n\n", error);
    delay(50);
    SCB_AIRCR = 0x05FA0004; // Cortex-M system reset request
    while(1);
}

// Keep in sync with hal_teensy_uart.cpp's BTSTACK_TEENSY_UART_FLOW_CONTROL --
// that's what actually enables/disables the RTS/CTS pins on the LPUART
// peripheral; this just tells BTstack's H4 transport whether to expect it.
#ifndef BTSTACK_TEENSY_UART_FLOW_CONTROL
#define BTSTACK_TEENSY_UART_FLOW_CONTROL 0
#endif

// baudrate_main temporarily 0 (disables the mid-init baud rate switch --
// BTstack stays at 115200 for the whole session) while debugging garbled
// data seen right after switching to 921600. Once that's root-caused, put
// 921600 back to get full firmware-download speed.
static hci_transport_config_uart_t config = {
    HCI_TRANSPORT_CONFIG_UART,
    115200,
    0,
    BTSTACK_TEENSY_UART_FLOW_CONTROL,
    NULL,
};

static btstack_packet_callback_registration_t hci_event_callback_registration;

void BTstackTeensyManager::setup(void){

    setup_stdio_over_usb_serial();

    printf("BTstackTeensyManager::setup()\n");

    btstack_memory_init();
    btstack_run_loop_init(btstack_run_loop_embedded_get_instance());

    const hci_transport_t * transport = hci_transport_h4_instance(btstack_uart_block_embedded_instance());
    hci_init(transport, (void*) &config);
    hci_set_chipset(btstack_chipset_bcm_instance());
    hci_set_control(teensy_bt_control_instance());

    if (have_custom_addr){
        hci_set_bd_addr(public_bd_addr);
    }

    hci_set_hardware_error_callback(&bluetooth_hardware_error);

    // inform about BTstack state
    hci_event_callback_registration.callback = &packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);

    l2cap_init();
    sdp_init();
    classic_sdp_initialized = true;

    // setup central device db
    le_device_db_init();

    sm_init();

    att_server_init(att_db_util_get_address(),att_read_callback, att_write_callback);

    gatt_client_init();

    // setup advertisements params
    uint16_t adv_int_min = 0x0030;
    uint16_t adv_int_max = 0x0030;
    uint8_t adv_type = 0;
    bd_addr_t null_addr;
    memset(null_addr, 0, 6);
    gap_advertisements_set_params(adv_int_min, adv_int_max, adv_type, 0, null_addr, 0x07, 0x00);

    // setup advertisements data
    int pos = 0;
    const uint8_t flags[] = { 0x02, 0x01, 0x02 };
    memcpy(&adv_data[pos], flags, sizeof(flags));
    pos += sizeof(flags);
    size_t name_len = strlen(device_name);
    adv_data[pos++] = (uint8_t) (name_len + 1);
    adv_data[pos++] = 0x09;
    memcpy(&adv_data[pos], device_name, name_len);
    pos += name_len;
    adv_data_len = pos;
    gap_advertisements_set_data(adv_data_len, adv_data);

    // turn on!
    printf("cyw43439: powering on module and starting HCI bring-up...\n");
    btstack_state = 0;
    hci_power_control(HCI_POWER_ON);
}

void BTstackTeensyManager::enablePacketLogger(void){
    hci_dump_init(hci_dump_embedded_stdout_get_instance());
}

void BTstackTeensyManager::enableDebugLogger(){
    // enable LOG_INFO messages
    hci_dump_enable_log_level(HCI_DUMP_LOG_LEVEL_INFO, 1);
}


void BTstackTeensyManager::loop(void){
    // process data from/to the module's HCI UART
    hal_teensy_uart_poll();
    // BTstack Run Loop
    btstack_run_loop_embedded_execute_once();
}

void BTstackTeensyManager::bleStartScanning(void){
    printf("Start scanning\n");
    gap_start_scan();
}
void BTstackTeensyManager::bleStopScanning(void){
    gap_stop_scan();
}

void BTstackTeensyManager::setGATTCharacteristicRead(uint16_t (*cb)(uint16_t characteristic_id, uint8_t * buffer, uint16_t buffer_size)){
    gattReadCallback = cb;
}
void BTstackTeensyManager::setGATTCharacteristicWrite(int (*cb)(uint16_t characteristic_id, uint8_t *buffer, uint16_t buffer_size)){
    gattWriteCallback = cb;
}
void BTstackTeensyManager::addGATTService(BTUUID * uuid){
    att_db_util_add_service_uuid128((uint8_t*)uuid->getUuid());
}
uint16_t BTstackTeensyManager::addGATTCharacteristic(BTUUID * uuid, uint16_t flags, const char * text){
    return att_db_util_add_characteristic_uuid128((uint8_t*)uuid->getUuid(), flags, ATT_SECURITY_NONE, ATT_SECURITY_NONE, (uint8_t*)text, strlen(text));
}
uint16_t BTstackTeensyManager::addGATTCharacteristic(BTUUID * uuid, uint16_t flags, uint8_t * data, uint16_t data_len){
    return att_db_util_add_characteristic_uuid128((uint8_t*)uuid->getUuid(), flags, ATT_SECURITY_NONE, ATT_SECURITY_NONE, data, data_len);
}
uint16_t BTstackTeensyManager::addGATTCharacteristicDynamic(BTUUID * uuid, uint16_t flags, uint16_t characteristic_id){
    return att_db_util_add_characteristic_uuid128((uint8_t*)uuid->getUuid(), flags | ATT_PROPERTY_DYNAMIC, ATT_SECURITY_NONE, ATT_SECURITY_NONE, NULL, 0);
}
void BTstackTeensyManager::setAdvData(uint16_t adv_data_len, const uint8_t * adv_data){
    gap_advertisements_set_data(adv_data_len, (uint8_t*) adv_data);
}
void BTstackTeensyManager::startAdvertising(){
    gap_advertisements_enable(1);
}
void BTstackTeensyManager::stopAdvertising(){
    gap_advertisements_enable(0);
}
void BTstackTeensyManager::iBeaconConfigure(BTUUID * uuid, uint16_t major_id, uint16_t minor_id, uint8_t measured_power){
    memcpy(adv_data, iBeaconAdvertisement01,  sizeof(iBeaconAdvertisement01));
    adv_data[2] = 0x06;
    memcpy(&adv_data[3], iBeaconAdvertisement38, sizeof(iBeaconAdvertisement38));
    memcpy(&adv_data[9], uuid->getUuid(), 16);
    big_endian_store_16(adv_data, 25, major_id);
    big_endian_store_16(adv_data, 27, minor_id);
    adv_data[29] = measured_power;
    adv_data_len = 30;
    gap_advertisements_set_data(adv_data_len, adv_data);
}

// ---------------------------------------------------------------------
// Classic: shared discoverability

void BTstackTeensyManager::enableClassicDiscoverable(const char * local_name, uint32_t class_of_device){
    gap_set_local_name(local_name);
    gap_discoverable_control(1);
    gap_connectable_control(1);
    gap_set_class_of_device(class_of_device);
    gap_set_default_link_policy_settings(LM_LINK_POLICY_ENABLE_ROLE_SWITCH | LM_LINK_POLICY_ENABLE_SNIFF_MODE);
    gap_set_allow_role_switch(true);
}

// ---------------------------------------------------------------------
// Classic: A2DP Sink + AVRCP (adapted from example/a2dp_sink_demo.c)

#define A2DP_NUM_CHANNELS 2
#define A2DP_BYTES_PER_FRAME (2 * A2DP_NUM_CHANNELS)
#define A2DP_MAX_SBC_FRAME_SIZE 120
#define A2DP_SBC_MAX_AUDIO_FRAMES_PER_BLOCK 128
#define A2DP_RESAMPLE_ADDITIONAL_FRAMES 16
#define A2DP_RESAMPLE_OUTPUT_FRAMES (A2DP_SBC_MAX_AUDIO_FRAMES_PER_BLOCK + A2DP_RESAMPLE_ADDITIONAL_FRAMES)
#define A2DP_OPTIMAL_FRAMES_MIN 60
#define A2DP_OPTIMAL_FRAMES_MAX 80
#define A2DP_ADDITIONAL_FRAMES 30

static uint8_t sdp_avdtp_sink_service_buffer[150];
static uint8_t sdp_avrcp_target_service_buffer[150];
static uint8_t sdp_avrcp_controller_service_buffer[200];
static uint8_t device_id_sdp_service_buffer[100];

// Accept every SBC configuration (bitpool 2-53); negotiate down as needed.
static uint8_t a2dp_media_sbc_codec_capabilities[] = {0xFF, 0xFF, 2, 53};

typedef struct {
    uint8_t reconfigure;
    uint8_t num_channels;
    uint16_t sampling_frequency;
    uint8_t block_length;
    uint8_t subbands;
    uint8_t min_bitpool_value;
    uint8_t max_bitpool_value;
    btstack_sbc_channel_mode_t channel_mode;
    btstack_sbc_allocation_method_t allocation_method;
} a2dp_sbc_configuration_t;

static uint8_t a2dp_local_seid;
static uint8_t a2dp_media_sbc_codec_configuration[4];
static a2dp_sbc_configuration_t a2dp_sbc_configuration;

static uint8_t a2dp_sbc_frame_storage[(A2DP_OPTIMAL_FRAMES_MAX + A2DP_ADDITIONAL_FRAMES) * A2DP_MAX_SBC_FRAME_SIZE];
static btstack_ring_buffer_t a2dp_sbc_frame_ring_buffer;
static unsigned int a2dp_sbc_frame_size;

static uint8_t a2dp_decoded_audio_storage[A2DP_RESAMPLE_OUTPUT_FRAMES * A2DP_BYTES_PER_FRAME];
static btstack_ring_buffer_t a2dp_decoded_audio_ring_buffer;

static int a2dp_media_initialized = 0;
static int a2dp_audio_stream_started = 0;
static btstack_resample_t a2dp_resample_instance;
static uint32_t a2dp_resampling_min_factor;

static int16_t * a2dp_request_buffer;
static int a2dp_request_frames;

static const btstack_sbc_decoder_t * a2dp_sbc_decoder_instance;
static btstack_sbc_decoder_bluedroid_t a2dp_sbc_decoder_context;

static void a2dp_playback_handler(int16_t *buffer, uint16_t num_audio_frames, const btstack_audio_context_t *context) {
    (void)context;

    uint32_t bytes_read;
    btstack_ring_buffer_read(&a2dp_decoded_audio_ring_buffer, (uint8_t *)buffer, num_audio_frames * A2DP_BYTES_PER_FRAME, &bytes_read);
    buffer += bytes_read / A2DP_NUM_CHANNELS;
    num_audio_frames -= bytes_read / A2DP_BYTES_PER_FRAME;

    a2dp_request_buffer = buffer;
    a2dp_request_frames = num_audio_frames;
    while (a2dp_request_frames > 0 && btstack_ring_buffer_bytes_available(&a2dp_sbc_frame_ring_buffer) >= a2dp_sbc_frame_size) {
        uint8_t sbc_frame[A2DP_MAX_SBC_FRAME_SIZE];
        btstack_ring_buffer_read(&a2dp_sbc_frame_ring_buffer, sbc_frame, a2dp_sbc_frame_size, &bytes_read);
        a2dp_sbc_decoder_instance->decode_signed_16(&a2dp_sbc_decoder_context, 0, sbc_frame, a2dp_sbc_frame_size);
    }

    // silence anything we couldn't fill (e.g. right after stream start)
    if (a2dp_request_frames > 0) {
        memset(a2dp_request_buffer, 0, a2dp_request_frames * A2DP_BYTES_PER_FRAME);
    }
}

static void a2dp_handle_pcm_data(int16_t *data, int num_audio_frames, int num_channels, int sample_rate, void *context) {
    (void)sample_rate;
    (void)context;
    (void)num_channels; // bluedroid decoder always emits stereo here

    const btstack_audio_sink_t *audio_sink = btstack_audio_sink_get_instance();
    if (audio_sink == NULL) {
        return;
    }

    int16_t output_buffer[A2DP_RESAMPLE_OUTPUT_FRAMES * A2DP_NUM_CHANNELS];
    uint32_t resampled_frames = btstack_resample_block(&a2dp_resample_instance, data, num_audio_frames, output_buffer);

    int frames_to_copy = btstack_min((int)resampled_frames, a2dp_request_frames);
    memcpy(a2dp_request_buffer, output_buffer, frames_to_copy * A2DP_BYTES_PER_FRAME);
    a2dp_request_frames -= frames_to_copy;
    a2dp_request_buffer += frames_to_copy * A2DP_NUM_CHANNELS;

    int frames_to_store = resampled_frames - frames_to_copy;
    if (frames_to_store > 0) {
        btstack_ring_buffer_write(&a2dp_decoded_audio_ring_buffer, (uint8_t *)&output_buffer[frames_to_copy * A2DP_NUM_CHANNELS], frames_to_store * A2DP_BYTES_PER_FRAME);
    }
}

static void a2dp_media_processing_init(a2dp_sbc_configuration_t *configuration) {
    if (a2dp_media_initialized) {
        return;
    }
    a2dp_sbc_decoder_instance = btstack_sbc_decoder_bluedroid_init_instance(&a2dp_sbc_decoder_context);
    a2dp_sbc_decoder_instance->configure(&a2dp_sbc_decoder_context, SBC_MODE_STANDARD, a2dp_handle_pcm_data, NULL);

    btstack_ring_buffer_init(&a2dp_sbc_frame_ring_buffer, a2dp_sbc_frame_storage, sizeof(a2dp_sbc_frame_storage));
    btstack_ring_buffer_init(&a2dp_decoded_audio_ring_buffer, a2dp_decoded_audio_storage, sizeof(a2dp_decoded_audio_storage));
    btstack_resample_init(&a2dp_resample_instance, configuration->num_channels);
    a2dp_resampling_min_factor = btstack_resample_get_min_factor_for_output_capacity(A2DP_SBC_MAX_AUDIO_FRAMES_PER_BLOCK, A2DP_RESAMPLE_OUTPUT_FRAMES);

    const btstack_audio_sink_t *audio = btstack_audio_sink_get_instance();
    if (audio != NULL) {
        audio->init(A2DP_NUM_CHANNELS, configuration->sampling_frequency, &a2dp_playback_handler);
    }

    a2dp_audio_stream_started = 0;
    a2dp_media_initialized = 1;
}

static void a2dp_media_processing_start(void) {
    if (!a2dp_media_initialized) {
        return;
    }
    const btstack_audio_sink_t *audio = btstack_audio_sink_get_instance();
    if (audio != NULL) {
        audio->start_stream();
    }
    a2dp_audio_stream_started = 1;
}

static void a2dp_media_processing_pause(void) {
    if (!a2dp_media_initialized) {
        return;
    }
    a2dp_audio_stream_started = 0;
    const btstack_audio_sink_t *audio = btstack_audio_sink_get_instance();
    if (audio != NULL) {
        audio->stop_stream();
    }
    btstack_ring_buffer_reset(&a2dp_decoded_audio_ring_buffer);
    btstack_ring_buffer_reset(&a2dp_sbc_frame_ring_buffer);
}

static void a2dp_media_processing_close(void) {
    if (!a2dp_media_initialized) {
        return;
    }
    a2dp_media_initialized = 0;
    a2dp_audio_stream_started = 0;
    a2dp_sbc_frame_size = 0;
    const btstack_audio_sink_t *audio = btstack_audio_sink_get_instance();
    if (audio != NULL) {
        audio->close();
    }
}

static int a2dp_read_media_data_header(uint8_t *packet, int size, int *offset, avdtp_media_packet_header_t *media_header) {
    int packet_size = size;
    if (packet_size - *offset < 12) {
        return 0;
    }
    media_header->version = packet[*offset] & 0x03;
    media_header->padding = get_bit16(packet[*offset], 2);
    media_header->extension = get_bit16(packet[*offset], 3);
    media_header->csrc_count = (packet[*offset] >> 4) & 0x0F;
    (*offset)++;
    media_header->marker = get_bit16(packet[*offset], 0);
    media_header->payload_type = (packet[*offset] >> 1) & 0x7F;
    (*offset)++;
    media_header->sequence_number = big_endian_read_16(packet, *offset);
    (*offset) += 2;
    media_header->timestamp = big_endian_read_32(packet, *offset);
    (*offset) += 4;
    media_header->synchronization_source = big_endian_read_32(packet, *offset);
    (*offset) += 4;
    return 1;
}

static int a2dp_read_sbc_header(uint8_t *packet, int size, int *offset, avdtp_sbc_codec_header_t *sbc_header) {
    int sbc_header_len = 12; // without crc
    int pos = *offset;
    if (size - pos < sbc_header_len) {
        return 0;
    }
    sbc_header->fragmentation = get_bit16(packet[pos], 7);
    sbc_header->starting_packet = get_bit16(packet[pos], 6);
    sbc_header->last_packet = get_bit16(packet[pos], 5);
    sbc_header->num_frames = packet[pos] & 0x0F;
    pos++;
    *offset = pos;
    return 1;
}

static void a2dp_handle_l2cap_media_data_packet(uint8_t seid, uint8_t *packet, uint16_t size) {
    (void)seid;
    int pos = 0;

    avdtp_media_packet_header_t media_header;
    if (!a2dp_read_media_data_header(packet, size, &pos, &media_header)) {
        return;
    }

    avdtp_sbc_codec_header_t sbc_header;
    if (!a2dp_read_sbc_header(packet, size, &pos, &sbc_header)) {
        return;
    }

    int packet_length = size - pos;
    uint8_t *packet_begin = packet + pos;
    if (sbc_header.num_frames == 0) {
        return;
    }

    const btstack_audio_sink_t *audio = btstack_audio_sink_get_instance();
    if (audio == NULL) {
        a2dp_sbc_decoder_instance->decode_signed_16(&a2dp_sbc_decoder_context, 0, packet_begin, packet_length);
        return;
    }

    a2dp_sbc_frame_size = packet_length / sbc_header.num_frames;
    btstack_ring_buffer_write(&a2dp_sbc_frame_ring_buffer, packet_begin, packet_length);

    int sbc_frames_in_buffer = btstack_ring_buffer_bytes_available(&a2dp_sbc_frame_ring_buffer) / a2dp_sbc_frame_size;

    // buffer-fill based drift compensation (same heuristic as a2dp_sink_demo.c)
    uint32_t nominal_factor = 0x10000;
    uint32_t compensation = 0x00100;
    uint32_t resampling_factor;
    if (sbc_frames_in_buffer < A2DP_OPTIMAL_FRAMES_MIN) {
        resampling_factor = nominal_factor - compensation;
    } else if (sbc_frames_in_buffer <= A2DP_OPTIMAL_FRAMES_MAX) {
        resampling_factor = nominal_factor;
    } else {
        resampling_factor = nominal_factor + compensation;
    }
    if (resampling_factor < a2dp_resampling_min_factor) {
        resampling_factor = a2dp_resampling_min_factor;
    }
    btstack_resample_set_factor(&a2dp_resample_instance, resampling_factor);

    if (!a2dp_audio_stream_started && sbc_frames_in_buffer >= A2DP_OPTIMAL_FRAMES_MIN) {
        a2dp_media_processing_start();
    }
}

static void a2dp_sink_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t event_size) {
    (void)channel;
    (void)event_size;
    if (packet_type != HCI_EVENT_PACKET) {
        return;
    }
    if (hci_event_packet_get_type(packet) != HCI_EVENT_A2DP_META) {
        return;
    }

    switch (hci_event_a2dp_meta_get_subevent_code(packet)) {
        case A2DP_SUBEVENT_SIGNALING_MEDIA_CODEC_SBC_CONFIGURATION: {
            a2dp_sbc_configuration_t *c = &a2dp_sbc_configuration;
            c->reconfigure = a2dp_subevent_signaling_media_codec_sbc_configuration_get_reconfigure(packet);
            c->num_channels = a2dp_subevent_signaling_media_codec_sbc_configuration_get_num_channels(packet);
            c->sampling_frequency = a2dp_subevent_signaling_media_codec_sbc_configuration_get_sampling_frequency(packet);
            c->block_length = a2dp_subevent_signaling_media_codec_sbc_configuration_get_block_length(packet);
            c->subbands = a2dp_subevent_signaling_media_codec_sbc_configuration_get_subbands(packet);
            c->min_bitpool_value = a2dp_subevent_signaling_media_codec_sbc_configuration_get_min_bitpool_value(packet);
            c->max_bitpool_value = a2dp_subevent_signaling_media_codec_sbc_configuration_get_max_bitpool_value(packet);
            c->channel_mode = (btstack_sbc_channel_mode_t)a2dp_subevent_signaling_media_codec_sbc_configuration_get_channel_mode(packet);
            c->allocation_method = (btstack_sbc_allocation_method_t)a2dp_subevent_signaling_media_codec_sbc_configuration_get_allocation_method(packet);
            printf("A2DP Sink: SBC %u Hz, %u ch, blocks %u, subbands %u, bitpool %u-%u\n",
                   c->sampling_frequency, c->num_channels, c->block_length, c->subbands, c->min_bitpool_value, c->max_bitpool_value);
            a2dp_media_processing_init(c);
            break;
        }
        case A2DP_SUBEVENT_STREAM_ESTABLISHED:
            if (a2dpStreamStateCallback) a2dpStreamStateCallback(A2DP_STREAM_ESTABLISHED);
            break;
        case A2DP_SUBEVENT_STREAM_STARTED:
            a2dp_audio_stream_started = 0; // wait for jitter buffer to fill, see a2dp_handle_l2cap_media_data_packet
            if (a2dpStreamStateCallback) a2dpStreamStateCallback(A2DP_STREAM_STARTED);
            break;
        case A2DP_SUBEVENT_STREAM_SUSPENDED:
            a2dp_media_processing_pause();
            if (a2dpStreamStateCallback) a2dpStreamStateCallback(A2DP_STREAM_SUSPENDED);
            break;
        case A2DP_SUBEVENT_STREAM_RELEASED:
            a2dp_media_processing_close();
            if (a2dpStreamStateCallback) a2dpStreamStateCallback(A2DP_STREAM_RELEASED);
            break;
        case A2DP_SUBEVENT_SIGNALING_CONNECTION_RELEASED:
            a2dp_media_processing_close();
            break;
        default:
            break;
    }
}

static void avrcp_target_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    (void)packet_type;
    (void)channel;
    (void)packet;
    (void)size;
    // Volume-from-phone and similar Category 2 commands land here; left as a
    // hook to wire to a physical volume control if you add one.
}

void BTstackTeensyManager::enableA2DPSink(void){
    a2dp_sink_init();
    avrcp_init();
    avrcp_controller_init();
    avrcp_target_init();

    a2dp_sink_register_packet_handler(&a2dp_sink_packet_handler);
    a2dp_sink_register_media_handler(&a2dp_handle_l2cap_media_data_packet);
    avdtp_stream_endpoint_t *local_stream_endpoint = a2dp_sink_create_stream_endpoint(
        AVDTP_AUDIO, AVDTP_CODEC_SBC, a2dp_media_sbc_codec_capabilities, sizeof(a2dp_media_sbc_codec_capabilities),
        a2dp_media_sbc_codec_configuration, sizeof(a2dp_media_sbc_codec_configuration));
    btstack_assert(local_stream_endpoint != NULL);
    a2dp_local_seid = avdtp_local_seid(local_stream_endpoint);

    avrcp_target_register_packet_handler(&avrcp_target_packet_handler);

    memset(sdp_avdtp_sink_service_buffer, 0, sizeof(sdp_avdtp_sink_service_buffer));
    a2dp_sink_create_sdp_record(sdp_avdtp_sink_service_buffer, sdp_create_service_record_handle(),
                                 AVDTP_SINK_FEATURE_MASK_HEADPHONE, NULL, NULL);
    sdp_register_service(sdp_avdtp_sink_service_buffer);

    memset(sdp_avrcp_controller_service_buffer, 0, sizeof(sdp_avrcp_controller_service_buffer));
    uint16_t controller_supported_features = 1 << AVRCP_CONTROLLER_SUPPORTED_FEATURE_CATEGORY_PLAYER_OR_RECORDER;
    avrcp_controller_create_sdp_record(sdp_avrcp_controller_service_buffer, sdp_create_service_record_handle(),
                                        controller_supported_features, NULL, NULL);
    sdp_register_service(sdp_avrcp_controller_service_buffer);

    memset(sdp_avrcp_target_service_buffer, 0, sizeof(sdp_avrcp_target_service_buffer));
    uint16_t target_supported_features = 1 << AVRCP_TARGET_SUPPORTED_FEATURE_CATEGORY_MONITOR_OR_AMPLIFIER;
    avrcp_target_create_sdp_record(sdp_avrcp_target_service_buffer, sdp_create_service_record_handle(),
                                    target_supported_features, NULL, NULL);
    sdp_register_service(sdp_avrcp_target_service_buffer);

    memset(device_id_sdp_service_buffer, 0, sizeof(device_id_sdp_service_buffer));
    device_id_create_sdp_record(device_id_sdp_service_buffer, sdp_create_service_record_handle(),
                                 DEVICE_ID_VENDOR_ID_SOURCE_BLUETOOTH, BLUETOOTH_COMPANY_ID_BLUEKITCHEN_GMBH, 1, 1);
    sdp_register_service(device_id_sdp_service_buffer);
}

void BTstackTeensyManager::setA2DPStreamStateCallback(void (*callback)(A2DPStreamState state)){
    a2dpStreamStateCallback = callback;
}

// ---------------------------------------------------------------------
// Classic: HFP Hands-Free (adapted from example/hfp_hf_demo.c) -- signaling
// only, audio is bridged directly between the module and Teensy over
// BT_PCM_*, not through this API (see hal_teensy_pcm.cpp).

static uint8_t hfp_service_buffer[200];
static const char hfp_hf_service_name[] = "Teensy Hands-Free";
static const int hfp_rfcomm_channel_nr = 1;
static const uint16_t hfp_indicators[1] = {0x01};
static const uint8_t hfp_codecs[] = {HFP_CODEC_CVSD, HFP_CODEC_MSBC};

static hci_con_handle_t hfp_acl_handle = HCI_CON_HANDLE_INVALID;
static hci_con_handle_t hfp_sco_handle = HCI_CON_HANDLE_INVALID;

static void hfp_hf_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *event, uint16_t event_size) {
    (void)channel;
    (void)event_size;
    if (packet_type != HCI_EVENT_PACKET) {
        return;
    }
    if (hci_event_packet_get_type(event) != HCI_EVENT_HFP_META) {
        return;
    }

    uint8_t status;
    switch (hci_event_hfp_meta_get_subevent_code(event)) {
        case HFP_SUBEVENT_SERVICE_LEVEL_CONNECTION_ESTABLISHED:
            status = hfp_subevent_service_level_connection_established_get_status(event);
            if (status != ERROR_CODE_SUCCESS) {
                break;
            }
            hfp_acl_handle = hfp_subevent_service_level_connection_established_get_acl_handle(event);
            if (hfpEventCallback) hfpEventCallback(HFP_SLC_CONNECTED, hfp_acl_handle);
            break;
        case HFP_SUBEVENT_SERVICE_LEVEL_CONNECTION_RELEASED:
            if (hfpEventCallback) hfpEventCallback(HFP_SLC_DISCONNECTED, hfp_acl_handle);
            hfp_acl_handle = HCI_CON_HANDLE_INVALID;
            break;
        case HFP_SUBEVENT_AUDIO_CONNECTION_ESTABLISHED: {
            status = hfp_subevent_audio_connection_established_get_status(event);
            if (status != ERROR_CODE_SUCCESS) {
                break;
            }
            hfp_sco_handle = hfp_subevent_audio_connection_established_get_sco_handle(event);
            uint8_t negotiated_codec = hfp_subevent_audio_connection_established_get_negotiated_codec(event);
            pcm_bridge_start(negotiated_codec == HFP_CODEC_MSBC ? 1 : 0);
            if (hfpEventCallback) hfpEventCallback(HFP_AUDIO_CONNECTED, hfp_sco_handle);
            break;
        }
        case HFP_SUBEVENT_AUDIO_CONNECTION_RELEASED:
            pcm_bridge_stop();
            if (hfpEventCallback) hfpEventCallback(HFP_AUDIO_DISCONNECTED, hfp_sco_handle);
            hfp_sco_handle = HCI_CON_HANDLE_INVALID;
            break;
        case HFP_SUBEVENT_CALL_ANSWERED:
            if (hfpEventCallback) hfpEventCallback(HFP_CALL_ANSWERED, hfp_acl_handle);
            break;
        case HFP_SUBEVENT_CALL_TERMINATED:
            if (hfpEventCallback) hfpEventCallback(HFP_CALL_TERMINATED, hfp_acl_handle);
            break;
        case HFP_SUBEVENT_START_RINGING:
            if (hfpEventCallback) hfpEventCallback(HFP_START_RINGING, hfp_acl_handle);
            break;
        case HFP_SUBEVENT_STOP_RINGING:
            if (hfpEventCallback) hfpEventCallback(HFP_STOP_RINGING, hfp_acl_handle);
            break;
        default:
            break;
    }
}

void BTstackTeensyManager::enableHFPHandsFree(void){
    if (!rfcomm_initialized){
        rfcomm_init();
        rfcomm_initialized = true;
    }

    uint16_t hf_supported_features = (1 << HFP_HFSF_ESCO_S4) | (1 << HFP_HFSF_CLI_PRESENTATION_CAPABILITY) |
                                      (1 << HFP_HFSF_CODEC_NEGOTIATION) | (1 << HFP_HFSF_ENHANCED_CALL_STATUS) |
                                      (1 << HFP_HFSF_EC_NR_FUNCTION) | (1 << HFP_HFSF_REMOTE_VOLUME_CONTROL);

    hfp_hf_init(hfp_rfcomm_channel_nr);
    hfp_hf_init_supported_features(hf_supported_features);
    hfp_hf_init_hf_indicators(sizeof(hfp_indicators) / sizeof(uint16_t), hfp_indicators);
    hfp_hf_init_codecs(sizeof(hfp_codecs), hfp_codecs);
    hfp_hf_register_packet_handler(&hfp_hf_packet_handler);

    memset(hfp_service_buffer, 0, sizeof(hfp_service_buffer));
    hfp_hf_create_sdp_record_with_codecs(hfp_service_buffer, sdp_create_service_record_handle(), hfp_rfcomm_channel_nr,
                                          hfp_hf_service_name, hf_supported_features, sizeof(hfp_codecs), hfp_codecs);
    sdp_register_service(hfp_service_buffer);
}

void BTstackTeensyManager::setHFPEventCallback(void (*callback)(HFPEvent event, hci_con_handle_t handle)){
    hfpEventCallback = callback;
}

void BTstackTeensyManager::hfpAnswer(void){
    hfp_hf_answer_incoming_call(hfp_acl_handle);
}

void BTstackTeensyManager::hfpHangup(void){
    if (hfp_sco_handle != HCI_CON_HANDLE_INVALID) {
        hfp_hf_terminate_call(hfp_acl_handle);
    } else {
        hfp_hf_reject_incoming_call(hfp_acl_handle);
    }
}

void BTstackTeensyManager::hfpDial(const char * number){
    hfp_hf_dial_number(hfp_acl_handle, (char *) number);
}


BTstackTeensyManager BTstackTeensy;
