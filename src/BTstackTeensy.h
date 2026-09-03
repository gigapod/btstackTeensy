/**
 * Arduino Wrapper for BTstack -- Teensy 4.x + Murata Type 1YN (CYW43439) port
 *
 * This mirrors the API of BlueKitchen's original Arduino port (BTstack.h/
 * BTstackManager, see port/arduino) for BLE GAP/GATT central & peripheral
 * use, and adds a small set of methods for Classic Bluetooth (A2DP Sink +
 * AVRCP, HFP Hands-Free) on top of it -- see README.md for wiring and
 * feature details.
 *
 * The class/global names here are deliberately different from the original
 * port's BTstackManager/BTstack (this header/lib is BTstackTeensyManager/
 * BTstackTeensy) so both libraries can be installed side by side without a
 * symbol clash.
 */
#ifndef __ARDUINO_TEENSY_BTSTACK_H
#define __ARDUINO_TEENSY_BTSTACK_H

#if defined __cplusplus
extern "C" {
#endif

#include "ble/att_db.h"
#include "btstack_util.h"
#include "ble/gatt_client.h"
#include "hci.h"
#include <stdint.h>

typedef enum BLEStatus {
    BLE_STATUS_OK,
    BLE_STATUS_DONE,    // e.g. for service or characteristic discovery done
    BLE_STATUS_CONNECTION_TIMEOUT,
    BLE_STATUS_CONNECTION_ERROR,
    BLE_STATUS_OTHER_ERROR
} BLEStatus;

typedef void (*btstack_packet_handler_t) (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);

class BTUUID {
private:
    uint8_t uuid[16];
public:
    BTUUID();
    BTUUID(const uint8_t uuid[16]);
    BTUUID(const char * uuidStr);
    const char * getUuidString()    const;
    const char * getUuid128String() const;
    const uint8_t * getUuid(void)   const;
    bool matches(BTUUID *uuid)        const;
};

typedef enum BTBD_ADDR_TYPE {
    PUBLIC_ADDRESS = 0,
    PRIVAT_ADDRESS
} BTBD_ADDR_TYPE;

class BTBD_ADDR {
private:
    uint8_t address[6];
    BTBD_ADDR_TYPE address_type;
public:
    BTBD_ADDR();
    BTBD_ADDR(const char * address_string, BTBD_ADDR_TYPE address_type = PUBLIC_ADDRESS);
    BTBD_ADDR(const uint8_t address[6], BTBD_ADDR_TYPE address_type = PUBLIC_ADDRESS);
    const uint8_t * getAddress();
    const char * getAddressString();
    BTBD_ADDR_TYPE getAddressType();
};

class BTBLEAdvertisement {
private:
    uint8_t advertising_event_type;
    uint8_t rssi;
    uint8_t data_length;
    uint8_t data[10 + LE_ADVERTISING_DATA_SIZE];
    BTBD_ADDR  bd_addr;
    BTUUID * iBeacon_UUID;
    char name_buffer[LE_ADVERTISING_DATA_SIZE + 1];
public:
    BTBLEAdvertisement(uint8_t * event_packet);
    ~BTBLEAdvertisement();
    BTBD_ADDR * getBdAddr();
    BTBD_ADDR_TYPE getBdAddrType();
    int getRssi();
    bool containsService(BTUUID * service);
    bool nameHasPrefix(const char * namePrefix);
    // Complete or shortened local name from the advertisement, or NULL if
    // neither AD type is present.
    const char * getName();
    const uint8_t * getAdvData();
    bool isIBeacon();
    const BTUUID * getIBeaconUUID();
    uint16_t     getIBeaconMajorID();
    uint16_t     getIBecaonMinorID();
    uint8_t      getiBeaconMeasuredPower();
};

class BTBLECharacteristic {
private:
    gatt_client_characteristic_t characteristic;
    BTUUID uuid;
public:
    BTBLECharacteristic();
    BTBLECharacteristic(gatt_client_characteristic_t characteristic);
    const BTUUID * getUUID();
    bool matches(BTUUID * uuid);
    bool isValueHandle(uint16_t value_handle);
    const gatt_client_characteristic_t * getCharacteristic();
};

class BTBLEService {
private:
    gatt_client_service_t service;
    BTUUID uuid;
public:
    BTBLEService();
    BTBLEService(gatt_client_service_t service);
    const BTUUID * getUUID();
    bool matches(BTUUID * uuid);
    const gatt_client_service_t * getService();
};

class BTBLEDevice {
private:
    hci_con_handle_t handle;
public:
    BTBLEDevice();
    BTBLEDevice(hci_con_handle_t handle);
    hci_con_handle_t getHandle();

    // discovery of services and characteristics
    int discoverGATTServices();
    int discoverCharacteristicsForService(BTBLEService * service);

    // read/write
    int  readCharacteristic(BTBLECharacteristic * characteristic);
    int  writeCharacteristic(BTBLECharacteristic * characteristic, uint8_t * data, uint16_t size);
    int  writeCharacteristicWithoutResponse(BTBLECharacteristic * characteristic, uint8_t * data, uint16_t size);

    // subscribe/unsubscribe
    int subscribeForNotifications(BTBLECharacteristic * characteristic);
    int unsubscribeFromNotifications(BTBLECharacteristic * characteristic);
    int subscribeForIndications(BTBLECharacteristic * characteristic);
    int unsubscribeFromIndications(BTBLECharacteristic * characteristic);
};

// Classic (A2DP Sink / AVRCP) stream lifecycle, mirrors example/a2dp_sink_demo.c's
// A2DP_SUBEVENT_STREAM_* handling.
typedef enum A2DPStreamState {
    A2DP_STREAM_ESTABLISHED,
    A2DP_STREAM_STARTED,
    A2DP_STREAM_SUSPENDED,
    A2DP_STREAM_RELEASED
} A2DPStreamState;

// Classic (HFP Hands-Free) call/audio lifecycle, mirrors example/hfp_hf_demo.c's
// HFP_SUBEVENT_* handling. Audio itself is not delivered through this callback --
// it's routed directly between the module and the Teensy over the PCM/I2S2
// pins (see hal_teensy_pcm.h) once HFP_AUDIO_CONNECTED fires.
typedef enum HFPEvent {
    HFP_SLC_CONNECTED,
    HFP_SLC_DISCONNECTED,
    HFP_AUDIO_CONNECTED,
    HFP_AUDIO_DISCONNECTED,
    HFP_START_RINGING,
    HFP_STOP_RINGING,
    HFP_CALL_ANSWERED,
    HFP_CALL_TERMINATED
} HFPEvent;

class BTstackTeensyManager {
public:
    BTstackTeensyManager(void);

    // Bring up the module (BT_ON, HCI transport/chipset, run loop, L2CAP,
    // SDP, SM, ATT server, GATT client) and process events. Call setup()
    // once from the sketch's setup(), and loop() from every iteration of
    // the sketch's loop().
    void setup(void);
    void loop(void);

    void setPublicBdAddr(bd_addr_t addr);
    // Call before setup() -- sets the name advertised in the GAP "Complete
    // Local Name" field (defaults to "BTstack Teensy" if never called).
    void setDeviceName(const char * name);
    void enablePacketLogger();
    void enableDebugLogger();

    // ---- BLE (GAP/GATT) -------------------------------------------------

    void setAdvData(uint16_t size, const uint8_t * data);
    void iBeaconConfigure(BTUUID * uuid, uint16_t major_id, uint16_t minor_id, uint8_t measured_power = 0xc6);
    void startAdvertising();
    void stopAdvertising();

    void bleStartScanning();
    void bleStopScanning();

    // connection management
    void bleConnect(BTBD_ADDR_TYPE address_type, const uint8_t address[6], int timeout_ms);
    void bleConnect(BTBD_ADDR_TYPE address_type, const char * address, int timeout_ms);
    void bleConnect(BTBD_ADDR * address, int timeout_ms);
    void bleConnect(BTBLEAdvertisement * advertisement, int timeout_ms);
    void bleDisconnect(BTBLEDevice * device);

    // discovery of services and characteristics
    int discoverGATTServices(BTBLEDevice * device);
    int discoverCharacteristicsForService(BTBLEDevice * peripheral, BTBLEService * service);

    // read/write
    int  readCharacteristic(BTBLEDevice * device, BTBLECharacteristic * characteristic);
    int  writeCharacteristic(BTBLEDevice * device, BTBLECharacteristic * characteristic, uint8_t * data, uint16_t size);
    int  writeCharacteristicWithoutResponse(BTBLEDevice * device, BTBLECharacteristic * characteristic, uint8_t * data, uint16_t size);

    // subscribe/unsubscribe notification and indications
    int subscribeForNotifications(BTBLEDevice * device, BTBLECharacteristic * characteristic);
    int unsubscribeFromNotifications(BTBLEDevice * device, BTBLECharacteristic * characteristic);
    int subscribeForIndications(BTBLEDevice * device, BTBLECharacteristic * characteristic);
    int unsubscribeFromIndications(BTBLEDevice * device, BTBLECharacteristic * characteristic);

    // Callbacks
    void setBLEAdvertisementCallback(void (*)(BTBLEAdvertisement * bleAdvertisement));
    void setBLEDeviceConnectedCallback(void (*)(BLEStatus status, BTBLEDevice * device));
    void setBLEDeviceDisconnectedCallback(void (*)(BTBLEDevice * device));
    void setGATTServiceDiscoveredCallback(void (*)(BLEStatus status, BTBLEDevice * device, BTBLEService * bleService));
    void setGATTCharacteristicDiscoveredCallback(void (*)(BLEStatus status, BTBLEDevice * device, BTBLECharacteristic * characteristic));
    void setGATTCharacteristicReadCallback(void (*)(BLEStatus status, BTBLEDevice * device, uint8_t * value, uint16_t length));
    void setGATTCharacteristicNotificationCallback(void (*)(BTBLEDevice * device, uint16_t value_handle, uint8_t* value, uint16_t length));
    void setGATTCharacteristicIndicationCallback(void (*)(BTBLEDevice * device, uint16_t value_handle, uint8_t* value, uint16_t length));
    void setGATTDoneCallback(void (*)(BLEStatus status, BTBLEDevice * device));

    void setGATTCharacteristicWrittenCallback(void (*)(BLEStatus status, BTBLEDevice * device));
    void setGATTCharacteristicSubscribedCallback(void (*)(BLEStatus status, BTBLEDevice * device));
    void setGATTCharacteristicUnsubscribedCallback(void (*)(BLEStatus status, BTBLEDevice * device));

    void setGATTCharacteristicRead(uint16_t (*)(uint16_t characteristic_id, uint8_t * buffer, uint16_t buffer_size));
    void setGATTCharacteristicWrite(int (*)(uint16_t characteristic_id, uint8_t *buffer, uint16_t buffer_size));

    void addGATTService(BTUUID * uuid);
    uint16_t addGATTCharacteristic(BTUUID * uuid, uint16_t flags, const char * text);
    uint16_t addGATTCharacteristic(BTUUID * uuid, uint16_t flags, uint8_t * data, uint16_t data_len);
    uint16_t addGATTCharacteristicDynamic(BTUUID * uuid, uint16_t flags, uint16_t characteristic_id);

    // ---- Classic (BR/EDR) ------------------------------------------------

    // Makes the module discoverable/connectable for Classic profiles and
    // sets the local name + Class of Device reported to other devices.
    // Call once, before or after enableA2DPSink()/enableHFPHandsFree().
    void enableClassicDiscoverable(const char * local_name, uint32_t class_of_device);

    // A2DP Sink + AVRCP Controller/Target: registers SDP records, creates a
    // single SBC stream endpoint, and wires decoded audio out through
    // hal_audio_teensy.cpp (Teensy Audio Library I2S1, e.g. an SGTL5000
    // Audio Shield). See README.md wiring notes.
    void enableA2DPSink(void);
    void setA2DPStreamStateCallback(void (*)(A2DPStreamState state));

    // HFP Hands-Free: registers the SDP record and CVSD/mSBC codec
    // negotiation. Voice audio is bridged directly between the module and
    // Teensy over the PCM/I2S2 pins (hal_teensy_pcm.h/.cpp) -- it never
    // passes through this API.
    void enableHFPHandsFree(void);
    void setHFPEventCallback(void (*)(HFPEvent event, hci_con_handle_t handle));
    void hfpAnswer(void);
    void hfpHangup(void);
    void hfpDial(const char * number);
};

extern BTstackTeensyManager BTstackTeensy;

#if defined __cplusplus
}
#endif

#endif // __ARDUINO_TEENSY_BTSTACK_H
