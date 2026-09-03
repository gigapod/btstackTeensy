// BLE Peripheral Example -- adapted from port/arduino's LEPeripheral example
// for the BTstackTeensy library. See README.md for wiring.
#include <BTstackTeensy.h>

static char characteristic_data = 'H';

void setup(void){

    Serial.begin(115200);

    // If the previous run crashed (e.g. a hard fault), Teensy 4.x auto-reboots
    // silently -- print the fault details it saved so we can see why.
    if (CrashReport){
        Serial.print(CrashReport);
    }

    // set callbacks
    BTstackTeensy.setBLEDeviceConnectedCallback(deviceConnectedCallback);
    BTstackTeensy.setBLEDeviceDisconnectedCallback(deviceDisconnectedCallback);
    BTstackTeensy.setGATTCharacteristicRead(gattReadCallback);
    BTstackTeensy.setGATTCharacteristicWrite(gattWriteCallback);

    // setup GATT Database
    BTstackTeensy.addGATTService(new BTUUID("B8E06067-62AD-41BA-9231-206AE80AB551"));
    BTstackTeensy.addGATTCharacteristic(new BTUUID("f897177b-aee8-4767-8ecc-cc694fd5fcef"), ATT_PROPERTY_READ, "This is a String!");
    BTstackTeensy.addGATTCharacteristicDynamic(new BTUUID("f897177b-aee8-4767-8ecc-cc694fd5fce0"), ATT_PROPERTY_READ | ATT_PROPERTY_WRITE | ATT_PROPERTY_NOTIFY, 0);

    // name shown to scanning devices (defaults to "BTstack Teensy" if not set)
    BTstackTeensy.setDeviceName("My Teensy Peripheral");

    // startup Bluetooth and activate advertisements
    BTstackTeensy.setup();
    BTstackTeensy.startAdvertising();
}

void loop(void){
    BTstackTeensy.loop();
}

void deviceConnectedCallback(BLEStatus status, BTBLEDevice *device) {
    switch (status){
        case BLE_STATUS_OK:
            Serial.println("Device connected!");
            break;
        default:
            break;
    }
}

void deviceDisconnectedCallback(BTBLEDevice * device){
    Serial.println("Disconnected.");
}

uint16_t gattReadCallback(uint16_t value_handle, uint8_t * buffer, uint16_t buffer_size){
    if (buffer){
        Serial.print("gattReadCallback, value: ");
        Serial.println(characteristic_data, HEX);
        buffer[0] = characteristic_data;
    }
    return 1;
}

int gattWriteCallback(uint16_t value_handle, uint8_t *buffer, uint16_t size){
    characteristic_data = buffer[0];
    Serial.print("gattWriteCallback , value ");
    Serial.println(characteristic_data, HEX);
    return 0;
}
