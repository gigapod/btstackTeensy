// iBeacon Scanner -- adapted from port/arduino's iBeaconScanner example for
// the BTstackTeensy library. See README.md for wiring.
#include <BTstackTeensy.h>

void setup(void){
    Serial.begin(115200);
    BTstackTeensy.setup();
    BTstackTeensy.setBLEAdvertisementCallback(advertisementCallback);
    BTstackTeensy.bleStartScanning();
}

void loop(void){
    BTstackTeensy.loop();
}

void advertisementCallback(BTBLEAdvertisement *adv) {
    if (adv->isIBeacon()) {
        Serial.print("iBeacon found ");
        Serial.print(adv->getBdAddr()->getAddressString());
        Serial.print(", RSSI ");
        Serial.print(adv->getRssi());
        Serial.print(", BTUUID ");
        Serial.print(adv->getIBeaconUUID()->getUuidString());
        Serial.print(", MajorID ");
        Serial.print(adv->getIBeaconMajorID());
        Serial.print(", MinorID ");
        Serial.print(adv->getIBecaonMinorID());
        Serial.print(", Measured Power ");
        Serial.println(adv->getiBeaconMeasuredPower());
    } else {
        Serial.print("Device discovered: ");
        Serial.print(adv->getBdAddr()->getAddressString());
        Serial.print(", RSSI ");
        Serial.print(adv->getRssi());
        const char * name = adv->getName();
        if (name){
            Serial.print(", name ");
            Serial.print(name);
        }
        Serial.println();
    }
}
