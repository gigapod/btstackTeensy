// iBeacon Simulator -- adapted from port/arduino's iBeacon example for the
// BTstackTeensy library. See README.md for wiring.
#include <BTstackTeensy.h>
#include <stdio.h>

BTUUID uuid("E2C56DB5-DFFB-48D2-B060-D0F5A71096E0");
const uint16_t majorID = 4711;
const uint16_t minorID = 2;

void setup(void){
    Serial.begin(115200);
    Serial.println("iBeacon example starting...");

    BTstackTeensy.setup();

    Serial.print("Configuring iBeacon: BTUUID ");
    Serial.print(uuid.getUuidString());
    Serial.print(", major ");
    Serial.print(majorID);
    Serial.print(", minor ");
    Serial.println(minorID);
    BTstackTeensy.iBeaconConfigure(&uuid, majorID, minorID);

    Serial.println("Starting advertising");
    BTstackTeensy.startAdvertising();
}

void loop(void){
    BTstackTeensy.loop();
}
