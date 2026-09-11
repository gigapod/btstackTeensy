// btstack_control_t implementation for the Murata Type 1YN's BT_ON / REG_ON pin.
//
// BT_ON enables the module's internal regulators; it is not a reset strobe, so
// we just drive it and hold it. The module needs some time after BT_ON goes
// high before its UART is ready to receive the first HCI command --
// BTSTACK_TEENSY_BT_ON_SETTLE_MS below is a conservative starting point taken
// from other CYW43xxx designs; tune it down once you've verified bring-up on
// your board (see README.md).

#include <Arduino.h>

#include "teensy_bt_control.h"

#ifndef BTSTACK_TEENSY_BT_ON_PIN
#define BTSTACK_TEENSY_BT_ON_PIN 28
#endif

#ifndef BTSTACK_TEENSY_BT_ON_SETTLE_MS
#define BTSTACK_TEENSY_BT_ON_SETTLE_MS 150
#endif

namespace {

void control_init(const void *config) {
    (void)config;
    pinMode(BTSTACK_TEENSY_BT_ON_PIN, OUTPUT);
    digitalWrite(BTSTACK_TEENSY_BT_ON_PIN, LOW);
}

int control_on(void) {
    digitalWrite(BTSTACK_TEENSY_BT_ON_PIN, HIGH);
    delay(BTSTACK_TEENSY_BT_ON_SETTLE_MS);
    return 0;
}

int control_off(void) {
    digitalWrite(BTSTACK_TEENSY_BT_ON_PIN, LOW);
    return 0;
}

int control_sleep(void) {
    // Not implemented: the module is left fully powered between HCI activity.
    return 0;
}

int control_wake(void) {
    return 0;
}

void control_register_for_power_notifications(void (*cb)(POWER_NOTIFICATION_t event)) {
    (void)cb;
}

const btstack_control_t teensy_bt_control = {
    &control_init,
    &control_on,
    &control_off,
    &control_sleep,
    &control_wake,
    &control_register_for_power_notifications,
};

} // namespace

extern "C" const btstack_control_t *teensy_bt_control_instance(void) {
    return &teensy_bt_control;
}
