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

// How long to hold BT_ON low before driving it high again in control_on().
// BT_ON only enables the module's internal regulators -- it's not a reset
// strobe -- so it's not enough to just drive it low once at control_init()
// time and trust that. If a *previous* run left the module powered and
// running at a non-default UART baud rate (see config.baudrate_main in
// BTstackTeensy.cpp) and the Teensy resets/reflashes without enough of a gap
// between that low and hci_power_control(HCI_POWER_ON)'s subsequent high,
// the module's regulators (and whatever state its firmware is holding,
// including the UART baud rate) can ride through the "reset" unchanged --
// the next boot's HCI Reset then goes out at 115200 into a module still
// listening at 921600, gets no response ever, and the resend logic ends up
// tripping the run-loop's duplicate-timer assert. Forcing BT_ON low for a
// real hold time here, on every control_on(), guarantees the module's
// regulators actually discharge and it comes up fresh regardless of what
// state a previous run left it in.
#ifndef BTSTACK_TEENSY_BT_ON_OFF_HOLD_MS
#define BTSTACK_TEENSY_BT_ON_OFF_HOLD_MS 100
#endif

namespace {

void control_init(const void *config) {
    (void)config;
    printf("cyw43439: init BT_ON pin %u\n", BTSTACK_TEENSY_BT_ON_PIN);
    pinMode(BTSTACK_TEENSY_BT_ON_PIN, OUTPUT);
    digitalWrite(BTSTACK_TEENSY_BT_ON_PIN, LOW);
}

int control_on(void) {
    // Force a real power-cycle -- see the BTSTACK_TEENSY_BT_ON_OFF_HOLD_MS
    // comment above for why this can't just be "drive BT_ON high".
    digitalWrite(BTSTACK_TEENSY_BT_ON_PIN, LOW);
    delay(BTSTACK_TEENSY_BT_ON_OFF_HOLD_MS);

    printf("cyw43439: BT_ON high, waiting %u ms for module to settle...\n", BTSTACK_TEENSY_BT_ON_SETTLE_MS);
    digitalWrite(BTSTACK_TEENSY_BT_ON_PIN, HIGH);
    delay(BTSTACK_TEENSY_BT_ON_SETTLE_MS);
    printf("cyw43439: module settled, UART should be ready\n");
    return 0;
}

int control_off(void) {
    printf("cyw43439: BT_ON low, module powered off\n");
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
