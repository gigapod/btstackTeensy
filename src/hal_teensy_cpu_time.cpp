// hal_cpu.h / hal_time_ms.h implementation for Teensy 4.x (IMXRT106x / Cortex-M7)

#include <Arduino.h>

extern "C" {

#include "hal_cpu.h"
#include "hal_time_ms.h"

void hal_cpu_disable_irqs(void) {
    __disable_irq();
}

void hal_cpu_enable_irqs(void) {
    __enable_irq();
}

void hal_cpu_enable_irqs_and_sleep(void) {
    // No low-power sleep entry implemented -- just re-enable interrupts and
    // let the run loop poll again immediately. WFI is not used here because
    // it would also need to survive USB/millis() servicing on Teensy.
    __enable_irq();
}

uint32_t hal_time_ms(void) {
    return millis();
}

} // extern "C"
