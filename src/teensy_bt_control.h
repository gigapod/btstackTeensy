#ifndef TEENSY_BT_CONTROL_H
#define TEENSY_BT_CONTROL_H

#include "btstack_control.h"

#ifdef __cplusplus
extern "C" {
#endif

// btstack_control_t driving BT_ON (Murata Type 1YN REG_ON, Teensy pin 28 by default).
const btstack_control_t *teensy_bt_control_instance(void);

#ifdef __cplusplus
}
#endif

#endif // TEENSY_BT_CONTROL_H
