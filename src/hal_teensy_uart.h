#ifndef HAL_TEENSY_UART_H
#define HAL_TEENSY_UART_H

#ifdef __cplusplus
extern "C" {
#endif

// Pumps pending HCI UART TX/RX against the module's UART (Serial8 by default,
// see BTSTACK_TEENSY_UART_PORT). Call once per Arduino loop() iteration, as
// often as possible -- BTstackTeensyManager::loop() already does this for you.
void hal_teensy_uart_poll(void);

#ifdef __cplusplus
}
#endif

#endif // HAL_TEENSY_UART_H
