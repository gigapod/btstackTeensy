// hal_uart_dma.h implementation for the Teensy 4.x HCI UART (Serial8 / LPUART6
// by default -- the pins wired to a Murata Type 1YN module's 4-wire H4 UART).
//
// Teensy's HardwareSerial doesn't expose a byte-received interrupt callback to
// user code, so this follows the same polling pattern BTstack's own
// port/max32630-fthr port uses: hal_uart_dma_send_block()/receive_block() just
// record the pending buffer, and hal_teensy_uart_poll() -- called every
// iteration of the Arduino loop() -- drains/fills it against the UART and
// fires the block_received/block_sent callbacks BTstack registered. RTS/CTS
// are real hardware flow control on the LPUART peripheral (attachRts/
// attachCts), so nothing needs to be throttled in software.
//
// All pins/port are overridable by defining the macros below before this file
// is compiled (e.g. via a build flag), in case your wiring differs from the
// defaults documented in README.md.

#include <Arduino.h>

extern "C" {
#include "hal_uart_dma.h"
}

#include "hal_teensy_uart.h"

#ifndef BTSTACK_TEENSY_UART_PORT
#define BTSTACK_TEENSY_UART_PORT Serial8
#endif
#ifndef BTSTACK_TEENSY_UART_RTS_PIN
#define BTSTACK_TEENSY_UART_RTS_PIN 36
#endif
#ifndef BTSTACK_TEENSY_UART_CTS_PIN
#define BTSTACK_TEENSY_UART_CTS_PIN 33
#endif


#define BT_UART BTSTACK_TEENSY_UART_PORT

namespace
{

//----------------------------------------------------------------
// Add a buffer to the UART to handle overflow from the small built-in ring buffer.
//
// Serial8's built-in hardware ring buffer is only 64 bytes (see
// HardwareSerial8.cpp's SERIAL8_RX_BUFFER_SIZE). With hal_teensy_uart_poll()
// only draining it once per Arduino loop() iteration, a burst of HCI events
// (e.g. several BLE advertising reports arriving back to back during a scan)
// can overflow that buffer. The bytes get silently dropped, desyncing the H4
// framing state machine and crashing btstack_assert(false) in
// hci_transport_h4_block_read(). Give it a much larger backing buffer via
// addMemoryForRead() so bursts have somewhere to sit until polled.
//
uint8_t rx_overflow_buffer[2048];

//--------------------------------------------------------------
uint32_t current_baud = 115200;

uint8_t *rx_buffer_ptr = nullptr;
uint16_t rx_bytes_remaining = 0;

const uint8_t *tx_buffer_ptr = nullptr;
uint16_t tx_bytes_remaining = 0;

void (*block_received_cb)(void) = nullptr;
void (*block_sent_cb)(void) = nullptr;

void configure_uart(uint32_t baud) {
    BT_UART.begin(baud);
    // Hardware RTS/CTS flow control -- pins are fixed per LPUART instance on
    // Teensy 4.x, see README.md for the Serial8 <-> pin mapping.
    BT_UART.attachRts(BTSTACK_TEENSY_UART_RTS_PIN);
    BT_UART.attachCts(BTSTACK_TEENSY_UART_CTS_PIN);
}

} // namespace

extern "C" {

void hal_uart_dma_init(void) {
        rx_buffer_ptr = nullptr;
        rx_bytes_remaining = 0;
        tx_buffer_ptr = nullptr;
        tx_bytes_remaining = 0;
        configure_uart(current_baud);
        
        // Add the overflow buffer to the UART for reading.
        BT_UART.addMemoryForRead(rx_overflow_buffer, sizeof(rx_overflow_buffer));
    }

void hal_uart_dma_set_block_received(void (*callback)(void)) {
        block_received_cb = callback;
}

void hal_uart_dma_set_block_sent(void (*callback)(void)) {
        block_sent_cb = callback;
}

int hal_uart_dma_set_baud(uint32_t baud) {
        current_baud = baud;
        // Drain anything in flight before restarting the peripheral at the new baud.
        BT_UART.flush();
        configure_uart(baud);
        return 0;
}

void hal_uart_dma_send_block(const uint8_t *buffer, uint16_t length) {
        tx_buffer_ptr = buffer;
        tx_bytes_remaining = length;
}

void hal_uart_dma_receive_block(uint8_t *buffer, uint16_t len) {
        rx_buffer_ptr = buffer;
        rx_bytes_remaining = len;
}

void hal_uart_dma_set_csr_irq_handler(void (*csr_irq_handler)(void)) {
        // Not used: we don't put the module's UART link to sleep, so there's no
        // CTS wake pulse to react to.
        (void)csr_irq_handler;
}

void hal_uart_dma_set_sleep(uint8_t sleep) {
        (void)sleep;
}

} // extern "C"

extern "C" void hal_teensy_uart_poll(void) {
    if (tx_bytes_remaining > 0) {
        int avail = BT_UART.availableForWrite();
        if (avail > 0) {
            uint16_t n = (uint16_t)min((int)tx_bytes_remaining, avail);
            BT_UART.write(tx_buffer_ptr, n);
            tx_buffer_ptr += n;
            tx_bytes_remaining -= n;
            if (tx_bytes_remaining == 0 && block_sent_cb != nullptr) {
                block_sent_cb();
            }
        }
    }

    if (rx_bytes_remaining > 0) {
        while (rx_bytes_remaining > 0 && BT_UART.available() > 0) {
            *rx_buffer_ptr++ = (uint8_t)BT_UART.read();
            rx_bytes_remaining--;
        }
        if (rx_bytes_remaining == 0 && block_received_cb != nullptr) {
            block_received_cb();
        }
    }
}
