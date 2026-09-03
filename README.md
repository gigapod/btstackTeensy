# BTstackTeensy -- Arduino library for Teensy 4.x + Murata Type 1YN (CYW43439)

An Arduino library packaging of BTstack for the Teensy 4.x family, talking to
a Murata Type 1YN Wi-Fi+BT module (Cypress/Infineon CYW43439 silicon) over a
4-wire H4 UART. Provides:

- BLE Central & Peripheral (GAP/GATT), with the same wrapper API style as
  BlueKitchen's original `port/arduino` (`BTstackManager`/`BTstack`) --
  renamed `BTstackTeensyManager`/`BTstackTeensy` here so both libraries can be
  installed side by side.
- Classic Bluetooth: A2DP Sink + AVRCP (audio out over Teensy Audio Library
  I2S1, e.g. an SGTL5000 Audio Shield) and HFP Hands-Free (voice audio
  bridged directly between the module and Teensy over its PCM interface,
  bypassing the host).

This packages the port as a modern ["recursive src/"
layout](https://arduino.github.io/arduino-cli/latest/library-specification/)
Arduino library, unlike the original `port/arduino`'s flat/legacy layout.

## Wiring

| Signal          | Teensy pin | Notes                                           |
|------------------|:---------:|--------------------------------------------------|
| HCI UART TX      | 35        | Serial8 (LPUART6) TX -> module RX                |
| HCI UART RX      | 34        | Serial8 RX <- module TX                          |
| HCI UART RTS     | 36        | hardware flow control, Teensy -> module CTS      |
| HCI UART CTS     | 33        | hardware flow control, module -> Teensy          |
| BT_ON            | 28        | module REG_ON / power enable, active high        |
| BT_PCM_SYNC      | 3         | SAI2 frame sync (LRCLK), module is clock master  |
| BT_PCM_CLK       | 4         | SAI2 bit clock (BCLK), module is clock master    |
| BT_PCM_OUT       | 5         | SAI2 TX data: Teensy -> module (mic path)        |
| BT_PCM_IN        | 2         | SAI2 RX data: module -> Teensy (speaker path)    |

Pins 2/3/4/5 are Teensy 4.x's second SAI block (SAI2, "I2S2" in Teensy Audio
Library terms), wired in **slave** mode -- the CYW43439 drives the PCM clock
(see `ENABLE_SCO_OVER_PCM`/`ENABLE_BCM_PCM_WBS` in `src/btstack_config.h`).
This is separate from A2DP playback, which is decoded on the Teensy and sent
out over I2S1 (the *standard* Teensy Audio Shield pins) via
`src/hal_audio_teensy.cpp`.

All pins are overridable by defining `BTSTACK_TEENSY_UART_PORT`,
`BTSTACK_TEENSY_UART_RTS_PIN`, `BTSTACK_TEENSY_UART_CTS_PIN`,
`BTSTACK_TEENSY_BT_ON_PIN` and `BTSTACK_TEENSY_BT_ON_SETTLE_MS` as build
flags before the library sources are compiled, if your wiring differs.

## Firmware (patchram)

The CYW43439 needs its Bluetooth init script (patchram) loaded over HCI
before it does anything. `src/firmware/cyw43_btfw_1yn.h` +
`src/firmware/cyw43_btfw_1yn_patchram.c` provide a 1YN firmware blob from
[`georgerobotics/cyw43-driver`](https://github.com/georgerobotics/cyw43-driver)
(the open-source Infineon/Raspberry Pi driver package that also backs the
Pico 2 W's Bluetooth support -- see the file header there for provenance).
This firmware blob is Murata/Infineon IP redistributed for use with their
modules; it is not part of BTstack and isn't covered by this repository's
license. Read Murata's or cyw43-driver's license/README before shipping a
product with it. If you'd rather build your own from Murata's
[`cyw-bt-patch`](https://github.com/murata-wireless/cyw-bt-patch) repository
instead, see `chipset/bcm/convert_hcd.py` in the main BTstack repository and
replace the two files in `src/firmware/` with its output.

## Building the library

From this directory (`port/arduino-teensy`), with GNU Make, `zip` and a
checkout of the full BTstack repository (the Makefile pulls BTstack's core,
Classic, BLE and BCM chipset sources directly out of `../../src`,
`../../chipset` etc. -- nothing is vendored into this folder):

```
make            # assemble build/BTstackTeensy and zip it to
                 # ../../btstack-arduino-teensy-<version>.zip
make install    # also copy build/BTstackTeensy into
                 # ~/Documents/Arduino/libraries/BTstackTeensy
make clean      # remove build/ and any previously built archives
```

Then, in the Arduino IDE: Sketch -> Include Library -> Add .ZIP Library...
and select the generated archive (or just use `make install` and restart the
IDE). Select a Teensy 4.x board (Tools -> Board -> Teensyduino) before
compiling an example.

## Examples

- `BLEPeripheral`, `BLECentral`, `iBeacon`, `iBeaconScanner` -- ported from
  `port/arduino`'s BLE examples; the API is unchanged, only the global object
  name (`BTstackTeensy` instead of `BTstack`).
- `A2DPSinkHFP` -- Classic Bluetooth demo: A2DP Sink + AVRCP audio playback
  (wire an I2S DAC to Teensy's I2S1 pins to hear it) plus HFP Hands-Free call
  control over USB Serial (`a` answer, `h` hang up/reject, `d<number>` dial).

## Known limitations

- Not yet run against real 1YN hardware as part of this port -- BT_ON
  timing, UART bring-up baud rate and PCM clock polarity/justification all
  need bring-up and debugging on your board. Start by watching the USB
  Serial log during `hci_power_control(ON)`.
- No persistence: link keys and LE bonds live in RAM only -- every reboot
  forgets paired devices.
- A2DP only decodes SBC, and `hal_audio_teensy.cpp` doesn't resample: a
  source that negotiates 48/32/16 kHz instead of 44.1 kHz will play back at
  the wrong pitch/speed.
- AVRCP Target is a stub: the SDP record and connection are there, but
  volume/play-pause commands from the phone aren't wired to anything
  (`avrcp_target_packet_handler` in `src/BTstackTeensy.cpp`).
