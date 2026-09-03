/*
 * Adapts cyw43_btfw_1yn.h -- a CYW43439/Murata 1YN Bluetooth firmware blob in
 * Infineon/Raspberry Pi's open-source `cyw43-driver` format (as used by the
 * Pico 2 W SDK) -- to the symbol names btstack_chipset_bcm.c links against
 * on non-POSIX systems when nothing calls btstack_chipset_bcm_set_patchram():
 * brcm_patchram_buf / brcm_patch_ram_length / brcm_patch_version (see
 * ../../../chipset/bcm/btstack_chipset_bcm.h).
 *
 * PROVENANCE: cyw43_btfw_1yn.h is from
 * https://github.com/georgerobotics/cyw43-driver (confirmed source) -- the
 * open-source Infineon/Raspberry Pi driver package that also backs the
 * Pico 2 W's Bluetooth support. Its "// Source: CYW4343A2_..._1YN.hcd"
 * comment matches the same 1YN patch family documented in Murata's own
 * https://github.com/murata-wireless/cyw-bt-patch. It has not been tested
 * against real 1YN hardware as part of this port -- verify it against your
 * module's revision before shipping, and see README.md's firmware section
 * for the alternative of generating your own from Murata's patch repo via
 * convert_hcd.py.
 */

#include <stddef.h>
#include <stdint.h>

// Only meaningful for RP2040 XIP placement in the upstream cyw43-driver
// project this header comes from; unused here.
#define CYW43_RESOURCE_ATTRIBUTE

// Rename the array/length this header defines to the symbols
// btstack_chipset_bcm.c expects, instead of duplicating ~5KB of firmware
// bytes into a second array.
#define cyw43_btfw_1yn brcm_patchram_buf
#define cyw43_btfw_1yn_len brcm_patch_ram_length_u32

#include "cyw43_btfw_1yn.h"

#undef cyw43_btfw_1yn
#undef cyw43_btfw_1yn_len

const char brcm_patch_version[] =
    "cyw43_btfw_1yn (Infineon/Raspberry Pi cyw43-driver, source: CYW4343A2_001.003.016.0031.0000.1YN.hcd)";
const int brcm_patch_ram_length = (int)brcm_patch_ram_length_u32;
