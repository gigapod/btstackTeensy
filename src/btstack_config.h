//
// btstack_config.h for the Arduino Teensy port (Teensy 4.1 + Murata Type 1YN / CYW43439)
//
// Documentation: https://bluekitchen-gmbh.com/btstack/#how_to/
//

#ifndef BTSTACK_CONFIG_H
#define BTSTACK_CONFIG_H

// Port related features
#define HAVE_ASSERT
#define HAVE_MALLOC
#define HAVE_EMBEDDED_TIME_MS
#define HAVE_HAL_AUDIO

// BTstack features that can be enabled
#define ENABLE_BLE
#define ENABLE_CLASSIC
#define ENABLE_CROSS_TRANSPORT_KEY_DERIVATION
#define ENABLE_GOEP_L2CAP
#define ENABLE_HFP_WIDE_BAND_SPEECH
#define ENABLE_L2CAP_ENHANCED_RETRANSMISSION_MODE
#define ENABLE_L2CAP_LE_CREDIT_BASED_FLOW_CONTROL_MODE
#define ENABLE_LE_CENTRAL
#define ENABLE_LE_DATA_LENGTH_EXTENSION
#define ENABLE_LE_PERIPHERAL
#define ENABLE_LE_SECURE_CONNECTIONS
#define ENABLE_LOG_ERROR
#define ENABLE_LOG_INFO
#define ENABLE_MICRO_ECC_FOR_LE_SECURE_CONNECTIONS
#define ENABLE_PRINTF_HEXDUMP
#define ENABLE_SDP_DES_DUMP
#define ENABLE_SOFTWARE_AES128

// Route SCO (HFP voice) audio over the CYW43439's 4-wire PCM interface instead of
// over the HCI UART -- BT_PCM_SYNC/CLK/OUT/IN, see hal_teensy_pcm.cpp.
#define ENABLE_SCO_OVER_PCM
#define ENABLE_BCM_PCM_WBS

// BTstack configuration. buffers, sizes, ...
#define HCI_ACL_PAYLOAD_SIZE (1021 + 4)
#define HCI_INCOMING_PRE_BUFFER_SIZE 6

#define MAX_ATT_DB_SIZE 200
#define MAX_NR_GATT_CLIENTS 1
#define MAX_NR_HCI_CONNECTIONS 2
#define MAX_NR_L2CAP_CHANNELS  4
#define MAX_NR_L2CAP_SERVICES  4
#define MAX_NR_LE_DEVICE_DB_ENTRIES 4
#define MAX_NR_SM_LOOKUP_ENTRIES 3
#define MAX_NR_WHITELIST_ENTRIES 1

#define NVM_NUM_LINK_KEYS          8

// no flash-backed TLV in this port -> in-memory link key / LE device DB only
// (pairing is lost on reset). NVM_NUM_DEVICE_DB_ENTRIES must stay undefined:
// defining it selects ble/le_device_db_tlv.c (which requires a
// btstack_tlv_t backend we don't have here, and asserts on first use)
// instead of ble/le_device_db_memory.c's RAM-only implementation.

#endif
