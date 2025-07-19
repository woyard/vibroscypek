#pragma once

#include "driver/gpio.h"

// --- Hardware Pin Configuration ---
#define SENDER_BUTTON_GPIO      GPIO_NUM_0     // GPIO0 with internal pulldown
#define OUTPUT_PIN_GPIO         GPIO_NUM_10    // Output pin controlled by received commands
#define OUTPUT_LED_GPIO         GPIO_NUM_8     // output LED pin for status indication

// --- Timing Configuration ---
#define UNPAIR_PRESS_DURATION_MS 7000         // Milliseconds to hold button to unpair
#define LED_ON_DURATION_MS      100            // Milliseconds for LED to stay on
#define OUTPUT_PIN_ACTIVE_TIME_MS  250
#define PAIRING_REQUEST_INTERVAL_MS 3000       // Milliseconds between pairing request broadcasts

// --- Protocol Configuration ---
#define PAIRING_RSSI_THRESHOLD  -65            // RSSI threshold for pairing
#define PAIRING_CODE            0x1234         // Verification code for pairing
#define NVS_PEER_MAC_KEY        "peer_mac"     // NVS key for storing peer MAC

// --- GPIO Configuration ---
#define OUTPUT_PIN_ACTIVE_LEVEL 1              // Active level for the output pin (1 = high, 0 = low)
