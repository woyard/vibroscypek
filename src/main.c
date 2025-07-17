/*
 * ESP32-C3 ESP-NOW Pairing & Control Firmware
 *
 * This firmware allows two ESP32-C3 devices to be paired and for one to
 * control an LED on the other using a button press.
 *
 * Features:
 * 1. Pairing: Devices pair with the first available peer within a specific RSSI range.
 * 2. Control: A button press on one device sends a command to the other.
 * 3. Action: The receiving device turns on an LED for a set duration.
 * 4. Persistence: The pairing is saved in Non-Volatile Storage (NVS) and persists across reboots.
 * 5. Unpairing: A long press on the button (10 seconds) clears the pairing and restarts the process.
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_mac.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "iot_button.h"
#include "button_gpio.h"

// --- Configuration ---
#define SENDER_BUTTON_GPIO      GPIO_NUM_0      // GPIO0 with internal pulldown
#define OUTPUT_PIN_GPIO         GPIO_NUM_8      // Output pin controlled by received commands
#define OUTPUT_LED_GPIO         GPIO_NUM_8      // output LED pin for status indication
#define UNPAIR_PRESS_DURATION_S 10              // Seconds to hold button to unpair
#define LED_ON_DURATION_MS      100            // Milliseconds for LED to stay on
#define PAIRING_RSSI_THRESHOLD  -60             // RSSI threshold for pairing
#define NVS_PEER_MAC_KEY        "peer_mac"      // NVS key for storing peer MAC

static const char *TAG = "VIBROSCYPEK";

// --- Global Variables ---
static uint8_t s_peer_mac[ESP_NOW_ETH_ALEN] = {0};
static bool s_is_paired = false;
static bool s_is_pairing = false;
static TimerHandle_t s_led_timer_handle;
static TimerHandle_t s_pairing_timer_handle;

// --- Function Declarations ---
static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status);
static void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len);
static void start_pairing(void);
static void complete_pairing(const uint8_t *mac_addr);
static void unpair_device(void);
static esp_err_t load_peer_mac(void);
static esp_err_t save_peer_mac(const uint8_t *mac_addr);
static void status_blink(int blink_count, int delay_ms);

// --- ESP-NOW Callbacks ---

// Send callback: logs the status of a sent message
static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status) {
    if (mac_addr == NULL) {
        ESP_LOGE(TAG, "Send CB error: MAC address is NULL");
        return;
    }
    ESP_LOGI(TAG, "Send status to " MACSTR ": %s", MAC2STR(mac_addr),
             status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

// Receive callback: handles incoming data
static void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    if (recv_info->src_addr == NULL || data == NULL || len <= 0) {
        ESP_LOGE(TAG, "Receive CB error: Invalid data");
        return;
    }

    // If not paired, this could be a pairing request
    if (!s_is_paired) {
        // Check RSSI to ensure the device is nearby
        if (recv_info->rx_ctrl->rssi > PAIRING_RSSI_THRESHOLD) {
            ESP_LOGI(TAG, "Pairing request received from " MACSTR " with RSSI: %d",
                     MAC2STR(recv_info->src_addr), recv_info->rx_ctrl->rssi);
            complete_pairing(recv_info->src_addr);
        } else {
            ESP_LOGW(TAG, "Ignoring pairing request from " MACSTR " due to weak signal (RSSI: %d)",
                     MAC2STR(recv_info->src_addr), recv_info->rx_ctrl->rssi);
        }
    }
    // If paired, check if the message is from the known peer
    else if (memcmp(recv_info->src_addr, s_peer_mac, ESP_NOW_ETH_ALEN) == 0) {
        if (len == 1 && data[0] == 1) { // Simple "button pressed" command
            ESP_LOGI(TAG, "Button press command received from " MACSTR, MAC2STR(recv_info->src_addr));
            gpio_set_level(OUTPUT_PIN_GPIO, 1);
            // Start or reset the timer to turn the LED off
            xTimerReset(s_led_timer_handle, portMAX_DELAY);
        }
    }
}

// --- Pairing and NVS ---

// Starts the pairing process by broadcasting a message
static void start_pairing(void) {
    s_is_paired = false;
    s_is_pairing = true;
    ESP_LOGI(TAG, "Starting pairing process...");
    
    // Start the pairing blink animation
    if (s_pairing_timer_handle != NULL) {
        xTimerStart(s_pairing_timer_handle, portMAX_DELAY);
    }
    
    // Use the broadcast address to find any listening device
    uint8_t broadcast_mac[ESP_NOW_ETH_ALEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    
    esp_now_peer_info_t peer_info = {};
    memcpy(peer_info.peer_addr, broadcast_mac, ESP_NOW_ETH_ALEN);
    if (!esp_now_is_peer_exist(broadcast_mac)) {
        ESP_ERROR_CHECK(esp_now_add_peer(&peer_info));
    }
    
    // Send a simple byte to initiate pairing
    uint8_t pairing_req = 0;
    esp_now_send(broadcast_mac, &pairing_req, sizeof(pairing_req));
}

// Completes the pairing process once a suitable peer is found
static void complete_pairing(const uint8_t *mac_addr) {
    memcpy(s_peer_mac, mac_addr, ESP_NOW_ETH_ALEN);
    s_is_paired = true;
    s_is_pairing = false;

    // Stop the pairing blink animation and turn off LED
    if (s_pairing_timer_handle != NULL) {
        xTimerStop(s_pairing_timer_handle, portMAX_DELAY);
    }
    gpio_set_level(OUTPUT_LED_GPIO, 0);

    ESP_LOGI(TAG, "Pairing complete with " MACSTR, MAC2STR(s_peer_mac));

    // Quick double blink to indicate successful pairing
    status_blink(2, 100);

    // Add the paired device to the peer list for direct communication
    esp_now_peer_info_t peer_info = {};
    memcpy(peer_info.peer_addr, s_peer_mac, ESP_NOW_ETH_ALEN);
    if (!esp_now_is_peer_exist(s_peer_mac)) {
        ESP_ERROR_CHECK(esp_now_add_peer(&peer_info));
    }

    // Save the peer's MAC to NVS
    save_peer_mac(s_peer_mac);
}

// Unpairs the device by clearing the peer MAC from NVS and RAM
static void unpair_device(void) {
    ESP_LOGI(TAG, "Unpairing device...");
    
    // Stop pairing animation if running
    s_is_pairing = false;
    if (s_pairing_timer_handle != NULL) {
        xTimerStop(s_pairing_timer_handle, portMAX_DELAY);
    }
    
    // Delete from NVS
    nvs_handle_t nvs_handle;
    ESP_ERROR_CHECK(nvs_open("storage", NVS_READWRITE, &nvs_handle));
    nvs_erase_key(nvs_handle, NVS_PEER_MAC_KEY);
    nvs_commit(nvs_handle);
    nvs_close(nvs_handle);

    // Delete from ESP-NOW peer list
    if (s_is_paired && esp_now_is_peer_exist(s_peer_mac)) {
        esp_now_del_peer(s_peer_mac);
    }

    // Clear from RAM
    memset(s_peer_mac, 0, ESP_NOW_ETH_ALEN);
    s_is_paired = false;

    ESP_LOGI(TAG, "Device unpaired. Restarting pairing process.");
    
    // Triple blink to indicate unpairing
    status_blink(3, 150);
    
    start_pairing();
}

// Loads the peer MAC address from NVS on startup
static esp_err_t load_peer_mac(void) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("storage", NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        return err;
    }

    size_t required_size = sizeof(s_peer_mac);
    err = nvs_get_blob(nvs_handle, NVS_PEER_MAC_KEY, s_peer_mac, &required_size);
    nvs_close(nvs_handle);

    if (err == ESP_OK && required_size == ESP_NOW_ETH_ALEN) {
        s_is_paired = true;
        ESP_LOGI(TAG, "Loaded peer MAC from NVS: " MACSTR, MAC2STR(s_peer_mac));
    }
    
    return err;
}

// Saves the peer MAC address to NVS after pairing
static esp_err_t save_peer_mac(const uint8_t *mac_addr) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        return err;
    }

    err = nvs_set_blob(nvs_handle, NVS_PEER_MAC_KEY, mac_addr, ESP_NOW_ETH_ALEN);
    if (err == ESP_OK) {
        err = nvs_commit(nvs_handle);
    }
    nvs_close(nvs_handle);
    
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Saved peer MAC to NVS");
    } else {
        ESP_LOGE(TAG, "Failed to save peer MAC to NVS");
    }

    return err;
}


// --- Status Indication ---

// Quick status blink function for inline status indications
static void status_blink(int blink_count, int delay_ms) {
    // Temporarily stop pairing animation if running to avoid conflicts
    bool was_pairing = s_is_pairing;
    if (was_pairing && s_pairing_timer_handle != NULL) {
        xTimerStop(s_pairing_timer_handle, portMAX_DELAY);
    }
    
    // Remember the current LED state
    int current_level = gpio_get_level(OUTPUT_LED_GPIO);
    
    // Perform the blink sequence
    for (int i = 0; i < blink_count; i++) {
        gpio_set_level(OUTPUT_LED_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
        gpio_set_level(OUTPUT_LED_GPIO, 0);
        if (i < blink_count - 1) { // Don't delay after the last blink
            vTaskDelay(pdMS_TO_TICKS(delay_ms));
        }
    }
    
    // Restore the original LED state
    gpio_set_level(OUTPUT_LED_GPIO, current_level);
    
    // Resume pairing animation if it was running
    if (was_pairing && s_pairing_timer_handle != NULL) {
        xTimerStart(s_pairing_timer_handle, portMAX_DELAY);
    }
}

// --- Initialization ---

// Wi-Fi and ESP-NOW Initialization
static void wifi_espnow_init(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    // ESP-NOW Init
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));
}

// Timer callback to turn off the LED
static void led_off_timer_cb(TimerHandle_t xTimer) {
    gpio_set_level(OUTPUT_PIN_GPIO, 0);
    ESP_LOGI(TAG, "LED turned off");
}

// Timer callback for pairing LED blink animation
static void pairing_blink_timer_cb(TimerHandle_t xTimer) {
    static bool led_state = false;
    
    if (s_is_pairing) {
        led_state = !led_state;
        gpio_set_level(OUTPUT_LED_GPIO, led_state ? 1 : 0);
        ESP_LOGD(TAG, "Pairing blink: LED %s", led_state ? "ON" : "OFF");
    }
}

// Button press callbacks
static void button_single_click_cb(void *arg, void *usr_data) {
    if (s_is_paired) {
        ESP_LOGI(TAG, "Button pressed. Sending command to peer.");
        uint8_t command = 1; // Simple "button pressed" command
        esp_now_send(s_peer_mac, &command, sizeof(command));
    } else {
        ESP_LOGW(TAG, "Button pressed, but not paired. Ignoring.");
    }
}

static void button_long_press_cb(void *arg, void *usr_data) {
    ESP_LOGW(TAG, "Long press detected. Unpairing device.");
    unpair_device();
}


// GPIO and Button Initialization
static void gpio_init(void) {
    // Configure output pin (controlled by received commands)
    gpio_config_t output_pin_conf = {};
    output_pin_conf.pin_bit_mask = (1ULL << OUTPUT_PIN_GPIO);
    output_pin_conf.mode = GPIO_MODE_OUTPUT;
    output_pin_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&output_pin_conf);
    gpio_set_level(OUTPUT_PIN_GPIO, 0); // Start with output pin off

    // Configure LED pin (for pairing animation) - only if different from output pin
    if (OUTPUT_LED_GPIO != OUTPUT_PIN_GPIO) {
        gpio_config_t led_conf = {};
        led_conf.pin_bit_mask = (1ULL << OUTPUT_LED_GPIO);
        led_conf.mode = GPIO_MODE_OUTPUT;
        led_conf.intr_type = GPIO_INTR_DISABLE;
        gpio_config(&led_conf);
        gpio_set_level(OUTPUT_LED_GPIO, 0); // Start with LED off
    }

    // Create a one-shot timer for the LED
    s_led_timer_handle = xTimerCreate("led_off_timer", pdMS_TO_TICKS(LED_ON_DURATION_MS),
                                      pdFALSE, // One-shot timer
                                      (void *)0, led_off_timer_cb);

    // Create a repeating timer for pairing LED blink animation (1 second interval)
    s_pairing_timer_handle = xTimerCreate("pairing_blink_timer", pdMS_TO_TICKS(1000),
                                          pdTRUE, // Auto-reload timer (repeating)
                                          (void *)0, pairing_blink_timer_cb);

    // Configure Button
    button_config_t btn_cfg = {
        .long_press_time = UNPAIR_PRESS_DURATION_S * 1000,
        .short_press_time = 50,
    };
    
    button_gpio_config_t gpio_cfg = {
        .gpio_num = SENDER_BUTTON_GPIO,
        .active_level = 1, // Button is active high (pulls GPIO0 to 3.3V when pressed)
        .enable_power_save = false,
        .disable_pull = false, // Let iot_button automatically configure pulldown for active_level=1
    };
    
    button_handle_t btn_handle;
    ESP_ERROR_CHECK(iot_button_new_gpio_device(&btn_cfg, &gpio_cfg, &btn_handle));
    ESP_ERROR_CHECK(iot_button_register_cb(btn_handle, BUTTON_SINGLE_CLICK, NULL, button_single_click_cb, NULL));
    ESP_ERROR_CHECK(iot_button_register_cb(btn_handle, BUTTON_LONG_PRESS_START, NULL, button_long_press_cb, NULL));
}

void app_main(void) {
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    gpio_init();
    wifi_espnow_init();

    // Try to load a paired device from NVS
    if (load_peer_mac() == ESP_OK) {
        // If successful, add the peer to ESP-NOW
        esp_now_peer_info_t peer_info = {};
        memcpy(peer_info.peer_addr, s_peer_mac, ESP_NOW_ETH_ALEN);
        if (!esp_now_is_peer_exist(s_peer_mac)) {
            ESP_ERROR_CHECK(esp_now_add_peer(&peer_info));
        }
        ESP_LOGI(TAG, "Device is already paired. Ready to send/receive.");
    } else {
        // If not found in NVS, start the pairing process
        ESP_LOGI(TAG, "No paired device found in NVS. Starting pairing.");
        start_pairing();
    }
}
