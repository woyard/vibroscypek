#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_mac.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "iot_button.h"
#include "button_gpio.h"

// --- Configuration ---
#define SENDER_BUTTON_GPIO      GPIO_NUM_0     // GPIO0 with internal pulldown
#define OUTPUT_PIN_GPIO         GPIO_NUM_10    // Output pin controlled by received commands
#define OUTPUT_LED_GPIO         GPIO_NUM_8     // output LED pin for status indication
#define UNPAIR_PRESS_DURATION_MS 7000         // Milliseconds to hold button to unpair
#define LED_ON_DURATION_MS      100            // Milliseconds for LED to stay on
#define OUTPUT_PIN_ACTIVE_LEVEL 1              // Active level for the output pin (1 = high, 0 = low)
#define OUTPUT_PIN_ACTIVE_TIME_MS  250
#define PAIRING_RSSI_THRESHOLD  -65            // RSSI threshold for pairing
#define PAIRING_REQUEST_INTERVAL_MS 3000       // Milliseconds between pairing request broadcasts
#define NVS_PEER_MAC_KEY        "peer_mac"     // NVS key for storing peer MAC
#define PAIRING_CODE            0x1234         // Verification code for pairing

// Message structure for communication
typedef struct {
    uint16_t command_type;
    uint16_t payload;
} message_t;

// LED command structure for queue
typedef struct {
    uint8_t blink_count;
    uint16_t delay_ms;
    bool high_priority;  // If true, this command will be inserted at front of queue
} led_command_t;

// Command types
#define CMD_PAIRING_REQUEST     0x0001
#define CMD_PAIRING_RESPONSE    0x0002
#define CMD_BUTTON_PRESS        0x0003

// Button states for payload
#define BUTTON_STATE_PRESSED    0x0001
#define BUTTON_STATE_RELEASED   0x0000

// LED queue configuration
#define LED_QUEUE_SIZE          10
#define LED_TASK_STACK_SIZE     2048
#define LED_TASK_PRIORITY       3

static const char *TAG = "VIBROSCYPEK";

// --- Global Variables ---
static uint8_t s_peer_mac[ESP_NOW_ETH_ALEN] = {0};
static uint8_t s_broadcast_mac[ESP_NOW_ETH_ALEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static bool s_is_paired = false;
static bool s_is_pairing = false;
static TimerHandle_t s_output_pin_timer_handle;
static TimerHandle_t s_pairing_broadcast_timer_handle;
static QueueHandle_t s_led_queue = NULL;
static TaskHandle_t s_led_task_handle = NULL;

// --- Function Declarations ---
static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status);
static void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len);

// Message processing functions
static void process_pairing_message(const esp_now_recv_info_t *recv_info, const message_t *msg);
static void process_operational_message(const esp_now_recv_info_t *recv_info, const message_t *msg);
static void handle_pairing_request(const esp_now_recv_info_t *recv_info, const message_t *msg);
static void handle_pairing_response(const esp_now_recv_info_t *recv_info, const message_t *msg);
static void handle_incoming_button_press(const esp_now_recv_info_t *recv_info, const message_t *msg);

// Pairing management functions
static void start_pairing(void);
static void complete_pairing(const uint8_t *mac_addr);
static void unpair_device(void);
static esp_err_t load_peer_mac(void);
static esp_err_t save_peer_mac(const uint8_t *mac_addr);

// Peer management functions
static esp_err_t add_peer_if_needed(const uint8_t *mac_addr);
static esp_err_t remove_peer_if_exists(const uint8_t *mac_addr);

// Message creation functions
static message_t create_pairing_request_message(void);
static message_t create_pairing_response_message(void);
static message_t create_button_press_message(uint16_t button_state);

// LED management functions
static void led_task(void *pvParameters);
static esp_err_t queue_led_command(uint8_t blink_count, uint16_t delay_ms, bool high_priority);

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
    if (recv_info->src_addr == NULL || data == NULL || len < sizeof(message_t)) {
        ESP_LOGE(TAG, "Receive CB error: Invalid data or insufficient length");
        return;
    }

    message_t *msg = (message_t*)data;

    // Route message based on pairing status
    if (!s_is_paired) {
        process_pairing_message(recv_info, msg);
    } else {
        process_operational_message(recv_info, msg);
    }
}

// --- Message Processing Functions ---

// Processes messages when device is not paired (pairing mode)
static void process_pairing_message(const esp_now_recv_info_t *recv_info, const message_t *msg) {
    switch (msg->command_type) {
        case CMD_PAIRING_REQUEST:
            handle_pairing_request(recv_info, msg);
            break;
        case CMD_PAIRING_RESPONSE:
            handle_pairing_response(recv_info, msg);
            break;
        default:
            ESP_LOGD(TAG, "Ignoring non-pairing message while unpaired: 0x%04X", msg->command_type);
            break;
    }
}

// Processes messages when device is paired (operational mode)
static void process_operational_message(const esp_now_recv_info_t *recv_info, const message_t *msg) {
    // Only accept messages from the known peer
    if (memcmp(recv_info->src_addr, s_peer_mac, ESP_NOW_ETH_ALEN) != 0) {
        ESP_LOGW(TAG, "Ignoring message from unknown device " MACSTR " (not paired peer)", 
                 MAC2STR(recv_info->src_addr));
        return;
    }

    switch (msg->command_type) {
        case CMD_BUTTON_PRESS:
            handle_incoming_button_press(recv_info, msg);
            break;
        default:
            ESP_LOGW(TAG, "Unknown operational command: 0x%04X", msg->command_type);
            break;
    }
}

// Handles incoming pairing requests
static void handle_pairing_request(const esp_now_recv_info_t *recv_info, const message_t *msg) {
    // Verify pairing code
    if (msg->payload != PAIRING_CODE) {
        ESP_LOGW(TAG, "Invalid pairing code from " MACSTR " (expected: 0x%04X, got: 0x%04X)",
                 MAC2STR(recv_info->src_addr), PAIRING_CODE, msg->payload);
        return;
    }

    // Check RSSI to ensure the device is nearby
    if (recv_info->rx_ctrl->rssi <= PAIRING_RSSI_THRESHOLD) {
        ESP_LOGW(TAG, "Ignoring pairing request from " MACSTR " due to weak signal (RSSI: %d)",
                 MAC2STR(recv_info->src_addr), recv_info->rx_ctrl->rssi);
        return;
    }

    ESP_LOGI(TAG, "Valid pairing request received from " MACSTR " with RSSI: %d",
             MAC2STR(recv_info->src_addr), recv_info->rx_ctrl->rssi);

    // Add peer temporarily for response
    if (add_peer_if_needed(recv_info->src_addr) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add peer for pairing response");
        return;
    }

    // Send pairing response
    message_t response_msg = create_pairing_response_message();
    esp_err_t send_result = esp_now_send(recv_info->src_addr, (uint8_t*)&response_msg, sizeof(response_msg));
    if (send_result != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send pairing response: %s", esp_err_to_name(send_result));
        return;
    }

    // Complete pairing
    complete_pairing(recv_info->src_addr);
}

// Handles incoming pairing responses
static void handle_pairing_response(const esp_now_recv_info_t *recv_info, const message_t *msg) {
    // Verify pairing code in response
    if (msg->payload != PAIRING_CODE) { 
        ESP_LOGW(TAG, "Invalid pairing response code from " MACSTR " (expected: 0x%04X, got: 0x%04X)",
                 MAC2STR(recv_info->src_addr), PAIRING_CODE, msg->payload);
        return;
    }

    ESP_LOGI(TAG, "Valid pairing response received from " MACSTR,
             MAC2STR(recv_info->src_addr));
    complete_pairing(recv_info->src_addr);
}

// Handles incoming button press commands
static void handle_incoming_button_press(const esp_now_recv_info_t *recv_info, const message_t *msg) {
    ESP_LOGI(TAG, "Button press command received from " MACSTR " (state: %s)", 
             MAC2STR(recv_info->src_addr),
             msg->payload == BUTTON_STATE_PRESSED ? "PRESSED" : "RELEASED");
    
    if (msg->payload == BUTTON_STATE_PRESSED) {
        gpio_set_level(OUTPUT_PIN_GPIO, OUTPUT_PIN_ACTIVE_LEVEL);
        xTimerReset(s_output_pin_timer_handle, portMAX_DELAY);
    }
    queue_led_command(1, 100, true); // High priority for button press indication
}

// --- Peer Management Functions ---

static esp_err_t add_peer_if_needed(const uint8_t *mac_addr) {
    if (esp_now_is_peer_exist(mac_addr)) {
        return ESP_OK; // already exists
    }
    esp_now_peer_info_t peer_info = {};
    memcpy(peer_info.peer_addr, mac_addr, ESP_NOW_ETH_ALEN);
    
    esp_err_t result = esp_now_add_peer(&peer_info);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add peer " MACSTR ": %s", 
                 MAC2STR(mac_addr), esp_err_to_name(result));
    }
    
    return result;
}

static esp_err_t remove_peer_if_exists(const uint8_t *mac_addr) {
    if (!esp_now_is_peer_exist(mac_addr)) {
        return ESP_OK; // nothing to remove
    }
    esp_err_t result = esp_now_del_peer(mac_addr);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Failed to remove peer " MACSTR ": %s", 
                 MAC2STR(mac_addr), esp_err_to_name(result));
    }
    
    return result;
}

// --- Pairing and NVS ---

static void start_pairing(void) {
    s_is_paired = false;
    s_is_pairing = true;
    ESP_LOGI(TAG, "Starting pairing process...");
    
    // Add the broadcast address to ESP-NOW peer list for sending
    if (add_peer_if_needed(s_broadcast_mac) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add broadcast peer for pairing");
        return;
    }
    
    // Send initial pairing request
    message_t pairing_msg = create_pairing_request_message();
    esp_now_send(s_broadcast_mac, (uint8_t*)&pairing_msg, sizeof(pairing_msg));
    
    // Start the periodic pairing broadcast timer
    if (s_pairing_broadcast_timer_handle != NULL) {
        xTimerStart(s_pairing_broadcast_timer_handle, portMAX_DELAY);
    }
}

static void complete_pairing(const uint8_t *mac_addr) {
    memcpy(s_peer_mac, mac_addr, ESP_NOW_ETH_ALEN);
    s_is_paired = true;
    s_is_pairing = false;

    if (s_pairing_broadcast_timer_handle != NULL) {
        xTimerStop(s_pairing_broadcast_timer_handle, portMAX_DELAY);
    }

    ESP_LOGI(TAG, "Pairing complete with " MACSTR, MAC2STR(s_peer_mac));

    queue_led_command(2, 100, true); // High priority for pairing success

    if (add_peer_if_needed(s_peer_mac) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add paired device to peer list");
        return;
    }

    // Save the peer's MAC to NVS
    save_peer_mac(s_peer_mac);
}

// Unpairs the device by clearing the peer MAC from NVS and RAM
static void unpair_device(void) {
    ESP_LOGI(TAG, "Unpairing device...");
    
    // Stop pairing broadcast timer if running
    s_is_pairing = false;
    if (s_pairing_broadcast_timer_handle != NULL) {
        xTimerStop(s_pairing_broadcast_timer_handle, portMAX_DELAY);
    }
    
    // Delete from NVS
    nvs_handle_t nvs_handle;
    ESP_ERROR_CHECK(nvs_open("storage", NVS_READWRITE, &nvs_handle));
    nvs_erase_key(nvs_handle, NVS_PEER_MAC_KEY);
    nvs_commit(nvs_handle);
    nvs_close(nvs_handle);

    // Remove peer from ESP-NOW peer list
    if (s_is_paired) {
        remove_peer_if_exists(s_peer_mac);
    }

    // Clear from RAM
    memset(s_peer_mac, 0, ESP_NOW_ETH_ALEN);
    s_is_paired = false;

    ESP_LOGI(TAG, "Device unpaired. Restarting pairing process.");
    queue_led_command(5, 150, true); // High priority for unpair indication
    
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

// Creates a pairing request message with the verification code
static message_t create_pairing_request_message(void) {
    message_t msg = {
        .command_type = CMD_PAIRING_REQUEST,
        .payload = PAIRING_CODE
    };
    return msg;
}

// Creates a pairing response message with the verification code
static message_t create_pairing_response_message(void) {
    message_t msg = {
        .command_type = CMD_PAIRING_RESPONSE,
        .payload = PAIRING_CODE
    };
    return msg;
}

// Creates a button press message with the specified button state
static message_t create_button_press_message(uint16_t button_state) {
    message_t msg = {
        .command_type = CMD_BUTTON_PRESS,
        .payload = button_state
    };
    return msg;
}


// --- Status Indication ---

// LED task that processes LED commands from the queue
static void led_task(void *pvParameters) {
    led_command_t led_cmd;
    
    while (1) {
        // Wait for a command in the queue
        if (xQueueReceive(s_led_queue, &led_cmd, portMAX_DELAY) == pdTRUE) {
            ESP_LOGD(TAG, "LED task: Processing %d blinks with %d ms delay (priority: %s)", 
                     led_cmd.blink_count, led_cmd.delay_ms, led_cmd.high_priority ? "HIGH" : "LOW");
            
            // Execute the blink pattern
            for (int i = 0; i < led_cmd.blink_count; i++) {
                gpio_set_level(OUTPUT_LED_GPIO, 1);
                vTaskDelay(pdMS_TO_TICKS(led_cmd.delay_ms));
                gpio_set_level(OUTPUT_LED_GPIO, 0);
                if (i < led_cmd.blink_count - 1) { // Don't delay after the last blink
                    vTaskDelay(pdMS_TO_TICKS(led_cmd.delay_ms));
                }
            }
        }
    }
}

// adds a command to the LED queue with the position based on priority
static esp_err_t queue_led_command(uint8_t blink_count, uint16_t delay_ms, bool high_priority) {
    if (s_led_queue == NULL) {
        ESP_LOGE(TAG, "LED queue not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    led_command_t led_cmd = {
        .blink_count = blink_count,
        .delay_ms = delay_ms,
        .high_priority = high_priority
    };
    
    BaseType_t result;
    if (high_priority) { // Send to front of queue for high priority commands
        result = xQueueSendToFront(s_led_queue, &led_cmd, 0);
    } else {  // Send to back of queue for normal priority commands
        result = xQueueSendToBack(s_led_queue, &led_cmd, 0);
    }
    
    if (result != pdTRUE) {
        ESP_LOGW(TAG, "Failed to queue LED command (queue full)");
        return ESP_ERR_NO_MEM;
    }
    
    return ESP_OK;
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

// Timer callback to turn off the output pin
static void output_pin_off_timer_cb(TimerHandle_t xTimer) {
    gpio_set_level(OUTPUT_PIN_GPIO, !OUTPUT_PIN_ACTIVE_LEVEL);
    ESP_LOGI(TAG, "Output pin turned off");
}

// Timer callback for pairing broadcast and LED blink animation
static void pairing_broadcast_timer_cb(TimerHandle_t xTimer) {
    if (s_is_pairing) {
        ESP_LOGI(TAG, "Sending pairing request broadcast");
        
        message_t pairing_msg = create_pairing_request_message();
        esp_now_send(s_broadcast_mac, (uint8_t*)&pairing_msg, sizeof(pairing_msg));
        
        queue_led_command(1, 1000, false); // Low priority for pairing broadcast indication
    }
}

// Button press callbacks
static void button_single_click_cb(void *arg, void *usr_data) {
    queue_led_command(1, 50, true); // High priority for button press indication
    if (s_is_paired) {
        ESP_LOGI(TAG, "Button pressed. Sending command to peer.");
        message_t button_msg = create_button_press_message(BUTTON_STATE_PRESSED);
        esp_now_send(s_peer_mac, (uint8_t*)&button_msg, sizeof(button_msg));
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
    gpio_set_level(OUTPUT_PIN_GPIO, !OUTPUT_PIN_ACTIVE_LEVEL); // Start with pin off

    gpio_config_t led_conf = {};
    led_conf.pin_bit_mask = (1ULL << OUTPUT_LED_GPIO);
    led_conf.mode = GPIO_MODE_OUTPUT;
    led_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&led_conf);
    gpio_set_level(OUTPUT_LED_GPIO, 0); // Start with LED off
    
    // Create LED queue and task for coordinating LED status indications
    s_led_queue = xQueueCreate(LED_QUEUE_SIZE, sizeof(led_command_t));
    if (s_led_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create LED queue");
        return;
    }
    
    BaseType_t task_result = xTaskCreate(
        led_task,                    // Task function
        "led_task",                  // Task name
        LED_TASK_STACK_SIZE,         // Stack size
        NULL,                        // Parameters
        LED_TASK_PRIORITY,           // Priority
        &s_led_task_handle           // Task handle
    );
    
    if (task_result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create LED task");
        return;
    }
    
    // Create a one-shot timer for the output pin
    s_output_pin_timer_handle = xTimerCreate("pin_off_timer", pdMS_TO_TICKS(OUTPUT_PIN_ACTIVE_TIME_MS),
                                      pdFALSE, // One-shot timer
                                      (void *)0, output_pin_off_timer_cb);

    // Create a repeating timer for pairing broadcast and LED blink animation
    s_pairing_broadcast_timer_handle = xTimerCreate("pairing_broadcast_timer", pdMS_TO_TICKS(PAIRING_REQUEST_INTERVAL_MS),
                                                    pdTRUE, // Auto-reload timer (repeating)
                                                    (void *)0, pairing_broadcast_timer_cb);

    // Configure Button
    button_config_t btn_cfg = {
        .long_press_time = UNPAIR_PRESS_DURATION_MS,
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
        if (add_peer_if_needed(s_peer_mac) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to add stored peer to ESP-NOW");
            start_pairing();
        } else {
            ESP_LOGI(TAG, "Device is already paired. Ready to send/receive.");
        }
    } else {
        // If not found in NVS, start the pairing process
        ESP_LOGI(TAG, "No paired device found in NVS. Starting pairing.");
        start_pairing();
    }
}
