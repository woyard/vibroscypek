#include "led_control.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/gpio.h"

static const char *TAG = "LED_CONTROL";

// Module state
typedef struct {
    QueueHandle_t led_queue;
    TaskHandle_t led_task_handle;
    gpio_num_t led_gpio;
    bool initialized;
} led_control_state_t;

static led_control_state_t s_led_state = {
    .led_queue = NULL,
    .led_task_handle = NULL,
    .led_gpio = GPIO_NUM_NC,
    .initialized = false
};

// LED task that processes LED commands from the queue
static void led_task(void *pvParameters) {
    led_command_t led_cmd;
    
    while (1) {
        // Wait for a command in the queue
        if (xQueueReceive(s_led_state.led_queue, &led_cmd, portMAX_DELAY) == pdTRUE) {
            ESP_LOGD(TAG, "LED task: Processing %d blinks with %d ms delay (priority: %s)", 
                     led_cmd.blink_count, led_cmd.delay_ms, led_cmd.high_priority ? "HIGH" : "LOW");
            
            // Execute the blink pattern
            for (int i = 0; i < led_cmd.blink_count; i++) {
                gpio_set_level(s_led_state.led_gpio, 1);
                vTaskDelay(pdMS_TO_TICKS(led_cmd.delay_ms));
                gpio_set_level(s_led_state.led_gpio, 0);
                if (i < led_cmd.blink_count - 1) { // Don't delay after the last blink
                    vTaskDelay(pdMS_TO_TICKS(led_cmd.delay_ms));
                }
            }
        }
    }
}

// Public API: Queue a command to the LED controller
esp_err_t led_control_queue_command(uint8_t blink_count, uint16_t delay_ms, bool high_priority) {
    if (!s_led_state.initialized || s_led_state.led_queue == NULL) {
        ESP_LOGE(TAG, "LED controller not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    led_command_t led_cmd = {
        .blink_count = blink_count,
        .delay_ms = delay_ms,
        .high_priority = high_priority
    };
    
    BaseType_t result;
    if (high_priority) { // Send to front of queue for high priority commands
        result = xQueueSendToFront(s_led_state.led_queue, &led_cmd, 0);
    } else {  // Send to back of queue for normal priority commands
        result = xQueueSendToBack(s_led_state.led_queue, &led_cmd, 0);
    }
    
    if (result != pdTRUE) {
        ESP_LOGW(TAG, "Failed to queue LED command (queue full)");
        return ESP_ERR_NO_MEM;
    }
    
    return ESP_OK;
}

// Public API: Initialize the LED controller
esp_err_t led_control_init(const led_config_t *config) {
    if (s_led_state.initialized) {
        ESP_LOGW(TAG, "LED controller already initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (config == NULL) {
        ESP_LOGE(TAG, "Invalid configuration");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Configure LED GPIO
    gpio_config_t led_conf = {
        .pin_bit_mask = (1ULL << config->led_gpio),
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE
    };
    
    esp_err_t ret = gpio_config(&led_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure LED GPIO: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Start with LED off
    gpio_set_level(config->led_gpio, 0);
    
    // Create LED queue
    s_led_state.led_queue = xQueueCreate(config->queue_size, sizeof(led_command_t));
    if (s_led_state.led_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create LED queue");
        return ESP_ERR_NO_MEM;
    }
    
    // Create LED task
    BaseType_t task_result = xTaskCreate(
        led_task,                    // Task function
        "led_task",                  // Task name
        config->task_stack_size,     // Stack size
        NULL,                        // Parameters
        config->task_priority,       // Priority
        &s_led_state.led_task_handle // Task handle
    );
    
    if (task_result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create LED task");
        vQueueDelete(s_led_state.led_queue);
        s_led_state.led_queue = NULL;
        return ESP_ERR_NO_MEM;
    }
    
    // Store configuration and mark as initialized
    s_led_state.led_gpio = config->led_gpio;
    s_led_state.initialized = true;
    
    ESP_LOGI(TAG, "LED controller initialized on GPIO %d", config->led_gpio);
    return ESP_OK;
}

// Public API: Deinitialize the LED controller
esp_err_t led_control_deinit(void) {
    if (!s_led_state.initialized) {
        ESP_LOGW(TAG, "LED controller not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Delete task
    if (s_led_state.led_task_handle != NULL) {
        vTaskDelete(s_led_state.led_task_handle);
        s_led_state.led_task_handle = NULL;
    }
    
    // Delete queue
    if (s_led_state.led_queue != NULL) {
        vQueueDelete(s_led_state.led_queue);
        s_led_state.led_queue = NULL;
    }
    
    // Turn off LED
    if (s_led_state.led_gpio != GPIO_NUM_NC) {
        gpio_set_level(s_led_state.led_gpio, 0);
    }
    
    // Reset state
    s_led_state.led_gpio = GPIO_NUM_NC;
    s_led_state.initialized = false;
    
    ESP_LOGI(TAG, "LED controller deinitialized");
    return ESP_OK;
}