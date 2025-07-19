#pragma once
#include <esp_err.h>
#include <stdint.h>
#include <stdbool.h>
#include "driver/gpio.h"

// LED queue configuration
#define LED_QUEUE_SIZE          10
#define LED_TASK_STACK_SIZE     2048
#define LED_TASK_PRIORITY       3

// LED command structure for queue
typedef struct {
    uint8_t blink_count;
    uint16_t delay_ms;
    bool high_priority;  // If true, this command will be inserted at front of queue
} led_command_t;

// LED controller configuration
typedef struct {
    gpio_num_t led_gpio;        // GPIO pin for the LED
    uint8_t queue_size;         // Size of the command queue
    uint32_t task_stack_size;   // Stack size for LED task
    uint8_t task_priority;      // Priority of LED task
} led_config_t;

// Default configuration
#define LED_CONFIG_DEFAULT() { \
    .led_gpio = GPIO_NUM_8, \
    .queue_size = LED_QUEUE_SIZE, \
    .task_stack_size = LED_TASK_STACK_SIZE, \
    .task_priority = LED_TASK_PRIORITY \
}

// Public API functions
esp_err_t led_control_init(const led_config_t *config);
esp_err_t led_control_deinit(void);
esp_err_t led_control_queue_command(uint8_t blink_count, uint16_t delay_ms, bool high_priority);
