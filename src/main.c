
#include <FreeRTOS.h>
#include <pico/cyw43_arch.h>
#include <pico/stdlib.h>
#include <task.h>

#include "display_run.h"
#include "temp_display_queue.h"

#define MAIN_LED_DELAY 800
#define LED_PIN 25
#define button 0
#define blink_ms
#define polls_ms

static void blink_task(void *pvParamters) {
    (void)pvParamters;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    
    for(;;) {
        gpio_put(LED_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(blink_ms));
        gpio_put(LED_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(blink_ms));

    }
}

static void button_task(void *vParameters) {
    (void) vParameters;
    gpio_init(button);
    gpio_set_dir(button, GPIO_IN);
    gpio_pull_up(button);

    for (;;) {
        if (!gpio_get(button)) {
            printf("freertos is running nigga");

            while (!gpio_get(button)) {
                vTaskDelay(pdMS_TO_TICKS(polls_ms));
            }
        }
        vTaskDelay(pdMS_TO_TICKS(polls_ms));
    }
}

// void led_task(void *pvParameters);

// void display_task(void *pvParameters);

// void start_tasks();

int main() {
   // Initialize USB serial for printf()
    stdio_init_all();
    printf("Starting FreeRTOS button/LED demo...\n");

    // Create the two tasks
    xTaskCreate(blink_task, "Blink",  256, NULL, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(button_task, "Button",256, NULL, tskIDLE_PRIORITY + 2, NULL);

    // Start the scheduler (does not return)
    vTaskStartScheduler();

    // Should never reach here
    for (;;) { tight_loop_contents(); }
}

// void start_tasks() {
//     // Create Your Task
//     xTaskCreate(
//         led_task,    // Task to be run
//         "LED_TASK",  // Name of the Task for debugging and managing its Task Handle
//         1024,        // Stack depth to be allocated for use with task's stack (see docs)
//         NULL,        // Arguments needed by the Task (NULL because we don't have any)
//         1,           // Task Priority - Higher the number the more priority [max is (configMAX_PRIORITIES - 1) provided in FreeRTOSConfig.h]
//         NULL         // Task Handle if available for managing the task
//     );

//     // Should start you scheduled Tasks (such as the LED_Task above)
//     vTaskStartScheduler();

//     while (true) {
//         // Your program should never get here
//     };
// }

// void display_task(void *pvParameters) {
//     run_display();
// }

// void led_task(void *pvParameters) {
//     bool is_connected = true;
//     if (cyw43_arch_init()) {
//         printf("WiFi init failed\n");
//         is_connected = false;
//     }

//     while (is_connected) {
//         cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
//         printf("LED tuned ON!\n");
//         vTaskDelay(MAIN_LED_DELAY);  // Delay by TICKS defined by FreeRTOS priorities
//         cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
//         printf("LED turned OFF\n");
//         vTaskDelay(MAIN_LED_DELAY);
//     }
// }
