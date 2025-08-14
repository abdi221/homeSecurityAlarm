#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"

// Prints once per second so we can verify the scheduler is alive
static void vPrintTask(void *arg) {
    (void)arg;
    for (;;) {
        printf("FreeRTOS is running\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

int main(void) {
    stdio_init_all();
    // Give USB CDC a moment to enumerate before first print
    sleep_ms(1500);

    // Start one simple task
    xTaskCreate(
        vPrintTask,
        "PrintTask",
        1024,          // stack words (tweak if your config uses bytes)
        NULL,
        tskIDLE_PRIORITY + 1,
        NULL
    );

    // Hand control to the scheduler (never returns if ok)
    vTaskStartScheduler();

    // If we ever get here, there wasn't enough heap for the idle/timer tasks
    while (1) { /* trap */ }
}
