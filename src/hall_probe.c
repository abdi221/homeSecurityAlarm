#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "FreeRTOS.h"
#include "task.h"

#define HALL_ADC_GPIO 26   // GPIO26 == ADC0

static uint16_t median3(uint16_t a, uint16_t b, uint16_t c) {
    if (a > b) { uint16_t t=a; a=b; b=t; }
    if (b > c) { uint16_t t=b; b=c; c=t; }
    if (a > b) { uint16_t t=a; a=b; b=t; }
    return b;
}

static void adc_task(void *arg) {
    (void)arg;

    stdio_init_all();
    adc_init();
    adc_gpio_init(HALL_ADC_GPIO);
    adc_select_input(0);

    vTaskDelay(pdMS_TO_TICKS(300)); // let USB come up

    float ema = 0.0f;
    bool ema_init = false;

    for (;;) {
        // Collect ~20 ms worth of data: 64 samples
        const int N = 64;
        uint32_t acc = 0;

        for (int i = 0; i < N; ++i) {
            // simple median-of-3 to suppress spikes
            uint16_t s1 = adc_read();
            uint16_t s2 = adc_read();
            uint16_t s3 = adc_read();
            uint16_t m = median3(s1, s2, s3);
            acc += m;
            // short pause spreads samples across mains noise; tweak if you like
            busy_wait_us_32(200); // ~0.2 ms
        }

        uint16_t raw = acc / N;         // averaged 12-bit
        float volts = raw * 3.3f / 4095.0f;

        // slow EMA for a buttery print
        if (!ema_init) { ema = volts; ema_init = true; }
        else           { ema = 0.85f * ema + 0.15f * volts; }

        printf("ADC0 raw=%u  V=%.3f  EMA=%.3f\r\n", raw, volts, ema);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

int main(void) {
    sleep_ms(200);
    xTaskCreate(adc_task, "adc", 1024, NULL, tskIDLE_PRIORITY + 1, NULL);
    vTaskStartScheduler();
    while (true) { __wfi(); }
}
