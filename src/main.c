// main.c
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "pico/cyw43_arch.h"

#include "lwip/dns.h"
#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"

#define WIFI_SSID          "the wifi ssid"
#define WIFI_PASS          "the wifi password"

#define AIO_USERNAME       "YOUR_ADAFRUIT_USERNAME"             // e.g., "iykyk_123"
#define AIO_KEY            "YOUR_ADAFRUIT_AIO_KEY"              // e.g., "aio_xxx"
#define AIO_HOST           "io.adafruit.com"
#define AIO_PORT           1883

// Feeds: <username>/feeds/Alarm_ON and Alarm_OFF
static char FEED_ALARM_ON[64];
static char FEED_ALARM_OFF[64];

// PINS 
#define HALL_GPIO          26    // ADC0
#define BUTTON_GPIO        0     // Pull-up input, active-low (pressed == 0)
#define BUZZER_GPIO        16    // PWM output

// TIMING / LOGIC
#define ARMING_DELAY_SEC   5
// MicroPython used a 16-bit ADC reading (0..65535) with threshold=49000.
// Pico SDK adc_read() returns 12-bit (0..4095). Scale 49000 >> 4 ≈ 3062.
#define HALL_THRESHOLD_12B 3062
#define BUZZER_FREQ_HZ     1500
#define BUZZER_DUTY_16B    45000

// ---------- Globals ----------
static mqtt_client_t *g_mqtt = NULL;
static volatile bool g_mqtt_connected = false;

// ---------- Helpers: Buzzer ----------
static uint buzzer_slice;

static void buzzer_init(uint gpio) {
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    buzzer_slice = pwm_gpio_to_slice_num(gpio);

    // Set TOP to 65535 so “duty” is 16-bit-like
    pwm_set_wrap(buzzer_slice, 65535);

    // Compute clkdiv so freq ~= BUZZER_FREQ_HZ
    // f_pwm = 125e6 / (clkdiv * (TOP+1)) -> clkdiv = 125e6 / (f_pwm * 65536)
    float clkdiv = 125000000.0f / (BUZZER_FREQ_HZ * 65536.0f);
    pwm_set_clkdiv(buzzer_slice, clkdiv);

    pwm_set_chan_level(buzzer_slice, pwm_gpio_to_channel(gpio), 0);
    pwm_set_enabled(buzzer_slice, true);
}

static void buzzer_on(void) {
    pwm_set_chan_level(buzzer_slice, pwm_gpio_to_channel(BUZZER_GPIO), BUZZER_DUTY_16B);
}

static void buzzer_off(void) {
    pwm_set_chan_level(buzzer_slice, pwm_gpio_to_channel(BUZZER_GPIO), 0);
}

// ---------- MQTT callbacks ----------
static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) {
    if (status == MQTT_CONNECT_ACCEPTED) {
        g_mqtt_connected = true;
        printf("MQTT: connected\n");
    } else {
        g_mqtt_connected = false;
        printf("MQTT: connection lost (status=%d)\n", status);
    }
}

static void mqtt_pub(const char *topic, const char *msg) {
    if (!g_mqtt || !g_mqtt_connected) return;
    mqtt_publish(g_mqtt, topic, msg, (u16_t)strlen(msg), 0, 0, NULL, NULL);
}

// ---------- DNS + Connect ----------
static ip_addr_t aio_addr;
static volatile bool dns_done = false;
static volatile bool dns_ok = false;

static void dns_found_cb(const char *name, const ip_addr_t *ipaddr, void *callback_arg) {
    if (ipaddr) {
        aio_addr = *ipaddr;
        dns_ok = true;
    }
    dns_done = true;
}

static void mqtt_connect_now(void) {
    mqtt_connect_client_info_t ci = {0};
    ci.client_id   = "pico_client";   // You can randomize this if you like
    ci.keep_alive  = 30;
    ci.client_user = AIO_USERNAME;
    ci.client_pass = AIO_KEY;

    g_mqtt = mqtt_client_new();
    if (!g_mqtt) {
        printf("MQTT: failed to allocate client\n");
        return;
    }
    mqtt_client_connect(g_mqtt, &aio_addr, AIO_PORT, mqtt_connection_cb, NULL, &ci);
}

// ---------- Hall Sensor ----------
static inline uint16_t hall_read_12b(void) {
    // ADC0 -> GPIO26
    adc_select_input(0);
    return adc_read();  // 0..4095
}

int main() {
    stdio_init_all();
    sleep_ms(300);

    // Build feed topics once
    snprintf(FEED_ALARM_ON,  sizeof(FEED_ALARM_ON),  "%s/feeds/Alarm_ON",  AIO_USERNAME);
    snprintf(FEED_ALARM_OFF, sizeof(FEED_ALARM_OFF), "%s/feeds/Alarm_OFF", AIO_USERNAME);

    // Init ADC (Hall sensor on GPIO26 -> ADC0)
    adc_init();
    adc_gpio_init(HALL_GPIO);

    // Init button (pull-up, active-low)
    gpio_init(BUTTON_GPIO);
    gpio_set_dir(BUTTON_GPIO, GPIO_IN);
    gpio_pull_up(BUTTON_GPIO);

    // Init buzzer (PWM)
    buzzer_init(BUZZER_GPIO);
    buzzer_off();

    // Init Wi-Fi (use the *threadsafe background* variant in CMake)
    if (cyw43_arch_init()) {
        printf("CYW43 init failed\n");
        return -1;
    }
    cyw43_arch_enable_sta_mode();

    printf("Connecting to Wi-Fi...\n");
    int rc = cyw43_arch_wifi_connect_timeout_ms(
        WIFI_SSID, WIFI_PASS, CYW43_AUTH_WPA2_AES_PSK, 30000
    );
    if (rc) {
        printf("Wi-Fi connect failed: %d\n", rc);
        return -1;
    }
    printf("Wi-Fi connected\n");

    // Resolve io.adafruit.com
    err_t derr = dns_gethostbyname(AIO_HOST, &aio_addr, dns_found_cb, NULL);
    if (derr == ERR_OK) {
        dns_ok = true; dns_done = true;
    }
    while (!dns_done) sleep_ms(50);
    if (!dns_ok) {
        printf("DNS failed for %s\n", AIO_HOST);
        return -1;
    }

    // Connect MQTT
    mqtt_connect_now();

    // Arming delay
    bool armed = false;
    printf("Arming the alarm in: %d s:\n", ARMING_DELAY_SEC);
    for (int i = ARMING_DELAY_SEC; i >= 0; --i) {
        printf("%d ", i); fflush(stdout);
        sleep_ms(1000);
    }
    printf("\n");

    // Only arm if initial reading is below threshold (mirrors your Python)
    uint16_t initial = hall_read_12b();
    if (initial < HALL_THRESHOLD_12B) {
        armed = true;
        printf("Alarm is now ARMED\n");
    } else {
        printf("Magnet not detected — alarm remains DISARMED\n");
    }

    // Main loop
    while (true) {
        // Button pressed?
        bool pressed = (gpio_get(BUTTON_GPIO) == 0);

        // Read hall sensor
        uint16_t hall_val = hall_read_12b();
        bool magnet_near = (hall_val > HALL_THRESHOLD_12B);

        if (pressed) {
            if (armed) {
                printf("Manual disarmed\n");
                buzzer_off();
                mqtt_pub(FEED_ALARM_OFF, "Alarm disarmed");
                armed = false;
            }
            sleep_ms(500); // crude debounce
            continue;
        } else if (armed && magnet_near) {
            printf("Magnet near - alarm triggered (val=%u)\n", hall_val);
            buzzer_on();
            mqtt_pub(FEED_ALARM_ON, "Alarm Triggered: Intruder alert!");
        } else {
            // Be silent
            buzzer_off();
        }

        sleep_ms(500);
    }

    // Not reached
    // cyw43_arch_deinit();
    // return 0;
}
