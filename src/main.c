// main.c
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "pico/cyw43_arch.h"

#ifdef ENABLE_MQTT
  #include "lwip/dns.h"
  #include "lwip/ip_addr.h"
  #include "lwip/apps/mqtt.h"
#endif

#define WIFI_SSID    "the wifi ssid"
#define WIFI_PASS    "the wifi password"

#define AIO_USERNAME "YOUR_ADAFRUIT_USERNAME"
#define AIO_KEY      "YOUR_ADAFRUIT_AIO_KEY"
#define AIO_HOST     "io.adafruit.com"
#define AIO_PORT     1883

// Feeds: <username>/feeds/Alarm_ON and Alarm_OFF
static char FEED_ALARM_ON[64];
static char FEED_ALARM_OFF[64];

// PINS
#define HALL_GPIO   26
#define BUTTON_GPIO 0
#define BUZZER_GPIO 16

// TIMING / LOGIC
#define ARMING_DELAY_SEC   5
#define HALL_THRESHOLD_12B 3062
#define BUZZER_FREQ_HZ     1500
#define BUZZER_DUTY_16B    45000

#ifdef ENABLE_MQTT
static mqtt_client_t *g_mqtt = NULL;
static volatile bool g_mqtt_connected = false;
static ip_addr_t aio_addr;
static volatile bool dns_done = false;
static volatile bool dns_ok = false;
#endif

// ---------- Buzzer ----------
static uint buzzer_slice;
static void buzzer_init(uint gpio) {
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    buzzer_slice = pwm_gpio_to_slice_num(gpio);
    pwm_set_wrap(buzzer_slice, 65535);
    float clkdiv = 125000000.0f / (BUZZER_FREQ_HZ * 65536.0f);
    pwm_set_clkdiv(buzzer_slice, clkdiv);
    pwm_set_chan_level(buzzer_slice, pwm_gpio_to_channel(gpio), 0);
    pwm_set_enabled(buzzer_slice, true);
}
static void buzzer_on(void)  { pwm_set_chan_level(buzzer_slice, pwm_gpio_to_channel(BUZZER_GPIO), BUZZER_DUTY_16B); }
static void buzzer_off(void) { pwm_set_chan_level(buzzer_slice, pwm_gpio_to_channel(BUZZER_GPIO), 0); }

#ifdef ENABLE_MQTT
// ---------- MQTT callbacks ----------
static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) {
    g_mqtt_connected = (status == MQTT_CONNECT_ACCEPTED);
    printf("MQTT: %s (status=%d)\n", g_mqtt_connected ? "connected" : "disconnected", status);
}

static void dns_found_cb(const char *name, const ip_addr_t *ipaddr, void *callback_arg) {
    if (ipaddr) { aio_addr = *ipaddr; dns_ok = true; }
    dns_done = true;
}

// Real publish
static void mqtt_pub(const char *topic, const char *msg) {
    if (!g_mqtt || !g_mqtt_connected) { printf("MQTT: not connected, drop: %s -> %s\n", topic, msg); return; }
    mqtt_publish(g_mqtt, topic, msg, (u16_t)strlen(msg), 0, 0, NULL, NULL);
}

static void mqtt_connect_now(void) {
    struct mqtt_connect_client_info_t ci = {0};   // <-- use 'struct'
    ci.client_id   = "pico_client";
    ci.keep_alive  = 30;
    ci.client_user = AIO_USERNAME;
    ci.client_pass = AIO_KEY;

    g_mqtt = mqtt_client_new();
    if (!g_mqtt) { printf("MQTT: failed to allocate client\n"); return; }
    mqtt_client_connect(g_mqtt, &aio_addr, AIO_PORT, mqtt_connection_cb, NULL, &ci);
}
#else
// ---------- Stubs when MQTT is disabled ----------
static void mqtt_pub(const char *topic, const char *msg) {
    printf("[SIM] would publish %s: %s\n", topic, msg);
}
static void mqtt_connect_now(void) { /* no-op */ }
#endif

// ---------- Hall Sensor ----------
static inline uint16_t hall_read_12b(void) {
    adc_select_input(0);      // ADC0 -> GPIO26
    return adc_read();        // 0..4095
}

int main() {
    stdio_init_all();
    sleep_ms(300);

    snprintf(FEED_ALARM_ON,  sizeof(FEED_ALARM_ON),  "%s/feeds/Alarm_ON",  AIO_USERNAME);
    snprintf(FEED_ALARM_OFF, sizeof(FEED_ALARM_OFF), "%s/feeds/Alarm_OFF", AIO_USERNAME);

    adc_init();              adc_gpio_init(HALL_GPIO);
    gpio_init(BUTTON_GPIO);  gpio_set_dir(BUTTON_GPIO, GPIO_IN);  gpio_pull_up(BUTTON_GPIO);
    buzzer_init(BUZZER_GPIO); buzzer_off();

    if (cyw43_arch_init()) { printf("CYW43 init failed\n"); return -1; }
    cyw43_arch_enable_sta_mode();

    printf("Connecting to Wi-Fi...\n");
    int rc = cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASS, CYW43_AUTH_WPA2_AES_PSK, 30000);
    if (rc) { printf("Wi-Fi connect failed: %d\n", rc); return -1; }
    printf("Wi-Fi connected\n");

#ifdef ENABLE_MQTT
    // Resolve and connect MQTT only when enabled
    err_t derr = dns_gethostbyname(AIO_HOST, &aio_addr, dns_found_cb, NULL);
    if (derr == ERR_OK) { dns_ok = true; dns_done = true; }
    while (!dns_done) sleep_ms(50);
    if (!dns_ok) { printf("DNS failed for %s\n", AIO_HOST); }
    else { mqtt_connect_now(); }
#endif

    bool armed = false;
    printf("Arming the alarm in: %d s:\n", ARMING_DELAY_SEC);
    for (int i = ARMING_DELAY_SEC; i >= 0; --i) { printf("%d ", i); fflush(stdout); sleep_ms(1000); }
    printf("\n");

    uint16_t initial = hall_read_12b();
    if (initial < HALL_THRESHOLD_12B) { armed = true; printf("Alarm is now ARMED\n"); }
    else { printf("Magnet not detected — alarm remains DISARMED\n"); }

    while (true) {
        bool pressed = (gpio_get(BUTTON_GPIO) == 0);
        uint16_t hall_val = hall_read_12b();
        bool magnet_near = (hall_val > HALL_THRESHOLD_12B);

        if (pressed) {
            if (armed) {
                printf("Manual disarmed\n");
                buzzer_off();
                mqtt_pub(FEED_ALARM_OFF, "Alarm disarmed");
                armed = false;
            }
            sleep_ms(500);
            continue;
        } else if (armed && magnet_near) {
            printf("Magnet near - alarm triggered (val=%u)\n", hall_val);
            buzzer_on();
            mqtt_pub(FEED_ALARM_ON, "Alarm Triggered: Intruder alert!");
        } else {
            buzzer_off();
        }
        sleep_ms(500);
    }
}
