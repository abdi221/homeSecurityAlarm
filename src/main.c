#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "pico/cyw43_arch.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#ifdef ENABLE_MQTT
  #include "lwip/dns.h"
  #include "lwip/ip_addr.h"
  #include "lwip/apps/mqtt.h"
#endif

// -------- user config (no secrets here) --------
#define WIFI_SSID    "YOUR_WIFI_SSID"
#define WIFI_PASS    "YOUR_WIFI_PASSWORD"
#define AIO_USERNAME "YOUR_ADAFRUIT_USERNAME"
#define AIO_KEY      "YOUR_ADAFRUIT_AIO_KEY"
#define AIO_HOST     "io.adafruit.com"
#define AIO_PORT     1883

#define HALL_GPIO    26   // ADC0
#define BUTTON_GPIO  0    // active-low with pull-up
#define BUZZER_GPIO  16

#define HALL_THRESHOLD_12B 3062     // ≈ 49000 >> 4
#define ARMING_DELAY_SEC   5
#define BUZZER_FREQ_HZ     1500
#define BUZZER_DUTY_16B    45000

// -------- topics --------
static char FEED_ALARM_ON[64];
static char FEED_ALARM_OFF[64];

// -------- buzzer --------
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
static inline void buzzer_on(void)  { pwm_set_chan_level(buzzer_slice, pwm_gpio_to_channel(BUZZER_GPIO), BUZZER_DUTY_16B); }
static inline void buzzer_off(void) { pwm_set_chan_level(buzzer_slice, pwm_gpio_to_channel(BUZZER_GPIO), 0); }

// -------- shared state --------
static volatile bool g_armed = false;
static volatile bool g_triggered = false;

// -------- publish queue --------
typedef struct {
  char topic[64];
  char payload[96];
} NetMsg;

static QueueHandle_t g_qNet = NULL;

#ifdef ENABLE_MQTT
// ---- MQTT state owned by TaskNet ----
static mqtt_client_t *g_mqtt = NULL;
static volatile bool g_mqtt_connected = false;
static ip_addr_t aio_addr;
static volatile bool dns_done = false, dns_ok = false;

static void mqtt_connection_cb(mqtt_client_t *c, void *arg, mqtt_connection_status_t st) {
  g_mqtt_connected = (st == MQTT_CONNECT_ACCEPTED);
  printf("MQTT: %s (status=%d)\n", g_mqtt_connected ? "connected" : "disconnected", st);
}

static void dns_found_cb(const char *name, const ip_addr_t *ipaddr, void *cbarg) {
  if (ipaddr) { aio_addr = *ipaddr; dns_ok = true; }
  dns_done = true;
}

static void mqtt_pub_real(const char *topic, const char *msg) {
  if (!g_mqtt || !g_mqtt_connected) { printf("MQTT drop: %s -> %s\n", topic, msg); return; }
  mqtt_publish(g_mqtt, topic, msg, (u16_t)strlen(msg), 0, 0, NULL, NULL);
}
#else
static void mqtt_pub_real(const char *topic, const char *msg) {
  printf("[SIM] would publish %s: %s\n", topic, msg);
}
#endif

// -------- tasks --------
static void TaskLog(void *arg) {
  (void)arg;
  for (;;) { printf("FreeRTOS is running\n"); vTaskDelay(pdMS_TO_TICKS(1000)); }
}

static void TaskButton(void *arg) {
  (void)arg;
  const TickType_t dt = pdMS_TO_TICKS(10);
  uint8_t stable = 1, cnt = 0;
  for (;;) {
    bool pressed = (gpio_get(BUTTON_GPIO) == 0);
    if (!pressed) { cnt = 0; stable = 1; }
    else if (cnt < 3) { cnt++; if (cnt == 3) stable = 0; } // ~30ms debounce

    if (!stable && pressed) {
      // disarm immediately
      if (g_armed) {
        g_armed = false; g_triggered = false; buzzer_off();
        if (g_qNet) {
          NetMsg m; snprintf(m.topic, sizeof m.topic, "%s/feeds/Alarm_OFF", AIO_USERNAME);
          snprintf(m.payload, sizeof m.payload, "Alarm disarmed");
          xQueueSend(g_qNet, &m, 0);
        }
        printf("Manual disarmed\n");
      }
      // wait until button released
      while (gpio_get(BUTTON_GPIO) == 0) vTaskDelay(dt);
      cnt = 0; stable = 1;
    }
    vTaskDelay(dt);
  }
}

static void TaskHall(void *arg) {
  (void)arg;
  // arming delay
  printf("Arming in %d s:\n", ARMING_DELAY_SEC);
  for (int i = ARMING_DELAY_SEC; i >= 0; --i) { printf("%d ", i); vTaskDelay(pdMS_TO_TICKS(1000)); }
  printf("\n");

  // Only arm if magnet initially “present” (your earlier logic)
  adc_select_input(0);
  uint16_t initial = adc_read();
  g_armed = (initial < HALL_THRESHOLD_12B);
  printf("Initial hall=%u ⇒ %s\n", initial, g_armed ? "ARMED" : "DISARMED");

  TickType_t last = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&last, pdMS_TO_TICKS(1));  // 1 kHz sampling
    adc_select_input(0);
    uint16_t v = adc_read();
    bool magnet_near = (v > HALL_THRESHOLD_12B);

    if (g_armed && magnet_near && !g_triggered) {
      g_triggered = true;
      buzzer_on();
      if (g_qNet) {
        NetMsg m; snprintf(m.topic, sizeof m.topic, "%s/feeds/Alarm_ON", AIO_USERNAME);
        snprintf(m.payload, sizeof m.payload, "Alarm Triggered: val=%u", v);
        xQueueSend(g_qNet, &m, 0);
      }
      printf("ALARM TRIGGERED (val=%u)\n", v);
    }
    if (!magnet_near && !g_armed) buzzer_off(); // stay silent when disarmed
  }
}

static void TaskNet(void *arg) {
  (void)arg;
  // Wi-Fi init
  if (cyw43_arch_init()) { printf("CYW43 init failed\n"); vTaskDelete(NULL); }
  cyw43_arch_enable_sta_mode();
  printf("Connecting Wi-Fi…\n");
  int rc = cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASS, CYW43_AUTH_WPA2_AES_PSK, 30000);
  if (rc) { printf("Wi-Fi connect failed: %d\n", rc); vTaskDelete(NULL); }
  printf("Wi-Fi connected\n");

#ifdef ENABLE_MQTT
  // DNS resolve
  err_t derr = dns_gethostbyname(AIO_HOST, &aio_addr, dns_found_cb, NULL);
  if (derr == ERR_OK) { dns_ok = true; dns_done = true; }
  while (!dns_done) vTaskDelay(pdMS_TO_TICKS(50));
  if (!dns_ok) { printf("DNS failed for %s\n", AIO_HOST); vTaskDelete(NULL); }

  // Connect MQTT
  struct mqtt_connect_client_info_t ci = {0};
  ci.client_id   = "pico_client";
  ci.keep_alive  = 30;
  ci.client_user = AIO_USERNAME;
  ci.client_pass = AIO_KEY;

  g_mqtt = mqtt_client_new();
  if (!g_mqtt) { printf("MQTT alloc failed\n"); vTaskDelete(NULL); }
  mqtt_client_connect(g_mqtt, &aio_addr, AIO_PORT, mqtt_connection_cb, NULL, &ci);
#endif

  // pump the queue and publish
  NetMsg m;
  for (;;) {
    if (xQueueReceive(g_qNet, &m, portMAX_DELAY) == pdPASS) {
      mqtt_pub_real(m.topic, m.payload);
    }
  }
}

// -------- main --------
int main(void) {
  stdio_init_all(); sleep_ms(1500);

  // build topics once
  snprintf(FEED_ALARM_ON,  sizeof FEED_ALARM_ON,  "%s/feeds/Alarm_ON",  AIO_USERNAME);
  snprintf(FEED_ALARM_OFF, sizeof FEED_ALARM_OFF, "%s/feeds/Alarm_OFF", AIO_USERNAME);

  // hardware init
  adc_init(); adc_gpio_init(HALL_GPIO);
  gpio_init(BUTTON_GPIO); gpio_set_dir(BUTTON_GPIO, GPIO_IN); gpio_pull_up(BUTTON_GPIO);
  buzzer_init(BUZZER_GPIO); buzzer_off();

  // queue
  g_qNet = xQueueCreate(8, sizeof(NetMsg));

  // tasks
  xTaskCreate(TaskLog,   "log",   1024, NULL, 1, NULL);
  xTaskCreate(TaskButton,"btn",   1024, NULL, 3, NULL);   // high prio
  xTaskCreate(TaskHall,  "hall",  1024, NULL, 3, NULL);   // high prio
  xTaskCreate(TaskNet,   "net",   2048, NULL, 1, NULL);   // low prio

  vTaskStartScheduler();
  while (1) {}
}
