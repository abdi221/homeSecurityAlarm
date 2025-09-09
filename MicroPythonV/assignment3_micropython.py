from machine import ADC, Pin, PWM
import time, network
from umqtt.simple import MQTTClient

# User config
ADAFRUIT_USERNAME = "iykyk_123"
ADAFRUIT_KEY = "aio_RvRc43b6oATYrQwMhjA5AHYcRx46"
AIO_HOST = "io.adafruit.com"

SSID     = "Tele2_60aebd"
PASSWORD = "rhyk34zc"

# Sensor/logic
TH_ON_12B   = 3990      # trigger when raw12 >= this (near saturation)
K_CONSEC    = 5         # require K consecutive samples (≈5 ms at 1 kHz)
ARM_DELAY_S = 5
BUZZ_FREQ   = 1500
BUZZ_DUTY16 = 45000

# Pins
ADC_PIN    = 26         # ADC0
BUTTON_PIN = 0          # active-low with pull-up
BUZZ_PIN   = 16

# Hardware
hall   = ADC(ADC_PIN)
button = Pin(BUTTON_PIN, Pin.IN, Pin.PULL_UP)
buzzer = PWM(Pin(BUZZ_PIN))
buzzer.freq(BUZZ_FREQ)
buzzer.duty_u16(0)      # silent

# MQTT topics
FEED_ALARM_ON  = bytes(f"{ADAFRUIT_USERNAME}/feeds/Alarm_ON",  "utf-8")
FEED_ALARM_OFF = bytes(f"{ADAFRUIT_USERNAME}/feeds/Alarm_OFF", "utf-8")

# Wi-Fi
wlan = network.WLAN(network.STA_IF)
wlan.active(True)
print("Connecting to Wi-Fi…")
wlan.connect(SSID, PASSWORD)
while not wlan.isconnected():
    time.sleep(0.2)
print("Wi-Fi:", wlan.ifconfig()[0])

# MQTT
CLIENT_ID = f"pico_{int(time.time()*1000)}"
client = MQTTClient(client_id=CLIENT_ID,
                    server=AIO_HOST,
                    user=ADAFRUIT_USERNAME,
                    password=ADAFRUIT_KEY,
                    keepalive=30)
client.connect()
print("MQTT connected")

def publish(topic_b, msg_s):
    try:
        client.publish(topic_b, msg_s.encode())
    except Exception as e:
        print("MQTT publish error:", e)

# ---- ONE demo burst function (kept) ----
def blocking_publish_burst(duration_ms=300, mode="cpu"):
    t0 = time.ticks_ms()
    print("BURST start", t0, f"mode={mode}, dur={duration_ms}ms")
    if mode == "sleep":
        # simplest: block the loop with a single sleep
        time.sleep_ms(duration_ms)
    else:
        # cpu: busy work loop for ~duration_ms without network
        deadline = time.ticks_add(t0, duration_ms)
        x = 0x12345678
        while time.ticks_diff(deadline, time.ticks_ms()) > 0:
            # cheap integer churn; no allocations
            x = (x * 1664525 + 1013904223) & 0xFFFFFFFF
            x ^= (x << 13) & 0xFFFFFFFF
            x ^= (x >> 17)
            x ^= (x << 5) & 0xFFFFFFFF
    t1 = time.ticks_ms()
    print("BURST end", t1, "Δ=", time.ticks_diff(t1, t0), "ms")

# Helpers
def read_raw12():
    s1 = hall.read_u16() >> 4
    s2 = hall.read_u16() >> 4
    s3 = hall.read_u16() >> 4
    a, b, c = s1, s2, s3
    if a > b: a, b = b, a
    if b > c: b, c = c, b
    if a > b: a, b = b, a
    return b

def buzzer_on():
    buzzer.freq(BUZZ_FREQ)
    buzzer.duty_u16(BUZZ_DUTY16)

def buzzer_off():
    buzzer.duty_u16(0)

# Arming delay
print(f"Arming in {ARM_DELAY_S}s:")
for i in range(ARM_DELAY_S, -1, -1):
    print(i, end=" ")
    time.sleep(1)
print()

# Guard against “arm-hot”
initial = read_raw12()
armed = initial < (TH_ON_12B - 50)
triggered = False
print(f"Initial raw12={initial}  -> {'ARMED' if armed else 'DISARMED'}  (TH={TH_ON_12B})")

# Main loop state
above = 0
next_us   = time.ticks_add(time.ticks_us(), 1000)
last_dbg  = time.ticks_ms()
last_ping = time.ticks_ms()

# ---- define last_burst BEFORE using it ----
ENABLE_BURST_DEMO = True
BURST_PERIOD_MS   = 4000
BURST_DURATION_MS = 300
last_burst = time.ticks_ms()

while True:
    # 1) Intentionally block the loop with a periodic MQTT burst (for the video demo)
    if ENABLE_BURST_DEMO and time.ticks_diff(time.ticks_ms(), last_burst) >= BURST_PERIOD_MS:
        blocking_publish_burst(BURST_DURATION_MS)
        last_burst = time.ticks_ms()

    # 2) Keep MQTT alive
    if time.ticks_diff(time.ticks_ms(), last_ping) >= 20000:
        try:
            client.ping()
        except Exception as e:
            print("MQTT ping error:", e)
        last_ping = time.ticks_ms()

    # 3) Button: manual disarm
    if button.value() == 0:
        if armed:
            armed = False
            triggered = False
            buzzer_off()
            publish(FEED_ALARM_OFF, "Alarm Disarmed")
            print("Manual disarmed")
        time.sleep_ms(300)
        continue

    # 4) Sensor path (1 kHz)
    v12 = read_raw12()

    if not armed:
        buzzer_off()
    else:
        if not triggered:
            if v12 >= TH_ON_12B:
                above += 1
                if above >= K_CONSEC:
                    triggered = True
                    buzzer_on()
                    publish(FEED_ALARM_ON, "Alarm Triggered")
                    print(f"ALARM TRIGGERED  raw12={v12} >= {TH_ON_12B}")
                    above = 0
            else:
                above = 0
        # else: stay triggered until manual disarm

    # 5) Optional debug once per second
    if time.ticks_diff(time.ticks_ms(), last_dbg) >= 1000:
        print(f"raw12={v12}  armed={armed}  trig={triggered}")
        last_dbg = time.ticks_ms()

    # 6) Pace loop to ~1 kHz
    now = time.ticks_us()
    if time.ticks_diff(now, next_us) > 0:
        next_us = now
    next_us = time.ticks_add(next_us, 1000)
    dt = time.ticks_diff(next_us, time.ticks_us())
    if dt > 0:
        time.sleep_us(dt)

