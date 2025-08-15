from machine import ADC, Pin, PWM
import time
from umqtt.simple import MQTTClient
import network

ADAFRUIT_USERNAME = "iykyk_123"
ADAFRUIT_KEY = "update"
AIO_HOST  = "io.adafruit.com"
CLIENT_ID = "pico_" + str(int(time.time()*1000))         

alarm_on = bytes(f"{ADAFRUIT_USERNAME}/feeds/Alarm_ON", "utf-8")
alarm_off = bytes(f"{ADAFRUIT_USERNAME}/feeds/Alarm_OFF", "utf-8")
#SSID = "lnuiot"
#PASSWORD = "modermodemet"
SSID = "wifissid"
PASSWORD = "wifipassword"
threshold = 49000

hall = ADC(26)
button = Pin(0, Pin.IN, Pin.PULL_UP)
buzzer = PWM(Pin(16))



wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect(SSID, PASSWORD)
print("Connecting to WIFI...")
while not wlan.isconnected():
    time.sleep(0.2)
print("Wi-Fi: ", wlan.ifconfig()[0])

def mqtt_callback(topic, msg):
    print(f"MQTT: {topic.decode()}: {msg.decode()}")    
    
client = MQTTClient(client_id=CLIENT_ID,
                    server=AIO_HOST,
                    user=ADAFRUIT_USERNAME,
                    password=ADAFRUIT_KEY,
                    keepalive=30
                    )
client.set_callback(mqtt_callback)
client.connect()
#client.subscribe(FEED_CMD)
#print("MQTT connected, subscribed to", FEED_CMD.decode())

armed = False
armed_delay = 5

print(f"Arming the alarm in: {armed_delay} s:")
for i in range(armed_delay, -1, -1):
    print(i, end=", ")
    time.sleep(1)

# only arm if the magnet is actually sensed
if hall.read_u16() < threshold:
    armed = True
    print("Alarm is now ARMED")
else:
    print("Magnet not detected — alarm remains DISARMED")

    
while True:
    hall_val = hall.read_u16()
    magnet_near = (hall_val > threshold)
    
    if button.value() == 0:
        if armed:
            print("Manual disarmed")
            buzzer.duty_u16(0)
            client.publish(alarm_off, "Alarm disarmed")
            armed = False
        time.sleep(0.5)
        continue
    elif armed and magnet_near:
        print("magnet near - alarmed triggered ")
        buzzer.freq(1500)
        buzzer.duty_u16(45000)
        try:
            client.publish(alarm_on, "Alarm Triggered: Intruder alert!")
        except Exception as e:
            print("MQTT error duing alert:", e)
    time.sleep(0.5)
    continue

    #otherwise be silent
    buzzer.duty_u16(0)
    time.sleep(0.2)



