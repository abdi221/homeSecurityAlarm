Home Security (Pico W + FreeRTOS)

I built a small, real-time home-security simulation on a Raspberry Pi Pico W. It uses a Hall sensor (door/window), a button (disarm), and a buzzer (alarm). It can optionally publish Alarm_ON / Alarm_OFF to Adafruit IO over MQTT. The point is to show why an RTOS is helpful when I need fast reactions even while Wi-Fi/MQTT is busy.

⸻

What this repo is
	•	Hardware: Pico W, Hall sensor on ADC0 (GPIO26), button on GPIO0 (pull-up, active-low), buzzer on PWM (GPIO16).
	•	UI: optional Adafruit IO dashboard (two feeds: Alarm_ON, Alarm_OFF).
	•	Goal: buzzer reacts in ≲50 ms when the Hall threshold is crossed, and the button disarms instantly—even if Wi-Fi is working.

⸻

Quick start (SIM / no-MQTT build)

This build proves the scheduler and core logic without touching MQTT. It prints to USB and logs when it would publish.

# one-time SDK setup (adjust path if yours is different)
cd ~/pico/pico-sdk
git submodule update --init --recursive

# build the project
cd /path/to/PicoW-FreeRTOS-Template
rm -rf build
cmake -S . -B build -DPICO_BOARD=pico_w
cmake --build build -j

Flash the UF2:

# hold BOOTSEL while plugging in the Pico W
cp build/src/home_security_sys.uf2 /media/$USER/RPI-RP2/

Open the serial console:

minicom -o -D /dev/ttyACM0 -b 115200   # exit: Ctrl+A, X, Y
# or: screen /dev/ttyACM0 115200

You should see a heartbeat like: FreeRTOS is running.

⸻

Enable real MQTT

I keep MQTT behind a build flag so I can flip it on only when I’m ready.

rm -rf build
cmake -S . -B build -DPICO_BOARD=pico_w -DENABLE_MQTT=ON
cmake --build build -j
cp build/src/home_security_sys.uf2 /media/$USER/RPI-RP2/

Before doing that, I put real credentials in code (or better, in a local secrets.h that’s git-ignored):

#define WIFI_SSID    "YOUR_WIFI_SSID"
#define WIFI_PASS    "YOUR_WIFI_PASSWORD"
#define AIO_USERNAME "YOUR_ADAFRUIT_USERNAME"
#define AIO_KEY      "YOUR_ADAFRUIT_AIO_KEY"

Recommended pattern:
	•	commit src/secrets.example.h with placeholders
	•	add src/secrets.h to .gitignore and include it from main.c

On first boot with MQTT enabled, the log shows Wi-Fi connect → DNS → “MQTT: connected”. When the Hall sensor triggers I publish to <username>/feeds/Alarm_ON, and when I press the button I publish Alarm_OFF.

⸻

How I wired the build (CMake)

Top-level stays simple and in the right order:

set(PICO_BOARD pico_w)
include(pico_sdk_import.cmake)
include(lwip_import.cmake)
include(FreeRTOS_Kernel_import.cmake)

project(home_security_sys C CXX ASM)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
pico_sdk_init()
add_subdirectory(src)

App target links only what’s needed; I let libraries bring their own include paths:

add_executable(${NAME} main.c)

target_link_libraries(${NAME}
  pico_stdlib
  FreeRTOS-Kernel-Heap4
  FREERTOS_PORT
  pico_cyw43_arch_lwip_threadsafe_background
  hardware_adc
  hardware_pwm
  pico_rand
  LWIP_PORT
)

option(ENABLE_MQTT "Build with lwIP MQTT" OFF)
if(ENABLE_MQTT)
  target_link_libraries(${NAME} pico_lwip_mqtt)
  target_compile_definitions(${NAME} PRIVATE ENABLE_MQTT=1)
endif()

pico_enable_stdio_usb(${NAME} 1)
pico_enable_stdio_uart(${NAME} 0)
pico_add_extra_outputs(${NAME})

Why this works well: I don’t hardcode random include paths; I use the background lwIP arch during development (forgiving and stable), and I can switch to the FreeRTOS sys-arch later if I need it.

⸻

How the firmware behaves

I split work into small FreeRTOS tasks so timing is predictable:
	•	TaskHall (high prio): samples ADC at 1 kHz with vTaskDelayUntil(1 ms). If armed and threshold crossed, it turns the buzzer on right away and drops a message in a queue for networking.
	•	TaskButton (high prio): polls every ~10 ms with basic debounce. If pressed, it disarms immediately, stops the buzzer, and enqueues Alarm_OFF.
	•	TaskNet (low prio): owns Wi-Fi+MQTT. It connects once and then blocks on a queue and publishes messages. If Wi-Fi stalls, my sensor/button logic is unaffected.
	•	TaskLog (lowest): prints FreeRTOS is running every second so I know the scheduler’s alive.

This keeps the control path (sensor/button → buzzer) fast and non-blocking, while networking happens in the background.

⸻

What I expect to see
	•	SIM build:
FreeRTOS is running every second, and lines like [SIM] would publish Alarm_ON: … on trigger and Alarm_OFF when I press the button.
	•	MQTT build:
Wi-Fi connect messages, MQTT: connected, then publishes to the two Adafruit feeds. The buzzer still reacts instantly because I don’t block in the fast tasks.

⸻

Common gotchas I hit (and how I avoid them now)
	•	FreeRTOS.h not found → link FreeRTOS-Kernel-Heap4 (or FreeRTOS-Kernel) and keep FREERTOS_PORT.
	•	CYW43 / lwIP headers missing → link pico_cyw43_arch_lwip_threadsafe_background and import SDK/lwIP/FreeRTOS in the right order at the top-level.
	•	MQTT type mismatch → use struct mqtt_connect_client_info_t (the lwIP API wants the struct).
	•	USB prints don’t show → call stdio_init_all(); and give USB a little time: sleep_ms(1500);.
	•	Minicom opens /dev/modem → use uppercase -D: minicom -o -D /dev/ttyACM0 -b 115200.

If IntelliSense is noisy, I export compile commands (set(CMAKE_EXPORT_COMPILE_COMMANDS ON)) and point VS Code to build/compile_commands.json.

⸻

MicroPython vs RTOS (for my project write-up)

I can show a simple MicroPython/CircuitPython version that tries to read the sensor, sound the buzzer, and publish over Wi-Fi. Under load, the networking path tends to block enough that the buzzer reaction slips past the timing I want. The RTOS version keeps timing stable because the fast tasks never wait on network I/O—they just queue messages for TaskNet.

⸻

Handy commands I actually use

# clean build (SIM mode)
rm -rf build && cmake -S . -B build -DPICO_BOARD=pico_w && cmake --build build -j

# enable MQTT
rm -rf build && cmake -S . -B build -DPICO_BOARD=pico_w -DENABLE_MQTT=ON && cmake --build build -j

# flash (BOOTSEL)
cp build/src/home_security_sys.uf2 /media/$USER/RPI-RP2/

# serial
minicom -o -D /dev/ttyACM0 -b 115200

That’s it. This layout keeps things reliable while I iterate. If I ever need stricter control of lwIP’s threading later, I can swap to the pico_cyw43_arch_lwip_sys_freertos arch with a one-line change in CMake, and the task split still protects my timing.