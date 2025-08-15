add_executable(${NAME}
    main.c
)

# Minimal set to boot RTOS and print over USB
target_link_libraries(${NAME}
    pico_stdlib
    FreeRTOS-Kernel-Heap4
    FREERTOS_PORT                              # your template’s FreeRTOS kernel
    pico_cyw43_arch_lwip_threadsafe_background # simpler arch; fine even if Wi-Fi unused
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
