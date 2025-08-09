add_executable(${NAME}  
        main.c
        )

# pull in common dependencies
target_link_libraries(${NAME}
    pico_stdlib
    hardware_adc
    hardware_pwm
    FreeRTOS-Kernel-Heap4
    pico_cyw43_arch_lwip_sys_freertos   # <- Wi-Fi+lwIP (FreeRTOS)
    pico_cyw43_driver
    pico_lwip_mqtt                      # <- brings lwip headers (dns.h, mqtt.h, ip_addr.h)
    pico_rand
    LWIP_PORT
    FREERTOS_PORT
)


        
# Enable print functionality on usb & disable on uart
pico_enable_stdio_usb(${NAME}  1) 
pico_enable_stdio_uart(${NAME}  0)

# create map/bin/hex file etc.
pico_add_extra_outputs(${NAME} )

target_include_directories(${NAME} PRIVATE
        ${PICO_SDK_PATH}/src/rp2_common/pico_cyw43_arch/include
        ${PICO_SDK_PATH}/src/rp2_common/pico_lwip/include
        ${PICO_SDK_PATH}/lib/lwip/src/include
        ${PICO_SDK_PATH}/lib/cyw43-driver/src
        ${PICO_SDK_PATH}/src/common/pico_rand/include
        )
