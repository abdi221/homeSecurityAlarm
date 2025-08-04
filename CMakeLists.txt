cmake_minimum_required(VERSION 3.12)

# Change your executable name to something creative!
set(NAME home_security_sys) # <-- Name your project/executable here!

if(NOT DEFINED PICO_SDK_PATH)
  set(PICO_SDK_PATH "$ENV{HOME}/pico/pico-sdk"
  CACHE PATH "Path to the Raspberry Pi Pico SDK")
endif()

# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)

project(${NAME} C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(PICO_BOARD pico_w) # Set the board used


if(NOT DEFINED FREERTOS_KERNEL_PATH)
  set(FREERTOS_KERNEL_PATH
    "${CMAKE_CURRENT_LIST_DIR}/external/FreeRTOS-Kernel"
     CACHE PATH "Path to FreeRTOS Kernel source")
endif()

# Set config files paths to be used
include(lwip_import.cmake) # lwip config files

SET(FREERTOS_CONFIG_FILE_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/configs/FreeRTOS-Kernel" CACHE STRING "Local Config")
include_directories("${FREERTOS_CONFIG_FILE_DIRECTORY}") # FreeRTOS config files

# Initialize the SDK
pico_sdk_init()

# Initialize all other external libs & dependencies
include(FreeRTOS_Kernel_import.cmake)

add_subdirectory(src)
