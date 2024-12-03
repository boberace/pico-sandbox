# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.0)
set(toolchainVersion 13_3_Rel1)
set(picotoolVersion 2.1.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2 CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(pio_ws2812 C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

file(MAKE_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/generated)

# add url via pico_set_program_url

add_executable(pio_ws2812_parallel)

pico_generate_pio_header(pio_ws2812_parallel ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio OUTPUT_DIR ${CMAKE_CURRENT_LIST_DIR}/generated)

target_sources(pio_ws2812_parallel PRIVATE ws2812_parallel.c)

target_compile_definitions(pio_ws2812_parallel PRIVATE
        PIN_DBG1=3)

target_link_libraries(pio_ws2812_parallel PRIVATE pico_stdlib hardware_pio hardware_dma pico_multicore)
pico_add_extra_outputs(pio_ws2812_parallel)

