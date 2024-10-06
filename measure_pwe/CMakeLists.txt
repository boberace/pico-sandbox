# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NEVER EDIT THE NEXT LINES for Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.0.0)
set(toolchainVersion 13_2_Rel1)
set(picotoolVersion 2.0.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(measure_pwe C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(measure_pwe measure_pwe.c )

pico_set_program_name(measure_pwe "measure_pwe")
pico_set_program_version(measure_pwe "0.1")

# Generate PIO header
pico_generate_pio_header(measure_pwe ${CMAKE_CURRENT_LIST_DIR}/measure_pwe.pio)
pico_generate_pio_header(measure_pwe ${CMAKE_CURRENT_LIST_DIR}/ws281x.pio)

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(measure_pwe 1)
pico_enable_stdio_usb(measure_pwe 0)

# Add the standard library to the build
target_link_libraries(measure_pwe
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(measure_pwe PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts or any other standard includes, if required
)

# Add any user requested libraries
target_link_libraries(measure_pwe 
        hardware_dma
        hardware_pio
        )

pico_add_extra_outputs(measure_pwe)

