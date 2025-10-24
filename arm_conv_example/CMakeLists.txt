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
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2 CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(arm_conv_example C CXX ASM)


# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(arm_conv_example 
    arm_conv_example.c 
    ${USERHOME}/pico/CMSISDSP/CMSIS-DSP/Examples/ARM/arm_convolution_example/math_helper.c
    )

pico_set_program_name(arm_conv_example "arm_conv_example")
pico_set_program_version(arm_conv_example "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(arm_conv_example 1)
pico_enable_stdio_usb(arm_conv_example 0)

# Add the standard library to the build
target_link_libraries(arm_conv_example
        pico_stdlib
        ${USERHOME}/pico/CMSISDSP/build/bin_dsp/libCMSISDSP.a
        )

# Add the standard include files to the build
target_include_directories(arm_conv_example PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${USERHOME}/pico/CMSISDSP/CMSIS-DSP/Include
        ${USERHOME}/pico/CMSISDSP/CMSIS_6/CMSIS/Core/Include
        ${USERHOME}/pico/CMSISDSP/CMSIS-DSP/Examples/ARM/arm_convolution_example
)

target_compile_definitions(arm_conv_example PRIVATE
    PICO_DEFAULT_UART=0
    PICO_DEFAULT_UART_TX_PIN=12
    PICO_DEFAULT_UART_RX_PIN=13
)

pico_add_extra_outputs(arm_conv_example)

