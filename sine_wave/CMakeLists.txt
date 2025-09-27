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
set(picotoolVersion 2.2.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2 CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

# We also need PICO EXTRAS
include(pico_extras_import.cmake)

project(sine_wave_pwm C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# only build I2S example if library is available
if (TARGET pico_audio_i2s)
    add_executable(sine_wave_i2s
            sine_wave.c
            )

    target_link_libraries(sine_wave_i2s PRIVATE
            pico_stdlib
            pico_audio_i2s
            )

    target_compile_definitions(sine_wave_i2s PRIVATE
            # compile time configuration of I2S
            PICO_AUDIO_I2S_MONO_INPUT=0
            #define for our example code
            USE_AUDIO_I2S=1
            PICO_AUDIO_I2S_DATA_PIN=28
            PICO_AUDIO_I2S_CLOCK_PIN_BASE=26
            PICO_DEFAULT_UART=0
            PICO_DEFAULT_UART_TX_PIN=16
            PICO_DEFAULT_UART_RX_PIN=17
            )
    # create map/bin/hex file etc.
    pico_add_extra_outputs(sine_wave_i2s)
    pico_enable_stdio_uart(sine_wave_i2s 1)
    pico_enable_stdio_usb(sine_wave_i2s 1)

endif ()

# only build PWM example if library is available
if (TARGET pico_audio_pwm)
    add_executable(sine_wave_pwm
            sine_wave.c
            )

    target_link_libraries(sine_wave_pwm PRIVATE
            pico_stdlib
            pico_audio_pwm
            )

    target_compile_definitions(sine_wave_pwm PRIVATE
            #define for our example code
            USE_AUDIO_PWM=1

            PICO_DEFAULT_UART=0
            PICO_DEFAULT_UART_TX_PIN=16
            PICO_DEFAULT_UART_RX_PIN=17
            )
    # create map/bin/hex file etc.
    pico_add_extra_outputs(sine_wave_pwm)
    pico_enable_stdio_uart(sine_wave_pwm 1)
    pico_enable_stdio_usb(sine_wave_pwm 1)

endif ()

# only build S/PDIF example if library is available
if (TARGET pico_audio_spdif)
    add_executable(sine_wave_spdif
            sine_wave.c
            )

    target_link_libraries(sine_wave_spdif PRIVATE
            pico_stdlib
            pico_audio_spdif
            )

    target_compile_definitions(sine_wave_spdif PRIVATE
            # compile time configuration of S/PDIF
            PICO_AUDIO_SPDIF_MONO_INPUT=1
            #define for our example code
            USE_AUDIO_SPDIF=1
            )
    # create map/bin/hex file etc.
    pico_add_extra_outputs(sine_wave_spdif)
endif ()

