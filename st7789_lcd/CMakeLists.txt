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
# if (TARGET tinyusb_device)
    set(PICO_BOARD pico2 CACHE STRING "Board type")

    # Pull in Raspberry Pi Pico SDK (must be before project)
    include(pico_sdk_import.cmake)

    project(pio_st7789_lcd C CXX ASM)

    # Initialise the Raspberry Pi Pico SDK
    pico_sdk_init()

    # Add executable. Default name is the project name, version 0.1

    add_executable(${PROJECT_NAME} st7789_lcd.c )

    pico_set_program_name(${PROJECT_NAME} "${PROJECT_NAME}")
    pico_set_program_version(${PROJECT_NAME} "0.1")

    target_compile_definitions(${PROJECT_NAME} PRIVATE
        PICO_DEFAULT_UART=0
        PICO_DEFAULT_UART_TX_PIN=12
        PICO_DEFAULT_UART_RX_PIN=13
    )

    # Generate PIO header
    pico_generate_pio_header(${PROJECT_NAME} ${CMAKE_CURRENT_LIST_DIR}/st7789_lcd.pio)

    target_sources(${PROJECT_NAME} PRIVATE st7789_lcd.c)


    # Modify the below lines to enable/disable output over UART/USB
    pico_enable_stdio_uart(${PROJECT_NAME} 1)
    pico_enable_stdio_usb(${PROJECT_NAME} 0)

    # Add the standard library to the build
    target_link_libraries(${PROJECT_NAME} PRIVATE 
            pico_stdlib 
            hardware_pio 
            hardware_interp
            hardware_dma
            # tinyusb_device 
            # tinyusb_board
            )

    pico_add_extra_outputs(${PROJECT_NAME})

    add_compile_options(-Ofast)

# elseif(PICO_ON_DEVICE)
#     message("Skipping hello_usb because TinyUSB submodule is not initialized in the SDK")
# endif()
