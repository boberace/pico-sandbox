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

# cmake version
cmake_minimum_required(VERSION 3.13)

# include the sdk.cmake file
include(pico_sdk_import.cmake)

# give the project a name (anything you want)
project(vga16-graphics-project)

# initialize the sdk
pico_sdk_init()

#######

# name anything you want
add_executable(graphics_test)

# must match with pio filename and executable name from above
pico_generate_pio_header(graphics_test ${CMAKE_CURRENT_LIST_DIR}/hsync.pio)
pico_generate_pio_header(graphics_test ${CMAKE_CURRENT_LIST_DIR}/vsync.pio)
pico_generate_pio_header(graphics_test ${CMAKE_CURRENT_LIST_DIR}/rgb.pio)

# must match with executable name and source file names
target_sources(graphics_test PRIVATE 
	# vga16_graphics_demo.c 
	# vga_pt_spectrogram.c
	vga16_fft_ifft.c
	vga16_graphics.c)

# must match with executable name
target_link_libraries(graphics_test PRIVATE 
	pico_stdlib 
	pico_bootsel_via_double_reset 
	hardware_pio 
	hardware_dma 
	hardware_adc 
	hardware_sync
	hardware_irq
	hardware_spi
	hardware_timer
	pico_multicore)

target_compile_definitions(graphics_test PRIVATE
		PICO_DEFAULT_UART=0
		PICO_DEFAULT_UART_TX_PIN=12
		PICO_DEFAULT_UART_RX_PIN=13
		)

# must match with executable name
pico_add_extra_outputs(graphics_test)

add_compile_options(-Ofast)
