cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD pico_w)

include(pico_sdk_import.cmake)

project(fancu)

pico_sdk_init()

add_executable(main main.c config.h lwipopts.h usb_descriptors.c usb_descriptors.h tusb_config.h)

pico_enable_stdio_usb(main 0)
pico_enable_stdio_uart(main 1)

pico_add_extra_outputs(main)

target_include_directories(main PRIVATE ${CMAKE_CURRENT_LIST_DIR} )

target_link_libraries(main
        pico_stdlib
#        pico_cyw43_arch_lwip_threadsafe_background
        pico_cyw43_arch_lwip_poll
        tinyusb_device
        tinyusb_board
        pico_multicore
        pico_sync
)
