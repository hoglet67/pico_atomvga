cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)

include(pico_extras_import.cmake)

project(atomvga_project)

pico_sdk_init()

add_executable(atomvga
  atomvga.c
  )

pico_generate_pio_header(atomvga ${CMAKE_CURRENT_LIST_DIR}/atomvga.pio)

target_link_libraries(atomvga PRIVATE
  pico_multicore
  pico_stdlib
  pico_scanvideo_dpi
  hardware_pio
  )

pico_add_extra_outputs(atomvga)

pico_enable_stdio_uart(atomvga 1)
# pico_enable_stdio_usb(atomvga 1)
