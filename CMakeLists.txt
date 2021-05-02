cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)

include(pico_extras_import.cmake)

project(test_pattern_project)

pico_sdk_init()

add_executable(test_pattern
  test_pattern.c
  )

pico_generate_pio_header(test_pattern ${CMAKE_CURRENT_LIST_DIR}/test.pio)

target_link_libraries(test_pattern PRIVATE
  pico_multicore
  pico_stdlib
  pico_scanvideo_dpi
  hardware_pio
  )

pico_add_extra_outputs(test_pattern)

pico_enable_stdio_uart(test_pattern 1)
# pico_enable_stdio_usb(test_pattern 1)
