cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)

include(pico_extras_import.cmake)

project(atomvga_project)

pico_sdk_init()

set( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Wno-unused-parameter" )

#
# Atom Build
#

add_executable(atomvga
  atomvga.c
  )

pico_generate_pio_header(atomvga ${CMAKE_CURRENT_LIST_DIR}/atomvga.pio)

pico_generate_pio_header(atomvga ${CMAKE_CURRENT_LIST_DIR}/atomvga_out.pio)

target_compile_definitions(atomvga PUBLIC -DPLATFORM=PLATFORM_ATOM -DPICO_SCANVIDEO_MAX_SCANLINE_BUFFER_WORDS=800 -DPICO_SCANVIDEO_SCANLINE_BUFFER_COUNT=16)
target_compile_definitions(atomvga PUBLIC PICO_SCANVIDEO_COLOR_PIN_BASE=10 PICO_SCANVIDEO_COLOR_PIN_COUNT=6) 
target_compile_definitions(atomvga PUBLIC PICO_SCANVIDEO_DPI_PIXEL_RCOUNT=2 PICO_SCANVIDEO_DPI_PIXEL_GCOUNT=2 PICO_SCANVIDEO_DPI_PIXEL_BCOUNT=2)
target_compile_definitions(atomvga PUBLIC PICO_SCANVIDEO_DPI_PIXEL_RSHIFT=0u PICO_SCANVIDEO_DPI_PIXEL_GSHIFT=2u PICO_SCANVIDEO_DPI_PIXEL_BSHIFT=4u)

target_link_libraries(atomvga PRIVATE
  pico_multicore
  pico_stdlib
  pico_scanvideo_dpi
  hardware_pio
  )

pico_add_extra_outputs(atomvga)

pico_enable_stdio_uart(atomvga 1)

#
# Atom R65C02 Build
#

add_executable(atomvga_r65c02
  atomvga.c
  )

pico_generate_pio_header(atomvga_r65c02 ${CMAKE_CURRENT_LIST_DIR}/r65c02.pio)

pico_generate_pio_header(atomvga_r65c02 ${CMAKE_CURRENT_LIST_DIR}/atomvga_out.pio)

target_compile_definitions(atomvga_r65c02 PUBLIC -DPLATFORM=PLATFORM_ATOM -DPICO_SCANVIDEO_MAX_SCANLINE_BUFFER_WORDS=800 -DPICO_SCANVIDEO_SCANLINE_BUFFER_COUNT=16 -DR65C02=1)
target_compile_definitions(atomvga_r65c02 PUBLIC PICO_SCANVIDEO_COLOR_PIN_BASE=10 PICO_SCANVIDEO_COLOR_PIN_COUNT=6) 
target_compile_definitions(atomvga_r65c02 PUBLIC PICO_SCANVIDEO_DPI_PIXEL_RCOUNT=2 PICO_SCANVIDEO_DPI_PIXEL_GCOUNT=2 PICO_SCANVIDEO_DPI_PIXEL_BCOUNT=2)
target_compile_definitions(atomvga_r65c02 PUBLIC PICO_SCANVIDEO_DPI_PIXEL_RSHIFT=0u PICO_SCANVIDEO_DPI_PIXEL_GSHIFT=2u PICO_SCANVIDEO_DPI_PIXEL_BSHIFT=4u)

target_link_libraries(atomvga_r65c02 PRIVATE
  pico_multicore
  pico_stdlib
  pico_scanvideo_dpi
  hardware_pio
  )

pico_add_extra_outputs(atomvga_r65c02)

pico_enable_stdio_uart(atomvga_r65c02 1)

#
# Dragon Build
#

add_executable(dragonvga
  atomvga.c
  eeprom.c
  )

pico_generate_pio_header(dragonvga ${CMAKE_CURRENT_LIST_DIR}/atomvga.pio)

pico_generate_pio_header(dragonvga ${CMAKE_CURRENT_LIST_DIR}/atomvga_out.pio)

target_compile_definitions(dragonvga PUBLIC -DPLATFORM=PLATFORM_DRAGON -DPICO_SCANVIDEO_MAX_SCANLINE_BUFFER_WORDS=800 -DPICO_SCANVIDEO_SCANLINE_BUFFER_COUNT=16)
target_compile_definitions(dragonvga PUBLIC PICO_SCANVIDEO_COLOR_PIN_BASE=10 PICO_SCANVIDEO_COLOR_PIN_COUNT=6) 
target_compile_definitions(dragonvga PUBLIC PICO_SCANVIDEO_DPI_PIXEL_RCOUNT=2 PICO_SCANVIDEO_DPI_PIXEL_GCOUNT=2 PICO_SCANVIDEO_DPI_PIXEL_BCOUNT=2)
target_compile_definitions(dragonvga PUBLIC PICO_SCANVIDEO_DPI_PIXEL_RSHIFT=0u PICO_SCANVIDEO_DPI_PIXEL_GSHIFT=2u PICO_SCANVIDEO_DPI_PIXEL_BSHIFT=4u)

target_link_libraries(dragonvga PRIVATE
  pico_multicore
  pico_stdlib
  pico_scanvideo_dpi
  hardware_pio
  hardware_i2c
  )

pico_add_extra_outputs(dragonvga)

pico_enable_stdio_uart(dragonvga 1)
