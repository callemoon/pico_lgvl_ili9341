cmake_minimum_required(VERSION 3.13...3.27)

# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()
include(pico_sdk_import.cmake)

project(my_project)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

set(LV_USE_ILI9341 1)

set(LVGL_ROOT_DIR ${CMAKE_CURRENT_LIST_DIR}/lvgl/)
include(${CMAKE_CURRENT_LIST_DIR}/lvgl/env_support/cmake/custom.cmake)

# rest of your project

add_executable(hello_world
    hello_world.c
    281.c
)

target_include_directories(hello_world PRIVATE
    /home/calle/picotest/lvgl/
)


# Add pico_stdlib library which aggregates commonly used features
target_link_libraries(hello_world pico_time pico_stdlib "/home/calle/picotest/build/lib/liblvgl.a" hardware_spi hardware_gpio) 

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(hello_world)