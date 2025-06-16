cmake_minimum_required(VERSION 3.13...3.27)

# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()
set(PICO_BOARD pico2_w CACHE STRING "Board type")

include(pico_sdk_import.cmake)

project(my_project)


# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

add_subdirectory(lvgl)

# rest of your project

add_executable(hello_world
    main.c
    281.c
)

target_include_directories(hello_world PRIVATE
    /home/calle/ili/pico_lgvl_ili9341/lvgl/
)


# Add pico_stdlib library which aggregates commonly used features
target_link_libraries(hello_world pico_time pico_stdlib lvgl hardware_spi hardware_gpio) 

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(hello_world)