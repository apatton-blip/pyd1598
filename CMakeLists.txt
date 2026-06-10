cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(pyd1598)

list(APPEND EXTRA_DTC_FLAGS "-I${CMAKE_CURRENT_SOURCE_DIR}/dts/bindings")

target_sources( app PRIVATE 
    src/main.c
    src/pyd1598.c
    )
