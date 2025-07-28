cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr HINTS $ENV{ZEPHYR_BASE})

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

project(nrfmic)

file(GLOB SOURCES "src/*.c")

target_sources(app PRIVATE ${SOURCES})
target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)