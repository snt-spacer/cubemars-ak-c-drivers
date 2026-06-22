cmake_minimum_required(VERSION 3.28)

project(cubemars-drivers C)

set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Curses REQUIRED)

# Inference library

file(GLOB LIB_SOURCES ${PROJECT_SOURCE_DIR}/src/*.c)

add_library(cubemars-drivers ${LIB_SOURCES})

target_include_directories(cubemars-drivers
PUBLIC
${PROJECT_SOURCE_DIR}/inc
)

# Example executables

add_executable(demo demo/demo.c)
target_link_libraries(demo PRIVATE cubemars-drivers ${CURSES_LIBRARIES})
target_include_directories(demo PRIVATE ${CURSES_INCLUDE_DIRS})
target_compile_options(demo PRIVATE -Wno-pedantic -Wno-shadow -Wno-conversion)

add_executable(nn_demo demo/nn_demo.c)
target_link_libraries(nn_demo PRIVATE cubemars-drivers ${CURSES_LIBRARIES})
target_include_directories(nn_demo PRIVATE ${CURSES_INCLUDE_DIRS})
target_compile_options(nn_demo PRIVATE -Wno-pedantic -Wno-shadow -Wno-conversion)

add_executable(serial_demo demo/serial_demo.c)
target_link_libraries(serial_demo PRIVATE cubemars-drivers ${CURSES_LIBRARIES})
target_include_directories(serial_demo PRIVATE ${CURSES_INCLUDE_DIRS})
target_compile_options(serial_demo PRIVATE -Wno-pedantic -Wno-shadow -Wno-conversion)

# Compiler options and static analysis

add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
    -Werror
    -Wshadow
    -Wconversion
    -Wformat=2
)