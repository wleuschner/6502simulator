cmake_minimum_required(VERSION 3.5)

add_library(6502 STATIC "")
target_include_directories(6502 PUBLIC include)
add_subdirectory(source)

