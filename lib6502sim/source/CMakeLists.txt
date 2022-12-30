cmake_minimum_required(VERSION 3.5)

target_sources(6502 PRIVATE ${CMAKE_CURRENT_LIST_DIR}/init.cpp ${CMAKE_CURRENT_LIST_DIR}/opcode_table.cpp)
