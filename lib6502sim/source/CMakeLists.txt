cmake_minimum_required(VERSION 3.5)

target_sources(6502 PRIVATE ${CMAKE_CURRENT_LIST_DIR}/simulator.cpp ${CMAKE_CURRENT_LIST_DIR}/decode.cpp ${CMAKE_CURRENT_LIST_DIR}/helper.cpp ${CMAKE_CURRENT_LIST_DIR}/instructions.cpp ${CMAKE_CURRENT_LIST_DIR}/opcode_table.cpp)
target_include_directories(6502 PRIVATE ${CMAKE_CURRENT_LIST_DIR}/include)
