#pragma once
#include <cinttypes>
#include <vector>
#include "opcode.h"
#include "register_file.h"

void init(uint16_t memory_size);
bool load_program(const char* file_name, uint16_t offset);
std::vector<instruction> disassemble_program();
void register_to_string(char* buffer, size_t size, register_file* registers);
void opcode_to_string(char* buffer, size_t size,instruction op);
void deinit();
instruction step();
uint32_t get_cycle_count();
register_file get_registers();
