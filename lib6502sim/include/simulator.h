#pragma once
#include <cinttypes>
#include <vector>
#include "opcode.h"

void init(uint16_t memory_size);
bool load_program(const char* file_name, uint16_t offset);
std::vector<instruction> disassemble_program();
void register_to_string(char* buffer, size_t size);
void opcode_to_string(char* buffer, size_t size,instruction op);
void deinit();
instruction step();
