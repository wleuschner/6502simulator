#pragma once
#include "opcode.h"
#include "register_file.h"

instruction decode_instruction(uint8_t* memory, uint16_t offset);
uint16_t decode_effective_adress(instruction instr, uint8_t* memory, register_file* registers);
