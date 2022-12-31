#include "decode.h"
#include "opcode_table.h"
#include "register_file.h"

instruction decode_instruction(uint8_t* memory, uint16_t offset)
{
    uint32_t op = memory[offset];
    instruction instr = opcode_table[op];
    offset++;
    instr.instr_length = 1;
    switch(instr.opcode_mode)
    {
        case ADDRESS_MODE::ABSOLUTE:
        case ADDRESS_MODE::ABSOLUTE_X:
        case ADDRESS_MODE::ABSOLUTE_Y:
        case ADDRESS_MODE::INDIRECT:
        {
            instr.instr_operand = *((uint16_t*)(memory+offset));
            instr.instr_length = 3;
            break;
        }
        case ADDRESS_MODE::ZEROPAGE:
        case ADDRESS_MODE::ZEROPAGEX:
        case ADDRESS_MODE::ZEROPAGEY:
        case ADDRESS_MODE::IMMIDIATE:
        case ADDRESS_MODE::RELATIVE:
        case ADDRESS_MODE::INDEXED_X:
        case ADDRESS_MODE::INDEXED_Y:
        {
            instr.instr_operand = *((uint8_t*)(memory+offset));
            instr.instr_length = 2;
            break;
        }
    }
    return instr;
}

uint16_t decode_effective_adress(instruction instr, uint8_t* memory, register_file* registers)
{
    switch(instr.opcode_mode)
    {
        case ADDRESS_MODE::ABSOLUTE:
        case ADDRESS_MODE::ZEROPAGE:
        {
            return instr.instr_operand;
        }
        case ADDRESS_MODE::ZEROPAGEX:
        case ADDRESS_MODE::ABSOLUTE_X:
        {
            return instr.instr_operand + (int8_t)registers->reg_x;
        }
        case ADDRESS_MODE::ZEROPAGEY:
        case ADDRESS_MODE::ABSOLUTE_Y:
        {
            return instr.instr_operand + (int8_t)registers->reg_y;
        }
        case ADDRESS_MODE::RELATIVE:
        {
            return registers->reg_pc + (int8_t)instr.instr_operand;
        }
        case ADDRESS_MODE::INDIRECT:
        {
            return memory[instr.instr_operand];
        }
        case ADDRESS_MODE::INDEXED_X:
        {
            return memory[instr.instr_operand + registers->reg_x];
        }
        case ADDRESS_MODE::INDEXED_Y:
        {
            return memory[instr.instr_operand] + registers->reg_y;
        }
    }
    return 0;
}
