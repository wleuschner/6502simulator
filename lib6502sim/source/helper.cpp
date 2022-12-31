#include "register_file.h"
#include <simulator.h>

#include <cinttypes>
#include <cstdlib>
#include <cstring>
#include <cstdio>

void register_to_string(char* buffer, size_t size, register_file* registers)
{
    bool flag_n = (registers->reg_flags & FLAG_NEGATIVE) != 0;
    bool flag_v = (registers->reg_flags & FLAG_OVERFLOW) != 0;
    bool flag_b = (registers->reg_flags & FLAG_BREAK) != 0;
    bool flag_d = (registers->reg_flags & FLAG_DECIMAL) != 0;
    bool flag_i = (registers->reg_flags & FLAG_INTERRUPT) != 0;
    bool flag_z = (registers->reg_flags & FLAG_ZERO) != 0;
    bool flag_c = (registers->reg_flags & FLAG_CARRY) != 0;
    snprintf(buffer,size,"PC=$%04x A=$%02x X=$%02x Y=$%02x S=$%02x\r\nN=%d V=%d B=%d D=%d I=%d Z=%d C=%d", registers->reg_pc, registers->reg_a, registers->reg_x, registers->reg_y, registers->reg_s, flag_n, flag_v, flag_b, flag_d, flag_i, flag_z, flag_c);
}

void opcode_to_string(char* buffer, size_t size,instruction op)
{
    INSTRUCTION_STRINGS[op.opcode_name];
    switch(op.opcode_mode)
    {
        case ADDRESS_MODE::ABSOLUTE:
        {
            snprintf(buffer,size,"%s $%04x", INSTRUCTION_STRINGS[op.opcode_name], op.instr_operand);
            break;
        }
        case ADDRESS_MODE::ABSOLUTE_X:
        {
            snprintf(buffer,size,"%s $%04x,X", INSTRUCTION_STRINGS[op.opcode_name], op.instr_operand);
            break;
        }
        case ADDRESS_MODE::ABSOLUTE_Y:
        {
            snprintf(buffer,size,"%s $%04x,Y", INSTRUCTION_STRINGS[op.opcode_name], op.instr_operand);
            break;
        }
        case ADDRESS_MODE::ZEROPAGE:
        {
            snprintf(buffer,size,"%s $%02x", INSTRUCTION_STRINGS[op.opcode_name], op.instr_operand);
            break;
        }
        case ADDRESS_MODE::ZEROPAGEX:
        {
            snprintf(buffer,size,"%s $%02x,X", INSTRUCTION_STRINGS[op.opcode_name], op.instr_operand);
            break;
        }
        case ADDRESS_MODE::ZEROPAGEY:
        {
            snprintf(buffer,size,"%s $%02x,Y", INSTRUCTION_STRINGS[op.opcode_name], op.instr_operand);
            break;
        }
        case ADDRESS_MODE::IMMIDIATE:
        {
            snprintf(buffer,size,"%s #$%02x", INSTRUCTION_STRINGS[op.opcode_name], op.instr_operand);
            break;
        }
        case ADDRESS_MODE::INDIRECT:
        {
            snprintf(buffer,size,"%s ($%04x)", INSTRUCTION_STRINGS[op.opcode_name], op.instr_operand);
            break;
        }
        case ADDRESS_MODE::RELATIVE:
        {
            snprintf(buffer,size,"%s $%02x", INSTRUCTION_STRINGS[op.opcode_name], op.instr_operand);
            break;
        }
        case ADDRESS_MODE::INDEXED_X:
        {
            snprintf(buffer,size,"%s ($%02x,X)", INSTRUCTION_STRINGS[op.opcode_name], op.instr_operand);
            break;
        }
        case ADDRESS_MODE::INDEXED_Y:
        {
            snprintf(buffer,size,"%s ($%02x),Y", INSTRUCTION_STRINGS[op.opcode_name], op.instr_operand);
            break;
        }
        case ADDRESS_MODE::IMPLICIT:
        {
            snprintf(buffer,size,"%s", INSTRUCTION_STRINGS[op.opcode_name]);
            break;
        }
    }
}
