#include "opcode.h"
#include "opcode_table.h"
#include "register_file.h"
#include <simulator.h>

#include <cinttypes>
#include <cstdlib>
#include <cstring>
#include <cstdio>

uint8_t* memory = nullptr;
uint16_t memory_size = 0;
register_file* registers = nullptr;
uint16_t program_size = 0;

void init(uint16_t mem_size)
{
    memory_size = mem_size;
    memory = (uint8_t*)malloc(memory_size);
    memset(memory, 0, memory_size);

    registers = (register_file*)malloc(sizeof(register_file));
    memset(registers, 0, sizeof(register_file));
    registers->reg_s = 0xFF;
    registers->reg_flags = 0x36;
}

bool load_program(const char* file_name, uint16_t offset=0)
{
    FILE* binary6502 = fopen(file_name,"rb");
    if(binary6502==NULL)
    {
        printf("Could not open file %s\n",file_name);
        return false;
    }
    fseek(binary6502, 0, SEEK_END);
    program_size = ftell(binary6502);
    fseek(binary6502, 0, SEEK_SET);

    fread(memory+offset, sizeof(uint8_t), program_size, binary6502);
    fclose(binary6502);
    return true;
}

void deinit()
{
    memory_size = 0;
    program_size = 0;

    free(registers);
    registers = nullptr;
    free(memory);
    memory = nullptr;
}

void register_to_string(char* buffer, size_t size)
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

instruction decode_instruction(uint16_t offset)
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

std::vector<instruction> disassemble_program()
{
    uint16_t offset = 0;
    std::vector<instruction> opcodes;
    while(offset<program_size)
    {
        instruction instr = decode_instruction(offset);
        opcodes.push_back(instr);
        offset += instr.instr_length;
    }
    return opcodes;
}

uint16_t decode_effective_adress(instruction instr)
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

void instr_load(instruction instr,uint8_t* reg, uint8_t* flags)
{
    if(instr.opcode_mode==ADDRESS_MODE::IMMIDIATE)
    {
        *reg = instr.instr_operand;
    }
    else
    {
        *reg = memory[decode_effective_adress(instr)];
    }

    //Change Zero Flag
    if(*reg == 0)
    {
        *flags |= FLAG_ZERO;
    }
    else
    {
        *flags &= ~FLAG_ZERO;
    }

    //Change Negative Flag
    if(*reg & 128)
    {
        *flags |= FLAG_NEGATIVE;
    }
    else
    {
        *flags &= ~FLAG_NEGATIVE;
    }
}

void instr_store(instruction instr,uint8_t* reg)
{
    if(instr.opcode_mode==ADDRESS_MODE::IMMIDIATE)
    {
        memory[instr.instr_operand] = *reg;
    }
    else
    {
        memory[decode_effective_adress(instr)] = *reg;
    }
}

void instr_transfer(uint8_t* from,uint8_t* to, uint8_t* flags)
{
    *to = *from;

    //Change Zero Flag
    if(*to == 0)
    {
        *flags |= FLAG_ZERO;
    }
    else
    {
        *flags &= ~FLAG_ZERO;
    }

    //Change Negative Flag
    if(*to & 128)
    {
        *flags |= FLAG_NEGATIVE;
    }
    else
    {
        *flags &= ~FLAG_NEGATIVE;
    }
}

void instr_inc(instruction instr,uint8_t* reg, uint8_t* flags)
{
    *reg = (*reg)+1;
    //Change Zero Flag
    if(*reg == 0)
    {
        *flags |= FLAG_ZERO;
    }
    else
    {
        *flags &= ~FLAG_ZERO;
    }

    //Change Negative Flag
    if(*reg & 128)
    {
        *flags |= FLAG_NEGATIVE;
    }
    else
    {
        *flags &= ~FLAG_NEGATIVE;
    }
}

void instr_dec(instruction instr,uint8_t* reg, uint8_t* flags)
{
    *reg = (*reg)-1;
    //Change Zero Flag
    if(*reg == 0)
    {
        *flags |= FLAG_ZERO;
    }
    else
    {
        *flags &= ~FLAG_ZERO;
    }

    //Change Negative Flag
    if(*reg & 128)
    {
        *flags |= FLAG_NEGATIVE;
    }
    else
    {
        *flags &= ~FLAG_NEGATIVE;
    }
}

void instr_and(instruction instr,uint8_t* reg, uint8_t* flags)
{
    if(instr.opcode_mode==ADDRESS_MODE::IMMIDIATE)
    {
        *reg &= instr.instr_operand;
    }
    else
    {
        *reg &= memory[decode_effective_adress(instr)];
    }

    //Change Zero Flag
    if(*reg == 0)
    {
        *flags |= FLAG_ZERO;
    }
    else
    {
        *flags &= ~FLAG_ZERO;
    }

    //Change Negative Flag
    if(*reg & 128)
    {
        *flags |= FLAG_NEGATIVE;
    }
    else
    {
        *flags &= ~FLAG_NEGATIVE;
    }
}

void instr_eor(instruction instr,uint8_t* reg, uint8_t* flags)
{
    if(instr.opcode_mode==ADDRESS_MODE::IMMIDIATE)
    {
        *reg ^= instr.instr_operand;
    }
    else
    {
        *reg ^= memory[decode_effective_adress(instr)];
    }

    //Change Zero Flag
    if(*reg == 0)
    {
        *flags |= FLAG_ZERO;
    }
    else
    {
        *flags &= ~FLAG_ZERO;
    }

    //Change Negative Flag
    if(*reg & 128)
    {
        *flags |= FLAG_NEGATIVE;
    }
    else
    {
        *flags &= ~FLAG_NEGATIVE;
    }
}

void instr_ora(instruction instr,uint8_t* reg, uint8_t* flags)
{
    if(instr.opcode_mode==ADDRESS_MODE::IMMIDIATE)
    {
        *reg |= instr.instr_operand;
    }
    else
    {
        *reg |= memory[decode_effective_adress(instr)];
    }

    //Change Zero Flag
    if(*reg == 0)
    {
        *flags |= FLAG_ZERO;
    }
    else
    {
        *flags &= ~FLAG_ZERO;
    }

    //Change Negative Flag
    if(*reg & 128)
    {
        *flags |= FLAG_NEGATIVE;
    }
    else
    {
        *flags &= ~FLAG_NEGATIVE;
    }
}

void instr_pha()
{
    memory[0x0100|registers->reg_s] = registers->reg_a;
    registers->reg_s--;
}

void instr_php()
{
    memory[0x0100|registers->reg_s] = registers->reg_flags | FLAG_UNUSED | FLAG_BREAK;
    registers->reg_s--;
}

void instr_pla(uint8_t* reg, uint8_t* flags)
{
    registers->reg_s++;
    *reg = memory[0x0100|registers->reg_s];

    //Change Zero Flag
    if(*reg == 0)
    {
        *flags |= FLAG_ZERO;
    }
    else
    {
        *flags &= ~FLAG_ZERO;
    }

    //Change Negative Flag
    if(*reg & 128)
    {
        *flags |= FLAG_NEGATIVE;
    }
    else
    {
        *flags &= ~FLAG_NEGATIVE;
    }
}

void instr_plp()
{
    registers->reg_s++;
    registers->reg_flags |= memory[0x0100|registers->reg_s] & ~(FLAG_UNUSED | FLAG_BREAK);
}

void instr_jsr(instruction instr)
{            ;
    memory[0x0100|registers->reg_s] = (uint8_t)(registers->reg_pc & 0x00FF);
    memory[0x0100|registers->reg_s-1] = (uint8_t)((registers->reg_pc & 0xFF00)>>8);
    registers->reg_s -= 2;
    registers->reg_pc = instr.instr_operand;
}

void instr_rts()
{
    registers->reg_pc = ((((uint16_t)memory[0x0100|(registers->reg_s+1)])&0x00FF)<<8) | (((uint16_t)memory[0x0100|(registers->reg_s+2)])&0x00FF);
    registers->reg_s += 2;
}

void instr_branch(instruction instr, bool take)
{
    if(take)
    {
        registers->reg_pc += (int8_t)instr.instr_operand;
    }
}

void instr_adc(instruction instr, uint8_t* flags)
{
    if(instr.opcode_mode==ADDRESS_MODE::IMMIDIATE)
    {
        uint16_t temp_result = (uint16_t)(registers->reg_a) + ((uint16_t)instr.instr_operand) + (*flags & FLAG_CARRY);
        if(temp_result>0xFF)
        {
            registers->reg_flags |= FLAG_CARRY;
        }
        else
        {
            registers->reg_flags &= ~FLAG_CARRY;
        }
        registers->reg_a = temp_result&0xFF;
    }
    else
    {
        uint16_t temp_result = (uint16_t)(registers->reg_a) + ((uint16_t)memory[decode_effective_adress(instr)]) + (*flags & FLAG_CARRY);
        if(temp_result>0xFF)
        {
            registers->reg_flags |= FLAG_CARRY;
        }
        else
        {
            registers->reg_flags &= ~FLAG_CARRY;
        }
        registers->reg_a = temp_result&0xFF;
    }
    //Change Zero Flag
    if(registers->reg_a == 0)
    {
        *flags |= FLAG_ZERO;
    }
    else
    {
        *flags &= ~FLAG_ZERO;
    }

    //Change Negative Flag
    if(registers->reg_a & 128)
    {
        *flags |= FLAG_NEGATIVE;
    }
    else
    {
        *flags &= ~FLAG_NEGATIVE;
    }
}

void instr_sbb(instruction instr, uint8_t* flags)
{
    if(instr.opcode_mode==ADDRESS_MODE::IMMIDIATE)
    {
        uint16_t temp_result = (uint16_t)(registers->reg_a) - ((uint16_t)instr.instr_operand) - (*flags & FLAG_CARRY);
        if(temp_result>0xFF)
        {
            registers->reg_flags &= ~FLAG_CARRY;
        }
        else
        {
            registers->reg_flags |= FLAG_CARRY;
        }
        registers->reg_a = temp_result&0xFF;
    }
    else
    {
        uint16_t temp_result = (uint16_t)(registers->reg_a) - ((uint16_t)memory[decode_effective_adress(instr)]) - (*flags & FLAG_CARRY);
        if(temp_result>0xFF)
        {
            registers->reg_flags &= ~FLAG_CARRY;
        }
        else
        {
            registers->reg_flags |= FLAG_CARRY;
        }
        registers->reg_a = temp_result&0xFF;
    }
    //Change Zero Flag
    if(registers->reg_a == 0)
    {
        *flags |= FLAG_ZERO;
    }
    else
    {
        *flags &= ~FLAG_ZERO;
    }

    //Change Negative Flag
    if(registers->reg_a & 128)
    {
        *flags |= FLAG_NEGATIVE;
    }
    else
    {
        *flags &= ~FLAG_NEGATIVE;
    }
}

void instr_cmp(instruction instr,uint8_t* reg, uint8_t* flags)
{
    uint16_t result = 0;
    if(instr.opcode_mode==ADDRESS_MODE::IMMIDIATE)
    {
        result = (uint16_t)(*reg) - ((uint16_t)instr.instr_operand);
        if(result>0xFF)
        {
            *flags &= ~FLAG_CARRY;
        }
        else
        {
            *flags |= FLAG_CARRY;
        }
    }
    else
    {
        result = (uint16_t)(registers->reg_a) - ((uint16_t)memory[decode_effective_adress(instr)]);
        if(result>0xFF)
        {
            *flags &= ~FLAG_CARRY;
        }
        else
        {
            *flags |= FLAG_CARRY;
        }
    }
    //Change Zero Flag
    if(result == 0)
    {
        *flags |= FLAG_ZERO;
    }
    else
    {
        *flags &= ~FLAG_ZERO;
    }

    //Change Negative Flag
    if(result & 128)
    {
        *flags |= FLAG_NEGATIVE;
    }
    else
    {
        *flags &= ~FLAG_NEGATIVE;
    }
}

void instr_asl(instruction instr, uint8_t* flags)
{
    if(instr.opcode_mode==ADDRESS_MODE::IMPLICIT)
    {
        uint16_t temp_result = ((uint16_t)(registers->reg_a))<<1;
        if(temp_result>0xFF)
        {
            registers->reg_flags |= FLAG_CARRY;
        }
        else
        {
            registers->reg_flags &= ~FLAG_CARRY;
        }
        registers->reg_a = temp_result&0xFF;
    }
    else
    {
        uint16_t temp_result = ((uint16_t)memory[decode_effective_adress(instr)])<<1;
        if(temp_result>0xFF)
        {
            registers->reg_flags |= FLAG_CARRY;
        }
        else
        {
            registers->reg_flags &= ~FLAG_CARRY;
        }
        memory[decode_effective_adress(instr)] = temp_result&0xFF;
    }
    //Change Zero Flag
    if(registers->reg_a == 0)
    {
        *flags |= FLAG_ZERO;
    }
    else
    {
        *flags &= ~FLAG_ZERO;
    }

    //Change Negative Flag
    if(registers->reg_a & 128)
    {
        *flags |= FLAG_NEGATIVE;
    }
    else
    {
        *flags &= ~FLAG_NEGATIVE;
    }
}

void instr_lsr(instruction instr, uint8_t* flags)
{
    if(instr.opcode_mode==ADDRESS_MODE::IMPLICIT)
    {
        uint16_t temp_result = (((uint16_t)(registers->reg_a))<<7);
        if(temp_result & 0xFF)
        {
            registers->reg_flags |= FLAG_CARRY;
        }
        else
        {
            registers->reg_flags &= ~FLAG_CARRY;
        }
        registers->reg_a = (temp_result>>8) & 0xFF;
    }
    else
    {
        uint16_t temp_result = ((uint16_t)memory[decode_effective_adress(instr)])<<7;
        if(temp_result & 0xFF)
        {
            registers->reg_flags |= FLAG_CARRY;
        }
        else
        {
            registers->reg_flags &= ~FLAG_CARRY;
        }
        memory[decode_effective_adress(instr)] = (temp_result>>8)&0xFF;
    }
    //Change Zero Flag
    if(registers->reg_a == 0)
    {
        *flags |= FLAG_ZERO;
    }
    else
    {
        *flags &= ~FLAG_ZERO;
    }

    //Change Negative Flag
    if(registers->reg_a & 128)
    {
        *flags |= FLAG_NEGATIVE;
    }
    else
    {
        *flags &= ~FLAG_NEGATIVE;
    }
}

void instr_rol(instruction instr, uint8_t* flags)
{
    if(instr.opcode_mode==ADDRESS_MODE::IMPLICIT)
    {
        uint16_t temp_result = (((uint16_t)(registers->reg_a))<<1) | (registers->reg_flags & FLAG_CARRY);
        if(temp_result>0xFF)
        {
            registers->reg_flags |= FLAG_CARRY;
        }
        else
        {
            registers->reg_flags &= ~FLAG_CARRY;
        }
        registers->reg_a = temp_result&0xFF;
    }
    else
    {
        uint16_t temp_result = ((uint16_t)memory[decode_effective_adress(instr)])<<1 | (registers->reg_flags & FLAG_CARRY);
        if(temp_result>0xFF)
        {
            registers->reg_flags |= FLAG_CARRY;
        }
        else
        {
            registers->reg_flags &= ~FLAG_CARRY;
        }
        memory[decode_effective_adress(instr)] = temp_result&0xFF;
    }
    //Change Zero Flag
    if(registers->reg_a == 0)
    {
        *flags |= FLAG_ZERO;
    }
    else
    {
        *flags &= ~FLAG_ZERO;
    }

    //Change Negative Flag
    if(registers->reg_a & 128)
    {
        *flags |= FLAG_NEGATIVE;
    }
    else
    {
        *flags &= ~FLAG_NEGATIVE;
    }
}

void instr_ror(instruction instr, uint8_t* flags)
{
    if(instr.opcode_mode==ADDRESS_MODE::IMPLICIT)
    {
        uint16_t temp_result = (((uint16_t)(registers->reg_a))<<7) | (registers->reg_flags & FLAG_CARRY)<<15;
        if(temp_result & 0xFF)
        {
            registers->reg_flags |= FLAG_CARRY;
        }
        else
        {
            registers->reg_flags &= ~FLAG_CARRY;
        }
        registers->reg_a = (temp_result>>8)&0xFF;
    }
    else
    {
        uint16_t temp_result = ((uint16_t)memory[decode_effective_adress(instr)])<<7 | (registers->reg_flags & FLAG_CARRY)<<15;
        if(temp_result & 0xFF)
        {
            registers->reg_flags |= FLAG_CARRY;
        }
        else
        {
            registers->reg_flags &= ~FLAG_CARRY;
        }
        memory[decode_effective_adress(instr)] = (temp_result>>8)&0xFF;
    }
    //Change Zero Flag
    if(registers->reg_a == 0)
    {
        *flags |= FLAG_ZERO;
    }
    else
    {
        *flags &= ~FLAG_ZERO;
    }

    //Change Negative Flag
    if(registers->reg_a & 128)
    {
        *flags |= FLAG_NEGATIVE;
    }
    else
    {
        *flags &= ~FLAG_NEGATIVE;
    }
}

void instr_bit(instruction instr, uint8_t* flags)
{
    uint8_t result = registers->reg_a & memory[decode_effective_adress(instr)];

    //Change Zero Flag
    if(result == 0)
    {
        *flags |= FLAG_ZERO;
    }
    else
    {
        *flags &= ~FLAG_ZERO;
    }
    *flags = (*flags & ~(FLAG_NEGATIVE | FLAG_OVERFLOW)) | (result & (FLAG_NEGATIVE | FLAG_OVERFLOW));
}

instruction step()
{
    instruction instr = decode_instruction(registers->reg_pc);
    registers->reg_pc += instr.instr_length;
    switch(instr.opcode_name)
    {
        //Jump and Subroutine operations
        case OPCODE::JMP:
        {
            registers->reg_pc = decode_effective_adress(instr);
            break;
        }
        case OPCODE::JSR:
        {
            instr_jsr(instr);
            break;
        }
        case OPCODE::RTS:
        {
            instr_rts();
            break;
        }

        //Compare operations
        case OPCODE::CMP:
        {
            instr_cmp(instr, &registers->reg_a, &registers->reg_flags);
            break;
        }
        case OPCODE::CPX:
        {
            instr_cmp(instr, &registers->reg_x, &registers->reg_flags);
            break;
        }
        case OPCODE::CPY:
        {
            instr_cmp(instr, &registers->reg_y, &registers->reg_flags);
            break;
        }

        //Branch operations
        case OPCODE::BCC:
        {
            instr_branch(instr, !(registers->reg_flags & FLAG_CARRY));
            break;
        }
        case OPCODE::BCS:
        {
            instr_branch(instr, (registers->reg_flags & FLAG_CARRY));
            break;
        }
        case OPCODE::BNE:
        {
            instr_branch(instr, !(registers->reg_flags & FLAG_ZERO));
            break;
        }
        case OPCODE::BEQ:
        {
            instr_branch(instr, (registers->reg_flags & FLAG_ZERO));
            break;
        }
        case OPCODE::BPL:
        {
            instr_branch(instr, !(registers->reg_flags & FLAG_NEGATIVE));
            break;
        }
        case OPCODE::BMI:
        {
            instr_branch(instr, (registers->reg_flags & FLAG_NEGATIVE));
            break;
        }
        case OPCODE::BVC:
        {
            instr_branch(instr, !(registers->reg_flags & FLAG_OVERFLOW));
            break;
        }
        case OPCODE::BVS:
        {
            instr_branch(instr, (registers->reg_flags & FLAG_OVERFLOW));
            break;
        }

        //Load
        case OPCODE::LDA:
        {
            instr_load(instr, &registers->reg_a, &registers->reg_flags);
            break;
        }
        case OPCODE::LDX:
        {
            instr_load(instr, &registers->reg_x, &registers->reg_flags);
            break;
        }
        case OPCODE::LDY:
        {
            instr_load(instr, &registers->reg_y, &registers->reg_flags);
            break;
        }

        //Store
        case OPCODE::STA:
        {
            instr_store(instr, &registers->reg_a);
            break;
        }
        case OPCODE::STX:
        {
            instr_store(instr, &registers->reg_x);
            break;
        }
        case OPCODE::STY:
        {
            instr_store(instr, &registers->reg_y);
            break;
        }

        //Transfer
        case OPCODE::TAX:
        {
            instr_transfer(&registers->reg_a, &registers->reg_x, &registers->reg_flags);
            break;
        }
        case OPCODE::TAY:
        {
            instr_transfer(&registers->reg_a, &registers->reg_y, &registers->reg_flags);
            break;
        }
        case OPCODE::TSX:
        {
            instr_transfer(&registers->reg_s, &registers->reg_x, &registers->reg_flags);
            break;
        }
        case OPCODE::TXA:
        {
            instr_transfer(&registers->reg_x, &registers->reg_a, &registers->reg_flags);
            break;
        }
        case OPCODE::TXS:
        {
            instr_transfer(&registers->reg_x, &registers->reg_s, &registers->reg_flags);
            break;
        }
        case OPCODE::TYA:
        {
            instr_transfer(&registers->reg_y, &registers->reg_a, &registers->reg_flags);
            break;
        }

        //Set Flags
        case OPCODE::SEC:
        {
            registers->reg_flags |= FLAG_CARRY;
            break;
        }
        case OPCODE::SED:
        {
            registers->reg_flags |= FLAG_DECIMAL;
            break;
        }
        case OPCODE::SEI:
        {
            registers->reg_flags |= FLAG_INTERRUPT;
            break;
        }

        //Clear Flags
        case OPCODE::CLC:
        {
            registers->reg_flags &= ~FLAG_CARRY;
            break;
        }
        case OPCODE::CLD:
        {
            registers->reg_flags &= ~FLAG_DECIMAL;
            break;
        }
        case OPCODE::CLI:
        {
            registers->reg_flags &= ~FLAG_INTERRUPT;
            break;
        }
        case OPCODE::CLV:
        {
            registers->reg_flags &= ~FLAG_OVERFLOW;
            break;
        }

        //Increment
        case OPCODE::INC:
        {
            instr_inc(instr,memory + decode_effective_adress(instr), &registers->reg_flags);
            break;
        }
        case OPCODE::INX:
        {
            instr_inc(instr,&registers->reg_x, &registers->reg_flags);
            break;
        }
        case OPCODE::INY:
        {
            instr_inc(instr,&registers->reg_y, &registers->reg_flags);
            break;
        }

        //Decrement
        case OPCODE::DEC:
        {
            instr_dec(instr,memory + decode_effective_adress(instr), &registers->reg_flags);
            break;
        }
        case OPCODE::DEX:
        {
            instr_dec(instr,&registers->reg_x, &registers->reg_flags);
            break;
        }
        case OPCODE::DEY:
        {
            instr_dec(instr,&registers->reg_y, &registers->reg_flags);
            break;
        }

        //Logic operations
        case OPCODE::AND:
        {
            instr_and(instr,&registers->reg_a,&registers->reg_flags);
            break;
        }
        case OPCODE::EOR:
        {
            instr_eor(instr,&registers->reg_a,&registers->reg_flags);
            break;
        }
        case OPCODE::ORA:
        {
            instr_ora(instr,&registers->reg_a,&registers->reg_flags);
            break;
        }
        //Shift operations
        case OPCODE::ASL:
        {
            instr_asl(instr,&registers->reg_flags);
            break;
        }
        case OPCODE::LSR:
        {
            instr_lsr(instr,&registers->reg_flags);
            break;
        }
        case OPCODE::ROL:
        {
            instr_rol(instr,&registers->reg_flags);
            break;
        }
        case OPCODE::ROR:
        {
            instr_ror(instr,&registers->reg_flags);
            break;
        }

        //Arithmetic operations
        case OPCODE::ADC:
        {
            instr_adc(instr, &registers->reg_flags);
            break;
        }
        case OPCODE::SBC:
        {
            instr_sbb(instr, &registers->reg_flags);
            break;
        }

        //Stack operations
        case OPCODE::PHA:
        {
            instr_pha();
            break;
        }
        case OPCODE::PHP:
        {
            instr_php();
            break;
        }
        case OPCODE::PLA:
        {
            instr_pla(&registers->reg_a, &registers->reg_flags);
            break;
        }
        case OPCODE::PLP:
        {
            instr_plp();
            break;
        }

        //Misc operations
        case OPCODE::BIT:
        {
            instr_bit(instr, &registers->reg_flags);
            break;
        }
    }
    return instr;
}
