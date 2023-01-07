#include "opcode.h"
#include "opcode_table.h"
#include "register_file.h"
#include "decode.h"
#include <simulator.h>

#include <cinttypes>
#include <cstdlib>
#include <cstring>
#include <cstdio>

#define STACK_PAGE 0x0100
#define IRQ_VECTOR 0xFFFE


void alu_flags_update(uint8_t* reg, uint8_t* flags)
{
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

void instr_load(instruction instr, uint8_t* memory, register_file* registers, uint8_t* reg)
{
    if(instr.opcode_mode==ADDRESS_MODE::IMMIDIATE)
    {
        *reg = instr.instr_operand;
    }
    else
    {
        *reg = memory[decode_effective_adress(instr, memory, registers)];
    }
    alu_flags_update(reg, &registers->reg_flags);
}

void instr_store(instruction instr, uint8_t* memory, register_file* registers, uint8_t* reg)
{
    if(instr.opcode_mode==ADDRESS_MODE::IMMIDIATE)
    {
        memory[instr.instr_operand] = *reg;
    }
    else
    {
        memory[decode_effective_adress(instr, memory, registers)] = *reg;
    }
}

void instr_transfer(register_file* registers, uint8_t* from,uint8_t* to)
{
    *to = *from;
    alu_flags_update(to, &registers->reg_flags);
}

void instr_inc(register_file* registers, uint8_t* reg)
{
    *reg = (*reg)+1;
    alu_flags_update(reg, &registers->reg_flags);
}

void instr_dec(register_file* registers, uint8_t* reg)
{
    *reg = (*reg)-1;
    alu_flags_update(reg, &registers->reg_flags);
}

void instr_and(instruction instr, uint8_t* memory, register_file* registers)
{
    if(instr.opcode_mode==ADDRESS_MODE::IMMIDIATE)
    {
        registers->reg_a &= instr.instr_operand;
    }
    else
    {
        registers->reg_a &= memory[decode_effective_adress(instr, memory, registers)];
    }
    alu_flags_update(&registers->reg_a, &registers->reg_flags);
}

void instr_eor(instruction instr, uint8_t* memory, register_file* registers)
{
    if(instr.opcode_mode==ADDRESS_MODE::IMMIDIATE)
    {
        registers->reg_a ^= instr.instr_operand;
    }
    else
    {
        registers->reg_a ^= memory[decode_effective_adress(instr, memory, registers)];
    }
    alu_flags_update(&registers->reg_a, &registers->reg_flags);
}

void instr_ora(instruction instr, uint8_t* memory, register_file* registers)
{
    if(instr.opcode_mode==ADDRESS_MODE::IMMIDIATE)
    {
        registers->reg_a |= instr.instr_operand;
    }
    else
    {
        registers->reg_a |= memory[decode_effective_adress(instr, memory, registers)];
    }
    alu_flags_update(&registers->reg_a, &registers->reg_flags);
}

void instr_pha(uint8_t* memory, register_file* registers)
{
    memory[STACK_PAGE+registers->reg_s] = registers->reg_a;
    registers->reg_s--;
}

void instr_php(uint8_t* memory, register_file* registers)
{
    memory[STACK_PAGE+registers->reg_s] = registers->reg_flags | FLAG_UNUSED | FLAG_BREAK;
    registers->reg_s--;
}

void instr_pla(uint8_t* memory, register_file* registers)
{
    registers->reg_s++;
    registers->reg_a = memory[STACK_PAGE+registers->reg_s];
    alu_flags_update(&registers->reg_a, &registers->reg_flags);
}

void instr_plp(uint8_t* memory, register_file* registers)
{
    registers->reg_s++;
    registers->reg_flags |= memory[STACK_PAGE+registers->reg_s] & ~FLAG_BREAK;
}

void instr_jsr(instruction instr, uint8_t* memory, register_file* registers)
{            ;
    memory[STACK_PAGE+registers->reg_s] = (uint8_t)((registers->reg_pc & 0xFF00)>>8);
    memory[STACK_PAGE+registers->reg_s-1] = (uint8_t)(registers->reg_pc & 0x00FF);
    registers->reg_s -= 2;
    registers->reg_pc = instr.instr_operand;
}

void instr_rts(uint8_t* memory, register_file* registers)
{
    registers->reg_pc = ((((uint16_t)memory[STACK_PAGE+(registers->reg_s+2)])&0x00FF)<<8) | (((uint16_t)memory[STACK_PAGE+(registers->reg_s+1)])&0x00FF);
    registers->reg_s += 2;
}

void instr_branch(instruction instr, register_file* registers, bool take)
{
    if(take)
    {
        registers->reg_pc += (int8_t)instr.instr_operand;
    }
}

void instr_adc(instruction instr, uint8_t* memory, register_file* registers)
{
    uint16_t temp_result;
    uint16_t operand;
    if(instr.opcode_mode==ADDRESS_MODE::IMMIDIATE)
    {
        operand = ((uint16_t)instr.instr_operand);
    }
    else
    {
        operand = ((uint16_t)memory[decode_effective_adress(instr, memory, registers)]);
    }
    temp_result = (uint16_t)(registers->reg_a) + operand + (registers->reg_flags & FLAG_CARRY);


    //Check Sign Overflow
    if((registers->reg_a^temp_result)&(operand^temp_result)&0x80)
    {
        registers->reg_flags |= FLAG_OVERFLOW;
    }
    else
    {
        registers->reg_flags &= ~FLAG_OVERFLOW;
    }

    //Check Carry
    if(temp_result>0xFF)
    {
        registers->reg_flags |= FLAG_CARRY;
    }
    else
    {
        registers->reg_flags &= ~FLAG_CARRY;
    }
    registers->reg_a = temp_result&0xFF;
    alu_flags_update(&registers->reg_a, &registers->reg_flags);
}

void instr_sbb(instruction instr, uint8_t* memory, register_file* registers)
{
    uint16_t temp_result;
    uint16_t operand;
    if(instr.opcode_mode==ADDRESS_MODE::IMMIDIATE)
    {
        operand = ((uint16_t)instr.instr_operand);
    }
    else
    {
        operand = ((uint16_t)memory[decode_effective_adress(instr, memory, registers)]);
    }
    temp_result = (uint16_t)(registers->reg_a) - ((uint16_t)instr.instr_operand) - (registers->reg_flags & FLAG_CARRY);


    //Check Sign Overflow
    if((registers->reg_a^temp_result)&(operand^temp_result)&0x80)
    {
        registers->reg_flags |= FLAG_OVERFLOW;
    }
    else
    {
        registers->reg_flags &= ~FLAG_OVERFLOW;
    }

    //Check Carry
    if(temp_result>0xFF)
    {
        registers->reg_flags &= ~FLAG_CARRY;
    }
    else
    {
        registers->reg_flags |= FLAG_CARRY;
    }
    registers->reg_a = temp_result&0xFF;
    alu_flags_update(&registers->reg_a, &registers->reg_flags);
}

void instr_cmp(instruction instr, uint8_t* memory, register_file* registers,uint8_t* reg)
{
    uint16_t result = 0;
    if(instr.opcode_mode==ADDRESS_MODE::IMMIDIATE)
    {
        result = (uint16_t)(*reg) - ((uint16_t)instr.instr_operand);
    }
    else
    {
        result = (uint16_t)(registers->reg_a) - ((uint16_t)memory[decode_effective_adress(instr, memory, registers)]);
    }

    if(result>0xFF)
    {
        registers->reg_flags &= ~FLAG_CARRY;
    }
    else
    {
        registers->reg_flags |= FLAG_CARRY;
    }
    alu_flags_update((uint8_t*)&result, &registers->reg_flags);
}

void instr_asl(instruction instr, uint8_t* memory, register_file* registers)
{
    uint16_t temp_result;
    if(instr.opcode_mode==ADDRESS_MODE::IMPLICIT)
    {
        temp_result = ((uint16_t)(registers->reg_a))<<1;
        registers->reg_a = temp_result&0xFF;
    }
    else
    {
        temp_result = ((uint16_t)memory[decode_effective_adress(instr, memory, registers)])<<1;
        memory[decode_effective_adress(instr, memory, registers)] = temp_result&0xFF;
    }

    if(temp_result>0xFF)
    {
        registers->reg_flags |= FLAG_CARRY;
    }
    else
    {
        registers->reg_flags &= ~FLAG_CARRY;
    }
    alu_flags_update(&registers->reg_a, &registers->reg_flags);
}

void instr_lsr(instruction instr, uint8_t* memory, register_file* registers)
{
    uint16_t temp_result;
    if(instr.opcode_mode==ADDRESS_MODE::IMPLICIT)
    {
        temp_result = (((uint16_t)(registers->reg_a))<<7);
        registers->reg_a = (temp_result>>8) & 0xFF;
    }
    else
    {
        temp_result = ((uint16_t)memory[decode_effective_adress(instr, memory, registers)])<<7;
        memory[decode_effective_adress(instr, memory, registers)] = (temp_result>>8)&0xFF;
    }

    if(temp_result & 0xFF)
    {
        registers->reg_flags |= FLAG_CARRY;
    }
    else
    {
        registers->reg_flags &= ~FLAG_CARRY;
    }
    alu_flags_update(&registers->reg_a, &registers->reg_flags);
}

void instr_rol(instruction instr, uint8_t* memory, register_file* registers)
{
    uint16_t temp_result;
    if(instr.opcode_mode==ADDRESS_MODE::IMPLICIT)
    {
        temp_result = (((uint16_t)(registers->reg_a))<<1) | (registers->reg_flags & FLAG_CARRY);
        registers->reg_a = temp_result&0xFF;
    }
    else
    {
        temp_result = ((uint16_t)memory[decode_effective_adress(instr, memory, registers)])<<1 | (registers->reg_flags & FLAG_CARRY);
        memory[decode_effective_adress(instr, memory, registers)] = temp_result&0xFF;
    }

    if(temp_result>0xFF)
    {
        registers->reg_flags |= FLAG_CARRY;
    }
    else
    {
        registers->reg_flags &= ~FLAG_CARRY;
    }
    alu_flags_update(&registers->reg_a, &registers->reg_flags);
}

void instr_ror(instruction instr, uint8_t* memory, register_file* registers)
{
    uint16_t temp_result;
    if(instr.opcode_mode==ADDRESS_MODE::IMPLICIT)
    {
        temp_result = (((uint16_t)(registers->reg_a))<<7) | (registers->reg_flags & FLAG_CARRY)<<15;
        registers->reg_a = (temp_result>>8)&0xFF;
    }
    else
    {
        temp_result = ((uint16_t)memory[decode_effective_adress(instr, memory, registers)])<<7 | (registers->reg_flags & FLAG_CARRY)<<15;
        memory[decode_effective_adress(instr, memory, registers)] = (temp_result>>8)&0xFF;
    }

    if(temp_result & 0xFF)
    {
        registers->reg_flags |= FLAG_CARRY;
    }
    else
    {
        registers->reg_flags &= ~FLAG_CARRY;
    }
    alu_flags_update(&registers->reg_a, &registers->reg_flags);
}

void instr_bit(instruction instr, uint8_t* memory, register_file* registers)
{
    uint8_t result = registers->reg_a & memory[decode_effective_adress(instr, memory, registers)];

    //Change Zero Flag
    if(result == 0)
    {
        registers->reg_flags |= FLAG_ZERO;
    }
    else
    {
        registers->reg_flags &= ~FLAG_ZERO;
    }
    registers->reg_flags = (registers->reg_flags & ~(FLAG_NEGATIVE | FLAG_OVERFLOW)) | (result & (FLAG_NEGATIVE | FLAG_OVERFLOW));
}

void instr_brk(uint8_t* memory, register_file* registers)
{
    memory[STACK_PAGE+registers->reg_s] = (uint8_t)((registers->reg_pc & 0xFF00)>>8);
    memory[STACK_PAGE+registers->reg_s-1] = (uint8_t)(registers->reg_pc & 0x00FF);
    memory[STACK_PAGE+registers->reg_s-2] = registers->reg_flags | FLAG_UNUSED | FLAG_BREAK;
    registers->reg_s -=3;
    registers->reg_pc = *((uint16_t*)(memory+IRQ_VECTOR));
}

void instr_rti(uint8_t* memory, register_file* registers)
{
    registers->reg_flags = memory[STACK_PAGE+(registers->reg_s+1)] & ~FLAG_BREAK;
    registers->reg_pc = ((((uint16_t)memory[STACK_PAGE+(registers->reg_s+3)])&0x00FF)<<8) | (((uint16_t)memory[STACK_PAGE+(registers->reg_s+2)])&0x00FF);
    registers->reg_s += 3;
}
