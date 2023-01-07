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

uint32_t cycle_page_crossing_panalty(uint16_t old_address, uint16_t new_address)
{
    uint8_t old_page = old_address>>8;
    uint8_t new_page = new_address>>8;

    if(old_page!=new_page)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

uint32_t instr_jmp(instruction instr, uint8_t* memory, register_file* registers)
{
    registers->reg_pc = decode_effective_adress(instr, memory, registers);
    return instr.instr_cycles;
}

uint32_t instr_load(instruction instr, uint8_t* memory, register_file* registers, uint8_t* reg)
{
    uint32_t extra_cycles = 0;
    if(instr.opcode_mode==ADDRESS_MODE::IMMIDIATE)
    {
        *reg = instr.instr_operand;
    }
    else
    {
        uint32_t address = decode_effective_adress(instr, memory, registers);
        extra_cycles = cycle_page_crossing_panalty(instr.instr_operand, address);
        *reg = memory[address];
    }
    alu_flags_update(reg, &registers->reg_flags);
    return instr.instr_cycles + extra_cycles;
}

uint32_t instr_store(instruction instr, uint8_t* memory, register_file* registers, uint8_t* reg)
{
    uint32_t extra_cycles = 0;
    if(instr.opcode_mode==ADDRESS_MODE::IMMIDIATE)
    {
        memory[instr.instr_operand] = *reg;
    }
    else
    {
        uint32_t address = decode_effective_adress(instr, memory, registers);
        extra_cycles = cycle_page_crossing_panalty(instr.instr_operand, address);
        memory[decode_effective_adress(instr, memory, registers)] = *reg;
    }
    return instr.instr_cycles + extra_cycles;
}

uint32_t instr_transfer(instruction instr, register_file* registers, uint8_t* from,uint8_t* to)
{
    *to = *from;
    alu_flags_update(to, &registers->reg_flags);
    return instr.instr_cycles;
}

uint32_t instr_set_flag(instruction instr, register_file* registers, uint8_t flag)
{
    registers->reg_flags |= flag;
    return instr.instr_cycles;
}

uint32_t instr_clear_flag(instruction instr, register_file* registers, uint8_t flag)
{
    registers->reg_flags &= ~flag;
    return instr.instr_cycles;
}

uint32_t instr_inc(instruction instr, register_file* registers, uint8_t* reg)
{
    *reg = (*reg)+1;
    alu_flags_update(reg, &registers->reg_flags);
    return instr.instr_cycles;
}

uint32_t instr_dec(instruction instr, register_file* registers, uint8_t* reg)
{
    *reg = (*reg)-1;
    alu_flags_update(reg, &registers->reg_flags);
    return instr.instr_cycles;
}

uint32_t instr_and(instruction instr, uint8_t* memory, register_file* registers)
{
    uint32_t extra_cycles = 0;
    if(instr.opcode_mode==ADDRESS_MODE::IMMIDIATE)
    {
        registers->reg_a &= instr.instr_operand;
    }
    else
    {
        uint32_t address = decode_effective_adress(instr, memory, registers);
        extra_cycles = cycle_page_crossing_panalty(instr.instr_operand, address);
        registers->reg_a &= memory[decode_effective_adress(instr, memory, registers)];
    }
    alu_flags_update(&registers->reg_a, &registers->reg_flags);
    return instr.instr_cycles + extra_cycles;
}

uint32_t instr_eor(instruction instr, uint8_t* memory, register_file* registers)
{
    uint32_t extra_cycles = 0;
    if(instr.opcode_mode==ADDRESS_MODE::IMMIDIATE)
    {
        registers->reg_a ^= instr.instr_operand;
    }
    else
    {
        uint32_t address = decode_effective_adress(instr, memory, registers);
        extra_cycles = cycle_page_crossing_panalty(instr.instr_operand, address);
        registers->reg_a ^= memory[decode_effective_adress(instr, memory, registers)];
    }
    alu_flags_update(&registers->reg_a, &registers->reg_flags);
    return instr.instr_cycles + extra_cycles;
}

uint32_t instr_ora(instruction instr, uint8_t* memory, register_file* registers)
{
    uint32_t extra_cycles = 0;
    if(instr.opcode_mode==ADDRESS_MODE::IMMIDIATE)
    {
        registers->reg_a |= instr.instr_operand;
    }
    else
    {
        uint32_t address = decode_effective_adress(instr, memory, registers);
        extra_cycles = cycle_page_crossing_panalty(instr.instr_operand, address);
        registers->reg_a |= memory[decode_effective_adress(instr, memory, registers)];
    }
    alu_flags_update(&registers->reg_a, &registers->reg_flags);
    return instr.instr_cycles + extra_cycles;
}

uint32_t instr_pha(instruction instr, uint8_t* memory, register_file* registers)
{
    memory[STACK_PAGE+registers->reg_s] = registers->reg_a;
    registers->reg_s--;
    return instr.instr_cycles;
}

uint32_t instr_php(instruction instr, uint8_t* memory, register_file* registers)
{
    memory[STACK_PAGE+registers->reg_s] = registers->reg_flags | FLAG_UNUSED | FLAG_BREAK;
    registers->reg_s--;
    return instr.instr_cycles;
}

uint32_t instr_pla(instruction instr, uint8_t* memory, register_file* registers)
{
    registers->reg_s++;
    registers->reg_a = memory[STACK_PAGE+registers->reg_s];
    alu_flags_update(&registers->reg_a, &registers->reg_flags);
    return instr.instr_cycles;
}

uint32_t instr_plp(instruction instr, uint8_t* memory, register_file* registers)
{
    registers->reg_s++;
    registers->reg_flags |= memory[STACK_PAGE+registers->reg_s] & ~FLAG_BREAK;
    return instr.instr_cycles;
}

uint32_t instr_jsr(instruction instr, uint8_t* memory, register_file* registers)
{            ;
    memory[STACK_PAGE+registers->reg_s] = (uint8_t)((registers->reg_pc & 0xFF00)>>8);
    memory[STACK_PAGE+registers->reg_s-1] = (uint8_t)(registers->reg_pc & 0x00FF);
    registers->reg_s -= 2;
    registers->reg_pc = instr.instr_operand;
    return instr.instr_cycles;
}

uint32_t instr_rts(instruction instr, uint8_t* memory, register_file* registers)
{
    registers->reg_pc = ((((uint16_t)memory[STACK_PAGE+(registers->reg_s+2)])&0x00FF)<<8) | (((uint16_t)memory[STACK_PAGE+(registers->reg_s+1)])&0x00FF);
    registers->reg_s += 2;
    return instr.instr_cycles;
}

uint32_t instr_branch(instruction instr, register_file* registers, bool take)
{
    uint32_t extra_cycles = 0;
    if(take)
    {
        uint16_t branch_address = registers->reg_pc+(int8_t)instr.instr_operand;
        extra_cycles = cycle_page_crossing_panalty(registers->reg_pc, branch_address) + 1;
        registers->reg_pc = branch_address;
    }
    return instr.instr_cycles + extra_cycles;
}

uint32_t instr_adc(instruction instr, uint8_t* memory, register_file* registers)
{
    uint32_t extra_cycles = 0;
    uint16_t temp_result;
    uint16_t operand;
    if(instr.opcode_mode==ADDRESS_MODE::IMMIDIATE)
    {
        operand = ((uint16_t)instr.instr_operand);
    }
    else
    {
        uint32_t address = decode_effective_adress(instr, memory, registers);
        extra_cycles = cycle_page_crossing_panalty(instr.instr_operand, address);
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
    return instr.instr_cycles + extra_cycles;
}

uint32_t instr_sbb(instruction instr, uint8_t* memory, register_file* registers)
{
    uint32_t extra_cycles = 0;
    uint16_t temp_result;
    uint16_t operand;
    if(instr.opcode_mode==ADDRESS_MODE::IMMIDIATE)
    {
        operand = ((uint16_t)instr.instr_operand);
    }
    else
    {
        uint32_t address = decode_effective_adress(instr, memory, registers);
        extra_cycles = cycle_page_crossing_panalty(instr.instr_operand, address);
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
    return instr.instr_cycles + extra_cycles;
}

uint32_t instr_cmp(instruction instr, uint8_t* memory, register_file* registers,uint8_t* reg)
{
    uint32_t extra_cycles = 0;
    uint16_t result = 0;
    if(instr.opcode_mode==ADDRESS_MODE::IMMIDIATE)
    {
        result = (uint16_t)(*reg) - ((uint16_t)instr.instr_operand);
    }
    else
    {
        uint32_t address = decode_effective_adress(instr, memory, registers);
        extra_cycles = cycle_page_crossing_panalty(instr.instr_operand, address);
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
    return instr.instr_cycles + extra_cycles;
}

uint32_t instr_asl(instruction instr, uint8_t* memory, register_file* registers)
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
    return instr.instr_cycles;
}

uint32_t instr_lsr(instruction instr, uint8_t* memory, register_file* registers)
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
    return instr.instr_cycles;
}

uint32_t instr_rol(instruction instr, uint8_t* memory, register_file* registers)
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
    return instr.instr_cycles;
}

uint32_t instr_ror(instruction instr, uint8_t* memory, register_file* registers)
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
    return instr.instr_cycles;
}

uint32_t instr_bit(instruction instr, uint8_t* memory, register_file* registers)
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
    return instr.instr_cycles;
}

uint32_t instr_brk(instruction instr, uint8_t* memory, register_file* registers)
{
    memory[STACK_PAGE+registers->reg_s] = (uint8_t)((registers->reg_pc & 0xFF00)>>8);
    memory[STACK_PAGE+registers->reg_s-1] = (uint8_t)(registers->reg_pc & 0x00FF);
    memory[STACK_PAGE+registers->reg_s-2] = registers->reg_flags | FLAG_UNUSED | FLAG_BREAK;
    registers->reg_s -=3;
    registers->reg_pc = *((uint16_t*)(memory+IRQ_VECTOR));
    return instr.instr_cycles;
}

uint32_t instr_rti(instruction instr, uint8_t* memory, register_file* registers)
{
    registers->reg_flags = memory[STACK_PAGE+(registers->reg_s+1)] & ~FLAG_BREAK;
    registers->reg_pc = ((((uint16_t)memory[STACK_PAGE+(registers->reg_s+3)])&0x00FF)<<8) | (((uint16_t)memory[STACK_PAGE+(registers->reg_s+2)])&0x00FF);
    registers->reg_s += 3;
    return instr.instr_cycles;
}
