#pragma once
#include "opcode.h"
#include "register_file.h"
#include <cinttypes>

uint32_t instr_jmp(instruction instr, uint8_t* memory, register_file* registers);
uint32_t instr_load(instruction instr, uint8_t* memory, register_file* registers,uint8_t* reg);
uint32_t instr_store(instruction instr, uint8_t* memory, register_file* registers, uint8_t* reg);
uint32_t instr_transfer(instruction instr, register_file* registers, uint8_t* from,uint8_t* to);
uint32_t instr_set_flag(instruction instr, register_file* registers, uint8_t flag);
uint32_t instr_clear_flag(instruction instr, register_file* registers, uint8_t flag);
uint32_t instr_inc(instruction instr, register_file* registers, uint8_t* reg);
uint32_t instr_dec(instruction instr, register_file* registers, uint8_t* reg);
uint32_t instr_and(instruction instr, uint8_t* memory, register_file* registers);
uint32_t instr_eor(instruction instr, uint8_t* memory, register_file* registers);
uint32_t instr_ora(instruction instr, uint8_t* memory, register_file* registers);
uint32_t instr_pha(instruction instr, uint8_t* memory, register_file* registers);
uint32_t instr_php(instruction instr, uint8_t* memory, register_file* registers);
uint32_t instr_pla(instruction instr, uint8_t* memory, register_file* registers);
uint32_t instr_plp(instruction instr, uint8_t* memory, register_file* registers);
uint32_t instr_jsr(instruction instr, uint8_t* memory, register_file* registers);
uint32_t instr_rts(instruction instr, uint8_t* memory, register_file* registers);
uint32_t instr_branch(instruction instr, register_file* registers, bool take);
uint32_t instr_adc(instruction instr, uint8_t* memory, register_file* registers);
uint32_t instr_sbb(instruction instr, uint8_t* memory, register_file* registers);
uint32_t instr_cmp(instruction instr, uint8_t* memory, register_file* registers,uint8_t* reg);
uint32_t instr_asl(instruction instr, uint8_t* memory, register_file* registers);
uint32_t instr_lsr(instruction instr, uint8_t* memory, register_file* registers);
uint32_t instr_rol(instruction instr, uint8_t* memory, register_file* registers);
uint32_t instr_ror(instruction instr, uint8_t* memory, register_file* registers);
uint32_t instr_bit(instruction instr, uint8_t* memory, register_file* registers);
uint32_t instr_brk(instruction instr, uint8_t* memory, register_file* registers);
uint32_t instr_rti(instruction instr, uint8_t* memory, register_file* registers);
