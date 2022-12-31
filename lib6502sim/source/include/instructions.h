#pragma once
#include "opcode.h"
#include "register_file.h"
#include <cinttypes>

void instr_load(instruction instr, uint8_t* memory, register_file* registers,uint8_t* reg);
void instr_store(instruction instr, uint8_t* memory, register_file* registers, uint8_t* reg);
void instr_transfer(register_file* registers, uint8_t* from,uint8_t* to);
void instr_inc(register_file* registers, uint8_t* reg);
void instr_dec(register_file* registers, uint8_t* reg);
void instr_and(instruction instr, uint8_t* memory, register_file* registers);
void instr_eor(instruction instr, uint8_t* memory, register_file* registers);
void instr_ora(instruction instr, uint8_t* memory, register_file* registers);
void instr_pha(uint8_t* memory, register_file* registers);
void instr_php(uint8_t* memory, register_file* registers);
void instr_pla(uint8_t* memory, register_file* registers);
void instr_plp(uint8_t* memory, register_file* registers);
void instr_jsr(instruction instr, uint8_t* memory, register_file* registers);
void instr_rts(uint8_t* memory, register_file* registers);
void instr_branch(instruction instr, register_file* registers, bool take);
void instr_adc(instruction instr, uint8_t* memory, register_file* registers);
void instr_sbb(instruction instr, uint8_t* memory, register_file* registers);
void instr_cmp(instruction instr, uint8_t* memory, register_file* registers,uint8_t* reg);
void instr_asl(instruction instr, uint8_t* memory, register_file* registers);
void instr_lsr(instruction instr, uint8_t* memory, register_file* registers);
void instr_rol(instruction instr, uint8_t* memory, register_file* registers);
void instr_ror(instruction instr, uint8_t* memory, register_file* registers);
void instr_bit(instruction instr, uint8_t* memory, register_file* registers);
void instr_brk(uint8_t* memory, register_file* registers);
void instr_rti(uint8_t* memory, register_file* registers);
