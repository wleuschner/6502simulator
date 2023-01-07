#include "opcode.h"
#include "opcode_table.h"
#include "register_file.h"
#include "instructions.h"
#include "decode.h"
#include <simulator.h>

#include <cinttypes>
#include <cstdlib>
#include <cstring>
#include <cstdio>

uint8_t* memory = nullptr;
uint16_t memory_size = 0;
register_file* registers = nullptr;
uint16_t program_size = 0;
uint32_t cycle_count = 0;

void init(uint16_t mem_size)
{
    cycle_count = 0;
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

uint32_t get_cycle_count()
{
    return cycle_count;
}

register_file get_registers()
{
    return *registers;
}

std::vector<instruction> disassemble_program()
{
    uint16_t offset = 0;
    std::vector<instruction> opcodes;
    while(offset<program_size)
    {
        instruction instr = decode_instruction(memory, offset);
        opcodes.push_back(instr);
        offset += instr.instr_length;
    }
    return opcodes;
}

instruction step()
{
    instruction instr = decode_instruction(memory, registers->reg_pc);
    registers->reg_pc += instr.instr_length;
    switch(instr.opcode_name)
    {
        //Jump and Subroutine operations
        case OPCODE::JMP:
        {
            cycle_count += instr_jmp(instr, memory, registers);
            break;
        }
        case OPCODE::JSR:
        {
            cycle_count += instr_jsr(instr, memory, registers);
            break;
        }
        case OPCODE::RTS:
        {
            cycle_count += instr_rts(instr, memory, registers);
            break;
        }

        //Interrupt operations
        case OPCODE::BRK:
        {
            cycle_count += instr_brk(instr, memory, registers);
            break;
        }
        case OPCODE::RTI:
        {
            cycle_count += instr_rti(instr, memory, registers);
            break;
        }

        //Compare operations
        case OPCODE::CMP:
        {
            cycle_count += instr_cmp(instr, memory, registers, &registers->reg_a);
            break;
        }
        case OPCODE::CPX:
        {
            cycle_count += instr_cmp(instr, memory, registers, &registers->reg_x);
            break;
        }
        case OPCODE::CPY:
        {
            cycle_count += instr_cmp(instr, memory, registers, &registers->reg_y);
            break;
        }

        //Branch operations
        case OPCODE::BCC:
        {
            cycle_count += instr_branch(instr, registers, !(registers->reg_flags & FLAG_CARRY));
            break;
        }
        case OPCODE::BCS:
        {
            cycle_count += instr_branch(instr, registers, (registers->reg_flags & FLAG_CARRY));
            break;
        }
        case OPCODE::BNE:
        {
            cycle_count += instr_branch(instr, registers, !(registers->reg_flags & FLAG_ZERO));
            break;
        }
        case OPCODE::BEQ:
        {
            cycle_count += instr_branch(instr, registers, (registers->reg_flags & FLAG_ZERO));
            break;
        }
        case OPCODE::BPL:
        {
            cycle_count += instr_branch(instr, registers, !(registers->reg_flags & FLAG_NEGATIVE));
            break;
        }
        case OPCODE::BMI:
        {
            cycle_count += instr_branch(instr, registers, (registers->reg_flags & FLAG_NEGATIVE));
            break;
        }
        case OPCODE::BVC:
        {
            cycle_count += instr_branch(instr, registers, !(registers->reg_flags & FLAG_OVERFLOW));
            break;
        }
        case OPCODE::BVS:
        {
            cycle_count += instr_branch(instr, registers, (registers->reg_flags & FLAG_OVERFLOW));
            break;
        }

        //Load
        case OPCODE::LDA:
        {
            cycle_count += instr_load(instr, memory, registers, &registers->reg_a);
            break;
        }
        case OPCODE::LDX:
        {
            cycle_count += instr_load(instr, memory, registers, &registers->reg_x);
            break;
        }
        case OPCODE::LDY:
        {
            cycle_count += instr_load(instr, memory, registers, &registers->reg_y);
            break;
        }

        //Store
        case OPCODE::STA:
        {
            cycle_count += instr_store(instr, memory, registers, &registers->reg_a);
            break;
        }
        case OPCODE::STX:
        {
            cycle_count += instr_store(instr, memory, registers, &registers->reg_x);
            break;
        }
        case OPCODE::STY:
        {
            cycle_count += instr_store(instr, memory, registers, &registers->reg_y);
            break;
        }

        //Transfer
        case OPCODE::TAX:
        {
            cycle_count += instr_transfer(instr, registers, &registers->reg_a, &registers->reg_x);
            break;
        }
        case OPCODE::TAY:
        {
            cycle_count += instr_transfer(instr, registers, &registers->reg_a, &registers->reg_y);
            break;
        }
        case OPCODE::TSX:
        {
            cycle_count += instr_transfer(instr, registers, &registers->reg_s, &registers->reg_x);
            break;
        }
        case OPCODE::TXA:
        {
            cycle_count += instr_transfer(instr, registers, &registers->reg_x, &registers->reg_a);
            break;
        }
        case OPCODE::TXS:
        {
            cycle_count += instr_transfer(instr, registers, &registers->reg_x, &registers->reg_s);
            break;
        }
        case OPCODE::TYA:
        {
            cycle_count += instr_transfer(instr, registers, &registers->reg_y, &registers->reg_a);
            break;
        }

        //Set Flags
        case OPCODE::SEC:
        {
            cycle_count += instr_set_flag(instr, registers, FLAG_CARRY);
            break;
        }
        case OPCODE::SED:
        {
            cycle_count += instr_set_flag(instr, registers, FLAG_DECIMAL);
            break;
        }
        case OPCODE::SEI:
        {
            cycle_count += instr_set_flag(instr, registers, FLAG_INTERRUPT);
            break;
        }

        //Clear Flags
        case OPCODE::CLC:
        {
            cycle_count += instr_clear_flag(instr, registers, FLAG_CARRY);
            break;
        }
        case OPCODE::CLD:
        {
            cycle_count += instr_clear_flag(instr, registers, FLAG_DECIMAL);
            break;
        }
        case OPCODE::CLI:
        {
            cycle_count += instr_clear_flag(instr, registers, FLAG_INTERRUPT);
            break;
        }
        case OPCODE::CLV:
        {
            cycle_count += instr_clear_flag(instr, registers, FLAG_OVERFLOW);
            break;
        }

        //Increment
        case OPCODE::INC:
        {
            cycle_count += instr_inc(instr, registers, memory + decode_effective_adress(instr, memory, registers));
            break;
        }
        case OPCODE::INX:
        {
            cycle_count += instr_inc(instr, registers, &registers->reg_x);
            break;
        }
        case OPCODE::INY:
        {
            cycle_count += instr_inc(instr, registers, &registers->reg_y);
            break;
        }

        //Decrement
        case OPCODE::DEC:
        {
            cycle_count += instr_dec(instr, registers, memory + decode_effective_adress(instr, memory, registers));
            break;
        }
        case OPCODE::DEX:
        {
            cycle_count += instr_dec(instr, registers, &registers->reg_x);
            break;
        }
        case OPCODE::DEY:
        {
            cycle_count += instr_dec(instr, registers, &registers->reg_y);
            break;
        }

        //Logic operations
        case OPCODE::AND:
        {
            cycle_count += instr_and(instr, memory, registers);
            break;
        }
        case OPCODE::EOR:
        {
            cycle_count += instr_eor(instr, memory, registers);
            break;
        }
        case OPCODE::ORA:
        {
            cycle_count += instr_ora(instr, memory, registers);
            break;
        }
        //Shift operations
        case OPCODE::ASL:
        {
            cycle_count += instr_asl(instr, memory, registers);
            break;
        }
        case OPCODE::LSR:
        {
            cycle_count += instr_lsr(instr, memory, registers);
            break;
        }
        case OPCODE::ROL:
        {
            cycle_count += instr_rol(instr, memory, registers);
            break;
        }
        case OPCODE::ROR:
        {
            cycle_count += instr_ror(instr, memory, registers);
            break;
        }

        //Arithmetic operations
        case OPCODE::ADC:
        {
            cycle_count += instr_adc(instr, memory, registers);
            break;
        }
        case OPCODE::SBC:
        {
            cycle_count += instr_sbb(instr, memory, registers);
            break;
        }

        //Stack operations
        case OPCODE::PHA:
        {
            cycle_count += instr_pha(instr, memory, registers);
            break;
        }
        case OPCODE::PHP:
        {
            cycle_count += instr_php(instr, memory, registers);
            break;
        }
        case OPCODE::PLA:
        {
            cycle_count += instr_pla(instr, memory, registers);
            break;
        }
        case OPCODE::PLP:
        {
            cycle_count += instr_plp(instr, memory, registers);
            break;
        }

        //Misc operations
        case OPCODE::BIT:
        {
            cycle_count += instr_bit(instr, memory, registers);
            break;
        }
        case OPCODE::NOP:
        {
            cycle_count += instr.instr_cycles;
            break;
        }
    }
    return instr;
}
