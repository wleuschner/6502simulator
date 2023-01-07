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
    cycle_count += instr.instr_cycles;
    registers->reg_pc += instr.instr_length;
    switch(instr.opcode_name)
    {
        //Jump and Subroutine operations
        case OPCODE::JMP:
        {
            registers->reg_pc = decode_effective_adress(instr, memory, registers);
            break;
        }
        case OPCODE::JSR:
        {
            instr_jsr(instr, memory, registers);
            break;
        }
        case OPCODE::RTS:
        {
            instr_rts(memory, registers);
            break;
        }

        //Interrupt operations
        case OPCODE::BRK:
        {
            instr_brk(memory, registers);
            break;
        }
        case OPCODE::RTI:
        {
            instr_rti(memory, registers);
            break;
        }

        //Compare operations
        case OPCODE::CMP:
        {
            instr_cmp(instr, memory, registers, &registers->reg_a);
            break;
        }
        case OPCODE::CPX:
        {
            instr_cmp(instr, memory, registers, &registers->reg_x);
            break;
        }
        case OPCODE::CPY:
        {
            instr_cmp(instr, memory, registers, &registers->reg_y);
            break;
        }

        //Branch operations
        case OPCODE::BCC:
        {
            instr_branch(instr, registers, !(registers->reg_flags & FLAG_CARRY));
            break;
        }
        case OPCODE::BCS:
        {
            instr_branch(instr, registers, (registers->reg_flags & FLAG_CARRY));
            break;
        }
        case OPCODE::BNE:
        {
            instr_branch(instr, registers, !(registers->reg_flags & FLAG_ZERO));
            break;
        }
        case OPCODE::BEQ:
        {
            instr_branch(instr, registers, (registers->reg_flags & FLAG_ZERO));
            break;
        }
        case OPCODE::BPL:
        {
            instr_branch(instr, registers, !(registers->reg_flags & FLAG_NEGATIVE));
            break;
        }
        case OPCODE::BMI:
        {
            instr_branch(instr, registers, (registers->reg_flags & FLAG_NEGATIVE));
            break;
        }
        case OPCODE::BVC:
        {
            instr_branch(instr, registers, !(registers->reg_flags & FLAG_OVERFLOW));
            break;
        }
        case OPCODE::BVS:
        {
            instr_branch(instr, registers, (registers->reg_flags & FLAG_OVERFLOW));
            break;
        }

        //Load
        case OPCODE::LDA:
        {
            instr_load(instr, memory, registers, &registers->reg_a);
            break;
        }
        case OPCODE::LDX:
        {
            instr_load(instr, memory, registers, &registers->reg_x);
            break;
        }
        case OPCODE::LDY:
        {
            instr_load(instr, memory, registers, &registers->reg_y);
            break;
        }

        //Store
        case OPCODE::STA:
        {
            instr_store(instr, memory, registers, &registers->reg_a);
            break;
        }
        case OPCODE::STX:
        {
            instr_store(instr, memory, registers, &registers->reg_x);
            break;
        }
        case OPCODE::STY:
        {
            instr_store(instr, memory, registers, &registers->reg_y);
            break;
        }

        //Transfer
        case OPCODE::TAX:
        {
            instr_transfer(registers, &registers->reg_a, &registers->reg_x);
            break;
        }
        case OPCODE::TAY:
        {
            instr_transfer(registers, &registers->reg_a, &registers->reg_y);
            break;
        }
        case OPCODE::TSX:
        {
            instr_transfer(registers, &registers->reg_s, &registers->reg_x);
            break;
        }
        case OPCODE::TXA:
        {
            instr_transfer(registers, &registers->reg_x, &registers->reg_a);
            break;
        }
        case OPCODE::TXS:
        {
            instr_transfer(registers, &registers->reg_x, &registers->reg_s);
            break;
        }
        case OPCODE::TYA:
        {
            instr_transfer(registers, &registers->reg_y, &registers->reg_a);
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
            instr_inc(registers, memory + decode_effective_adress(instr, memory, registers));
            break;
        }
        case OPCODE::INX:
        {
            instr_inc(registers, &registers->reg_x);
            break;
        }
        case OPCODE::INY:
        {
            instr_inc(registers, &registers->reg_y);
            break;
        }

        //Decrement
        case OPCODE::DEC:
        {
            instr_dec(registers, memory + decode_effective_adress(instr, memory, registers));
            break;
        }
        case OPCODE::DEX:
        {
            instr_dec(registers, &registers->reg_x);
            break;
        }
        case OPCODE::DEY:
        {
            instr_dec(registers, &registers->reg_y);
            break;
        }

        //Logic operations
        case OPCODE::AND:
        {
            instr_and(instr, memory, registers);
            break;
        }
        case OPCODE::EOR:
        {
            instr_eor(instr, memory, registers);
            break;
        }
        case OPCODE::ORA:
        {
            instr_ora(instr, memory, registers);
            break;
        }
        //Shift operations
        case OPCODE::ASL:
        {
            instr_asl(instr, memory, registers);
            break;
        }
        case OPCODE::LSR:
        {
            instr_lsr(instr, memory, registers);
            break;
        }
        case OPCODE::ROL:
        {
            instr_rol(instr, memory, registers);
            break;
        }
        case OPCODE::ROR:
        {
            instr_ror(instr, memory, registers);
            break;
        }

        //Arithmetic operations
        case OPCODE::ADC:
        {
            instr_adc(instr, memory, registers);
            break;
        }
        case OPCODE::SBC:
        {
            instr_sbb(instr, memory, registers);
            break;
        }

        //Stack operations
        case OPCODE::PHA:
        {
            instr_pha(memory, registers);
            break;
        }
        case OPCODE::PHP:
        {
            instr_php(memory, registers);
            break;
        }
        case OPCODE::PLA:
        {
            instr_pla(memory, registers);
            break;
        }
        case OPCODE::PLP:
        {
            instr_plp(memory, registers);
            break;
        }

        //Misc operations
        case OPCODE::BIT:
        {
            instr_bit(instr, memory, registers);
            break;
        }
    }
    return instr;
}
