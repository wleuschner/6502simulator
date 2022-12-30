#pragma once
#include<inttypes.h>

extern const char* INSTRUCTION_STRINGS[];

enum ADDRESS_MODE
{
    IMPLICIT = 0,
    IMMIDIATE,
    ZEROPAGE,
    ZEROPAGEX,
    ZEROPAGEY,
    ABSOLUTE,
    ABSOLUTE_X,
    ABSOLUTE_Y,
    INDIRECT,
    INDEXED_X,
    INDEXED_Y,
    RELATIVE
};

enum OPCODE
{
    ADC = 0,
    AND,
    ASL,
    BCC,
    BCS,
    BEQ,
    BIT,
    BMI,
    BNE,
    BPL,
    BRK,
    BVC,
    BVS,
    CLC,
    CLD,
    CLI,
    CLV,
    CMP,
    CPX,
    CPY,
    DEC,
    DEX,
    DEY,
    EOR,
    INC,
    INX,
    INY,
    JMP,
    JSR,
    LDA,
    LDX,
    LDY,
    LSR,
    NOP,
    ORA,
    PHA,
    PHP,
    PLA,
    PLP,
    ROL,
    ROR,
    RTI,
    RTS,
    SBC,
    SEC,
    SED,
    SEI,
    STA,
    STX,
    STY,
    TAX,
    TAY,
    TSX,
    TXA,
    TXS,
    TYA,

    //Illegal Opcodes
    JAM,
    ALR,
    ANC,
    ANE,
    ARR,
    DCP,
    ISC,
    LAS,
    LAX,
    LXA,
    RLA,
    RRA,
    SAX,
    SBX,
    SHA,
    SHX,
    SHY,
    SLO,
    SRE,
    TAS,
    USBC,
};

struct instruction
{
    instruction(OPCODE opcode, ADDRESS_MODE mode)
        : opcode_name(opcode), opcode_mode(mode), instr_operand(0)
    {};

    OPCODE opcode_name = OPCODE::JAM;
    ADDRESS_MODE opcode_mode = ADDRESS_MODE::IMPLICIT;
    uint16_t instr_length = 1;
    uint16_t instr_operand = 0;
};
