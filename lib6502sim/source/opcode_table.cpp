#include "opcode_table.h"

instruction opcode_table[256] = {
    {OPCODE::BRK, ADDRESS_MODE::IMPLICIT},
    {OPCODE::ORA, ADDRESS_MODE::INDEXED_X},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::ORA, ADDRESS_MODE::ZEROPAGE},
    {OPCODE::ASL, ADDRESS_MODE::ZEROPAGE},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::PHP, ADDRESS_MODE::IMPLICIT},
    {OPCODE::ORA, ADDRESS_MODE::IMMIDIATE},
    {OPCODE::ASL, ADDRESS_MODE::IMPLICIT},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::ORA, ADDRESS_MODE::ABSOLUTE},
    {OPCODE::ASL, ADDRESS_MODE::ABSOLUTE},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::BPL, ADDRESS_MODE::RELATIVE},
    {OPCODE::ORA, ADDRESS_MODE::INDEXED_Y},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::ORA, ADDRESS_MODE::ZEROPAGEX},
    {OPCODE::ASL, ADDRESS_MODE::ZEROPAGEX},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::CLC, ADDRESS_MODE::IMPLICIT},
    {OPCODE::ORA, ADDRESS_MODE::ABSOLUTE_Y},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::ORA, ADDRESS_MODE::ABSOLUTE_X},
    {OPCODE::ASL, ADDRESS_MODE::ABSOLUTE_X},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::JSR, ADDRESS_MODE::ABSOLUTE},
    {OPCODE::AND, ADDRESS_MODE::INDEXED_X},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::BIT, ADDRESS_MODE::ZEROPAGE},
    {OPCODE::AND, ADDRESS_MODE::ZEROPAGE},
    {OPCODE::ROL, ADDRESS_MODE::ZEROPAGE},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::PLP, ADDRESS_MODE::IMPLICIT},
    {OPCODE::AND, ADDRESS_MODE::IMMIDIATE},
    {OPCODE::ROL, ADDRESS_MODE::IMPLICIT},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::BIT, ADDRESS_MODE::ABSOLUTE},
    {OPCODE::AND, ADDRESS_MODE::ABSOLUTE},
    {OPCODE::ROL, ADDRESS_MODE::ABSOLUTE},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::BMI, ADDRESS_MODE::RELATIVE},
    {OPCODE::AND, ADDRESS_MODE::INDEXED_Y},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::AND, ADDRESS_MODE::ZEROPAGEX},
    {OPCODE::ROL, ADDRESS_MODE::ZEROPAGEX},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::SEC, ADDRESS_MODE::IMPLICIT},
    {OPCODE::AND, ADDRESS_MODE::ABSOLUTE_Y},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::AND, ADDRESS_MODE::ABSOLUTE_X},
    {OPCODE::ROL, ADDRESS_MODE::ABSOLUTE_X},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::RTI, ADDRESS_MODE::IMPLICIT},
    {OPCODE::EOR, ADDRESS_MODE::INDEXED_X},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::EOR, ADDRESS_MODE::ZEROPAGE},
    {OPCODE::LSR, ADDRESS_MODE::ZEROPAGE},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::PHA, ADDRESS_MODE::IMPLICIT},
    {OPCODE::EOR, ADDRESS_MODE::IMMIDIATE},
    {OPCODE::LSR, ADDRESS_MODE::IMPLICIT},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::JMP, ADDRESS_MODE::ABSOLUTE},
    {OPCODE::EOR, ADDRESS_MODE::ABSOLUTE},
    {OPCODE::LSR, ADDRESS_MODE::ABSOLUTE},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::BVC, ADDRESS_MODE::RELATIVE},
    {OPCODE::EOR, ADDRESS_MODE::INDEXED_Y},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::EOR, ADDRESS_MODE::ZEROPAGEX},
    {OPCODE::LSR, ADDRESS_MODE::ZEROPAGEX},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::CLI, ADDRESS_MODE::IMPLICIT},
    {OPCODE::EOR, ADDRESS_MODE::ABSOLUTE_Y},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::EOR, ADDRESS_MODE::ABSOLUTE_X},
    {OPCODE::LSR, ADDRESS_MODE::ABSOLUTE_X},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::RTS, ADDRESS_MODE::IMPLICIT},
    {OPCODE::ADC, ADDRESS_MODE::INDEXED_X},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::ADC, ADDRESS_MODE::ZEROPAGE},
    {OPCODE::ROR, ADDRESS_MODE::ZEROPAGE},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::PLA, ADDRESS_MODE::IMPLICIT},
    {OPCODE::ADC, ADDRESS_MODE::IMMIDIATE},
    {OPCODE::ROR, ADDRESS_MODE::IMPLICIT},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::JMP, ADDRESS_MODE::INDIRECT},
    {OPCODE::ADC, ADDRESS_MODE::ABSOLUTE},
    {OPCODE::ROR, ADDRESS_MODE::ABSOLUTE},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::BVS, ADDRESS_MODE::RELATIVE},
    {OPCODE::ADC, ADDRESS_MODE::INDEXED_Y},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::ADC, ADDRESS_MODE::ZEROPAGEX},
    {OPCODE::ROR, ADDRESS_MODE::ZEROPAGEX},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::SEI, ADDRESS_MODE::IMPLICIT},
    {OPCODE::ADC, ADDRESS_MODE::ABSOLUTE_Y},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::ADC, ADDRESS_MODE::ABSOLUTE_X},
    {OPCODE::ROR, ADDRESS_MODE::ABSOLUTE_X},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::STA, ADDRESS_MODE::INDEXED_X},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::STY, ADDRESS_MODE::ZEROPAGE},
    {OPCODE::STA, ADDRESS_MODE::ZEROPAGE},
    {OPCODE::STX, ADDRESS_MODE::ZEROPAGE},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::DEY, ADDRESS_MODE::IMPLICIT},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::TXA, ADDRESS_MODE::IMPLICIT},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::STY, ADDRESS_MODE::ABSOLUTE},
    {OPCODE::STA, ADDRESS_MODE::ABSOLUTE},
    {OPCODE::STX, ADDRESS_MODE::ABSOLUTE},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::BCC, ADDRESS_MODE::RELATIVE},
    {OPCODE::STA, ADDRESS_MODE::INDEXED_Y},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::STY, ADDRESS_MODE::ZEROPAGEX},
    {OPCODE::STA, ADDRESS_MODE::ZEROPAGEX},
    {OPCODE::STX, ADDRESS_MODE::ZEROPAGEY},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::TYA, ADDRESS_MODE::IMPLICIT},
    {OPCODE::STA, ADDRESS_MODE::ABSOLUTE_Y},
    {OPCODE::TXS, ADDRESS_MODE::IMPLICIT},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::STA, ADDRESS_MODE::ABSOLUTE_X},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::LDY, ADDRESS_MODE::IMMIDIATE},
    {OPCODE::LDA, ADDRESS_MODE::INDEXED_X},
    {OPCODE::LDX, ADDRESS_MODE::IMMIDIATE},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::LDY, ADDRESS_MODE::ZEROPAGE},
    {OPCODE::LDA, ADDRESS_MODE::ZEROPAGE},
    {OPCODE::LDX, ADDRESS_MODE::ZEROPAGE},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::TAY, ADDRESS_MODE::IMPLICIT},
    {OPCODE::LDA, ADDRESS_MODE::IMMIDIATE},
    {OPCODE::TAX, ADDRESS_MODE::IMPLICIT},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::LDY, ADDRESS_MODE::ABSOLUTE},
    {OPCODE::LDA, ADDRESS_MODE::ABSOLUTE},
    {OPCODE::LDX, ADDRESS_MODE::ABSOLUTE},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::BCS, ADDRESS_MODE::RELATIVE},
    {OPCODE::LDA, ADDRESS_MODE::INDEXED_Y},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::LDY, ADDRESS_MODE::ZEROPAGEX},
    {OPCODE::LDA, ADDRESS_MODE::ZEROPAGEX},
    {OPCODE::LDX, ADDRESS_MODE::ZEROPAGEY},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::CLV, ADDRESS_MODE::IMPLICIT},
    {OPCODE::LDA, ADDRESS_MODE::ABSOLUTE_Y},
    {OPCODE::TSX, ADDRESS_MODE::IMPLICIT},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::LDY, ADDRESS_MODE::ABSOLUTE_X},
    {OPCODE::LDA, ADDRESS_MODE::ABSOLUTE_X},
    {OPCODE::LDX, ADDRESS_MODE::ABSOLUTE_Y},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::CPY, ADDRESS_MODE::IMMIDIATE},
    {OPCODE::CMP, ADDRESS_MODE::INDEXED_X},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::CPY, ADDRESS_MODE::ZEROPAGE},
    {OPCODE::CMP, ADDRESS_MODE::ZEROPAGE},
    {OPCODE::DEC, ADDRESS_MODE::ZEROPAGE},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::INY, ADDRESS_MODE::IMPLICIT},
    {OPCODE::CMP, ADDRESS_MODE::IMMIDIATE},
    {OPCODE::DEX, ADDRESS_MODE::IMPLICIT},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::CPY, ADDRESS_MODE::ABSOLUTE},
    {OPCODE::CMP, ADDRESS_MODE::ABSOLUTE},
    {OPCODE::DEC, ADDRESS_MODE::ABSOLUTE},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::BNE, ADDRESS_MODE::RELATIVE},
    {OPCODE::CMP, ADDRESS_MODE::INDEXED_Y},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::CMP, ADDRESS_MODE::ZEROPAGEX},
    {OPCODE::DEC, ADDRESS_MODE::ZEROPAGEX},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::CLD, ADDRESS_MODE::IMPLICIT},
    {OPCODE::CMP, ADDRESS_MODE::ABSOLUTE_Y},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::CMP, ADDRESS_MODE::ABSOLUTE_X},
    {OPCODE::DEC, ADDRESS_MODE::ABSOLUTE_X},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::CPX, ADDRESS_MODE::IMMIDIATE},
    {OPCODE::SBC, ADDRESS_MODE::INDEXED_X},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::CPX, ADDRESS_MODE::ZEROPAGE},
    {OPCODE::SBC, ADDRESS_MODE::ZEROPAGE},
    {OPCODE::INC, ADDRESS_MODE::ZEROPAGE},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::INX, ADDRESS_MODE::IMPLICIT},
    {OPCODE::SBC, ADDRESS_MODE::IMMIDIATE},
    {OPCODE::NOP, ADDRESS_MODE::IMPLICIT},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::CPX, ADDRESS_MODE::ABSOLUTE},
    {OPCODE::SBC, ADDRESS_MODE::ABSOLUTE},
    {OPCODE::INC, ADDRESS_MODE::ABSOLUTE},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::BEQ, ADDRESS_MODE::RELATIVE},
    {OPCODE::SBC, ADDRESS_MODE::INDEXED_Y},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::SBC, ADDRESS_MODE::ZEROPAGEX},
    {OPCODE::INC, ADDRESS_MODE::ZEROPAGEX},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::SED, ADDRESS_MODE::IMPLICIT},
    {OPCODE::SBC, ADDRESS_MODE::ABSOLUTE_Y},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT},
    {OPCODE::SBC, ADDRESS_MODE::ABSOLUTE_X},
    {OPCODE::INC, ADDRESS_MODE::ABSOLUTE_X},
    {OPCODE::JAM, ADDRESS_MODE::IMPLICIT}
};

const char* INSTRUCTION_STRINGS[] =
{
    "ADC",
    "AND",
    "ASL",
    "BCC",
    "BCS",
    "BEQ",
    "BIT",
    "BMI",
    "BNE",
    "BPL",
    "BRK",
    "BVC",
    "BVS",
    "CLC",
    "CLD",
    "CLI",
    "CLV",
    "CMP",
    "CPX",
    "CPY",
    "DEC",
    "DEX",
    "DEY",
    "EOR",
    "INC",
    "INX",
    "INY",
    "JMP",
    "JSR",
    "LDA",
    "LDX",
    "LDY",
    "LSR",
    "NOP",
    "ORA",
    "PHA",
    "PHP",
    "PLA",
    "PLP",
    "ROL",
    "ROR",
    "RTI",
    "RTS",
    "SBC",
    "SEC",
    "SED",
    "SEI",
    "STA",
    "STX",
    "STY",
    "TAX",
    "TAY",
    "TSX",
    "TXA",
    "TXS",
    "TYA",

    //Illegal Opcodes
    "JAM"
    "ALR",
    "ANC",
    "ANE",
    "ARR",
    "DCP",
    "ISC",
    "LAS",
    "LAX",
    "LXA",
    "RLA",
    "RRA",
    "SAX",
    "SBX",
    "SHA",
    "SHX",
    "SHY",
    "SLO",
    "SRE",
    "TAS",
    "USBC",
};
