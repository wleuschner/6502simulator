#pragma once
#include <inttypes.h>

#define FLAG_CARRY 1
#define FLAG_ZERO 2
#define FLAG_INTERRUPT 4
#define FLAG_DECIMAL 8
#define FLAG_BREAK 16
#define FLAG_UNUSED 32
#define FLAG_OVERFLOW 64
#define FLAG_NEGATIVE 128

struct register_file
{
    uint8_t reg_a = 0;
    uint8_t reg_x = 0;
    uint8_t reg_y = 0;
    uint8_t reg_s = 0;
    uint16_t reg_pc = 0;
    uint8_t reg_flags = 0;
};
