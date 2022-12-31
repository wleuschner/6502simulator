#include <cstdio>
#include <climits>
#include "simulator.h"

int main(int argc,char** argv)
{
    if(argc!=2)
    {
        printf("No 6502 binary specified");
        return 0;
    }
    init(USHRT_MAX);
    load_program(argv[1], 0);
    std::vector<instruction> opcodes = disassemble_program();
    char buffer[256];
    for (instruction op : opcodes) {
        opcode_to_string(buffer, 256, op);
        printf("%s\r\n",buffer);
    }
    deinit();
    return 0;
}
