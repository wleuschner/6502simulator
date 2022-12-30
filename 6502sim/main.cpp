#include <cstdio>
#include <climits>
#include "simulator.h"
#include "opcode_table.h"

int main(int argc,char** argv)
{
    if(argc!=2)
    {
        printf("No 6502 binary specified");
        return 0;
    }
    init(USHRT_MAX);
    load_program(argv[1], 0);
    char input;
    char buffer[256];
    do
    {
        printf("> ");
        scanf(" %c",&input);
        switch (input)
        {
            case 's':
            {
                instruction instr = step();
                opcode_to_string(buffer, 256, instr);
                printf("%s",buffer);
                break;
            }
            case 'r':
            {
                register_to_string(buffer,256);
                printf("%s",buffer);
                break;
            }
            case 'd':
            {
                std::vector<instruction> opcodes = disassemble_program();
                for (instruction op : opcodes) {
                    opcode_to_string(buffer, 256, op);
                    printf("%s\r\n",buffer);
                }
                break;
            }
        }
        printf("\r\n");
    }while(input!='q');
    deinit();
    return 0;
}
