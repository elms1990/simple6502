#include <stdio.h>
#include <stdlib.h>

#include "cpu.h"

int main(int argc, char *argv[]) {
    Cpu cpu;
    cpu_initialize(&cpu);

    FILE *f = fopen(argv[1], "r");
    fseek(f, 0, SEEK_END);
    
    int sz = ftell(f);
    fseek(f, 0, SEEK_SET);
    byte *buffer = calloc(sz, sizeof(byte));
    fread(buffer, 1, sz, f);

    fclose(f);

    while (cpu.pc < sz) {
        cpu.pc += cpu_debugDecodeInstruction(&cpu, buffer);
    }
    
    return 0;
}
