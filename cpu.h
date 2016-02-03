#ifndef CPU_H_INCLUDED_
#define CPU_H_INCLUDED_ 

#include <stdint.h>

#define MAX_MEMORY 8 * 1024

typedef unsigned char byte;

typedef struct _state {
    uint8_t sign:1; 
    uint8_t overflow:1;
    uint8_t breakpoint:1;
    uint8_t decimal:1;
    uint8_t interrupt:1;
    uint8_t zero:1;
    uint8_t carry:1;
} State;

typedef struct _cpu {
    uint8_t acc;
    uint8_t x;
    uint8_t y;
    State s;
    uint16_t sp; // 0x01ff -> 0x0100
    uint16_t pc;
    uint8_t memory[MAX_MEMORY];
} Cpu;

void cpu_initialize(Cpu *cpu);
int cpu_debugDecodeInstruction(Cpu *cpu, byte *buffer);

#endif /* CPU_H_INCLUDED_ */
