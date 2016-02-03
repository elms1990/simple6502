#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "cpu.h"

#define WORD_MAX 127
#define WORD_MIN -128

#define MASK_CARRY(x) ((x & 0x100) >> 8)
#define MASK_SIGN(x) ((x & 0x80) >> 7)

#define STATE_SIGN MASK_SIGN
#define STATE_OVERFLOW(x) ((x & 0x40) >> 6)
#define STATE_BREAKPOINT(x) ((x & 0x10) >> 4)
#define STATE_DECIMAL(x) ((x & 0x08) >> 3)
#define STATE_INTERRUPT(x) ((x & 0x04) >> 2)
#define STATE_ZERO(x) ((x & 0x02) >> 1)
#define STATE_CARRY(x) (x & 0x01)

#define MASK_BIT0(x) (x & 0x01)

static void cpu_setArithmeticFlags(Cpu *cpu, uint16_t result, 
        uint8_t op1, uint8_t op2);
static void cpu_setZNFlags(Cpu *cpu, uint16_t result);
static uint16_t cpu_fetchIIAX(Cpu *cpu, byte *buffer);
static uint16_t cpu_fetchIIAY(Cpu *cpu, byte *buffer);
static uint8_t cpu_lowerByte(uint16_t dword);
static uint8_t cpu_higherByte(uint16_t dword);
static uint16_t cpu_toDWORD(uint8_t higher, uint8_t lower);
static void cpu_setStateBit(Cpu *cpu, int bit);
static void cpu_clearStateBit(Cpu *cpu, int bit);
static uint8_t cpu_stateToWord(Cpu *cpu);
static void cpu_wordToState(Cpu *cpu, uint8_t word);

static inline void cpu_setArithmeticFlags(Cpu *cpu, uint16_t result, 
        uint8_t op1, uint8_t op2) {
    cpu->s.sign = MASK_SIGN(result);
    cpu->s.overflow = ((MASK_SIGN(op1) == MASK_SIGN(op2)) &&
            (MASK_SIGN(cpu_lowerByte(result)) != MASK_SIGN(op1)));
    cpu->s.zero = (result & 0xff) == 0 ? 1 : 0;
    cpu->s.carry = (result > WORD_MAX || result < WORD_MIN); 
}

static inline void cpu_setZNFlags(Cpu *cpu, uint16_t result) {
    cpu->s.sign = MASK_SIGN(result);
    cpu->s.zero = (result & 0xff) == 0 ? 1 : 0;
}

static inline uint16_t cpu_fetchIIAX(Cpu *cpu, byte *buffer) {
    uint8_t baseAddress = (uint8_t) buffer[cpu->pc + 1] + cpu->x;
    return ((uint16_t) cpu->memory[baseAddress + 1] << 8) | 
        cpu->memory[baseAddress];
}

static inline uint16_t cpu_fetchIIAY(Cpu *cpu, byte *buffer) {
    uint16_t address = (uint16_t) cpu->memory[buffer[cpu->pc + 1]] +
        (uint16_t) cpu->y;
    uint8_t lower = cpu_lowerByte(address);
    uint8_t higher = MASK_CARRY(address) + 
        (uint16_t) cpu->memory[buffer[cpu->pc + 1] + 1];
    address = cpu_toDWORD(higher, lower);

    return address;
}

static inline uint8_t cpu_lowerByte(uint16_t dword) {
    return (uint8_t) (dword & 0xff);
}

static inline uint8_t cpu_higherByte(uint16_t dword) {
    return (uint8_t) ((dword & 0xff00) >> 8);
}

static inline uint16_t cpu_toDWORD(uint8_t higher, uint8_t lower) {
    return (((uint16_t) higher) << 8) | ((uint16_t) lower);
}

static inline void cpu_setStateBit(Cpu *cpu, int bit) {
    switch (bit) {
        case 7:
            cpu->s.sign = 1;
            break;
        case 6:
            cpu->s.overflow = 1;
            break;
        case 4:
            cpu->s.breakpoint = 1;
            break;
        case 3:
            cpu->s.decimal = 1;
            break;
        case 2:
            cpu->s.interrupt = 1;
            break;
        case 1:
            cpu->s.zero = 1;
            break;
        case 0:
            cpu->s.carry = 1;
            break;
    }
}

static inline void cpu_clearStateBit(Cpu *cpu, int bit) {
    switch (bit) {
        case 7:
            cpu->s.sign = 0;
            break;
        case 6:
            cpu->s.overflow = 0;
            break;
        case 4:
            cpu->s.breakpoint = 0;
            break;
        case 3:
            cpu->s.decimal = 0;
            break;
        case 2:
            cpu->s.interrupt = 0;
            break;
        case 1:
            cpu->s.zero = 0;
            break;
        case 0:
            cpu->s.carry = 0;
            break;
    }
}

static inline uint8_t cpu_stateToWord(Cpu *cpu) {
    return (cpu->s.sign << 7) | (cpu->s.overflow << 6) | 
        (cpu->s.breakpoint << 4) | (cpu->s.decimal << 3) | 
        (cpu->s.interrupt << 2) | (cpu->s.zero << 1) | cpu->s.carry;
}

static inline void cpu_wordToState(Cpu *cpu, uint8_t word) {
    cpu->s.sign = STATE_OVERFLOW(word);
    cpu->s.overflow = STATE_SIGN(word);
    cpu->s.breakpoint = STATE_BREAKPOINT(word);
    cpu->s.decimal = STATE_DECIMAL(word);
    cpu->s.interrupt = STATE_INTERRUPT(word);
    cpu->s.zero = STATE_ZERO(word);
    cpu->s.carry = STATE_CARRY(word);
}

void cpu_initialize(Cpu *cpu) {
    memset(cpu, 0, sizeof(Cpu));

    cpu->sp = 0x01ff;
}

int cpu_debugDecodeInstruction(Cpu *cpu, byte *buffer) { 
    int opBytes = 1;

    switch(buffer[cpu->pc]) {
        case 0x00: // BRK
            {
                #ifdef DEBUG
                printf("BRK\n");
                #endif 

            }
            break;
        case 0x01: // ORA ($NN, X)
            {
                #ifdef DEBUG
                printf("ORA $%02x,X\n", buffer[cpu->pc + 1]);
                #endif 
                opBytes = 2;

                uint16_t address = cpu_fetchIIAX(cpu, buffer);
                uint16_t result = (uint16_t) cpu->acc | 
                    (uint16_t) cpu->memory[address];
                cpu_setZNFlags(cpu, result);
                cpu->acc = result;
            }
            break;
        case 0x05: // ORA $NN
            {
                #ifdef DEBUG
                printf("ORA $%02x\n", buffer[cpu->pc + 1]);
                #endif 
                opBytes = 2;

                uint16_t result = (uint16_t) cpu->acc | 
                    (uint16_t) cpu->memory[buffer[cpu->pc + 1]];
                cpu_setZNFlags(cpu, result);
                cpu->acc = result;
            }
            break;
        case 0x06: // ASL $NN
            {
                #ifdef DEBUG
                printf("ASL $%02x,X\n", buffer[cpu->pc + 1]);
                #endif 
                opBytes = 2;

                uint8_t address = buffer[cpu->pc + 1];
                uint8_t result = (uint16_t) cpu->memory[address];
                cpu->s.carry = MASK_SIGN(result);
                cpu->memory[address] = result << 1;
                cpu_setZNFlags(cpu, cpu->memory[address]);
            }
            break;
        case 0x08: // PHP
            {
                #ifdef DEBUG
                printf("PHP\n");
                #endif 

                cpu->memory[cpu->sp] = cpu_stateToWord(cpu);
                cpu->sp--;
            }
            break;
        case 0x09: // ORA #$NN
            {
                #ifdef DEBUG
                printf("ORA $%02x\n", buffer[cpu->pc + 1]);
                #endif 
                opBytes = 2;

                uint16_t result = (uint16_t) cpu->acc | 
                    (uint16_t) buffer[cpu->pc + 1];
                cpu_setZNFlags(cpu, result);
                cpu->acc = result;
            }
            break;
        case 0x0a: // ASL A
            {
                #ifdef DEBUG
                printf("ASL A\n");
                #endif 

                cpu->s.carry = MASK_SIGN(cpu->acc);
                cpu->acc <<= 1;
                cpu_setZNFlags(cpu, cpu->acc);
            }
            break;
        case 0x0d: // ORA $NNNN
            {
                #ifdef DEBUG
                printf("ORA $%02x%02x\n", buffer[cpu->pc + 2], buffer[cpu->pc + 1]);
                #endif 
                opBytes = 3;

                uint16_t address = cpu_toDWORD(buffer[cpu->pc + 2], 
                        buffer[cpu->pc + 1]);
                uint16_t result = (uint16_t) cpu->acc | 
                    (uint16_t) cpu->memory[address];
                cpu_setZNFlags(cpu, result);
                cpu->acc = result;
            }
            break;
        case 0x0e: // ASL $NNNN
            {
                #ifdef DEBUG
                printf("ASL $%02x%02x\n", buffer[cpu->pc + 2], buffer[cpu->pc + 1]);
                #endif 
                opBytes = 3;

                uint16_t address = cpu_toDWORD(buffer[cpu->pc + 2], 
                        buffer[cpu->pc + 1]);
                uint16_t result = cpu->memory[address];
                cpu->s.carry = MASK_SIGN(result);
                cpu->memory[address] = result << 1;
                cpu_setZNFlags(cpu, cpu->memory[address]);
            }
            break;
        case 0x10: // BPL $NN
            #ifdef DEBUG
            printf("BPL $%02x\n", buffer[cpu->pc + 1]);
            #endif 
            opBytes = 2;
            break;
        case 0x11: // ORA ($NN),Y
            {
                #ifdef DEBUG
                printf("ORA $%02x,Y\n", buffer[cpu->pc + 1]);
                #endif 
                opBytes = 2;

                uint16_t address = cpu_fetchIIAY(cpu, buffer);
                uint16_t result = cpu->acc | cpu->memory[address];
                cpu_setZNFlags(cpu, result);
                cpu->acc = result;
            }
            break;
        case 0x15: // ORA $NN,X
            {
                #ifdef DEBUG
                printf("ORA $%02x,X\n", buffer[cpu->pc + 1]);
                #endif 
                opBytes = 2;

                uint16_t address = (uint16_t) cpu->x + (uint16_t) buffer[cpu->pc + 1];
                uint16_t result = (uint16_t) cpu->acc | 
                    (uint16_t) cpu->memory[address];
                cpu_setZNFlags(cpu, result);
                cpu->acc = result;
            }
            break;
        case 0x16: // ASL $NN,X
            {
                #ifdef DEBUG
                printf("ASL $%02x,X\n", buffer[cpu->pc + 1]);
                #endif 
                opBytes = 2;

                uint16_t address = (uint16_t) cpu->x + (uint16_t) buffer[cpu->pc + 1];
                uint16_t result = cpu->memory[address];
                cpu->s.carry = MASK_SIGN(result);
                cpu->memory[address] = result << 1;
                cpu_setZNFlags(cpu, cpu->memory[address]);
            }
            break;
        case 0x18: // CLC
            {
                #ifdef DEBUG
                printf("CLC\n");
                #endif

                cpu->s.carry = 0;
            }
            break;
        case 0x19: // ORA $NNNN,Y
            {
                #ifdef DEBUG
                printf("ORA $%02x%02x,Y\n", buffer[cpu->pc + 2], buffer[cpu->pc + 1]);
                #endif 
                opBytes = 3;

                uint16_t address = cpu_toDWORD(buffer[cpu->pc + 2], 
                        buffer[cpu->pc + 1]) + (uint16_t) cpu->y;
                uint16_t result = (uint16_t) cpu->acc | 
                    (uint16_t) cpu->memory[address];
                cpu_setArithmeticFlags(cpu, result, cpu->acc, 
                        cpu->memory[address]);
                cpu->acc = result;
            }
            break;
        case 0x1d: // ORA $NNNN,X
            {
                #ifdef DEBUG
                printf("ORA $%02x%02x,X\n", buffer[cpu->pc + 2], buffer[cpu->pc + 1]);
                #endif 
                opBytes = 3;

                uint16_t address = cpu_toDWORD(buffer[cpu->pc + 2], 
                        buffer[cpu->pc + 1]) + (uint16_t) cpu->x;
                uint16_t result = (uint16_t) cpu->acc | 
                    (uint16_t) cpu->memory[address];
                cpu_setZNFlags(cpu, result);
                cpu->acc = result;
            }
            break;
        case 0x1e: // ASL $NNNN,X
            {
                #ifdef DEBUG
                printf("ASL $%02x%02x,X\n", buffer[cpu->pc + 2], buffer[cpu->pc + 1]);
                #endif 
                opBytes = 3;

                uint16_t address = cpu_toDWORD(buffer[cpu->pc + 2], 
                        buffer[cpu->pc + 1]) + (uint16_t) cpu->x;
                uint16_t result = cpu->memory[address];
                cpu->s.carry = MASK_SIGN(result);
                cpu->memory[address] = result << 1;
                cpu_setZNFlags(cpu, cpu->memory[address]);
            }
            break;
        case 0x20: // JSR $NNNN
            #ifdef DEBUG
            printf("JSR $%02x%02x\n", buffer[cpu->pc + 2], buffer[cpu->pc + 1]);
            #endif 
            opBytes = 3;
            break;
        case 0x21: // AND($NN,X)
            {
                #ifdef DEBUG
                printf("AND ($%02x,X)\n", buffer[cpu->pc + 1]);
                #endif 
                opBytes = 2;

                uint16_t address = cpu_fetchIIAX(cpu, buffer);
                uint16_t result = (uint16_t) cpu->acc & 
                    (uint16_t) cpu->memory[address];
                cpu_setZNFlags(cpu, result);
                cpu->acc = result;
            }
            break;
        case 0x24: // BIT $NN
            #ifdef DEBUG
            printf("BIT $%02x\n", buffer[cpu->pc + 1]);
            #endif 
            opBytes = 2;
            break;
        case 0x25: // AND $NN
            {
                #ifdef DEBUG
                printf("AND $%02x\n", buffer[cpu->pc + 1]);
                #endif 
                opBytes = 2;

                uint16_t result = (uint16_t) cpu->acc & 
                    (uint16_t) cpu->memory[buffer[cpu->pc + 1]];
                cpu_setZNFlags(cpu, result);
                cpu->acc = result;
            }
            break;
        case 0x26: // ROL $NN
            {
                #ifdef DEBUG
                printf("ROL $%02x\n", buffer[cpu->pc + 1]);
                #endif 
                opBytes = 2;

                uint8_t address = buffer[cpu->pc + 1];
                uint8_t result = (uint16_t) cpu->memory[address];
                cpu->s.carry = MASK_SIGN(result);
                cpu->memory[address] = (result << 1) | cpu->s.carry ;
                cpu_setZNFlags(cpu, cpu->memory[address]);
            }
            break;
        case 0x28: // PLP
            {
                #ifdef DEBUG
                printf("PLP\n");
                #endif

                cpu->sp++;
                cpu_wordToState(cpu, cpu->memory[cpu->sp]);
            }
            break;
        case 0x29: // AND #$NN
            {
                #ifdef DEBUG
                printf("AND #$%02x\n", buffer[cpu->pc + 1]);
                #endif 
                opBytes = 2;

                uint16_t result = (uint16_t) cpu->acc & 
                    (uint16_t) buffer[cpu->pc + 1];
                cpu_setZNFlags(cpu, result);
                cpu->acc = result;
            }
            break;
        case 0x2a: // ROL A 
            {
                #ifdef DEBUG
                printf("ROL A\n");
                #endif

                cpu->s.carry = MASK_SIGN(cpu->acc); 
                cpu->acc = (cpu->acc << 1) | cpu->s.carry;
                cpu_setZNFlags(cpu, cpu->acc);
            }
            break;
        case 0x2c: // BIT $NNNN
            #ifdef DEBUG
            printf("BIT $%02x%02x\n", buffer[cpu->pc + 2], buffer[cpu->pc + 1]);
            #endif 
            opBytes = 3;
            break;
        case 0x2d: // AND $NNNN
            {
                #ifdef DEBUG
                printf("AND $%02x%02x\n", buffer[cpu->pc + 2], buffer[cpu->pc + 1]);
                #endif 
                opBytes = 3;

                uint16_t address = cpu_toDWORD(buffer[cpu->pc + 2], 
                        buffer[cpu->pc + 1]);
                uint16_t result = (uint16_t) cpu->acc & 
                    (uint16_t) cpu->memory[address];
                cpu_setZNFlags(cpu, result);
                cpu->acc = result;
            }
            break;
        case 0x2e: // ROL $NNNN
            {
                #ifdef DEBUG
                printf("ROL $%02x%02x\n", buffer[cpu->pc + 2], buffer[cpu->pc + 1]);
                #endif 
                opBytes = 3;

                uint16_t address = cpu_toDWORD(buffer[cpu->pc + 2], 
                        buffer[cpu->pc + 1]);
                uint16_t result = cpu->memory[address];
                cpu->s.carry = MASK_SIGN(result);
                cpu->memory[address] = (result << 1) | cpu->s.carry;
                cpu_setZNFlags(cpu, cpu->memory[address]);
            }
            break;
        case 0x30: // BMI $NN
            #ifdef DEBUG
            printf("BMI $%02x\n", buffer[cpu->pc + 1]);
            #endif 
            opBytes = 2;
            break;
        case 0x31: // AND ($NN),Y
            {
                #ifdef DEBUG
                printf("AND $%02x,Y\n", buffer[cpu->pc + 1]);
                #endif 
                opBytes = 2;

                uint16_t address = cpu_fetchIIAY(cpu, buffer);
                uint16_t result = cpu->acc & cpu->memory[address];
                cpu_setZNFlags(cpu, result);
                cpu->acc = result;
            }
            break;
        case 0x35: // AND $NN,X
            {
                #ifdef DEBUG
                printf("AND $%02x,X\n", buffer[cpu->pc + 1]);
                #endif 
                opBytes = 2;

                uint16_t address = (uint16_t) cpu->x + (uint16_t) buffer[cpu->pc + 1];
                uint16_t result = (uint16_t) cpu->acc & 
                    (uint16_t) cpu->memory[address];
                cpu_setZNFlags(cpu, result);
                cpu->acc = result;
            }
            break;
        case 0x36: // ROL $NN,X
            {
                #ifdef DEBUG
                printf("ROL $%02x,X\n", buffer[cpu->pc + 1]);
                #endif 
                opBytes = 2;

                uint16_t address = (uint16_t) cpu->x + (uint16_t) buffer[cpu->pc + 1];
                uint16_t result = cpu->memory[address];
                cpu->s.carry = MASK_SIGN(result);
                cpu->memory[address] = (result << 1) | cpu->s.carry;
                cpu_setZNFlags(cpu, cpu->memory[address]);
            }
            break;
        case 0x38: // SEC
            {
                #ifdef DEBUG
                printf("SEC\n");
                #endif 

                cpu->s.carry = 1;
            }
            break;
        case 0x39: // AND $NNNN,Y
            {
                #ifdef DEBUG
                printf("AND $%02x%02x,Y\n", buffer[cpu->pc + 2], buffer[cpu->pc + 1]);
                #endif 
                opBytes = 3;

                uint16_t address = cpu_toDWORD(buffer[cpu->pc + 2], 
                        buffer[cpu->pc + 1]) + (uint16_t) cpu->y;
                uint16_t result = (uint16_t) cpu->acc & 
                    (uint16_t) cpu->memory[address];
                cpu_setArithmeticFlags(cpu, result, cpu->acc, 
                        cpu->memory[address]);
                cpu->acc = result;
            }
            break;
        case 0x3d: // AND $NNNN,X
            {
                #ifdef DEBUG
                printf("AND $%02x%02x,X\n", buffer[cpu->pc + 2], buffer[cpu->pc + 1]);
                #endif 
                opBytes = 3;

                uint16_t address = cpu_toDWORD(buffer[cpu->pc + 2], 
                        buffer[cpu->pc + 1]) + (uint16_t) cpu->x;
                uint16_t result = (uint16_t) cpu->acc & 
                    (uint16_t) cpu->memory[address];
                cpu_setZNFlags(cpu, result);
                cpu->acc = result;
            }
            break;
        case 0x3e: // ROL $NNNN,X
            {
                #ifdef DEBUG
                printf("ROL $%02x%02x,X\n", buffer[cpu->pc + 2], buffer[cpu->pc + 1]);
                #endif 
                opBytes = 3;

                uint16_t address = cpu_toDWORD(buffer[cpu->pc + 2], 
                        buffer[cpu->pc + 1]) + (uint16_t) cpu->x;
                uint16_t result = cpu->memory[address];
                cpu->s.carry = MASK_SIGN(result);
                cpu->memory[address] = (result << 1) | cpu->s.carry;
                cpu_setZNFlags(cpu, cpu->memory[address]);
            }
            break;
        case 0x40: // RTI
            #ifdef DEBUG
            printf("RTI\n");
            #endif 
            break;
        case 0x41: // EOR($NN,X)
            {
                #ifdef DEBUG
                printf("EOR $%02x,X\n", buffer[cpu->pc + 1]);
                #endif 
                opBytes = 2;

                uint16_t address = cpu_fetchIIAX(cpu, buffer);
                uint16_t result = (uint16_t) cpu->acc ^ 
                    (uint16_t) cpu->memory[address];
                cpu_setZNFlags(cpu, result);
                cpu->acc = result;
            }
            break;
        case 0x45: // EOR $NN
            { 
                #ifdef DEBUG
                printf("EOR $%02x\n", buffer[cpu->pc + 1]);
                #endif 
                opBytes = 2;

                uint16_t result = (uint16_t) cpu->acc ^ 
                    (uint16_t) cpu->memory[buffer[cpu->pc + 1]];
                cpu_setZNFlags(cpu, result);
                cpu->acc = result;
            }
            break;
        case 0x46: // LSR $NN
            {
                #ifdef DEBUG
                printf("LSR $%02x\n", buffer[cpu->pc + 1]);
                #endif 
                opBytes = 2;

                uint8_t address = buffer[cpu->pc + 1];
                uint8_t result = (uint16_t) cpu->memory[address];
                cpu->s.carry = MASK_BIT0(result);
                cpu->memory[address] = result >> 1;
                cpu_setZNFlags(cpu, cpu->memory[address]);
                cpu_clearStateBit(cpu, 7);
            }
            break;
        case 0x48: // PHA
            {
                #ifdef DEBUG
                printf("PHA\n");
                #endif

                cpu->memory[cpu->sp] = cpu->acc;
                cpu->sp--;
            }
            break;
        case 0x49: // EOR #$NN
            {
                #ifdef DEBUG
                printf("EOR #$%02x\n", buffer[cpu->pc + 1]);
                #endif 
                opBytes = 2;

                uint16_t result = (uint16_t) cpu->acc ^ 
                    (uint16_t) buffer[cpu->pc + 1];
                cpu_setZNFlags(cpu, result);
                cpu->acc = result;
            }
            break;
        case 0x4a: // LSR A
            {
                #ifdef DEBUG
                printf("LSR A\n");
                #endif 

                cpu->s.carry = MASK_BIT0(cpu->acc);
                cpu->acc >>= 1;
                cpu_setZNFlags(cpu, cpu->acc);
                cpu_clearStateBit(cpu, 7);
            }
            break;
        case 0x4c: // JMP $NNNN
            #ifdef DEBUG
            printf("JMP $%02x%02x,X\n", buffer[cpu->pc + 2], buffer[cpu->pc + 1]);
            #endif 
            opBytes = 3;
            break;
        case 0x4d: // EOR $NNNN 
            { 
                #ifdef DEBUG
                printf("EOR $%02x%02x,X\n", buffer[cpu->pc + 2], buffer[cpu->pc + 1]);
                #endif 
                opBytes = 3;

                uint16_t address = cpu_toDWORD(buffer[cpu->pc + 2], 
                        buffer[cpu->pc + 1]);
                uint16_t result = (uint16_t) cpu->acc ^ 
                    (uint16_t) cpu->memory[address];
                cpu_setZNFlags(cpu, result);
                cpu->acc = result;
            }
            break;
        case 0x4e: // LSR $NNNN
            {
                #ifdef DEBUG
                printf("LSR $%02x%02x,X\n", buffer[cpu->pc + 2], buffer[cpu->pc + 1]);
                #endif 
                opBytes = 3;

                uint16_t address = cpu_toDWORD(buffer[cpu->pc + 2], 
                        buffer[cpu->pc + 1]);
                uint16_t result = cpu->memory[address];
                cpu->s.carry = MASK_BIT0(result);
                cpu->memory[address] = result >> 1;
                cpu_setZNFlags(cpu, cpu->memory[address]);
                cpu_clearStateBit(cpu, 7);
            }
            break;
        case 0x50: // BVC $NN
            #ifdef DEBUG
            printf("BVC $%02x,X\n", buffer[cpu->pc + 1]);
            #endif 
            opBytes = 2;
            break;
        case 0x51: // EOR($NN),Y
            {
                #ifdef DEBUG
                printf("EOR $%02x,Y\n", buffer[cpu->pc + 1]);
                #endif 
                opBytes = 2;

                uint16_t address = cpu_fetchIIAY(cpu, buffer);
                uint16_t result = cpu->acc ^ cpu->memory[address];
                cpu_setZNFlags(cpu, result);
                cpu->acc = result;
            }
            break;
        case 0x55: // EOR $NN,X
            {
                #ifdef DEBUG
                printf("EOR $%02x,X\n", buffer[cpu->pc + 1]);
                #endif 
                opBytes = 2;

                uint16_t address = (uint16_t) cpu->x + (uint16_t) buffer[cpu->pc + 1];
                uint16_t result = (uint16_t) cpu->acc ^ 
                    (uint16_t) cpu->memory[address];
                cpu_setZNFlags(cpu, result);
                cpu->acc = result;

            }
            break;
        case 0x56: // LSR $NN,X
            {
                #ifdef DEBUG
                printf("LSR $%02x,X\n", buffer[cpu->pc + 1]);
                #endif 
                opBytes = 2;

                uint16_t address = (uint16_t) cpu->x + (uint16_t) buffer[cpu->pc + 1];
                uint16_t result = cpu->memory[address];
                cpu->s.carry = MASK_BIT0(result);
                cpu->memory[address] = result >> 1;
                cpu_setZNFlags(cpu, cpu->memory[address]);
                cpu_clearStateBit(cpu, 7);
            }
            break;
        case 0x58: // CLI
            {
                #ifdef DEBUG
                printf("CLI\n");
                #endif 

                cpu->s.interrupt = 0;
            }
            break;
        case 0x59: // EOR $NNNN,Y
            {
                #ifdef DEBUG
                printf("EOR $%02x%02x,Y\n", buffer[cpu->pc + 2], buffer[cpu->pc + 1]);
                #endif 
                opBytes = 3;

                uint16_t address = cpu_toDWORD(buffer[cpu->pc + 2], 
                        buffer[cpu->pc + 1]) + (uint16_t) cpu->y;
                uint16_t result = (uint16_t) cpu->acc ^ 
                    (uint16_t) cpu->memory[address];
                cpu_setArithmeticFlags(cpu, result, cpu->acc, 
                        cpu->memory[address]);
                cpu->acc = result;
            }
            break;
        case 0x5d: // EOR $NNNN,X
            {
                #ifdef DEBUG
                printf("EOR $%02x%02x,X\n", buffer[cpu->pc + 2], buffer[cpu->pc + 1]);
                #endif 
                opBytes = 3;

                uint16_t address = cpu_toDWORD(buffer[cpu->pc + 2], 
                        buffer[cpu->pc + 1]) + (uint16_t) cpu->x;
                uint16_t result = (uint16_t) cpu->acc ^ 
                    (uint16_t) cpu->memory[address];
                cpu_setZNFlags(cpu, result);
                cpu->acc = result;
            }
            break;
        case 0x5e: // LSR $NNNN,X
            {
                #ifdef DEBUG
                printf("LSR $%02x%02x,X\n", buffer[cpu->pc + 2], buffer[cpu->pc + 1]);
                #endif 
                opBytes = 3;

                uint16_t address = cpu_toDWORD(buffer[cpu->pc + 2], 
                        buffer[cpu->pc + 1]) + (uint16_t) cpu->x;
                uint16_t result = cpu->memory[address];
                cpu->s.carry = MASK_BIT0(result);
                cpu->memory[address] = result >> 1;
                cpu_setZNFlags(cpu, cpu->memory[address]);
                cpu_clearStateBit(cpu, 7);
            }
            break;
        case 0x60: // RTS
            #ifdef DEBUG
            printf("RTS\n");
            #endif
            break;
        case 0x61: // ADC($NN,X)
            {
                #ifdef DEBUG
                printf("ADC $%02x,X\n", buffer[cpu->pc + 1]);
                #endif 
                opBytes = 2;

                uint16_t address = cpu_fetchIIAX(cpu, buffer);
                uint16_t result = (uint16_t) cpu->acc + 
                    (uint16_t) cpu->memory[address];
                cpu_setArithmeticFlags(cpu, result, cpu->acc, cpu->memory[address]);
                cpu->acc = result;
            }
            break;
        case 0x65: // ADC $NN
            {
                #ifdef DEBUG
                printf("ADC $%02x\n", buffer[cpu->pc + 1]);
                #endif 
                opBytes = 2;

                uint16_t result = (uint16_t) cpu->acc + 
                    (uint16_t) cpu->memory[buffer[cpu->pc + 1]];
                cpu_setArithmeticFlags(cpu, result, cpu->acc, 
                        cpu->memory[buffer[cpu->pc + 1]]);
                cpu->acc = result;
            }
            break;
        case 0x66: // ROR $NN
            {
                #ifdef DEBUG
                printf("ROR $%02x\n", buffer[cpu->pc + 1]);
                #endif 
                opBytes = 2;

                uint8_t address = buffer[cpu->pc + 1];
                uint8_t result = (uint16_t) cpu->memory[address];
                cpu->s.carry = MASK_BIT0(result);
                cpu->memory[address] = (result >> 1) | (cpu->s.carry << 7) ;
                cpu_setZNFlags(cpu, cpu->memory[address]);
            }
            break;
        case 0x68: // PLA
            {
                #ifdef DEBUG
                printf("PLA\n");
                #endif

                cpu->sp++;
                cpu->acc = cpu->memory[cpu->sp];
            }
            break;
        case 0x69: // ADC #$NN
            {
                #ifdef DEBUG
                printf("ADC #$%02x\n", buffer[cpu->pc + 1]);
                #endif 
                opBytes = 2;

                uint16_t result = (uint16_t) cpu->acc + 
                    (uint16_t) buffer[cpu->pc + 1];
                cpu_setArithmeticFlags(cpu, result, cpu->acc, 
                        buffer[cpu->pc + 1]);
                cpu->acc = result;
            }
            break;
        case 0x6a: // ROR A
            {
                #ifdef DEBUG
                printf("ROR A\n");
                #endif 

                cpu->s.carry = MASK_BIT0(cpu->acc);
                cpu->acc = (cpu->acc >> 1) | (cpu->s.carry << 7);
                cpu_setZNFlags(cpu, cpu->acc);
            }
            break;
        case 0x6c: // JMP $NN
            #ifdef DEBUG
            printf("JMP $%02x\n", buffer[cpu->pc + 1]);
            #endif 
            opBytes = 2;
            break;
        case 0x6d: // ADC $NNNN
            {
                #ifdef DEBUG
                printf("ADC $%02x%02x\n", buffer[cpu->pc + 2], buffer[cpu->pc + 1]);
                #endif 
                opBytes = 3;

                uint16_t address = cpu_toDWORD(buffer[cpu->pc + 2], 
                        buffer[cpu->pc + 1]);
                uint16_t result = (uint16_t) cpu->acc + 
                    (uint16_t) cpu->memory[address];
                cpu_setArithmeticFlags(cpu, result, cpu->acc, 
                        cpu->memory[address]);
                cpu->acc = result;
            }
            break;
        case 0x6e: // ROR $NNNN,X
            {
                #ifdef DEBUG
                printf("ROR $%02x%02x\n", buffer[cpu->pc + 2], buffer[cpu->pc + 1]);
                #endif 
                opBytes = 3;

                uint16_t address = cpu_toDWORD(buffer[cpu->pc + 2], 
                        buffer[cpu->pc + 1]);
                uint16_t result = cpu->memory[address];
                cpu->s.carry = MASK_BIT0(result);
                cpu->memory[address] = (result >> 1) | (cpu->s.carry << 7);
                cpu_setZNFlags(cpu, cpu->memory[address]);
            }
            break;
        case 0x70: // BVS $NN
            #ifdef DEBUG
            printf("BVS $%02x\n", buffer[cpu->pc + 1]);
            #endif 
            opBytes = 2;
            break;
        case 0x71: // ADC($NN),Y
            {
                #ifdef DEBUG
                printf("ADC $%02x,Y\n", buffer[cpu->pc + 1]);
                #endif 
                opBytes = 2;

                uint16_t address = cpu_fetchIIAY(cpu, buffer);
                uint16_t result = cpu->acc + cpu->memory[address];
                cpu_setArithmeticFlags(cpu, result, cpu->acc, 
                        cpu->memory[address]);
                cpu->acc = result;
            }
            break;
        case 0x75: // ADC $NN,X
            {
                #ifdef DEBUG
                printf("ADC $%02x,X\n", buffer[cpu->pc + 1]);
                #endif 
                opBytes = 2;

                uint16_t address = (uint16_t) cpu->x + (uint16_t) buffer[cpu->pc + 1];
                uint16_t result = (uint16_t) cpu->acc + 
                    (uint16_t) cpu->memory[address];
                cpu_setArithmeticFlags(cpu, result, cpu->acc, cpu->memory[address]);
                cpu->acc = result;
            }
            break;
        case 0x76: // ROR $NN,X
            {
                #ifdef DEBUG
                printf("ROR $%02x,X\n", buffer[cpu->pc + 1]);
                #endif 
                opBytes = 2;

                uint16_t address = (uint16_t) cpu->x + (uint16_t) buffer[cpu->pc + 1];
                uint16_t result = cpu->memory[address];
                cpu->s.carry = MASK_BIT0(result);
                cpu->memory[address] = (result >> 1) | (cpu->s.carry << 7);
                cpu_setZNFlags(cpu, cpu->memory[address]);
            }
            break;
        case 0x78: // SEI
            {
                #ifdef DEBUG
                printf("SEI\n");
                #endif

                cpu->s.interrupt = 1;
            }
            break;
        case 0x79: // ADC $NNNN,Y
            {
                #ifdef DEBUG
                printf("ADC $%02x%02x,Y\n", buffer[cpu->pc + 2], buffer[cpu->pc + 1]);
                #endif 
                opBytes = 3;

                uint16_t address = cpu_toDWORD(buffer[cpu->pc + 2], 
                        buffer[cpu->pc + 1]) + (uint16_t) cpu->y;
                uint16_t result = (uint16_t) cpu->acc + 
                    (uint16_t) cpu->memory[address];
                cpu_setArithmeticFlags(cpu, result, cpu->acc, 
                        cpu->memory[address]);
                cpu->acc = result;
            }
            break;
        case 0x7d: // ADC $NNNN,X
            {
                #ifdef DEBUG
                printf("ADC $%02x%02x,X\n", buffer[cpu->pc + 2], buffer[cpu->pc + 1]);
                #endif 
                opBytes = 3;

                uint16_t address = cpu_toDWORD(buffer[cpu->pc + 2], 
                        buffer[cpu->pc + 1]) + (uint16_t) cpu->x;
                uint16_t result = (uint16_t) cpu->acc + 
                    (uint16_t) cpu->memory[address];
                cpu_setArithmeticFlags(cpu, result, cpu->acc, 
                        cpu->memory[address]);
                cpu->acc = result;
            }
            break;
        case 0x7e: // ROR $NNNN
            {
                #ifdef DEBUG
                printf("ROR $%02x%02x\n", buffer[cpu->pc + 2], buffer[cpu->pc + 1]);
                #endif 
                opBytes = 3;

                uint16_t address = cpu_toDWORD(buffer[cpu->pc + 2], 
                        buffer[cpu->pc + 1]) + (uint16_t) cpu->x;
                uint16_t result = cpu->memory[address];
                cpu->s.carry = MASK_BIT0(result);
                cpu->memory[address] = (result >> 1) | (cpu->s.carry << 7);
                cpu_setZNFlags(cpu, cpu->memory[address]);
            }
            break;
        case 0x81: // STA($NN,X)
            {
                #ifdef DEBUG
                printf("STA $%02x,X\n", buffer[cpu->pc + 1]);
                #endif 
                opBytes = 2;

                uint16_t address = cpu_fetchIIAX(cpu, buffer);
                cpu->memory[address] = cpu->acc;
            }
            break;
        case 0x84: // STY $NN
            {
                #ifdef DEBUG
                printf("STY $%02x\n", buffer[cpu->pc + 1]);
                #endif 
                opBytes = 2;

                uint16_t address = buffer[cpu->pc + 1];
                cpu->memory[address] = cpu->y;
            }
            break;
        case 0x85: // STA $NN
            {
                #ifdef DEBUG
                printf("STA $%02x\n", buffer[cpu->pc + 1]);
                #endif 
                opBytes = 2;

                uint16_t address = buffer[cpu->pc + 1];
                cpu->memory[address] = cpu->acc;
            }
            break;
        case 0x86: // STX $NN
            {
                #ifdef DEBUG
                printf("STX $%02x\n", buffer[cpu->pc + 1]);
                #endif 
                opBytes = 2;

                uint16_t address = buffer[cpu->pc + 1];
                cpu->memory[address] = cpu->x;
            }
            break;
        case 0x88: // DEY
            {
                #ifdef DEBUG
                printf("DEY\n");
                #endif

                uint16_t result = (uint16_t) cpu->y - 1;
                cpu_setZNFlags(cpu, result);
                cpu->y = result;               
            }
            break;
        case 0x8a: // TXA
            {
                #ifdef DEBUG
                printf("TXA\n");
                #endif

                cpu->acc = cpu->x;
                cpu_setZNFlags(cpu, cpu->acc); 
            }
            break;
        case 0x8c: // STY $NNNN
            {
                #ifdef DEBUG
                printf("STY $%02x%02x\n", buffer[cpu->pc + 2], buffer[cpu->pc + 1]);
                #endif 
                opBytes = 3;

                uint16_t address = cpu_toDWORD(buffer[cpu->pc + 2], 
                        buffer[cpu->pc + 1]);
                cpu->memory[address] = cpu->y;
            }
            break;
        case 0x8d: // STA $NNNN
            {
                #ifdef DEBUG
                printf("STA $%02x%02x\n", buffer[cpu->pc + 2], buffer[cpu->pc + 1]);
                #endif 
                opBytes = 3;

                uint16_t address = cpu_toDWORD(buffer[cpu->pc + 2], 
                        buffer[cpu->pc + 1]);
                cpu->memory[address] = cpu->acc;
            }
            break;
        case 0x8e: // STX $NNNN
            {
                #ifdef DEBUG
                printf("STX $%02x%02x\n", buffer[cpu->pc + 2], buffer[cpu->pc + 1]);
                #endif 
                opBytes = 3;

                uint16_t address = cpu_toDWORD(buffer[cpu->pc + 2], 
                        buffer[cpu->pc + 1]);
                cpu->memory[address] = cpu->x;
            }
            break;
        case 0x90: // BCC $NN
            #ifdef DEBUG
            printf("BCC $%02x\n", buffer[cpu->pc + 1]);
            #endif 
            opBytes = 2;
            break;
        case 0x91: // STA ($NN),Y
            {
                #ifdef DEBUG
                printf("STA $%02x,Y\n", buffer[cpu->pc + 1]);
                #endif 
                opBytes = 2;

                uint16_t address = cpu_fetchIIAY(cpu, buffer);
                cpu->memory[address] = cpu->acc;
            }
            break;
        case 0x94: // STY $NN,X
            {
                #ifdef DEBUG
                printf("STY $%02x,X\n", buffer[cpu->pc + 1]);
                #endif 
                opBytes = 2;

                uint16_t address = (uint16_t) cpu->x + (uint16_t) buffer[cpu->pc + 1];
                cpu->memory[address] = cpu->y;
            }
            break;
        case 0x95: // STA $NN,X
            {
                #ifdef DEBUG
                printf("STA $%02x,X\n", buffer[cpu->pc + 1]);
                #endif 
                opBytes = 2;

                uint16_t address = (uint16_t) cpu->x + (uint16_t) buffer[cpu->pc + 1];
                cpu->memory[address] = cpu->acc;
            }
            break;
        case 0x96: // STX $NN,Y
            {
                #ifdef DEBUG
                printf("STX $%02x,Y\n", buffer[cpu->pc + 1]);
                #endif 
                opBytes = 2;

                uint16_t address = (uint16_t) cpu->y + (uint16_t) buffer[cpu->pc + 1];
                cpu->memory[address] = cpu->x;
            }
            break;
        case 0x98: // TYA
            {
                #ifdef DEBUG
                printf("TYA\n");
                #endif 

                cpu->acc = cpu->y;
                cpu_setZNFlags(cpu, cpu->acc); 
            }
            break;
        case 0x99: // STA $NNNN,Y
            {
                #ifdef DEBUG
                printf("STA $%02x%02x,Y\n", buffer[cpu->pc + 2], buffer[cpu->pc + 1]);
                #endif 
                opBytes = 3;

                uint16_t address = cpu_toDWORD(buffer[cpu->pc + 2], 
                        buffer[cpu->pc + 1]) + (uint16_t) cpu->y;
                cpu->memory[address] = cpu->acc;
            }
            break;
        case 0x9a: // TXS
            {
                #ifdef DEBUG
                printf("TXS\n");
                #endif 

                cpu->sp = cpu->x;
            }
            break;
        case 0x9d: // STA $NNNN,X
            {
                #ifdef DEBUG
                printf("STA $%02x%02x,X\n", buffer[cpu->pc + 2], buffer[cpu->pc + 1]);
                #endif 
                opBytes = 3;

                uint16_t address = cpu_toDWORD(buffer[cpu->pc + 2], 
                        buffer[cpu->pc + 1]) + (uint16_t) cpu->x;
                cpu->memory[address] = cpu->acc;
            }
            break;
        case 0xa0: // LDY #$NN
            {
                #ifdef DEBUG
                printf("LDY #$%02x\n", buffer[cpu->pc + 1]);
                #endif 
                opBytes = 2;

                uint16_t result = buffer[cpu->pc + 1];
                cpu_setZNFlags(cpu, result);
                cpu->y = result;
            }
            break;
        case 0xa1: // LDA ($NN,X)
            {
                #ifdef DEBUG
                printf("LDA ($%02x,X)\n", buffer[cpu->pc + 1]);
                #endif 
                opBytes = 2;

                uint16_t address = cpu_fetchIIAX(cpu, buffer);
                uint16_t result = cpu->memory[address];
                cpu_setZNFlags(cpu, result);
                cpu->acc = result;
            }
            break;
        case 0xa2: // LDX #$NN
            {
                #ifdef DEBUG
                printf("LDX #$%02x\n", buffer[cpu->pc + 1]);
                #endif 
                opBytes = 2;
 
                uint16_t result = buffer[cpu->pc + 1];
                cpu_setZNFlags(cpu, result);
                cpu->x = result;
            }
            break;
        case 0xa4: // LDY $NN
            {
                #ifdef DEBUG
                printf("LDY $%02x\n", buffer[cpu->pc + 1]);
                #endif 
                opBytes = 2;

                uint16_t address = buffer[cpu->pc + 1];
                uint16_t result = cpu->memory[address];
                cpu_setZNFlags(cpu, result);
                cpu->y = result;
            }
            break;
        case 0xa5: // LDA $NN
            {
                #ifdef DEBUG
                printf("LDA $%02x\n", buffer[cpu->pc + 1]);
                #endif 
                opBytes = 2;

                uint16_t address = buffer[cpu->pc + 1];
                uint16_t result = cpu->memory[address];
                cpu_setZNFlags(cpu, result);
                cpu->acc = result;
            }
            break;
        case 0xa6: // LDX $NN
            {
                #ifdef DEBUG
                printf("LDX $%02x\n", buffer[cpu->pc + 1]);
                #endif 
                opBytes = 2;

                uint16_t address = buffer[cpu->pc + 1];
                uint16_t result = cpu->memory[address];
                cpu_setZNFlags(cpu, result);
                cpu->x = result;
            }
            break;
        case 0xa8: // TAY
            {
                #ifdef DEBUG
                printf("TAY\n");
                #endif 

                cpu->y = cpu->acc;
                cpu_setZNFlags(cpu, cpu->y); 
            }
            break;
        case 0xa9: // LDA #$NN
            {
                #ifdef DEBUG
                printf("LDA #$%02x\n", buffer[cpu->pc + 1]);
                #endif 
                opBytes = 2;

                uint16_t result = buffer[cpu->pc + 1];
                cpu_setZNFlags(cpu, result);
                cpu->acc = result;
            }
            break;
        case 0xaa: // TAX
            {
                #ifdef DEBUG
                printf("TAX\n");
                #endif 

                cpu->x = cpu->acc;
                cpu_setZNFlags(cpu, cpu->x); 
            }
            break;
        case 0xac: // LDY $NNNN
            {
                #ifdef DEBUG
                printf("LDY $%02x%02x\n", buffer[cpu->pc + 2], buffer[cpu->pc + 1]);
                #endif 
                opBytes = 3;

                uint16_t address = cpu_toDWORD(buffer[cpu->pc + 2], 
                        buffer[cpu->pc + 1]);
                uint16_t result = (uint16_t) cpu->memory[address];
                cpu_setZNFlags(cpu, result);
                cpu->y = result;
            }
            break;
        case 0xad: // LDA $NNNN
            {
                #ifdef DEBUG
                printf("LDA $%02x%02x\n", buffer[cpu->pc + 2], buffer[cpu->pc + 1]);
                #endif 
                opBytes = 3;

                uint16_t address = cpu_toDWORD(buffer[cpu->pc + 2], 
                        buffer[cpu->pc + 1]);
                uint16_t result = (uint16_t) cpu->memory[address];
                cpu_setZNFlags(cpu, result);
                cpu->acc = result;
            }
            break;
        case 0xae: // LDX $NNNN
            {
                #ifdef DEBUG
                printf("LDX $%02x%02x\n", buffer[cpu->pc + 2], buffer[cpu->pc + 1]);
                #endif 
                opBytes = 3;

                uint16_t address = cpu_toDWORD(buffer[cpu->pc + 2], 
                        buffer[cpu->pc + 1]);
                uint16_t result = (uint16_t) cpu->memory[address];
                cpu_setZNFlags(cpu, result);
                cpu->x = result;
            }
            break;
        case 0xb0: // BCS $NN
            #ifdef DEBUG
            printf("BCS $%02x\n", buffer[cpu->pc + 1]);
            #endif 
            opBytes = 2;
            break;
        case 0xb1: // LDA ($NN),Y
            {
                #ifdef DEBUG
                printf("LDA $%02x,Y\n", buffer[cpu->pc + 1]);
                #endif 
                opBytes = 2;

                uint16_t address = cpu_fetchIIAY(cpu, buffer);
                uint16_t result = cpu->memory[address];
                cpu_setZNFlags(cpu, result);
                cpu->acc = result;
            }
            break;
        case 0xb4: // LDY $NN,X
            {
                #ifdef DEBUG
                printf("LDY $%02x,X\n", buffer[cpu->pc + 1]);
                #endif 
                opBytes = 2;

                uint16_t address = (uint16_t) cpu->y + (uint16_t) buffer[cpu->pc + 1];
                uint16_t result = (uint16_t) cpu->memory[address];
                cpu_setZNFlags(cpu, result);
                cpu->y = result;
            }
            break;
        case 0xb5: // LDA $NN,X
            {
                #ifdef DEBUG
                printf("LDA $%02x,X\n", buffer[cpu->pc + 1]);
                #endif 
                opBytes = 2;

                uint16_t address = (uint16_t) cpu->x + (uint16_t) buffer[cpu->pc + 1];
                uint16_t result = (uint16_t) cpu->memory[address];
                cpu_setZNFlags(cpu, result);
                cpu->acc = result;
            }
            break;
        case 0xb6: // LDX $NN,Y
            {
                #ifdef DEBUG
                printf("LDX $%02x,Y\n", buffer[cpu->pc + 1]);
                #endif 
                opBytes = 2;

                uint16_t address = (uint16_t) cpu->y + (uint16_t) buffer[cpu->pc + 1];
                uint16_t result = (uint16_t) cpu->memory[address];
                cpu_setZNFlags(cpu, result);
                cpu->x = result;
            }
            break;
        case 0xb8: // CLV
            {
                #ifdef DEBUG
                printf("CLV\n");
                #endif

                cpu->s.overflow = 0;
            }
            break;
        case 0xb9: // LDA $NNNN,Y
            {
                #ifdef DEBUG
                printf("LDA $%02x%02x,Y\n", buffer[cpu->pc + 2], buffer[cpu->pc + 1]);
                #endif 
                opBytes = 3;

                uint16_t address = cpu_toDWORD(buffer[cpu->pc + 2], 
                        buffer[cpu->pc + 1]) + (uint16_t) cpu->y;
                uint16_t result = cpu->memory[address];
                cpu_setZNFlags(cpu, result);
                cpu->acc = result;
            }
            break;
        case 0xba: // TSX
            {
                #ifdef DEBUG
                printf("TSX\n");
                #endif 

                cpu->x = (uint8_t) (cpu->sp & 0xff);
            }
            break;
        case 0xbc: // LDY $NNNN,X
            {
                #ifdef DEBUG
                printf("LDY $%02x%02x,X\n", buffer[cpu->pc + 2], buffer[cpu->pc + 1]);
                #endif 
                opBytes = 3;

                uint16_t address = cpu_toDWORD(buffer[cpu->pc + 2], 
                        buffer[cpu->pc + 1]) + (uint16_t) cpu->x;
                uint16_t result = cpu->memory[address];
                cpu_setZNFlags(cpu, result);
                cpu->y = result;
            }
            break;
        case 0xbd: // LDA $NNNN,X
            {
                #ifdef DEBUG
                printf("LDA $%02x%02x,X\n", buffer[cpu->pc + 2], buffer[cpu->pc + 1]);
                #endif 
                opBytes = 3;

                uint16_t address = cpu_toDWORD(buffer[cpu->pc + 2], 
                        buffer[cpu->pc + 1]) + (uint16_t) cpu->x;
                uint16_t result = (uint16_t) cpu->memory[address];
                cpu_setZNFlags(cpu, result);
                cpu->acc = result;

            }
            break;
        case 0xbe: // LDX $NNNN,Y
            {
                #ifdef DEBUG
                printf("LDX $%02x%02x,Y\n", buffer[cpu->pc + 2], buffer[cpu->pc + 1]);
                #endif 
                opBytes = 3;

                uint16_t address = cpu_toDWORD(buffer[cpu->pc + 2], 
                        buffer[cpu->pc + 1]) + (uint16_t) cpu->y;
                uint16_t result = cpu->memory[address];
                cpu_setZNFlags(cpu, result);
                cpu->x = result;
            }
            break;
        case 0xc0: // CPY #$NN
            #ifdef DEBUG
            printf("CPY #$%02x\n", buffer[cpu->pc + 1]);
            #endif 
            opBytes = 2;
            break;
        case 0xc1: // CMP($NN,X)
            #ifdef DEBUG
            printf("CMP $%02x,X\n", buffer[cpu->pc + 1]);
            #endif 
            opBytes = 2;
            break;
        case 0xc4: // CPY $NN
            #ifdef DEBUG
            printf("CPY $%02x\n", buffer[cpu->pc + 1]);
            #endif 
            opBytes = 2;
            break;
        case 0xc5: // CMP $NN
            #ifdef DEBUG
            printf("CMP $%02x\n", buffer[cpu->pc + 1]);
            #endif 
            opBytes = 2;
            break;
        case 0xc6: // DEC $NN
            {
                #ifdef DEBUG
                printf("DEC $%02x\n", buffer[cpu->pc + 1]);
                #endif 
                opBytes = 2;

                uint16_t address = buffer[cpu->pc + 1];
                uint16_t result = (uint16_t) cpu->memory[address] - 1;
                cpu_setZNFlags(cpu, result);
                cpu->memory[address] = result;               
            }
            break;
        case 0xc8: // INY
            {
                #ifdef DEBUG
                printf("INY\n");
                #endif 

                uint16_t result = (uint16_t) cpu->y + 1;
                cpu_setZNFlags(cpu, result);
                cpu->y = result;               
            }
            break;
        case 0xc9: // CMP #$NN
            #ifdef DEBUG
            printf("CMP #$%02x\n", buffer[cpu->pc + 1]);
            #endif 
            opBytes = 2;
            break;
        case 0xca: // DEX
            {
                #ifdef DEBUG
                printf("DEX\n");
                #endif 

                uint16_t result = (uint16_t) cpu->x - 1;
                cpu_setZNFlags(cpu, result);
                cpu->x = result;               
            }
            break;
        case 0xcc: // CPY $NNNN
            #ifdef DEBUG
            printf("CPY $%02x%02x\n", buffer[cpu->pc + 2], buffer[cpu->pc + 1]);
            #endif 
            opBytes = 3;
            break;
        case 0xcd: // CMP $NNNN
            #ifdef DEBUG
            printf("CMP $%02x%02x\n", buffer[cpu->pc + 2], buffer[cpu->pc + 1]);
            #endif 
            opBytes = 3;
            break;
        case 0xce: // DEC $NNNN
            {
                #ifdef DEBUG
                printf("DEC $%02x%02x,Y\n", buffer[cpu->pc + 2], buffer[cpu->pc + 1]);
                #endif 
                opBytes = 3;

                uint16_t address = cpu_toDWORD(buffer[cpu->pc + 2], buffer[cpu->pc + 1]);
                uint16_t result = (uint16_t) cpu->memory[address] - 1;
                cpu_setZNFlags(cpu, result);
                cpu->memory[address] = result;      
            }
            break;
        case 0xd0: // BNE $NN
            #ifdef DEBUG
            printf("BNE $%02x\n", buffer[cpu->pc + 1]);
            #endif 
            opBytes = 2;
            break;
        case 0xd1: // CMP ($NN),Y
            #ifdef DEBUG
            printf("CMP $%02x,Y\n", buffer[cpu->pc + 1]);
            #endif 
            opBytes = 2;
            break;
        case 0xd5: // CMP $NN,X
            #ifdef DEBUG
            printf("CMP $%02x,X\n", buffer[cpu->pc + 1]);
            #endif 
            opBytes = 2;
            break;
        case 0xd6: // DEC $NN,X
            {
                #ifdef DEBUG
                printf("DEC $%02x,X\n", buffer[cpu->pc + 1]);
                #endif 
                opBytes = 2;

                uint16_t address = (uint16_t) cpu->x + (uint16_t) buffer[cpu->pc + 1];
                uint16_t result = (uint16_t) cpu->memory[address] - 1;
                cpu_setZNFlags(cpu, result);
                cpu->memory[address] = result;
            }
            break;
        case 0xd8: // CLD
            { 
                #ifdef DEBUG
                printf("CLD\n");
                #endif 

                cpu->s.decimal = 0;
            }
            break;
        case 0xd9: // CMP $NNNN,Y
            #ifdef DEBUG
            printf("CMP $%02x%02x,Y\n", buffer[cpu->pc + 2], buffer[cpu->pc + 1]);
            #endif 
            opBytes = 3;
            break;
        case 0xdd: // CMP $NNNN,X
            #ifdef DEBUG
            printf("CMP $%02x%02x,X\n", buffer[cpu->pc + 2], buffer[cpu->pc + 1]);
            #endif 
            opBytes = 3;
            break;
        case 0xde: // DEC $NNNN,X
            {
                #ifdef DEBUG
                printf("DEC $%02x%02x,X\n", buffer[cpu->pc + 2], buffer[cpu->pc + 1]);
                #endif 
                opBytes = 3;

                uint16_t address = cpu_toDWORD(buffer[cpu->pc + 2], 
                        buffer[cpu->pc + 1]) + (uint16_t) cpu->x;
                uint16_t result = (uint16_t) cpu->memory[address] - 1;
                cpu_setZNFlags(cpu, result);
            }
            break;
        case 0xe0: // CPX #$NN
            #ifdef DEBUG
            printf("CPX #$%02x\n", buffer[cpu->pc + 1]);
            #endif 
            opBytes = 2;
            break;
        case 0xe1: // SBC($NN,X)
            {
                #ifdef DEBUG
                printf("SBC $%02x,X\n", buffer[cpu->pc + 1]);
                #endif 
                opBytes = 2;

                uint16_t address = cpu_fetchIIAX(cpu, buffer);
                uint16_t result = (uint16_t) cpu->acc - 
                    (uint16_t) cpu->memory[address];
                cpu_setArithmeticFlags(cpu, result, cpu->acc, cpu->memory[address]);
                cpu->acc = result;
            }
            break;
        case 0xe4: // CPX $NN
            #ifdef DEBUG
            printf("CPX $%02x\n", buffer[cpu->pc + 1]);
            #endif 
            opBytes = 3;
            break;
        case 0xe6: // INC $NN
            {
                #ifdef DEBUG
                printf("INC $%02x\n", buffer[cpu->pc + 1]);
                #endif 
                opBytes = 2;

                uint16_t address = buffer[cpu->pc + 1];
                uint16_t result = (uint16_t) cpu->memory[address] + 1;
                cpu_setZNFlags(cpu, result);
                cpu->memory[address] = result;               
            }
            break;
        case 0xe8: // INX
            {
                #ifdef DEBUG
                printf("INX\n");
                #endif 

                uint16_t result = (uint16_t) cpu->x + 1;
                cpu_setZNFlags(cpu, result);
                cpu->x = result;               
            }
            break;
        case 0xe9: // SBC #$NN
            {
                #ifdef DEBUG
                printf("SBC #$%02x\n", buffer[cpu->pc + 1]);
                #endif 
                opBytes = 2;

                uint16_t result = (uint16_t) cpu->acc - 
                    (uint16_t) buffer[cpu->pc + 1];
                cpu_setArithmeticFlags(cpu, result, cpu->acc, 
                        buffer[cpu->pc + 1]);
                cpu->acc = result;
            }
            break;
        case 0xea: // NOP
            #ifdef DEBUG
            printf("NOP\n");
            #endif 
            break;
        case 0xec: // CPX $NNNN
            #ifdef DEBUG
            printf("CPX $%02x%02x\n", buffer[cpu->pc + 2], buffer[cpu->pc + 1]);
            #endif 
            opBytes = 3;
            break;
        case 0xed: // SBC $NNNN
            {
                #ifdef DEBUG
                printf("SBC $%02x%02x\n", buffer[cpu->pc + 2], buffer[cpu->pc + 1]);
                #endif 
                opBytes = 3;

                uint16_t address = cpu_toDWORD(buffer[cpu->pc + 2], 
                        buffer[cpu->pc + 1]);
                uint16_t result = (uint16_t) cpu->acc - 
                    (uint16_t) cpu->memory[address];
                cpu_setArithmeticFlags(cpu, result, cpu->acc, 
                        cpu->memory[address]);
                cpu->acc = result;
            }
            break;
        case 0xee: // INC $NNNN
            {
                #ifdef DEBUG
                printf("INC $%02x%02x\n", buffer[cpu->pc + 2], buffer[cpu->pc + 1]);
                #endif 
                opBytes = 3;

                uint16_t address = cpu_toDWORD(buffer[cpu->pc + 2], buffer[cpu->pc + 1]);
                uint16_t result = (uint16_t) cpu->memory[address] + 1;
                cpu_setZNFlags(cpu, result);
                cpu->memory[address] = result;      
            }
            break;
        case 0xf0: // BEQ $NN
            #ifdef DEBUG
            printf("BEQ $%02x\n", buffer[cpu->pc + 1]);
            #endif 
            opBytes = 2;
            break;
        case 0xf1: // SBC ($NN),Y
            {
                #ifdef DEBUG
                printf("SBC $%02x,Y\n", buffer[cpu->pc + 1]);
                #endif 
                opBytes = 2;

                uint16_t address = cpu_fetchIIAY(cpu, buffer);
                uint16_t result = cpu->acc - cpu->memory[address];
                cpu_setArithmeticFlags(cpu, result, cpu->acc, 
                        cpu->memory[address]);
                cpu->acc = result;
            }
            break;
        case 0xf5: // SBC $NN,X
            {
                #ifdef DEBUG
                printf("SBC $%02x,X\n", buffer[cpu->pc + 1]);
                #endif 
                opBytes = 2;

                uint16_t address = (uint16_t) cpu->x + (uint16_t) buffer[cpu->pc + 1];
                uint16_t result = (uint16_t) cpu->acc - 
                    (uint16_t) cpu->memory[address];
                cpu_setArithmeticFlags(cpu, result, cpu->acc, cpu->memory[address]);
                cpu->acc = result;
            }
            break;
        case 0xf6: // INC $NN,X
            {
                #ifdef DEBUG
                printf("INC $%02x,X\n", buffer[cpu->pc + 1]);
                #endif 
                opBytes = 2;
                
                uint16_t address = (uint16_t) cpu->x + (uint16_t) buffer[cpu->pc + 1];
                uint16_t result = (uint16_t) cpu->memory[address] + 1;
                cpu_setZNFlags(cpu, result);
                cpu->memory[address] = result;
            }
            break;
        case 0xf8: // SED
            {
                #ifdef DEBUG
                printf("SED\n");
                #endif 

                cpu->s.decimal = 1;
            }
            break;
        case 0xf9: // SBC $NNNN,Y
            {
                #ifdef DEBUG
                printf("SBC $%02x%02x,Y\n", buffer[cpu->pc + 2], buffer[cpu->pc + 1]);
                #endif 
                opBytes = 3;

                uint16_t address = cpu_toDWORD(buffer[cpu->pc + 2], 
                        buffer[cpu->pc + 1]) + (uint16_t) cpu->y;
                uint16_t result = (uint16_t) cpu->acc - 
                    (uint16_t) cpu->memory[address];
                cpu_setArithmeticFlags(cpu, result, cpu->acc, 
                        cpu->memory[address]);
                cpu->acc = result;
            }
            break;
        case 0xfd: // SBC $NNNN,X
            {
                #ifdef DEBUG
                printf("SBC $%02x%02x,X\n", buffer[cpu->pc + 2], buffer[cpu->pc + 1]);
                #endif 
                opBytes = 3;

                uint16_t address = cpu_toDWORD(buffer[cpu->pc + 2], 
                        buffer[cpu->pc + 1]) + (uint16_t) cpu->x;
                uint16_t result = (uint16_t) cpu->acc - 
                    (uint16_t) cpu->memory[address];
                cpu_setArithmeticFlags(cpu, result, cpu->acc, 
                        cpu->memory[address]);
                cpu->acc = result;
            }
            break;
        case 0xfe: // INC $NNNN,X
            {
                #ifdef DEBUG
                printf("INC $%02x%02x,X\n", buffer[cpu->pc + 2], buffer[cpu->pc + 1]);
                #endif 
                opBytes = 3;

                uint16_t address = cpu_toDWORD(buffer[cpu->pc + 2], 
                        buffer[cpu->pc + 1]) + (uint16_t) cpu->x;
                uint16_t result = (uint16_t) cpu->memory[address] + 1;
                cpu_setZNFlags(cpu, result);
                cpu->memory[address] = result;
            }
            break;
    }

    return opBytes;
}
