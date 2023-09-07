#include <stdint.h>

typedef struct p6502 
{	
	uint8_t A,X,Y,S,P,interrupt_line,current_byte_count;
	uint16_t PC;
	uint8_t *memory;
}p6502_t;


p6502_t *processor_init();
void set_mem(p6502_t *cpu, uint16_t addr, uint8_t byte);
void load_pc(p6502_t *cpu, uint16_t addr);
void cpu_cycle(p6502_t *cpu);
