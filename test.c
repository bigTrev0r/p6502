#include <stdio.h>
#include <stdlib.h>

#include "inc/cpu.h"


int main(void)
{
	p6502_t *cpu = processor_init();

	FILE *fp = fopen("count.txt","r");
	if(!fp) return 1; 
	for(size_t i = 0; fscanf(fp,"%hhx",(cpu->memory+0x2000+i)) != EOF; i++);
	fclose(fp);
	
	for(load_pc(cpu,0x2000); cpu->memory[cpu->PC]!=0xEA; cpu_cycle(cpu),printf("%x\n",cpu->X));
	
	return 0;
}
