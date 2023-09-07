#include <stdlib.h>
#include <stdio.h>

#include "../inc/cpu.h"

#define CLOCK_CYCLE 0.0005

#define FLAGS cpu->P
#define C 0x01
#define Z 0x02
#define I 0x04
#define D 0x08
#define B 0x10
#define V 0x40
#define N 0x80

#define IRQ_ADDR 0xFFFE
#define NMI_ADDR 0xFFFA
#define RESET_ADDR 0xFFFC

#define MEMORY cpu->memory
#define BYTE_COUNT cpu->current_byte_count

#define STACK (cpu->S)
#define STACK_ADDRESS (0x100 | cpu->S)
#define STACK_PUSH_16(b2) *(uint16_t *)(MEMORY+STACK_ADDRESS) = b2; STACK -=2
#define STACK_PUSH_8(b) MEMORY[STACK_ADDRESS] = b; STACK--
#define STACK_POP_16(dst) dst = *(uint16_t*)(MEMORY + STACK_ADDRESS); STACK+=2
#define STACK_POP_8(dst) dst = MEMORY[STACK_ADDRESS]; STACK++

#define PC cpu->PC
#define PC_INC(bytes) PC += bytes 
#define PC_LOAD(mem) PC = *(uint16_t *)(MEMORY+mem)

#define ACCUMULATOR cpu->A
#define INDEX_X	cpu->X
#define INDEX_Y cpu->Y



p6502_t *processor_init()
{
	p6502_t *new = calloc(1,sizeof(p6502_t));
	new->memory = malloc(0x10000);
	new->S = 0xFF;
	return new;
}


typedef enum ACCESS_MODE {READ,WRITE} ACCESS_MODE_T;

#define MODE_TERN return mode ? addr : MEMORY[addr]

static uint16_t IMMEDIATE(p6502_t   * restrict cpu, uint8_t *opcode, ACCESS_MODE_T mode) 
{
	BYTE_COUNT = 1;
	return (uint16_t)*(opcode+1);
}
static uint16_t ABSOLUTE(p6502_t    * restrict cpu, uint8_t *opcode, ACCESS_MODE_T mode) 
{
	BYTE_COUNT = 2;
	uint16_t addr = *(uint16_t*)(opcode+1); 
	MODE_TERN;
}
static uint16_t ZERO_PAGE(p6502_t   * restrict cpu, uint8_t *opcode, ACCESS_MODE_T mode) 
{
	BYTE_COUNT = 1;
	uint16_t addr = (uint16_t)*(opcode+1); 
	MODE_TERN;
}
static uint16_t ABSOLUTE_X(p6502_t  * restrict cpu, uint8_t *opcode, ACCESS_MODE_T mode) 
{
	BYTE_COUNT = 2;
	uint16_t addr = ABSOLUTE(cpu,opcode,mode)+INDEX_X; 
	MODE_TERN;
}
static uint16_t ABSOLUTE_Y(p6502_t  * restrict cpu, uint8_t *opcode, ACCESS_MODE_T mode) 
{
	BYTE_COUNT = 2;
	uint16_t addr = ABSOLUTE(cpu,opcode,mode)+INDEX_Y; 
	MODE_TERN;
}
static uint16_t ZERO_PAGE_X(p6502_t * restrict cpu, uint8_t *opcode, ACCESS_MODE_T mode) 
{
	BYTE_COUNT = 1;
	uint16_t addr = ZERO_PAGE(cpu,opcode,mode)+INDEX_X; 
	MODE_TERN;
}
static uint16_t ZERO_PAGE_Y(p6502_t * restrict cpu, uint8_t *opcode, ACCESS_MODE_T mode) 
{
	BYTE_COUNT = 1;
	uint16_t addr = ZERO_PAGE(cpu,opcode,mode)+INDEX_Y; 
	MODE_TERN;
}

static uint16_t INDEXED_INDIRECT(p6502_t * restrict cpu, uint8_t *opcode, ACCESS_MODE_T mode) 
{
	BYTE_COUNT = 1;
	uint16_t addr = *(uint16_t*)(MEMORY+((*(opcode+1)+INDEX_X)&0xFF));
	MODE_TERN;
}

static uint16_t INDIRECT_INDEXED(p6502_t * restrict cpu, uint8_t *opcode, ACCESS_MODE_T mode) 
{
	BYTE_COUNT++;
	uint16_t addr = (uint16_t)((ZERO_PAGE(cpu,opcode,mode) + INDEX_Y));
	MODE_TERN;
}

#undef MODE_TERN

static uint16_t (*ADRESSING_MODE_TABLE[9])(p6502_t*,uint8_t*,ACCESS_MODE_T) = 
{
	&IMMEDIATE, &ABSOLUTE, &ABSOLUTE_X, &ABSOLUTE_Y, &ZERO_PAGE, &ZERO_PAGE_X, &ZERO_PAGE_Y,
	&INDEXED_INDIRECT, &INDIRECT_INDEXED 
};

static uint8_t ARITH_MAP(uint8_t byte)
{
	uint8_t lo = byte & 0x0F, off = ((byte & 0xF0) >> 4) % 2;
	switch(lo)
	{
		case 0x01:
			return 7 + off;
		case 0x05:
			return 4 + off;
		case 0x09:
			return 0 + (3 * off);
		case 0x0D:
			return 1 + off;
	}
}

static uint8_t CMP_MAP(uint8_t byte)
{
	uint8_t lo = byte & 0x0F;
	return (lo == 0x0C) ? 2 : lo;  
}

static uint8_t INC_MAP(uint8_t byte)
{
	uint8_t lo = byte & 0x0F, off = ((byte & 0xF0) >> 4) % 2;
	return lo == 0x0E ? 1 + off : 4 + off; 
}

static uint8_t MVX_MAP(uint8_t byte)
{
	uint8_t lo = byte & 0x0F, off = ((byte & 0xF0) >> 4)  % 2;
	return lo == 0x02 ? 0 : lo == 0x06 ? 4 + 2*off : 2 + 2*off;
}

static uint8_t MVY_MAP(uint8_t byte)
{
	uint8_t lo = byte & 0x0F, off = ((byte & 0xF0) >> 4) % 2;
	return lo == 0x00 ? lo : lo == 0x04 ? 4 + off : 1 + off;
}

static inline uint16_t READ_ADDRESS(p6502_t *cpu, uint8_t *opcode, uint8_t read_type) {return ADRESSING_MODE_TABLE[read_type](cpu,opcode,READ);}
static inline uint16_t WRITE_ADDRESS(p6502_t *cpu, uint8_t *opcode, uint8_t read_type) {return ADRESSING_MODE_TABLE[read_type](cpu,opcode,WRITE);}

#define READ_ARITH (READ_ADDRESS(cpu,opcode,ARITH_MAP(*opcode)))
#define READ_CMP (READ_ADDRESS(cpu,opcode,CMP_MAP(*opcode)))
#define READ_MVX (READ_ADDRESS(cpu,opcode,MVX_MAP(*opcode)))
#define READ_MVY (READ_ADDRESS(cpu,opcode,MVY_MAP(*opcode)))
#define WRITE_ARITH MEMORY[(WRITE_ADDRESS(cpu,opcode,ARITH_MAP(*opcode)))]
#define WRITE_INC MEMORY[(WRITE_ADDRESS(cpu,opcode,INC_MAP(*opcode)))]
#define WRITE_MVX MEMORY[(WRITE_ADDRESS(cpu,opcode,MVX_MAP(*opcode)))]
#define WRITE_MVY MEMORY[(WRITE_ADDRESS(cpu,opcode,MVY_MAP(*opcode)))]

#define SET_LOGIC_FLAGS(byte) FLAGS = (!byte ? (FLAGS | Z) & ~N : (FLAGS | (byte & N)) & ~Z)
#define SET_CMP_FLAGS(byte) FLAGS = (!byte ? (FLAGS | Z | C) & ~N : (FLAGS | (byte & N)) & ~Z) | (byte & N)>>7

#define APC_INC PC_INC(BYTE_COUNT)

static void ORA(p6502_t * restrict cpu, uint8_t *opcode) {ACCUMULATOR |= READ_ARITH; APC_INC; SET_LOGIC_FLAGS(ACCUMULATOR);}
static void AND(p6502_t * restrict cpu, uint8_t *opcode) {ACCUMULATOR &= READ_ARITH; APC_INC; SET_LOGIC_FLAGS(ACCUMULATOR);}
static void EOR(p6502_t * restrict cpu, uint8_t *opcode) {ACCUMULATOR ^= READ_ARITH; APC_INC; SET_LOGIC_FLAGS(ACCUMULATOR);}

static void BIT(p6502_t * restrict cpu, uint8_t *opcode) 
{
	uint8_t operand = ABSOLUTE(cpu,opcode,READ);
	APC_INC;
	if(*opcode == 0x24) operand = ZERO_PAGE(cpu,opcode,READ);
	uint8_t test = ACCUMULATOR ^ operand; FLAGS |= (!test ? Z | (FLAGS & ~V) : (test | V | N));
}

static void LDA(p6502_t * restrict cpu, uint8_t *opcode) {ACCUMULATOR = READ_ARITH; APC_INC; SET_LOGIC_FLAGS(ACCUMULATOR);}
static void LDX(p6502_t * restrict cpu, uint8_t *opcode) {INDEX_X = READ_MVX; APC_INC; SET_LOGIC_FLAGS(INDEX_X);}
static void LDY(p6502_t * restrict cpu, uint8_t *opcode) {INDEX_Y = READ_MVY; APC_INC; SET_LOGIC_FLAGS(INDEX_Y);}
static void STA(p6502_t * restrict cpu, uint8_t *opcode) {WRITE_ARITH = ACCUMULATOR; APC_INC;}
static void STX(p6502_t * restrict cpu, uint8_t *opcode) {WRITE_MVX = INDEX_X; APC_INC;}
static void STY(p6502_t * restrict cpu, uint8_t *opcode) {WRITE_MVY = INDEX_Y; APC_INC;}

static void TAX(p6502_t * restrict cpu, uint8_t *opcode) {INDEX_X = ACCUMULATOR; SET_LOGIC_FLAGS(INDEX_X);}
static void TAY(p6502_t * restrict cpu, uint8_t *opcode) {INDEX_Y = ACCUMULATOR; SET_LOGIC_FLAGS(INDEX_Y);}
static void TSX(p6502_t * restrict cpu, uint8_t *opcode) {INDEX_X = STACK; SET_LOGIC_FLAGS(INDEX_X);}
static void TXA(p6502_t * restrict cpu, uint8_t *opcode) {ACCUMULATOR = INDEX_X; SET_LOGIC_FLAGS(ACCUMULATOR);}
static void TXS(p6502_t * restrict cpu, uint8_t *opcode) {STACK = INDEX_X;}
static void TYA(p6502_t * restrict cpu, uint8_t *opcode) {ACCUMULATOR = INDEX_Y; SET_LOGIC_FLAGS(ACCUMULATOR);}

static void PHA(p6502_t * restrict cpu, uint8_t *opcode) {MEMORY[STACK_ADDRESS] = ACCUMULATOR; STACK--;}
static void PHP(p6502_t * restrict cpu, uint8_t *opcode) {MEMORY[STACK_ADDRESS] = FLAGS; STACK--;}
static void PLA(p6502_t * restrict cpu, uint8_t *opcode) {STACK++; ACCUMULATOR = MEMORY[STACK_ADDRESS]; SET_LOGIC_FLAGS(ACCUMULATOR);}
static void PLP(p6502_t * restrict cpu, uint8_t *opcode) {STACK++; FLAGS = MEMORY[STACK_ADDRESS];}

static void ADC(p6502_t * restrict cpu, uint8_t *opcode)
{
	int8_t operand = READ_ARITH;
	APC_INC; 
	if(FLAGS & D)
	{
		int16_t low = (ACCUMULATOR & 0x0F) + (operand & 0x0F);
		int16_t high = (ACCUMULATOR & 0xF0) + (operand & 0xF0);

		high += low>0x09 ? 0x10 : 0;
		FLAGS |= (high | low) > 0x99 ? C : FLAGS & ~C;
		low %= 0x0A;
		high %= 0xA0;

		ACCUMULATOR = (low | high);
		SET_LOGIC_FLAGS(ACCUMULATOR);
		return;
	}

	uint16_t temp_V = ACCUMULATOR & V, temp_S = ACCUMULATOR + operand;
	FLAGS = temp_S > 0xFF ? FLAGS | C : FLAGS & ~C;
	FLAGS = temp_V == temp_S & V ? FLAGS | V : FLAGS & ~V;

	ACCUMULATOR = (uint8_t)temp_S;
	SET_LOGIC_FLAGS(ACCUMULATOR);
	
	return;
}

static void SBC(p6502_t * restrict cpu, uint8_t *opcode)
{
	int8_t operand = READ_ARITH;
	APC_INC; 
	if(FLAGS & D)
	{
		int16_t low = (ACCUMULATOR & 0x0F) - (operand & 0x0F);
		int16_t high = (ACCUMULATOR & 0xF0) - (operand & 0xF0);

		high += low>0x09 ? 0x10 : 0;
		FLAGS |= (high | low) >=0 ? C : FLAGS & ~C;
		low %= 0x0A;
		high %= 0xA0;

		ACCUMULATOR = (low | high);
		SET_LOGIC_FLAGS(ACCUMULATOR);
		return;
	}

	uint16_t temp_V = ACCUMULATOR & V, temp_D = ACCUMULATOR - operand;
	FLAGS = temp_D >= 0 ? FLAGS | C : FLAGS & ~C;
	FLAGS = temp_V == temp_D & V ? FLAGS | V : FLAGS & ~V;

	ACCUMULATOR = (uint8_t)temp_D;
	SET_LOGIC_FLAGS(ACCUMULATOR);

	return;
}

static void CMP(p6502_t * restrict cpu, uint8_t *opcode) {SET_CMP_FLAGS(ACCUMULATOR-READ_ARITH); APC_INC;}
static void CPX(p6502_t * restrict cpu, uint8_t *opcode) {SET_CMP_FLAGS(INDEX_X-READ_CMP); APC_INC;}
static void CPY(p6502_t * restrict cpu, uint8_t *opcode) {SET_CMP_FLAGS(INDEX_Y-READ_CMP); APC_INC;}

static void DEC(p6502_t * restrict cpu, uint8_t *opcode) {WRITE_INC--; APC_INC; SET_LOGIC_FLAGS(WRITE_INC);}
static void DEX(p6502_t * restrict cpu, uint8_t *opcode) {INDEX_X--; SET_LOGIC_FLAGS(INDEX_X);}
static void DEY(p6502_t * restrict cpu, uint8_t *opcode) {INDEX_Y--; SET_LOGIC_FLAGS(INDEX_Y);}
static void INC(p6502_t * restrict cpu, uint8_t *opcode) {WRITE_INC++; APC_INC; SET_LOGIC_FLAGS(WRITE_INC);}
static void INX(p6502_t * restrict cpu, uint8_t *opcode) {INDEX_X++; SET_LOGIC_FLAGS(INDEX_X);}
static void INY(p6502_t * restrict cpu, uint8_t *opcode) {INDEX_Y++; SET_LOGIC_FLAGS(INDEX_Y);}

static void BRK(p6502_t * restrict cpu, uint8_t *opcode) {STACK_PUSH_16(PC); STACK_PUSH_8(FLAGS); PC_LOAD(0xFFFE); FLAGS |= I;}

static void JMP(p6502_t * restrict cpu, uint8_t *opcode) {
	if(*opcode == 0x4C0) {PC_LOAD(ABSOLUTE(cpu,opcode,READ)); return;}
	PC_LOAD(MEMORY[ABSOLUTE(cpu,opcode,READ)]);
}

static void JSR(p6502_t * restrict cpu, uint8_t *opcode) {PC_INC(2); STACK_PUSH_16(PC); PC_LOAD(ABSOLUTE(cpu,opcode,READ));}
static void RTI(p6502_t * restrict cpu, uint8_t *opcode) {STACK_POP_8(FLAGS); STACK_POP_16(PC); PC++;}
static void RTS(p6502_t * restrict cpu, uint8_t *opcode) {STACK_POP_16(PC); PC++;}

#define OFFSET (int8_t)*(opcode+1)

static void BCC(p6502_t * restrict cpu, uint8_t *opcode) {PC_INC(1); PC += (OFFSET * (~FLAGS & C));}
static void BCS(p6502_t * restrict cpu, uint8_t *opcode) {PC_INC(1); PC += (OFFSET * (FLAGS & C));}
static void BEQ(p6502_t * restrict cpu, uint8_t *opcode) {PC_INC(1); PC += (OFFSET * (FLAGS & Z)>>1);}
static void BMI(p6502_t * restrict cpu, uint8_t *opcode) {PC_INC(1); PC += (OFFSET * (FLAGS & N)>>7);}
static void BNE(p6502_t * restrict cpu, uint8_t *opcode) {PC_INC(1); PC += (OFFSET * (~FLAGS & Z)>>1);}
static void BPL(p6502_t * restrict cpu, uint8_t *opcode) {PC_INC(1); PC += (OFFSET * (~FLAGS & N)>>7);}
static void BVC(p6502_t * restrict cpu, uint8_t *opcode) {PC_INC(1); PC += (OFFSET * (~FLAGS & V)>>6);}
static void BVS(p6502_t * restrict cpu, uint8_t *opcode) {PC_INC(1); PC += (OFFSET * (FLAGS & V)>>6);}

#undef OFFSET

//grrrrr why shift addressing weird grrrrrr
#define ADDR_FOR_SHIFTS(byte) WRITE_ADDRESS(cpu,opcode,INC_MAP(*opcode))
#define SHIFT_BR (*opcode & 0x0F) == 0x0A

#define DEF_TEMPS uint8_t temp,temp_a
#define TEMP_UP(byte)  temp = byte
#define TEMP_A_UP(byte) temp_a = ADDR_FOR_SHIFTS(byte)

#define SHIFT_FLAGS FLAGS &= ~(N | C | Z)
#define SR_ZERO(temp) FLAGS |= (temp ? 0 : Z)

static void ASL(p6502_t * restrict cpu, uint8_t *opcode) 
{
	DEF_TEMPS;
	if(SHIFT_BR) {TEMP_UP(ACCUMULATOR); ACCUMULATOR<<=1;}

	else 
	{
		TEMP_A_UP(*opcode);
		APC_INC;
		TEMP_UP(MEMORY[temp_a]);
		MEMORY[temp_a] <<= 1;
	}

	SHIFT_FLAGS;
	FLAGS |= ((temp & 0xF0) >> 7);
	FLAGS |= ((temp << 1) & 0xF0);
	SR_ZERO(temp<<1);

}
static void LSR(p6502_t * restrict cpu, uint8_t *opcode) 
{
	DEF_TEMPS;
	if(SHIFT_BR) {TEMP_UP(ACCUMULATOR); ACCUMULATOR >>= 1;}

	else
	{
		TEMP_A_UP(*opcode);
		APC_INC;
		TEMP_UP(MEMORY[temp_a]);
		MEMORY[temp_a] >>= 1;
	}

	SHIFT_FLAGS;
	FLAGS |= (temp & 1);
	SR_ZERO(temp>>1);
}
static void ROL(p6502_t * restrict cpu, uint8_t *opcode) 
{
	DEF_TEMPS,temp_s;
	if(SHIFT_BR) {TEMP_UP(ACCUMULATOR); temp_s = (temp << 1 | (FLAGS & C)); ACCUMULATOR = temp_s;}

	else
	{
		TEMP_A_UP(*opcode);
		APC_INC;
		TEMP_UP(MEMORY[temp_a]);
		temp_s = (temp << 1 | (FLAGS & C));

		MEMORY[temp_a] = temp_s;
	}

	SHIFT_FLAGS;
	FLAGS |= ((temp & N) >> 7);
	FLAGS |= ((temp & V) >> 6);
	SR_ZERO(temp_s);
}
static void ROR(p6502_t * restrict cpu, uint8_t *opcode) 
{
	DEF_TEMPS,temp_s,temp_c = FLAGS & C;
	if(SHIFT_BR) {TEMP_UP(ACCUMULATOR); temp_s = (temp >> 1 | ((FLAGS & C) >> 7 )); ACCUMULATOR = temp_s;}

	else
	{
		TEMP_A_UP(*opcode);
		APC_INC;
		TEMP_UP(MEMORY[temp_a]);
		temp_s = (temp >> 1 | ((FLAGS & C) >> 7));

		MEMORY[temp_a] = temp_s;
	}

	SHIFT_FLAGS;
	FLAGS |= (temp & 1);
	FLAGS |= temp_c;
	SR_ZERO(temp_s);
}

static void CLC(p6502_t * restrict cpu, uint8_t *opcode) {FLAGS &= ~C;}
static void CLD(p6502_t * restrict cpu, uint8_t *opcode) {FLAGS &= ~D;}
static void CLI(p6502_t * restrict cpu, uint8_t *opcode) {FLAGS &= ~I;}
static void CLV(p6502_t * restrict cpu, uint8_t *opcode) {FLAGS &= ~V;}
static void SEC(p6502_t * restrict cpu, uint8_t *opcode) {FLAGS |= C;}
static void SED(p6502_t * restrict cpu, uint8_t *opcode) {FLAGS |= D;}
static void SEI(p6502_t * restrict cpu, uint8_t *opcode) {FLAGS |= I;}

static void NOP(p6502_t * restrict cpu, uint8_t *opcode){return;}

static void(*OP_TABLE[256])(p6502_t*,uint8_t*) = 
{
/*      00   01   02   03   04   05   06   07   08   09   0A   0B   0C   0D   0E   0F */
/*00*/ &BRK,&ORA,&NOP,&NOP,&NOP,&ORA,&ASL,&NOP,&PHP,&ORA,&ASL,&NOP,&NOP,&ORA,&ASL,&NOP,
/*01*/ &BPL,&ORA,&NOP,&NOP,&NOP,&ORA,&ASL,&NOP,&CLC,&ORA,&NOP,&NOP,&NOP,&ORA,&ASL,&NOP,
/*02*/ &JSR,&AND,&NOP,&NOP,&BIT,&AND,&ROL,&NOP,&PLP,&AND,&ROL,&NOP,&BIT,&AND,&ROL,&NOP,
/*03*/ &BMI,&AND,&NOP,&NOP,&NOP,&AND,&ROL,&NOP,&SEC,&AND,&NOP,&NOP,&NOP,&AND,&ROL,&NOP,
/*04*/ &RTI,&EOR,&NOP,&NOP,&NOP,&EOR,&LSR,&NOP,&PHA,&EOR,&LSR,&NOP,&JMP,&EOR,&LSR,&NOP,
/*05*/ &BVC,&EOR,&NOP,&NOP,&NOP,&EOR,&LSR,&NOP,&CLI,&EOR,&NOP,&NOP,&NOP,&EOR,&LSR,&NOP,
/*06*/ &RTS,&ADC,&NOP,&NOP,&NOP,&ADC,&ROR,&NOP,&PLA,&ADC,&ROR,&NOP,&JMP,&ADC,&ROR,&NOP,
/*07*/ &BVS,&ADC,&NOP,&NOP,&NOP,&ADC,&ROR,&NOP,&SEI,&ADC,&NOP,&NOP,&NOP,&ADC,&ROR,&NOP,
/*08*/ &NOP,&STA,&NOP,&NOP,&STY,&STA,&STX,&NOP,&DEY,&NOP,&TXA,&NOP,&STY,&STA,&STX,&NOP,
/*09*/ &BCC,&STA,&NOP,&NOP,&STY,&STA,&STX,&NOP,&TYA,&STA,&TXS,&NOP,&NOP,&STA,&NOP,&NOP,
/*0A*/ &LDY,&LDA,&LDX,&NOP,&LDY,&LDA,&LDX,&NOP,&TAY,&LDA,&TAX,&NOP,&LDY,&LDA,&LDX,&NOP,
/*0B*/ &BCS,&LDA,&NOP,&NOP,&LDY,&LDA,&LDX,&NOP,&CLV,&LDA,&TSX,&NOP,&LDY,&LDA,&LDX,&NOP,
/*0C*/ &CPY,&CMP,&NOP,&NOP,&CPY,&CMP,&DEC,&NOP,&INY,&CMP,&DEX,&NOP,&CPY,&CMP,&DEC,&NOP,
/*0D*/ &BNE,&CMP,&NOP,&NOP,&NOP,&CMP,&DEC,&NOP,&CLD,&CMP,&NOP,&NOP,&NOP,&CMP,&DEC,&NOP,
/*0E*/ &CPX,&SBC,&NOP,&NOP,&CPX,&SBC,&INC,&NOP,&INX,&SBC,&NOP,&NOP,&CPX,&SBC,&INC,&NOP,
/*0F*/ &BEQ,&SBC,&NOP,&NOP,&NOP,&SBC,&INC,&NOP,&SED,&SBC,&NOP,&NOP,&NOP,&SBC,&INC,&NOP
};

#define STORE_STATUS STACK_PUSH_16(PC); STACK_PUSH_8(FLAGS)

static void IRQ(p6502_t * restrict cpu)
{
	STORE_STATUS;
	PC_LOAD(IRQ_ADDR);
	FLAGS |= I;
}

static void NMI(p6502_t * restrict cpu)
{
	STORE_STATUS;
	PC_LOAD(NMI_ADDR);
	FLAGS |= I;
}

static void RESET(p6502_t * restrict cpu)
{
	STORE_STATUS;
	PC_LOAD(RESET_ADDR);
	FLAGS |= I;
}

static void(*INTERRUPTS[3])(p6502_t *) = { &IRQ, &NMI, &RESET };

void load_pc(p6502_t * restrict cpu, uint16_t addr) {PC = addr;}
void set_mem(p6502_t * restrict cpu, uint16_t addr, uint8_t byte) {MEMORY[addr] = byte;}

void cpu_cycle(p6502_t * restrict cpu)
{
	PC_INC(1);
	OP_TABLE[*(MEMORY+PC-1)](cpu,(MEMORY+PC-1));
}
