#ifndef ARM6502_HEADER
#define ARM6502_HEADER

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	void (*opz[256])(void);
	u8 *memTbl[8];
	u8 (*readTbl[8])();
	void (*writeTbl[8])(u8 val);

// StateStart
	u32 regNz;
	u32 regA;
	u32 regX;
	u32 regY;
	u32 regSp;
	u32 cycles;
	u8 *regPc;
	u32 zeroPage;
	u8 irqPending;
	u8 nmiPin;
	u8 padding[2];

	u8 *lastBank;
#ifdef DEBUG
	u32 brkCount;
	u32 badOpCount;
#else
	int alignment[2];
#endif
} M6502Core;

/**
 * Initializes the opcode table.
 * @param  *cpu: The M6502Core cpu to initialize.
 */
void m6502Init(M6502Core *cpu);

void m6502Reset(M6502Core *cpu);

/**
 * Saves the state of the cpu to the destination.
 * @param  *destination: Where to save the state.
 * @param  *cpu: The M6502Core cpu to save.
 * @return The size of the state.
 */
int m6502SaveState(void *destination, const M6502Core *cpu);

/**
 * Loads the state of the cpu from the source.
 * @param  *cpu: The M6502Core cpu to load a state into.
 * @param  *source: Where to load the state from.
 * @return The size of the state.
 */
int m6502LoadState(M6502Core *cpu, const void *source);

/**
 * Gets the state size of an M6502Core cpu.
 * @return The size of the state.
 */
int m6502GetStateSize(void);

/**
 * Patch an opcode to a new function.
 * @param  *cpu: The M6502Core cpu to patch.
 * @param  opcode: Which opcode to redirect.
 * @param  *function: Pointer to new function .
 */
void m6502PatchOpcode(M6502Core *cpu, int opcode, void (*function)(void));

/**
 * Restore a previously patched opcode.
 * @param  *cpu: The M6502Core cpu to patch.
 * @param  opcode: Which opcode to restore.
 */
void m6502RestoreOpcode(M6502Core *cpu, int opcode);

void m6502RunXCyclesC(int cycles, M6502Core *cpu);

void m6502RestoreAndRunXCycles(int cycles);
void m6502RunXCycles(int cycles);
void m6502SetResetPin();
void m6502SetNMIPin(bool set);
void m6502SetIRQPin(bool set);

#ifdef __cplusplus
}
#endif

#endif // ARM6502_HEADER
