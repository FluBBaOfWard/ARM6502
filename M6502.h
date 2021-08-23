#ifndef ARM6502_HEADER
#define ARM6502_HEADER

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	u32 m6502Opz[256];
	u32 m6502ReadTbl[8];
	u32 m6502WriteTbl[8];
	u32 m6502MemTbl[8];
// m6502Regs[8]
	u32 m6502RegNz;
	u32 m6502RegA;
	u32 m6502RegX;
	u32 m6502RegY;
	u32 m6502RegSp;
	u32 m6502RegCy;
	u32 m6502RegPc;
	u32 m6502RegZp;
	u32 m6502US;
	u8 m6502IrqPending;
	u8 m6502NmiPin;
	u8 m6502Padding[2];

	void *m6502LastBank;
	int m6502OldCycles;
	void *m6502NextTimeout_;
	void *m6502NextTimeout;
} M6502Core;

extern M6502Core m6502OpTable;

void m6502Reset(int type);

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

void m6502SetIRQPin(bool set);
void m6502SetNMIPin(bool set);
void m6502RestoreAndRunXCycles(int cycles);
void m6502RunXCycles(int cycles);
void m6502CheckIrqs(void);

#ifdef __cplusplus
}
#endif

#endif // ARM6502_HEADER
