
				;@ r0,r1,r2=temp regs
	m6502nz		.req r3			;@ Bit 31=N, Z=1 if bits 0-7=0
	m6502a		.req r4			;@ Bits  0-23=0, also used to clear bytes in memory (vdc.s)
	m6502x		.req r5			;@ Bits  0-23=0
	m6502y		.req r6			;@ Bits  0-23=0
	m6502sp		.req r7			;@ Bits 24-31=SP, bit 0=1.
	cycles		.req r8
	m6502pc		.req r9
	m6502optbl	.req r10
	m6502zpage	.req r11		;@ ZeroPage RAM ptr
	addy		.req r12		;@ Keep this at r12 (scratch for APCS)

	.struct 0					;@ Changes section so make sure it's set before real code.
m6502Opz:			.space 256*4
m6502MemTbl:		.space 8*4
m6502ReadTbl:		.space 8*4
m6502WriteTbl:		.space 8*4
m6502Regs:
m6502RegNz:			.long 0
m6502RegA:			.long 0
m6502RegX:			.long 0
m6502RegY:			.long 0
m6502RegSp:			.long 0
m6502RegCy:			.long 0
m6502RegPc:			.long 0
m6502RegZp:			.long 0
m6502IrqPending:	.byte 0
m6502NMIPin:		.byte 0
m6502Padding:		.space 2

m6502LastBank:		.long 0
m6502OldCycles:		.long 0
m6502NextTimeout_:	.long 0
m6502NextTimeout:	.long 0
m6502Size:

;@----------------------------------------------------------------------------
