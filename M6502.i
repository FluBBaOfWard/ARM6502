
#if !__ASSEMBLER__
	#error This header file is only for use in assembly files!
#endif

				;@ r0,r1,r2=temp regs
	m6502nz		.req r3			;@ Bit 31=N, Z=1 if bits 0-7=0
	m6502a		.req r4			;@ Bits  0-23=0
	m6502x		.req r5			;@ Bits  0-23=0
	m6502y		.req r6			;@ Bits  0-23=0
	m6502sp		.req r7			;@ Bits 24-31=SP, bit 0=1.
	cycles		.req r8			;@ Also VDIC flags
	m6502pc		.req r9
	m6502ptr	.req r10
	m6502zpage	.req r11		;@ ZeroPage RAM ptr
	addy		.req r12		;@ Keep this at r12 (scratch for APCS)

;@----------------------------------------------------------------------------
	.equ CYC_SHIFT, 8
	.equ CYCLE, 1<<CYC_SHIFT	;@ One cycle
	.equ CYC_MASK, CYCLE-1		;@ Mask
;@----------------------------------------------------------------------------
;@ cycle flags- (stored in cycles reg for speed)
	.equ CYC_C, 0x01			;@ Carry bit
	.equ CYC_I, 0x04			;@ IRQ mask
	.equ CYC_D, 0x08			;@ Decimal bit
	.equ CYC_V, 0x40			;@ Overflow bit
;@----------------------------------------------------------------------------
;@ IRQ flags
	.equ IRQ_F,   0x04			;@ IRQ flag
	.equ NMI_F,   0x08			;@ NMI flag
	.equ RESET_F, 0x10			;@ Reset flag
;@----------------------------------------------------------------------------

	.struct 0					;@ Changes section so make sure it is set before real code.
m6502Start:
m6502Opz:			.space 256*4
m6502MemTbl:		.space 8*4
m6502ReadTbl:		.space 8*4
m6502WriteTbl:		.space 8*4
m6502StateStart:
m6502Regs:
m6502RegNZ:			.long 0
m6502RegA:			.long 0
m6502RegX:			.long 0
m6502RegY:			.long 0
m6502RegSP:			.long 0
m6502Cycles:		.long 0
m6502RegPC:			.long 0
m6502ZeroPage:		.long 0
m6502IrqPending:	.byte 0
m6502NMIPin:		.byte 0
m6502Padding:		.space 2
m6502StateEnd:

m6502LastBank:		.long 0
#ifdef DEBUG
m6502BRKCount:		.long 0
m6502BadOpCount:	.long 0
#else
m6502Alignment:		.space 2*4
#endif
m6502End:
m6502Size = m6502End-m6502Start
m6502StateSize = m6502StateEnd-m6502StateStart

;@----------------------------------------------------------------------------
