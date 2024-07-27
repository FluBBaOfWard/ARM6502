
#include "M6502.i"
							;@ ARM flags
	.equ PSR_N, 0x80000000		;@ Negative (Sign)
	.equ PSR_Z, 0x40000000		;@ Zero
	.equ PSR_C, 0x20000000		;@ Carry
	.equ PSR_V, 0x10000000		;@ Overflow


							;@ M6502 flags
	.equ N, 0x80				;@ Sign (negative)
	.equ V, 0x40				;@ Overflow
	.equ R, 0x20				;@ Reserved? allways 1
	.equ B, 0x10				;@ Interrupt by BRK opcode?
	.equ D, 0x08				;@ Decimal mode
	.equ I, 0x04				;@ Interrup Disable
	.equ Z, 0x02				;@ Zero
	.equ C, 0x01				;@ Carry

;@----------------------------------------------------------------------------


	.macro loadLastBank reg
	ldr \reg,[m6502ptr,#m6502LastBank]
	.endm

	.macro storeLastBank reg
	str \reg,[m6502ptr,#m6502LastBank]
	.endm

	.macro encodePC				;@ Translate m6502pc from M6502 PC to rom offset
	bl translate6502PCToOffset	;@ In=m6502pc, Out=m6502pc,r0=lastBank
	.endm

	.macro reEncodePC			;@ Retranslate m6502pc romoffset
	bl reTranslate6502PCToOffset
	.endm

	.macro encodeP extra	;@ Pack M6502 flags into r0
	and r0,cycles,#CYC_D+CYC_I+CYC_C+CYC_V
	tst m6502nz,#PSR_N
	orrne r0,r0,#N				;@ N
	tst m6502nz,#0xff
	orreq r0,r0,#Z				;@ Z
	orr r0,r0,#\extra			;@ R (& B)
	.endm

	.macro decodePF			;@ Unpack M6502 flags from r0
	bic cycles,cycles,#CYC_D+CYC_I+CYC_C+CYC_V
	bic m6502nz,r0,#0xFD		;@ r0 is signed
	eor m6502nz,m6502nz,#Z
	and r0,r0,#D+I+C+V
	orr cycles,cycles,r0		;@ DICV
	.endm

	.macro getNextOpcode
	ldrb r0,[m6502pc],#1
	.endm

#ifdef CPU_RP2A03
	.equ CYC_MULT, 3
#else
	.equ CYC_MULT, 1
#endif

#ifdef DEBUG
	.macro executeOpcode count
	subs cycles,cycles,#(\count)*CYC_MULT*CYCLE
	b fetchDebug
	.endm

	.macro executeOpcode_c count
	sbcs cycles,cycles,#(\count)*CYC_MULT*CYCLE
	b fetchDebug
	.endm

	.macro executeNext
	getNextOpcode
	b fetchDebug
	.endm
#else
	.macro executeOpcode count
	subs cycles,cycles,#(\count)*CYC_MULT*CYCLE
	ldrpl pc,[m6502ptr,r0,lsl#2]
	b m6502OutOfCycles
	.endm

	.macro executeOpcode_c count
	sbcs cycles,cycles,#(\count)*CYC_MULT*CYCLE
	ldrpl pc,[m6502ptr,r0,lsl#2]
	b m6502OutOfCycles
	.endm

	.macro executeNext
	getNextOpcode
	ldr pc,[m6502ptr,r0,lsl#2]
	.endm
#endif
	.macro fetch count
	getNextOpcode
	executeOpcode \count
	.endm

	.macro fetch_c count	;@ Same as fetch except it adds the Carry (bit 0) also.
	getNextOpcode
	executeOpcode_c \count
	.endm

	.macro eatCycles count
	sub cycles,cycles,#(\count)*CYC_MULT*CYCLE
	.endm

	.macro clearCycles
	and cycles,cycles,#CYC_MASK	;@ Save CPU bits
	.endm

/*	.macro readMem8
	add r0,m6502ptr,#m6502ReadTbl
	and r1,addy,#0xE000
	mov lr,pc
	ldr pc,[r0,r1,lsr#11]		;@ In: addy,r1=addy&0xe000
0:								;@ Out: r0=val (bits 8-31=0 (LSR,ROR,INC,DEC,ASL)), addy preserved for RMW instructions
	.endm*/

	.macro readMem8
	bl memRead8
	.endm

	.macro readMemABS
	bl memRead8ABS
	.endm

	.macro readMemIIX
	bl memRead8IIX
	.endm

	.macro readMemIIY
	bl memRead8IIY
	.endm

	.macro readMemAIY
	bl memRead8AIY
	.endm

	.macro readMemAIX
	bl memRead8AIX
	.endm

	.macro readMemZInd
	bl memRead8ZInd
	.endm

#if defined(CPU_6510)
	.macro readMemZP
	bl ram_R
	.endm

	.macro readMemZPI
	bl ram_low_R
	.endm
#else
	.macro readMemZP
	ldrb r0,[m6502zpage,addy]
	.endm

	.macro readMemZPI
	ldrb r0,[m6502zpage,addy,lsr#24]
	.endm
#endif

	.macro readMemZPS
	ldrsb m6502nz,[m6502zpage,addy]
	.endm

	.macro readMemIMM
	ldrb r0,[m6502pc],#1
	.endm

	.macro readMemIMMS
	ldrsb m6502nz,[m6502pc],#1
	.endm

	.macro readMem
	.ifeq AddressMode-_ABS
		readMem8
	.endif
	.ifeq AddressMode-_ZP
		readMemZP
	.endif
	.ifeq AddressMode-_ZPI
		readMemZPI
	.endif
	.ifeq AddressMode-_IMM
		readMemIMM
	.endif
	.endm

	.macro readMemS
	.ifeq AddressMode-_ABS
		readMem8
		orr m6502nz,r0,r0,lsl#24
	.endif
	.ifeq AddressMode-_ZP
		readMemZPS
	.endif
	.ifeq AddressMode-_IMM
		readMemIMMS
	.endif
	.endm


/*	.macro writeMem8
	add r2,m6502ptr,#m6502WriteTbl
	and r1,addy,#0xE000
	mov lr,pc
	ldr pc,[r2,r1,lsr#11]		;@ In: addy,r0=val(bits 8-31=?),r1=addy&0xe000(for CDRAM_W)
0:								;@ Out: r0,r1,r2,addy=?
	.endm
*/

	.macro writeMem8
	bl memWrite8
	.endm

#if defined(CPU_6510)
	.macro writeMemZP
	bl ram_W
	.endm

	.macro writeMemZPI
	bl ram_low_W
	.endm
#else
	.macro writeMemZP
	strb r0,[m6502zpage,addy]
	.endm

	.macro writeMemZPI
	strb r0,[m6502zpage,addy,lsr#24]
	.endm
#endif

	.macro writeMem
	.ifeq AddressMode-_ABS
		writeMem8
	.endif
	.ifeq AddressMode-_ZP
		writeMemZP
	.endif
	.ifeq AddressMode-_ZPI
		writeMemZPI
	.endif
	.endm

	.macro readWriteMem				;@ !! Crazy Maniacs use this feature !!
	.ifeq AddressMode-_ABS
		readMem8
#if defined(W65C02) || defined(W65C02_OLD)
		readMem8
#else
		writeMem8
#endif
	.endif
	.ifeq AddressMode-_ZP
		readMemZP
	.endif
	.ifeq AddressMode-_ZPI
		readMemZPI
	.endif
	.endm
;@----------------------------------------------------------------------------

	.macro push16				;@ Push r0
	mov r1,r0,lsr#8
	strb r1,[m6502zpage,m6502sp,ror#24]
	sub m6502sp,m6502sp,#0x01000000
	strb r0,[m6502zpage,m6502sp,ror#24]
	sub m6502sp,m6502sp,#0x01000000
	.endm						;@ r1,r2=?

	.macro push8 reg
	strb \reg,[m6502zpage,m6502sp,ror#24]
	sub m6502sp,m6502sp,#0x01000000
	.endm						;@r2=?

	.macro pop16				;@ Pop m6502pc
	add m6502sp,m6502sp,#0x01000000
	ldrb m6502pc,[m6502zpage,m6502sp,ror#24]
	add m6502sp,m6502sp,#0x01000000
	ldrb r0,[m6502zpage,m6502sp,ror#24]
	orr m6502pc,m6502pc,r0,lsl#8
	.endm						;@ r0,r1=?

	.macro pop8 reg
	add m6502sp,m6502sp,#0x01000000
	mov r2,m6502sp,ror#24
	ldrsb \reg,[m6502zpage,r2]	;@ Signed for PLA, PLX, PLY, PLP & RTI
	.endm						;@ r2=?

;@----------------------------------------------------------------------------
;@ doXXX: load addy, increment m6502pc


	.equ _IMM,	1				;@ Immediate
	.equ _ZP,	2				;@ Zero page
	.equ _ZPI,	3				;@ Zero page indexed
	.equ _ABS,	4				;@ Absolute

	.macro doABS			;@ Absolute				$nnnn
	.set AddressMode, _ABS
	ldrb addy,[m6502pc],#1
	ldrb r0,[m6502pc],#1
	orr addy,addy,r0,lsl#8
	.endm

	.macro doAIX			;@ Absolute indexed X	$nnnn,X
	.set AddressMode, _ABS
	ldrb addy,[m6502pc],#1
	ldrb r0,[m6502pc],#1
	cmn m6502x,addy,lsl#24
	subcs cycles,cycles,#CYC_MULT*CYCLE	;@ Waste a cycle if address crosses a page
	add addy,addy,m6502x,lsr#24
	add addy,addy,r0,lsl#8
;@	bic addy,addy,#0xff0000
	.endm

	.macro doAIX_W			;@ Absolute indexed X	$nnnn,X; Write
	.set AddressMode, _ABS
	ldrb addy,[m6502pc],#1
	ldrb r0,[m6502pc],#1
	add addy,addy,m6502x,lsr#24
	add addy,addy,r0,lsl#8
;@	bic addy,addy,#0xff0000
	.endm

	.macro doAIY			;@ Absolute indexed Y	$nnnn,Y
	.set AddressMode, _ABS
	ldrb addy,[m6502pc],#1
	ldrb r0,[m6502pc],#1
	cmn m6502y,addy,lsl#24
	subcs cycles,cycles,#CYC_MULT*CYCLE	;@ Waste a cycle if address crosses a page
	add addy,addy,m6502y,lsr#24
	add addy,addy,r0,lsl#8
;@	bic addy,addy,#0xff0000
	.endm

	.macro doAIY_W			;@ Absolute indexed Y	$nnnn,Y; Write
	.set AddressMode, _ABS
	ldrb addy,[m6502pc],#1
	ldrb r0,[m6502pc],#1
	add addy,addy,m6502y,lsr#24
	add addy,addy,r0,lsl#8
;@	bic addy,addy,#0xff0000
	.endm

	.macro doIMM			;@ Immediate			#$nn
	.set AddressMode, _IMM
	.endm

	.macro doIIX			;@ Indexed indirect X	($nn,X)
	.set AddressMode, _ABS
	ldrb r0,[m6502pc],#1
	add r0,m6502x,r0,lsl#24
	ldrb addy,[m6502zpage,r0,lsr#24]
	add r0,r0,#0x01000000
	ldrb r1,[m6502zpage,r0,lsr#24]
	orr addy,addy,r1,lsl#8
	.endm

	.macro doIIY			;@ Indirect indexed Y	($nn),Y
	.set AddressMode, _ABS
	ldrb r0,[m6502pc],#1
	mov r0,r0,lsl#24
	ldrb addy,[m6502zpage,r0,lsr#24]
	add r0,r0,#0x01000000
	ldrb r1,[m6502zpage,r0,lsr#24]
	cmn m6502y,addy,lsl#24
	subcs cycles,cycles,#CYC_MULT*CYCLE	;@ Waste a cycle if address crosses a page
	add addy,addy,m6502y,lsr#24
	add addy,addy,r1,lsl#8
;@	bic addy,addy,#0xff0000
	.endm

	.macro doIIY_W			;@ Indirect indexed Y	($nn),Y; Write
	.set AddressMode, _ABS
	ldrb r0,[m6502pc],#1
	mov r0,r0,lsl#24
	ldrb addy,[m6502zpage,r0,lsr#24]
	add r0,r0,#0x01000000
	ldrb r1,[m6502zpage,r0,lsr#24]
	add addy,addy,m6502y,lsr#24
	add addy,addy,r1,lsl#8
;@	bic addy,addy,#0xff0000
	.endm

	.macro doZPI			;@ Zeropage indirect	($nn)
	.set AddressMode, _ABS
	ldrb r0,[m6502pc],#1
	ldrb addy,[r0,m6502zpage]!
	ldrb r1,[r0,#1]
	orr addy,addy,r1,lsl#8
	.endm

	.macro doZ				;@ Zero page			$nn
	.set AddressMode, _ZP
	ldrb addy,[m6502pc],#1
	.endm

#if defined(W65C02)
	.macro doZ2				;@ Zero page			$nn
	.set AddressMode, _ZP
	ldrb addy,[m6502pc],#2	;@ Ugly thing for BBR/BBS
	.endm
#endif

	.macro doZIX			;@ Zero page indexed X	$nn,X
	.set AddressMode, _ZP
	ldrb addy,[m6502pc],#1
	add addy,m6502x,addy,lsl#24
	mov addy,addy,lsr#24
	.endm

	.macro doZIXf			;@ Zero page indexed X	$nn,X
	.set AddressMode, _ZPI
	ldrb addy,[m6502pc],#1
	add addy,m6502x,addy,lsl#24
	.endm

	.macro doZIY			;@ Zero page indexed Y	$nn,Y
	.set AddressMode, _ZP
	ldrb addy,[m6502pc],#1
	add addy,m6502y,addy,lsl#24
	mov addy,addy,lsr#24
	.endm

	.macro doZIYf			;@ Zero page indexed Y	$nn,Y
	.set AddressMode, _ZPI
	ldrb addy,[m6502pc],#1
	add addy,m6502y,addy,lsl#24
	.endm

;@----------------------------------------------------------------------------

	.macro opKIL					;@ Lock cpu
	mov r11,r11
	sub m6502pc,m6502pc,#1
	clearCycles
	fetch 1
	.endm

	.macro opADC cyc
#ifndef CPU_RP2A03
	tst cycles,#CYC_D
	bne opADC_Dec
#endif
	movs r1,cycles,lsr#1			;@ Get C
	subcs r0,r0,#0x00000100
	orr cycles,cycles,#CYC_C+CYC_V	;@ Prepare C & V
	adcs m6502a,m6502a,r0,ror#8
	bicvc cycles,cycles,#CYC_V		;@ V
	getNextOpcode
	mov m6502nz,m6502a,asr#24		;@ NZ
	executeOpcode_c \cyc
	.endm

#ifndef CPU_RP2A03
	.macro opADCD cyc
	movs r1,cycles,lsr#1        	;@ Get C
	adc m6502nz,r0,m6502a,lsr#24	;@ Z is set with normal addition

	mov r1,r0,lsl#28
	subcs r1,r1,#0xF0000001
	adcs m6502a,r1,m6502a,ror#28
	cmncc m6502a,#0x60000000
	addcs m6502a,m6502a,#0x60000000

	orr cycles,cycles,#CYC_C+CYC_V	;@ Prepare C & V

	mov r0,r0,lsr#4
	subcs r0,r0,#0x00000010
	mov m6502a,m6502a,ror#4
	adcs m6502a,m6502a,r0,ror#4
	orrmi m6502nz,m6502nz,#PSR_N	;@ N & V is set after high addition, before fixup
	bicvc cycles,cycles,#CYC_V		;@ V
	cmncc m6502a,#0x60000000
	addcs m6502a,m6502a,#0x60000000
	fetch_c \cyc
	.endm
#endif

	.macro opAND cyc
	and m6502a,m6502a,r0,lsl#24
	getNextOpcode
	mov m6502nz,m6502a,asr#24		;@ NZ
	executeOpcode \cyc
	.endm

	.macro opARR cyc
	readMem
	and r1,m6502a,r0,lsl#24
	tst cycles,cycles,lsr#1			;@ Get C
	orr cycles,cycles,#CYC_C+CYC_V	;@ Prepare C & V
	mov m6502a,r1,rrx
	eor r0,m6502a,r1
	tst r0,#0x40000000
	biceq cycles,cycles,#CYC_V		;@ V
	mov m6502nz,m6502a,asr#24		;@ NZ
#ifndef CPU_RP2A03
	tst cycles,#CYC_D
	bne .F0
#endif
	tst m6502a,m6502a,lsr#31
	b .F1
.F0:
	mov m6502a,m6502a,ror#28
	and r0,r1,#0x0F000000
	cmp r0,#0x05000000
	addpl m6502a,m6502a,#0x60000000
	mov m6502a,m6502a,ror#4
	and r0,r1,#0xF0000000
	cmp r0,#0x50000000
	addcs m6502a,m6502a,#0x60000000
.F1:
	and m6502a,m6502a,#0xff000000
	fetch_c \cyc
	.endm

	.macro opASL cyc
	readWriteMem
	 mov cycles,cycles,lsr#1		;@ Get C
	 add r0,r0,r0
	 orrs m6502nz,r0,r0,lsl#24		;@ NZ
	 adc cycles,cycles,cycles		;@ Set C
	writeMem
	fetch \cyc
	.endm

#if defined(W65C02)
	.macro opBBR bit
	doZ2
	readMemZP
	tst r0,#1<<(\bit)
	bne noBBranch
	ldrsbeq r0,[m6502pc,#-1]
	addeq m6502pc,m6502pc,r0
	fetch 6
	.endm

	.macro opBBRx bit
	doZ2
	readMemZP
	tst r0,#1<<(\bit)
	bne noBBranch
	ldrsbeq r0,[m6502pc,#-1]
	addeq m6502pc,m6502pc,r0
	cmp r0,#-3
	andeq cycles,cycles,#CYC_MASK	;@ Save CPU bits
	fetch 6
	.endm

	.macro opBBS bit
	doZ2
	readMemZP
	tst r0,#1<<(\bit)
	beq noBBranch
	ldrsbne r0,[m6502pc,#-1]
	addne m6502pc,m6502pc,r0
	fetch 6
	.endm

	.macro opBBSx bit
	doZ2
	readmemzp
	tst r0,#1<<(\bit)
	beq noBBranch
	ldrsbne r0,[m6502pc,#-1]
	addne m6502pc,m6502pc,r0
	cmp r0,#-3
	andeq cycles,cycles,#CYC_MASK	;@ Save CPU bits
	fetch 6
	.endm
#endif

	.macro opBIT cyc
	bic cycles,cycles,#CYC_V		;@ Clear V
	tst r0,#V
	and m6502nz,r0,m6502a,lsr#24	;@ Z
	orr m6502nz,m6502nz,r0,lsl#24	;@ N
	getNextOpcode
	orrne cycles,cycles,#CYC_V		;@ V
	executeOpcode \cyc
	.endm

	.macro opBCC
	ldrsb r0,[m6502pc],#1
	tst cycles,#CYC_C				;@ Test Carry
	mov r1,m6502pc,lsl#23			;@ Prepare check of page crossing
	subeq cycles,cycles,#CYC_MULT*CYCLE
	addeq m6502pc,m6502pc,r0
	eors r1,r1,m6502pc,lsl#23		;@ Check page crossing
	submi cycles,cycles,#CYC_MULT*CYCLE
	fetch 2
	.endm

	.macro opBCS
	ldrsb r0,[m6502pc],#1
	tst cycles,#CYC_C				;@ Test Carry
	mov r1,m6502pc,lsl#23			;@ Prepare check of page crossing
	subne cycles,cycles,#CYC_MULT*CYCLE
	addne m6502pc,m6502pc,r0
	eors r1,r1,m6502pc,lsl#23		;@ Check page crossing
	submi cycles,cycles,#CYC_MULT*CYCLE
	fetch 2
	.endm

	.macro opBEQ
	ldrsb r0,[m6502pc],#1
	tst m6502nz,#0xff
	mov r1,m6502pc,lsl#23			;@ Prepare check of page crossing
	subeq cycles,cycles,#CYC_MULT*CYCLE
	addeq m6502pc,m6502pc,r0
	eors r1,r1,m6502pc,lsl#23		;@ Check page crossing
	submi cycles,cycles,#CYC_MULT*CYCLE
	fetch 2
	.endm

	.macro opBMI
	ldrsb r0,[m6502pc],#1
	tst m6502nz,#0x80000000
	mov r1,m6502pc,lsl#23			;@ Prepare check of page crossing
	subne cycles,cycles,#CYC_MULT*CYCLE
	addne m6502pc,m6502pc,r0
	eors r1,r1,m6502pc,lsl#23		;@ Check page crossing
	submi cycles,cycles,#CYC_MULT*CYCLE
	fetch 2
	.endm

	.macro opBNE
	ldrsb r0,[m6502pc],#1
	tst m6502nz,#0xff
	mov r1,m6502pc,lsl#23			;@ Prepare check of page crossing
	subne cycles,cycles,#CYC_MULT*CYCLE
	addne m6502pc,m6502pc,r0
	eors r1,r1,m6502pc,lsl#23		;@ Check page crossing
	submi cycles,cycles,#CYC_MULT*CYCLE
	fetch 2
	.endm

	.macro opBPL
	ldrsb r0,[m6502pc],#1
	tst m6502nz,#0x80000000
	mov r1,m6502pc,lsl#23			;@ Prepare check of page crossing
	subeq cycles,cycles,#CYC_MULT*CYCLE
	addeq m6502pc,m6502pc,r0
	eors r1,r1,m6502pc,lsl#23		;@ Check page crossing
	submi cycles,cycles,#CYC_MULT*CYCLE
	fetch 2
	.endm

	.macro opBVC
	ldrsb r0,[m6502pc],#1
	tst cycles,#CYC_V
	mov r1,m6502pc,lsl#23			;@ Prepare check of page crossing
	subeq cycles,cycles,#CYC_MULT*CYCLE
	addeq m6502pc,m6502pc,r0
	eors r1,r1,m6502pc,lsl#23		;@ Check page crossing
	submi cycles,cycles,#CYC_MULT*CYCLE
	fetch 2
	.endm

	.macro opBVS
	ldrsb r0,[m6502pc],#1
	tst cycles,#CYC_V
	mov r1,m6502pc,lsl#23			;@ Prepare check of page crossing
	subne cycles,cycles,#CYC_MULT*CYCLE
	addne m6502pc,m6502pc,r0
	eors r1,r1,m6502pc,lsl#23		;@ Check page crossing
	submi cycles,cycles,#CYC_MULT*CYCLE
	fetch 2
	.endm

	.macro opCOMP reg cyc
	subs m6502nz,\reg,r0,lsl#24
	orr cycles,cycles,#CYC_C		;@ Prepare C
	getNextOpcode
	mov m6502nz,m6502nz,asr#24		;@ NZ
	executeOpcode_c \cyc
	.endm

	.macro opDCP cyc				;@ Decrease Compare
	readWriteMem
	sub r0,r0,#1
	subs m6502nz,m6502a,r0,lsl#24
	mov m6502nz,m6502nz,asr#24		;@ NZ
	orr cycles,cycles,#CYC_C		;@ Prepare C
	biccc cycles,cycles,#CYC_C
	writeMem
	fetch \cyc
	.endm

	.macro opDEC cyc
	readWriteMem
	sub r0,r0,#1
	orr m6502nz,r0,r0,lsl#24		;@ NZ
	writeMem
	fetch \cyc
	.endm

	.macro opEOR cyc
	eor m6502a,m6502a,r0,lsl#24
	getNextOpcode
	mov m6502nz,m6502a,asr#24		;@ NZ
	executeOpcode \cyc
	.endm


	.macro opINC cyc
	readWriteMem
	add r0,r0,#1
	orr m6502nz,r0,r0,lsl#24		;@ NZ
	writeMem
	fetch \cyc
	.endm

	.macro opISB cyc
	readWriteMem
	add r0,r0,#0x01
	and r0,r0,#0xFF					;@ This is only needed for decimal mode.
	writeMem
#ifndef CPU_RP2A03
	tst cycles,#CYC_D
	bne opSBC_Dec
#endif
	tst cycles,cycles,lsr#1			;@ Get C
	sbcs m6502a,m6502a,r0,lsl#24
	and m6502a,m6502a,#0xff000000
	mov m6502nz,m6502a,asr#24	 	;@ NZ
	orr cycles,cycles,#CYC_C+CYC_V	;@ Prepare C & V
	bicvc cycles,cycles,#CYC_V		;@ V
	fetch_c \cyc
	.endm

	.macro opLAS cyc
	and m6502a,m6502sp,r0,lsl#24
	and m6502x,m6502sp,r0,lsl#24
	orr m6502sp,m6502a,#1
	getNextOpcode
	mov m6502nz,m6502a,asr#24
	executeOpcode \cyc
	.endm

	.macro opLAX cyc
	readMemS
	mov m6502a,m6502nz,lsl#24
	getNextOpcode
	mov m6502x,m6502nz,lsl#24
	executeOpcode \cyc
	.endm

	.macro opLOAD reg cyc
	readMemS
	getNextOpcode
	mov \reg,m6502nz,lsl#24
	executeOpcode \cyc
	.endm

	.macro opLoadAbsX reg cyc
	readMemAIX
	orr m6502nz,r0,r0,lsl#24
	getNextOpcode
	mov \reg,m6502nz,lsl#24
	executeOpcode \cyc
	.endm

	.macro opLSR cyc
	.ifeq AddressMode-_ABS
		readWriteMem
		movs r0,r0,lsr#1
		orr cycles,cycles,#CYC_C	;@ Prepare C
		biccc cycles,cycles,#CYC_C
		mov m6502nz,r0				;@ Z, (N=0)
		writeMem
		fetch \cyc
	.endif
	.ifeq AddressMode-_ZP
		ldrb m6502nz,[m6502zpage,addy]
		orr cycles,cycles,#CYC_C	;@ Prepare C
		movs m6502nz,m6502nz,lsr#1	;@ Z, (N=0)
		getNextOpcode
		strb m6502nz,[m6502zpage,addy]
		executeOpcode_c \cyc
	.endif
	.ifeq AddressMode-_ZPI
		ldrb m6502nz,[m6502zpage,addy,lsr#24]
		orr cycles,cycles,#CYC_C	;@ Prepare C
		movs m6502nz,m6502nz,lsr#1	;@ Z, (N=0)
		getNextOpcode
		strb m6502nz,[m6502zpage,addy,lsr#24]
		executeOpcode_c \cyc
	.endif
	.endm

	.macro opLXA cyc
	readMem
#ifdef CPU_RP2A03
	orr m6502a,m6502a,#0xFF000000
#else
	orr m6502a,m6502a,#0xEE000000
#endif
	and m6502a,m6502a,r0,lsl#24
	mov m6502x,m6502a
	getNextOpcode
	mov m6502nz,m6502a,asr#24		;@ NZ
	executeOpcode \cyc
	.endm

	.macro opORA cyc
	orr m6502a,m6502a,r0,lsl#24
	getNextOpcode
	mov m6502nz,m6502a,asr#24		;@ NZ
	executeOpcode \cyc
	.endm

	.macro opRLA cyc
	readWriteMem
	movs cycles,cycles,lsr#1		;@ Get C
	adc r0,r0,r0
	ands m6502a,m6502a,r0,lsl#24
	mov m6502nz,m6502a,asr#24		;@ NZ
	adc cycles,cycles,cycles		;@ Set C
	writeMem
	fetch \cyc
	.endm

	.macro opRMB bit
	doZ
	readMemZP
	bic r0,r0,#1<<(\bit)
	writeMemZP
	fetch 5
	.endm

	.macro opROL cyc
	readWriteMem
	movs cycles,cycles,lsr#1		;@ Get C
	adc r0,r0,r0
	orrs m6502nz,r0,r0,lsl#24		;@ NZ
	adc cycles,cycles,cycles		;@ Set C
	writeMem
	fetch \cyc
	.endm

	.macro opROR cyc
	readWriteMem
	movs cycles,cycles,lsr#1		;@ Get C
	orrcs r0,r0,#0x100
	movs r0,r0,lsr#1
	adc cycles,cycles,cycles		;@ Set C
	orr m6502nz,r0,r0,lsl#24		;@ NZ
	writeMem
	fetch \cyc
	.endm

	.macro opRRA cyc
	readWriteMem
	movs cycles,cycles,lsr#1		;@ Get C
	orrcs r0,r0,#0x100
	movs r0,r0,lsr#1
	adc cycles,cycles,cycles		;@ Set C
	writeMem
#ifndef CPU_RP2A03
	tst cycles,#CYC_D
	bne opADC_Dec
#endif
	movs r1,cycles,lsr#1			;@ Get C
	subcs r0,r0,#0x00000100
	adcs m6502a,m6502a,r0,ror#8
	mov m6502nz,m6502a,asr#24		;@ NZ
	orr cycles,cycles,#CYC_C+CYC_V	;@ Prepare C & V
	bicvc cycles,cycles,#CYC_V		;@ V
	fetch_c \cyc
	.endm

	.macro opSAX cyc
	and r0,m6502a,m6502x
	mov r0,r0,lsr#24
	writeMem
	fetch \cyc
	.endm

	.macro opSBC cyc
#ifndef CPU_RP2A03
	tst cycles,#CYC_D
	bne opSBC_Dec
#endif
	movs r1,cycles,lsr#1			;@ Get C
	sbcs m6502a,m6502a,r0,lsl#24
	orr cycles,cycles,#CYC_C+CYC_V	;@ Prepare C & V
	and m6502a,m6502a,#0xff000000
	bicvc cycles,cycles,#CYC_V		;@ V
	getNextOpcode
	mov m6502nz,m6502a,asr#24		;@ NZ
	executeOpcode_c \cyc
	.endm

#ifndef CPU_RP2A03
	.macro opSBCD cyc
	movs r1,cycles,lsr#1			;@ Get C
	subcc r0,r0,#0x00000100
	sbcs m6502a,m6502a,r0,ror#8
	mov m6502nz,m6502a,asr#24 		;@ NZ
	bic cycles,cycles,#CYC_C+CYC_V	;@ Clear C & V
	orrcs cycles,cycles,#CYC_C		;@ C
	orrvs cycles,cycles,#CYC_V		;@ V
	subcc m6502a,m6502a,#0x60000000

	mov m6502a,m6502a,ror#28
	mvn r0,r0,ror#31
	cmp m6502a,r0,lsl#27
	subcs m6502a,m6502a,#0x60000000
	getNextOpcode
	mov m6502a,m6502a,ror#4
	executeOpcode \cyc
	.endm
#endif

	.macro opSBX cyc
	readMem
	and m6502x,m6502x,m6502a
	subs m6502x,m6502x,r0,lsl#24
	mov m6502nz,m6502x,asr#24	 	;@ NZ
	getNextOpcode
	orr cycles,cycles,#CYC_C		;@ Prepare C
	executeOpcode_c \cyc
	.endm

	.macro opSHA_ABS_Y cyc
	ldrb addy,[m6502pc],#1
	ldrb r1,[m6502pc],#1
	adds addy,m6502y,addy,lsl#24

	adc r0,r1,#0x01
	and r0,r0,m6502x,lsr#24
	and r0,r0,m6502a,lsr#24

	orrcc addy,addy,r1
	orrcs addy,addy,r0
	mov addy,addy,ror#24
	writeMem
	fetch \cyc
	.endm

	.macro opSHA_IND_Y cyc
	ldrb r0,[m6502pc],#1
	ldrb addy,[r0,m6502zpage]!
	ldrb r1,[r0,#1]
	adds addy,m6502y,addy,lsl#24

	adc r0,r1,#0x01
	and r0,r0,m6502x,lsr#24
	and r0,r0,m6502a,lsr#24

	orrcc addy,addy,r1
	orrcs addy,addy,r0
	mov addy,addy,ror#24
	writeMem
	fetch \cyc
	.endm

	.macro opSHS_ABS_Y cyc
	ldrb addy,[m6502pc],#1
	ldrb r1,[m6502pc],#1
	adds addy,m6502y,addy,lsl#24

	adc r0,r1,#0x01
	and r0,r0,m6502x,lsr#24
	and r0,r0,m6502a,lsr#24

	orrcc addy,addy,r1
	orrcs addy,addy,r0
	mov addy,addy,ror#24
	writeMem

	and m6502sp,m6502x,m6502a
	orr m6502sp,m6502sp,#1
	fetch \cyc
	.endm

	.macro opSHX_ABS_Y cyc
	ldrb addy,[m6502pc],#1
	ldrb r1,[m6502pc],#1
	adds addy,m6502y,addy,lsl#24

	adc r0,r1,#0x01
	and r0,r0,m6502x,lsr#24

	orrcc addy,addy,r1
	orrcs addy,addy,r0
	mov addy,addy,ror#24
	writeMem
	fetch \cyc
	.endm

	.macro opSHY_ABS_X cyc
	ldrb addy,[m6502pc],#1
	ldrb r1,[m6502pc],#1
	adds addy,m6502x,addy,lsl#24

	adc r0,r1,#0x01
	and r0,r0,m6502y,lsr#24

	orrcc addy,addy,r1
	orrcs addy,addy,r0
	mov addy,addy,ror#24
	writeMem
	fetch \cyc
	.endm

	.macro opSLO cyc
	readWriteMem
	add r0,r0,r0
	orrs m6502a,m6502a,r0,lsl#24
	mov m6502nz,m6502a,asr#24		;@ NZ
	orr cycles,cycles,#CYC_C		;@ Prepare C
	biccc cycles,cycles,#CYC_C		;@ C
	writeMem
	fetch \cyc
	.endm

	.macro opSMB bit
	doZ
	readMemZP
	orr r0,r0,#1<<(\bit)
	writeMemZP
	fetch 5
	.endm

	.macro opSRE cyc
	readWriteMem
	movs r0,r0,lsr#1
	eor m6502a,m6502a,r0,lsl#24
	mov m6502nz,m6502a,asr#24		;@ NZ
	orr cycles,cycles,#CYC_C		;@ Prepare C
	biccc cycles,cycles,#CYC_C		;@ C
	writeMem
	fetch \cyc
	.endm

	.macro opSTORE reg cyc
	mov r0,\reg,lsr#24
	writeMem
	fetch \cyc
	.endm

	.macro opSTZ cyc
	mov r0,#0
	writeMem
	fetch \cyc
	.endm

	.macro opTRB cyc
	readMem
	and r1,r0,m6502a,lsr#24			;@ Flags
	bic r0,r0,m6502a,lsr#24			;@ Result
	bic m6502nz,m6502nz,#0xFF
	orr m6502nz,m6502nz,r1			;@ Z
	writeMem
	fetch \cyc
	.endm

	.macro opTSB cyc
	readMem
	and r1,r0,m6502a,lsr#24			;@ Flags
	orr r0,r0,m6502a,lsr#24			;@ Result
	bic m6502nz,m6502nz,#0xFF
	orr m6502nz,m6502nz,r1			;@ Z
	writeMem
	fetch \cyc
	.endm

;@----------------------------------------------------------------------------
