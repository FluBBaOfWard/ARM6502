# ARM6502 V0.2.2
A 6502 cpu emulator for ARM32.

You can define W65C02, W65C02_OLD or CPU_RP2A03 to get different versions of the 6502.
You can also define NO_FASTMEM_6502 to not use itcm (on NDS) or iwram (on GBA) for cpu core.

First you need to allocate space for the cpu core state, either by using the struct from C or allocating/reserving memory using the "m6502Size"
Next call m6502Init with a pointer to that memory.
