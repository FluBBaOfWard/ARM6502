# ARM6502
A 6502 cpu emulator for ARM32.

You can define W65C02, W65C02_OLD or CPU_N2A03 to get different versions of the 6502.
You can also define NO_FASTMEM_6502 to not use itcm (on NDS) or iwram (on GBA) for cpu core, it will still use dtcm or iwram for tables.
