# ARM6502 V0.3.5

A 6502 cpu emulator for ARM32.

You can define W65C02, W65C02_OLD, CPU_6510 or CPU_RP2A03 to get different versions of the 6502.
You can also define NO_FASTMEM_6502 to not use itcm (on NDS) or iwram (on GBA) for cpu core.

## How to use

First you need to allocate space for the cpu core state, either by using the struct from C or allocating/reserving memory using the "m6502Size"
Next call m6502Init with a pointer to that memory.
All memory mapped to the cpu needs to aligned to 0x100 for page crossing checks for extra cycles.


## Projects That use this CPU core

* https://github.com/FluBBaOfWard/C64DS
* https://github.com/FluBBaOfWard/N2A03
* https://github.com/FluBBaOfWard/NesDS
* https://github.com/FluBBaOfWard/PunchOutDS
* https://github.com/FluBBaOfWard/RenegadeDS
* https://github.com/FluBBaOfWard/RenegadeGBA
* https://github.com/FluBBaOfWard/WasabiDS
* https://github.com/FluBBaOfWard/WasabiGBA

## Credits

```text
Most code is derived from PocketNES which was started by Loopy.
Dwedit helped with a lot of things. https://www.dwedit.org
```
