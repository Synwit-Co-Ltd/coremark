# coremark
coremark test for synwit mcu.

## SWM341
```
SystemCoreClock = 140000000

2K performance run parameters for coremark.
CoreMark Size    : 666
Total ticks      : 1574
Total time (secs): 15.740000
Iterations/Sec   : 381.194409
Iterations       : 6000
Compiler version : GCCClang 13.0.0 (ssh://ds-gerrit/armcompiler/llvm-project)
Compiler flags   : -O3
Memory location  : Flash
seedcrc          : 0xe9f5
[0]crclist       : 0xe714
[0]crcmatrix     : 0x1fd7
[0]crcstate      : 0x8e3a
[0]crcfinal      : 0xa14c
Correct operation validated. See README.md for run and reporting rules.
CoreMark 1.0 : 381.194409 / GCCClang 13.0.0 (ssh://ds-gerrit/armcompiler/llvm-project) -O3 / Flash
```
381.19 / 140MHz = 2.72/MHz
