.section .text
.globl _start

.equ IO_BASE, 0x40000000   # <<< THIS LINE WAS MISSING

_start:
    la   gp, IO_BASE       # initialize global pointer for MMIO
    la   sp, _stack_top
    call main

1:
    j 1b
