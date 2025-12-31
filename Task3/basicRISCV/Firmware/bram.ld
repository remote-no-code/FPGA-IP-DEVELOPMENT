MEMORY
{
   BRAM (RWX) : ORIGIN = 0x0000, LENGTH = 0x1800  /* 6kB RAM */
}

SECTIONS
{
    everything :
    {
        . = ALIGN(4);
        start.o (.text)
        *(.*)
    } >BRAM
}

/* Stack top at end of BRAM */
_stack_top = ORIGIN(BRAM) + LENGTH(BRAM);
