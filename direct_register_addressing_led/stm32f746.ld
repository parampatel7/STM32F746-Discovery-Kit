/* STM32F746 minimal linker script
   Adjust as needed for your application */

MEMORY
{
  FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 1024K
  RAM (xrw)   : ORIGIN = 0x20000000, LENGTH = 320K
}

/* Define stack size */
_estack = ORIGIN(RAM) + LENGTH(RAM); /* End of RAM (stack start) */

SECTIONS
{
  /* Vector table and code in FLASH */
  .isr_vector :
  {
    KEEP(*(.isr_vector))  /* Vector table */
  } >FLASH

  /* Code section */
  .text :
  {
    *(.text*)           /* Program code */
    *(.rodata*)         /* Read-only data */
    *(.glue_7) *(.glue_7t)
    KEEP(*(.init))
    KEEP(*(.fini))
  } >FLASH

  /* Initialized data section, load in FLASH but run in RAM */
  .data : AT (ADDR(.text) + SIZEOF(.text))
  {
    __data_start__ = .;
    *(.data*)
    __data_end__ = .;
  } >RAM

  /* Uninitialized data section */
  .bss :
  {
    __bss_start__ = .;
    *(.bss*)
    *(COMMON)
    __bss_end__ = .;
    _end = .;
  } >RAM

  /* Stack section (optional, can be just a symbol) */
  .stack (NOLOAD) :
  {
    . = ALIGN(8);
    _estack = ORIGIN(RAM) + LENGTH(RAM);
  } >RAM

  /* Heap section (optional) */
  .heap (NOLOAD) :
  {
    . = ALIGN(8);
    __heap_start__ = .;
    __heap_end__ = ORIGIN(RAM) + LENGTH(RAM) - 0x1000; /* leave 4KB for stack */
  } >RAM

  /* End of RAM */
  end = .;
}

PROVIDE(_estack = ORIGIN(RAM) + LENGTH(RAM));
