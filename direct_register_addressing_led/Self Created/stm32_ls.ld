ENTRY(Reset_Handler)

MEMORY
{
  FLASH(rx)  : ORIGIN = 0x08000000, LENGTH = 1024K
  SRAM(rwx)  : ORIGIN = 0x20010000, LENGTH = 256K   /* Using SRAM1 only */
}

SECTIONS
{
  .text :
  { 
      *(.isr_vector)
      *(.text)
      *(.rodata)
      _etext = .;
  } > FLASH   /* Correct: no AT> FLASH needed */

  .data : AT (LOADADDR(.text))   /* Here AT is needed */
  {
      _sdata = .;
      *(.data)
      _edata = .;
      _la_data = LOADADDR(.data);

  } > SRAM

  .bss (NOLOAD) :
  {
      _sbss = .;
      *(.bss)
      _ebss = .;
  } > SRAM
}

