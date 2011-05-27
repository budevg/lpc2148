MEMORY
{
  FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 0x0007FFFF
  RAM (rw)   : ORIGIN = 0x40000000, LENGTH = 0x00007FFF
}

_top_stack = 0x40008000;

SECTIONS
{
  .text :             /* code                    */
  {
    startup.o (.text) /* vectors & startup code  */
    *(.text)          /* remaining code          */
  } > FLASH
  . = ALIGN(4);
  
  .rodata :           /* read-only constants     */
  {
    *(.rodata)
    *(.rodata*)
  } > FLASH
  . = ALIGN(4);
  
  _text_end   = . ;

  .data : AT (_text_end) /* initialized data     */
  {
    _data_start = . ;
    *(.data)
  _data_end   = . ;
  } > RAM
  . = ALIGN(4);

  .bss :              /* uninitialized variables */
  {
    _bss_start  = . ;
    *(.bss)
  } > RAM
  . = ALIGN(4);
  
  _bss_end    = . ;
}