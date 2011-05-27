            .section .text
	        .code 32  /* ARM mode, not thumb  */

            .global _startup
            .func   _startup
_startup:
            B       InitReset
            B       . /* undef is not handled */
            B       . /* swi is not handled   */
            B       . /* pabt is not handled  */
            B       . /* dabt is not handled  */
            NOP       /* Reserved Vector      */
            LDR     PC, [PC, #-0x0FF0]
            LDR     PC, fiq_addr

fiq_addr:   .word   fiq_isr

fiq_isr:	B       fiq_isr
.weak		fiq_isr

InitReset:
            /* constants for setting stacks   */
            .equ    FIQ_Stack_Size, 0x00000100
            .equ    IRQ_Stack_Size, 0x00000400
            .equ    SYS_Stack_Size, 0x00000400

            .equ    Mode_USR,       0x10
            .equ    Mode_FIQ,       0x11
            .equ    Mode_IRQ,       0x12
            .equ    Mode_SVC,       0x13
            .equ    Mode_ABT,       0x17
            .equ    Mode_UND,       0x1B
            .equ    Mode_SYS,       0x1F

            .equ    I_Bit,          0x80    /* disable IRQ */
            .equ    F_Bit,          0x40    /* disable FIQ */

            LDR     R0, =_top_stack

            /* enter FIQ mode and set stack   */
            MSR     CPSR_c, #Mode_FIQ|I_Bit|F_Bit
            MOV     SP, R0
            SUB     R0, R0, #FIQ_Stack_Size

            /* enter IRQ mode and set stack   */
            MSR     CPSR_c, #Mode_IRQ|I_Bit|F_Bit
            MOV     SP, R0
            SUB     R0, R0, #IRQ_Stack_Size

            /* enter SYS mode and set stack   */
            MSR     CPSR_c, #Mode_SYS|I_Bit|F_Bit
            MOV     SP, R0
            /* stay in SYS mode from now on   */

            /* relocate .data section to RAM  */
            LDR     R1, =_text_end
            LDR     R2, =_data_start
            LDR     R3, =_data_end
LoopRel:    CMP     R2, R3
            LDRLO   R0, [R1], #4
            STRLO   R0, [R2], #4
            BLO     LoopRel

            /* clear .bss section             */
            MOV     R0, #0
            LDR     R1, =_bss_start
            LDR     R2, =_bss_end
LoopZI:     CMP     R1, R2
            STRLO   R0, [R1], #4
            BLO     LoopZI

JumpToC:
	        LDR     LR,=exit
	        LDR     R0,=main
	        BX      R0

           .size    _startup, . - _startup
           .endfunc

           .global  exit
           .func    exit
exit:       B       .
	       .size    exit, . - exit
           .endfunc
           .end

