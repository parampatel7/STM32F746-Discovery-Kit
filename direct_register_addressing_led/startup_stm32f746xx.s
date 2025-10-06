.syntax unified
.cpu cortex-m7
.thumb

.global _estack
.global Reset_Handler

.section .isr_vector, "a", %progbits
.type g_pfnVectors, %object
.size g_pfnVectors, .-g_pfnVectors

g_pfnVectors:
    .word _estack          /* Initial Stack Pointer */
    .word Reset_Handler    /* Reset Handler */
    /* Other vectors can be added here as needed */
    .word 0                /* NMI Handler */
    .word 0                /* HardFault Handler */
    /* ... */

.section .text.Reset_Handler
.type Reset_Handler, %function
Reset_Handler:
    /* Initialize stack pointer - done by hardware automatically */

    /* Normally: Copy .data from flash to RAM, zero .bss, etc. */
    /* Skipped here for brevity */

    bl main

hang:
    b hang

.size Reset_Handler, .-Reset_Handler

/* Define stack size */
.section .stack
.space 0x1000
_estack = . + 0x1000
