//! @file isr.c
//! @brief isr
//! @author Jean-Baptiste Trédez

#include "io/debug.h"

void isr_reset(void);
static void isr_nmi(void);
static void isr_hard_fault(void);
static void isr_mpu_fault(void);
static void isr_bus_fault(void);
static void isr_usage_fault(void);
static void isr_debug_monitor(void);
static void isr_unexpected(void);

extern void xPortPendSVHandler(void);
extern void xPortSysTickHandler(void);
extern void vPortSVCHandler( void );
extern int main(void);

extern unsigned long _etext;
extern unsigned long __data_start;
extern unsigned long _edata;
extern unsigned long __bss_start;
extern unsigned long _bss_end__;
extern unsigned long _stack;

__attribute__ ((section(".isr_vector")))
void (* const g_pfnVectors[])(void) =
{
	// core level - cortex-m3
	(void*)&_stack,                         // The initial stack pointer
	isr_reset,                              // The reset handler
	isr_nmi,                                // The NMI handler
	isr_hard_fault,                         // The hard fault handler
	isr_mpu_fault,                          // The MPU fault handler
	isr_bus_fault,                          // The bus fault handler
	isr_usage_fault,                        // The usage fault handler
	0,                                      // Reserved
	0,                                      // Reserved
	0,                                      // Reserved
	0,                                      // Reserved
	vPortSVCHandler,                        // SVCall handler
	isr_debug_monitor,                      // Debug monitor handler
	0,                                      // Reserved
	xPortPendSVHandler,                     // The PendSV handler
	xPortSysTickHandler,                    // The SysTick handler

	// chip level
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
	isr_unexpected,
};

void isr_reset(void)
{
	unsigned long *pulSrc, *pulDest;

	//
	// Copy the data segment initializers from flash to SRAM.
	//
	pulSrc = &_etext;
	for(pulDest = &__data_start; pulDest < &_edata; )
	{
		*pulDest++ = *pulSrc++;
	}

    //
    // Zero fill the bss segment.  This is done with inline assembly since this
    // will clear the value of pulDest if it is not kept in a register.
    //
    __asm("    ldr     r0, =__bss_start\n"
          "    ldr     r1, =_bss_end__\n"
          "    mov     r2, #0\n"
          "    .thumb_func\n"
          "zero_loop:\n"
          "        cmp     r0, r1\n"
          "        it      lt\n"
          "        strlt   r2, [r0], #4\n"
          "        blt     zero_loop");

	main();
}

static void isr_nmi(void)
{
	while(debug_mode())
	{

	}
}

static void isr_hard_fault(void)
{
	while(debug_mode())
	{

	}
}

static void isr_mpu_fault(void)
{
	while(debug_mode())
	{

	}
}

static void isr_bus_fault(void)
{
	while(debug_mode())
	{

	}
}

static void isr_usage_fault(void)
{
	while(debug_mode())
	{

	}
}

static void isr_debug_monitor(void)
{
	while(debug_mode())
	{

	}
}

static void isr_unexpected(void)
{
	while(debug_mode())
	{

	}
}
