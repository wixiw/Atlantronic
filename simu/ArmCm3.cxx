#include "ArmCm3.h"

ArmCm3::ArmCm3()
{

}

ArmCm3::~ArmCm3()
{

}

void ArmCm3::mem_write(uint64_t offset, uint32_t val)
{
	offset += PERIPH_BASE;

	if( offset >= RCC_BASE && offset < (RCC_BASE + 0x400) )
	{
		RCC.mem_write(offset - RCC_BASE, val);
	}
	else if( offset >= TIM1_BASE && offset < (TIM1_BASE + 0x1000) )
	{
		TIM1.mem_write(offset - TIM1_BASE, val);
	}
}

uint32_t ArmCm3::mem_read(uint64_t offset)
{
	uint32_t rep = 0;

	offset += PERIPH_BASE;

	if( offset >= RCC_BASE && offset < (RCC_BASE + 0x400) )
	{
		rep = RCC.mem_read(offset - RCC_BASE);
	}
	else if( offset >= TIM1_BASE && offset < (TIM1_BASE + 0x1000) )
	{
		rep = TIM1.mem_read(offset - TIM1_BASE);
	}

	return rep;
}
