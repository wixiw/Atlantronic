#ifndef ARM_TIM_H
#define ARM_TIM_H

//! @file ArmTim.h
//! @brief TIM
//! @author Jean-Baptiste Trédez

#include "ArmMem.h"

class ArmTim : public ArmMem<TIM_TypeDef>
{
public:
	ArmTim();
	~ArmTim();

	float getPwm(int num);
	void setEncoder(uint16_t val);
};

#endif
