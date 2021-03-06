#ifndef PUMP_H
#define PUMP_H

//! @file pump.h
//! @brief Pump
//! @author Atlantronic

#include <stdint.h>
#include "core/asm/asm_base_func.h"
#include "os/systick.h"

#ifndef WEAK_PUMP

#define WEAK_PUMP __attribute__((weak, alias("nop_function") ))
#endif

#define PUMP_SAMPLE_STDDEV              10

void pumps_update() WEAK_PUMP;
uint32_t pumps_get_state() WEAK_PUMP;

typedef enum
{
	PUMP_1,
	PUMP_2,
	PUMP_3,
	PUMP_4,
	PUMP_MAX
} PumpId;

class Pump
{
	public:
		Pump(uint8_t Pwm_id);

		void set(float percent);

		void update();

		inline bool isBlocked();

	protected:
		uint8_t pwm_id;
		uint8_t currentId;
		bool pumpBlocked;
		float val;
		float current[PUMP_SAMPLE_STDDEV];
		float stdDev2;
		systime startTime;
};

extern Pump pump[PUMP_MAX];

//------------------ interface usb -------------------
struct pump_cmd_arg
{
	uint8_t id;         //!< id de la pompe
	uint8_t val;        //!< puissance de 0 a 100
} __attribute__((packed));

//------------------ fonctions inline------------------

inline bool Pump::isBlocked()
{
	return pumpBlocked;
}

#endif
