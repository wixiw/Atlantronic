#include "core/module.h"
#include "core/rcc.h"
#include "os/os.h"
#include "led.h"
#include "core/gpio.h"
#include "master/color.h"

#define LED_STACK_SIZE           300

static void led_task(void *arg);
static void led_two_half_chaser();
static void led_chaser();
static void led_tetris();
static void led_blink(uint32_t maskOn, uint32_t maskOff);
static void led_slow_blink(uint32_t maskOn, uint32_t maskOff);

static eLedState led_mode;
static int led_step;
const portTickType periodInMs = ms_to_tick(100);

#define LED_CPU_RED      0x00004000
#define LED_CPU_BLUE     0x00008000
#define LED_EXT_BLUE     0x80000000
#define LED_EXT_GREEN    0x20000000
#define LED_EXT_YELLOW   0x00100000
#define LED_EXT_ORANGE2  0x00040000
#define LED_EXT_RED      0x01000000
#define LED_NOTHING      0x00000000

void setLed(uint32_t mask)
{
	GPIOB->BSRRL = (uint16_t)((mask & LED_EXT_RED) >> 16);
	GPIOB->BSRRH = (uint16_t)(((~mask) & LED_EXT_RED) >> 16);

	GPIOC->BSRRL = (uint16_t)((mask & (LED_EXT_BLUE | LED_EXT_GREEN)) >> 16);
	GPIOC->BSRRH = (uint16_t)(((~mask) & (LED_EXT_BLUE | LED_EXT_GREEN)) >> 16);

	GPIOD->BSRRL = (uint16_t)(mask & (LED_CPU_RED | LED_CPU_BLUE));
	GPIOD->BSRRH = (uint16_t)((~mask) & ( LED_CPU_RED | LED_CPU_BLUE));

	GPIOE->BSRRL = (uint16_t)((mask & (LED_EXT_YELLOW | LED_EXT_ORANGE2)) >> 16);
	GPIOE->BSRRH = (uint16_t)(((~mask) & (LED_EXT_YELLOW | LED_EXT_ORANGE2)) >> 16);
}


static int led_module_init(void)
{
	// LED (carte CPU) sur PD14 PD15 (led verte et orange sur PD12 et PD13 non utilisables, encodeur dessus !!)
	// LED (carte led deportee) sur PC15, PC13, PE4, PE2, PB8
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOEEN;
	gpio_pin_init(GPIOB, 8, GPIO_MODE_OUT, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_UP); // LED carte led deportee
	gpio_pin_init(GPIOC, 13, GPIO_MODE_OUT, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_UP); // LED carte led deportee
	gpio_pin_init(GPIOC, 15, GPIO_MODE_OUT, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_UP); // LED carte led deportee
	gpio_pin_init(GPIOD, 14, GPIO_MODE_OUT, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_UP); // LED rouge carte CPU
	gpio_pin_init(GPIOD, 15, GPIO_MODE_OUT, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_UP); // LED bleue carte CPU
	gpio_pin_init(GPIOE, 2, GPIO_MODE_OUT, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_UP); // LED carte led deportee
	gpio_pin_init(GPIOE, 4, GPIO_MODE_OUT, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_UP); // LED carte led deportee

	xTaskHandle xHandle;
	portBASE_TYPE err = xTaskCreate(led_task, "led", LED_STACK_SIZE, NULL, PRIORITY_TASK_LED, &xHandle);

	if(err != pdPASS)
	{
		return ERR_INIT_LED;
	}

	setLed( LED_NOTHING);
	led_mode = LED_MODE_WAIT_X86;

	return MODULE_INIT_SUCCESS;
}

module_init(led_module_init, INIT_LED);

static void led_task(void *arg)
{
	UNUSED(arg);

	portTickType lastWakeTime;

	while(1)
	{
		lastWakeTime = xTaskGetTickCount ();

		switch(led_mode)
		{
			case LED_MODE_WAIT_AU_UP:
				setLed(LED_EXT_RED);
				break;

			case LED_MODE_WAIT_X86:
				led_tetris();
				break;

			case LED_MODE_WAIT_COLOR_SELECTION:
				led_blink(LED_EXT_GREEN | LED_EXT_YELLOW, LED_NOTHING);
				break;

			case LED_MODE_COLOR_SELECTION_PREF:
				led_blink(LED_EXT_BLUE | LED_EXT_YELLOW, LED_EXT_YELLOW);
				break;

			case LED_MODE_COLOR_SELECTION_SYM:
				led_blink(LED_EXT_BLUE | LED_EXT_GREEN, LED_EXT_GREEN);
				break;

			case LED_MODE_COLOR_VALIDATED:
				if(getColor() == COLOR_PREF)
				{
					setLed(LED_EXT_YELLOW);
					break;
				}
				else
				{
					setLed(LED_EXT_GREEN);
					break;
				}

			case LED_MODE_WAIT_SELF_TEST:
				if(getColor() == COLOR_PREF)
				{
					led_blink(LED_EXT_BLUE | LED_EXT_YELLOW, LED_EXT_YELLOW);
					break;
				}
				else
				{
					led_blink(LED_EXT_BLUE | LED_EXT_GREEN, LED_EXT_GREEN);
					break;
				}
				break;

			case LED_MODE_SELF_TESTING:
				led_two_half_chaser();
				break;

			case LED_MODE_WAIT_MATCH:
				if(getColor() == COLOR_PREF)
				{
					led_slow_blink(
							LED_EXT_BLUE | LED_EXT_GREEN | /*LED_EXT_YELLOW | */LED_EXT_ORANGE2 | LED_EXT_RED,
							LED_EXT_BLUE | LED_EXT_YELLOW);
				}
				else
				{
					led_slow_blink(
							LED_EXT_BLUE | /*LED_EXT_GREEN | */ LED_EXT_YELLOW | LED_EXT_ORANGE2 | LED_EXT_RED,
							LED_EXT_BLUE |LED_EXT_GREEN);
				}
				break;

			case LED_MODE_MATCH_RUNNING:
				led_chaser();
				break;

			case LED_MODE_MATCH_ENDED:
				if(getColor() == COLOR_PREF)
				{
					setLed(LED_EXT_YELLOW);
				}
				else
				{
					setLed(LED_EXT_GREEN);
				}
				break;

			default:
				setLed(LED_NOTHING);
				break;
		}

		led_step++;
		vTaskDelayUntil( &lastWakeTime, periodInMs );
	}
}

static void led_two_half_chaser()
{
	switch(led_step)
	{
		default:
			led_step = 0;
		case 0:
			setLed(LED_EXT_BLUE | LED_EXT_RED);
			break;
		case 1:
			setLed(LED_EXT_GREEN | LED_EXT_ORANGE2);
			break;
		case 2:
			setLed(LED_EXT_YELLOW);
			break;
		case 3:
			setLed(0);
			break;
	}
}

static void led_chaser()
{
	switch(led_step)
	{
		default:
			led_step = 0;
		case 0:
			setLed(0);
			break;
		case 1:
			setLed(LED_EXT_BLUE);
			break;
		case 2:
			setLed(LED_EXT_GREEN);
			break;
		case 3:
			setLed(LED_EXT_YELLOW);
			break;
		case 4:
			setLed(LED_EXT_ORANGE2);
			break;
		case 5:
			setLed(LED_EXT_RED);
			break;
	}
}

static void led_tetris()
{
	switch(led_step)
	{
		default:
			led_step = 0;
		case 0:
			setLed(LED_EXT_RED);
			break;
		case 1:
			setLed(LED_EXT_RED | LED_EXT_ORANGE2);
			break;
		case 2:
			setLed(LED_EXT_RED | LED_EXT_ORANGE2 | LED_EXT_YELLOW);
			break;
		case 3:
			setLed(LED_EXT_RED | LED_EXT_ORANGE2 | LED_EXT_YELLOW | LED_EXT_GREEN);
			break;
		case 4:
			setLed(LED_EXT_RED | LED_EXT_ORANGE2 | LED_EXT_YELLOW | LED_EXT_GREEN | LED_EXT_BLUE);
			break;
		case 5:
			setLed(0);
			break;
	}
}

static void led_blink(uint32_t maskOn, uint32_t maskOff)
{
	switch(led_step)
	{
		default:
			led_step = 0;
		case 0:
		case 1:
			setLed(maskOn);
			break;
		case 2:
		case 3:
			setLed(maskOff);
			break;
	}
}

static void led_slow_blink(uint32_t maskOn, uint32_t maskOff)
{
	switch(led_step)
	{
		default:
			led_step = 0;
		case 0:
		case 1:
		case 2:
		case 3:
		case 4:
			setLed(maskOn);
			break;
		case 5:
		case 6:
		case 7:
		case 8:
		case 9:
			setLed(maskOff);
			break;
	}
}

void led_setState(eLedState state)
{
	led_mode = state;
}
