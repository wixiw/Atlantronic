/*
 * uiMiddleware.cpp
 *
 *  Created on: Feb 25, 2015
 *      Author: willy
 */

#include "uiMiddleware.h"
#include "core/gpio.h"
#include "os/module.h"
#include "os/Signal.h"
#include "components/led/led.h"
#include "components/log/log.h"
#include "match_time.h"

static Signal colorConfiguredSignal;
static Signal selfTestStartSignal;
static Signal matchStartSignal;

static eMatchColor colorRequest = COLOR_UNKNOWN;

static uint8_t color_choice_enable;
static uint8_t match_start_enable;
static uint8_t seltTest_start_enable;

static struct systime colorSwitch_change_time;

//
// UI Display
//

void ui_ubiquityWaitingStartIn()
{
	led_setState(LED_MODE_WAIT_START_IN);
	log(LOG_INFO, "Waiting start to be inserted before beginning.");
}

void ui_ubiquityBooting()
{
	led_setState(LED_MODE_WAIT_X86);
	log(LOG_INFO, "Waiting x86 boot...");
}

void ui_ubiquityReadyForMatch()
{
	led_setState(LED_MODE_READY_MATCH);
	log(LOG_INFO, "Please insert start for the match.");
}

void ui_ubiquityWaitForMatch()
{
	match_start_enable = 1;
	led_setState(LED_MODE_WAIT_MATCH);
	log(LOG_INFO, "Waiting for match begin !");
}

void ui_ubiquityReadyForSelfTests()
{
	seltTest_start_enable = 1;
	led_setState(LED_MODE_WAIT_SELF_TEST);
	log(LOG_INFO, "Waiting self tests.");
}

void ui_selfTesting()
{
	led_setState(LED_MODE_SELF_TESTING);
	log(LOG_INFO, "Self Testing...");
}

void ui_matchRuning()
{
	led_setState(LED_MODE_MATCH_RUNNING);
	log(LOG_INFO, "Match running ...");
}

void ui_matchEnded()
{
	led_setState(LED_MODE_MATCH_ENDED);
	log(LOG_INFO, "Match ended.");
}

void ui_displayEmergencyStopActive()
{
	led_setState(LED_MODE_WAIT_AU_UP);
	log(LOG_INFO, "AU is pushed !!");
}

//
// UI Request
//

eMatchColor ui_requestMatchColor()
{
	log(LOG_INFO, "Waiting color choice...");
	led_setState(LED_MODE_WAIT_COLOR_SELECTION);
	color_choice_enable = 1;
	colorConfiguredSignal.wait();

	color_choice_enable = 0;
	if( colorRequest == COLOR_PREF)
	{
		log(LOG_INFO, "Prefered color (yellow) choosed.");
	}
	else
	{
		log(LOG_INFO, "Symetric color (green) choosed.");
	}

	led_setState(LED_MODE_COLOR_VALIDATED);

	return colorRequest;
}

void ui_waitForMatchStart()
{
	matchStartSignal.wait();
}

void ui_waitForSelfTestStart()
{
	selfTestStartSignal.wait();
}

bool ui_isStartPlugged()
{
	return gpio_get_pin(GPIOD, 3);
}

//
// UIH
//

//As there is no debounce on inputs, ensure flags are not set without a start plugged in.s
void uih_startWithdraw()
{
	if( match_start_enable )
	{
		systick_start_match_from_isr();
		match_start_enable = 0;
		matchStartSignal.setFromIsr();
	}
	else if( seltTest_start_enable )
	{
		seltTest_start_enable = 0;
		selfTestStartSignal.setFromIsr();
	}
	else if( color_choice_enable && colorRequest != COLOR_UNKNOWN)
	{
		colorConfiguredSignal.setFromIsr();
	}

}

//This
void uih_colorButtonPushed()
{
	if( color_choice_enable )
	{
		switch (colorRequest)
		{
			default:
			case COLOR_UNKNOWN:
			case COLOR_SYM:
				colorRequest = COLOR_PREF;
				led_setState(LED_MODE_COLOR_SELECTION_PREF);
				break;
			case COLOR_PREF:
				colorRequest = COLOR_SYM;
				led_setState(LED_MODE_COLOR_SELECTION_SYM);
				break;
		}
	}
	else
	{
		//ui_debug1_event();
	}
}

//
// Plomberie interne
//


static int uimdw_module_init(void)
{
	// "io entrees"
	// boutons USR1 et USR2 carte led sur PC14 et PB7
	// bouton go sur PD3
	gpio_pin_init(GPIOB, 7, GPIO_MODE_IN, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_DOWN); // bouton USR2
	gpio_pin_init(GPIOC, 14, GPIO_MODE_IN, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_DOWN); // bouton USR1
	gpio_pin_init(GPIOD, 3, GPIO_MODE_IN, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_DOWN); // bouton go

	// boutons en IT sur front montant : USR1 et USR2
	// boutons en IT sur front descendant sur le GO
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI3_PD;
	SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI7_PB;
	SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI14_PC;
	EXTI->IMR |= EXTI_IMR_MR3 | EXTI_IMR_MR7 | EXTI_IMR_MR14;
	EXTI->RTSR |= EXTI_RTSR_TR7 | EXTI_RTSR_TR14;
	EXTI->FTSR |= EXTI_RTSR_TR3;

	NVIC_SetPriority(EXTI3_IRQn, PRIORITY_IRQ_EXTI3);
	//NVIC_SetPriority(EXTI9_5_IRQn, PRIORITY_IRQ_EXTI9_5);
	NVIC_SetPriority(EXTI15_10_IRQn, PRIORITY_IRQ_EXTI15_10);
	NVIC_EnableIRQ(EXTI3_IRQn);
	//NVIC_EnableIRQ(EXTI9_5_IRQn);
	NVIC_EnableIRQ(EXTI15_10_IRQn);

	return 0;
}

module_init(uimdw_module_init, INIT_UIMDW);

extern "C"
{

//Start IT
void isr_exti3(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken = 0;
	portSET_INTERRUPT_MASK_FROM_ISR();

	if( EXTI->PR & EXTI_PR_PR3)
	{
		EXTI->PR = EXTI_PR_PR3;

		uih_startWithdraw();
	}
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
	portCLEAR_INTERRUPT_MASK_FROM_ISR(0);
}

//Color button IT
void isr_exti15_10(void)
{
	portSET_INTERRUPT_MASK_FROM_ISR();

	if( EXTI->PR & EXTI_PR_PR14)
	{
		EXTI->PR = EXTI_PR_PR14;

		struct systime t = systick_get_time_from_isr();
		struct systime dt = timediff(t, colorSwitch_change_time);
		if( dt.ms > 300)
		{
			colorSwitch_change_time = t;
			uih_colorButtonPushed();
		}
	}

	portCLEAR_INTERRUPT_MASK_FROM_ISR(0);
}

//void isr_exti9_5(void)
//{
//	portSET_INTERRUPT_MASK_FROM_ISR();
//
//	if( EXTI->PR & EXTI_PR_PR7)
//	{
//		EXTI->PR = EXTI_PR_PR7;
//		ui_user2_event();
//	}
//
//	portCLEAR_INTERRUPT_MASK_FROM_ISR(0);
//}


}


