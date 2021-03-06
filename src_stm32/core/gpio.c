#include "gpio.h"
#include "os/module.h"
#include "os/rcc.h"
#include "os/os.h"
#include "components/log/log.h"


//Callback to be implemented by user code
extern void uih_colorButtonPushed();

static int gpio_module_init(void)
{
	// io "sorties"
	// puissance on/off sur PB2
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOEEN;
	gpio_pin_init(GPIOB, 2, GPIO_MODE_OUT, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_UP); // on/off

	// "io entrees"
	// boutons USR1 et USR2 carte led sur PC14 et PB7
	// bouton go sur PD3
	gpio_pin_init(GPIOB, 7, GPIO_MODE_IN, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_DOWN); // bouton USR2
	gpio_pin_init(GPIOC, 14, GPIO_MODE_IN, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_DOWN); // bouton USR1
	gpio_pin_init(GPIOD, 3, GPIO_MODE_IN, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_DOWN); // bouton go

	// io sur les 5 connecteurs io generiques
	// pull up pour les omron
	// PD11 : IN_1
	// PB13 : IN_2
	// PB12 : IN_3
	// PD10 : IN_4
	// PD7  : IN_5
	// PB11 : IN_6
	// PC11 : IN_7
	// PD6  : IN_8
	// PC9  : IN_9
	// PC8  : IN_10
	// PB14 : IN_11
	// PB15 : IN_12
	// PE6  : IN_13
	// PE5  : IN_14
	gpio_pin_init(GPIOD, 11, GPIO_MODE_IN, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_UP); // IN_1
	gpio_pin_init(GPIOB, 13, GPIO_MODE_IN, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_UP); // IN_2
	gpio_pin_init(GPIOB, 12, GPIO_MODE_IN, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_UP); // IN_3
	gpio_pin_init(GPIOD, 10, GPIO_MODE_IN, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_UP); // IN_4
	gpio_pin_init(GPIOD, 7, GPIO_MODE_IN, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_UP);  // IN_5
	gpio_pin_init(GPIOB, 11, GPIO_MODE_IN, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_UP); // IN_6
	gpio_pin_init(GPIOC, 11, GPIO_MODE_IN, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_UP); // IN_7
	gpio_pin_init(GPIOD, 6, GPIO_MODE_IN, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_UP);  // IN_8
	gpio_pin_init(GPIOC, 9, GPIO_MODE_IN, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_UP);  // IN_9
	gpio_pin_init(GPIOC, 8, GPIO_MODE_IN, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_UP);  // IN_10
	gpio_pin_init(GPIOB, 14, GPIO_MODE_IN, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_UP);  // IN_11
	gpio_pin_init(GPIOB, 15, GPIO_MODE_IN, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_UP);  // IN_12
	gpio_pin_init(GPIOE, 6, GPIO_MODE_IN, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_UP);  // IN_13
	gpio_pin_init(GPIOE, 5, GPIO_MODE_IN, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_UP);  // IN_14

	//TODO c'est moche de piloter ca ici, ca devrait aller dans le module power
	gpio_power_off();

	return 0;
}

module_init(gpio_module_init, INIT_GPIO);

void gpio_pin_init(GPIO_TypeDef* GPIOx, uint32_t pin, enum gpio_mode mode, enum gpio_speed speed, enum gpio_otype otype, enum gpio_pupd pupd)
{
	GPIOx->MODER &= ~(GPIO_MODER_MODER0 << (2 * pin));
	GPIOx->MODER |= ((uint32_t)mode) << (2 * pin);

	if( mode == GPIO_MODE_OUT || mode == GPIO_MODE_AF )
	{
		GPIOx->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR0 << (2 * pin));
		GPIOx->OSPEEDR |= ((uint32_t)speed) << (2 * pin);

		GPIOx->OTYPER &= ~((GPIO_OTYPER_OT_0) << pin);
		GPIOx->OTYPER |= ((uint16_t)otype) << pin;
	}

	GPIOx->PUPDR &= ~(GPIO_PUPDR_PUPDR0 << (pin * 2));
	GPIOx->PUPDR |= ((uint32_t)pupd) << (pin * 2);
}

void gpio_af_config(GPIO_TypeDef* GPIOx, uint32_t pin, uint32_t gpio_af)
{
	uint32_t temp = gpio_af << ((pin & 0x07) * 4);
	GPIOx->AFR[pin >> 0x03] &= ~(0xF << ((pin & 0x07) * 4));
	uint32_t temp_2 = GPIOx->AFR[pin >> 0x03] | temp;
	GPIOx->AFR[pin >> 0x03] = temp_2;
}

uint32_t gpio_get_state()
{
	uint32_t res = 0;

	res |= gpio_get_pin(GPIOD, 11);      // IN_1
	res |= gpio_get_pin(GPIOB, 13) << 1; // IN_2
	res |= gpio_get_pin(GPIOB, 12) << 2; // IN_3
	res |= gpio_get_pin(GPIOD, 10) << 3; // IN_4
	res |= gpio_get_pin(GPIOD, 7) << 4;  // IN_5
	res |= gpio_get_pin(GPIOB, 11) << 5; // IN_6
	res |= gpio_get_pin(GPIOC, 11) << 6; // IN_7
	res |= gpio_get_pin(GPIOD, 6) << 7;  // IN_8
	res |= gpio_get_pin(GPIOC, 9) << 8;  // IN_9
	res |= gpio_get_pin(GPIOC, 8) << 9;  // IN_10
	res |= gpio_get_pin(GPIOB, 14) << 10;  // IN_11
	res |= gpio_get_pin(GPIOB, 15) << 11;  // IN_12
	res |= gpio_get_pin(GPIOE, 6) << 12;  // IN_13
	res |= gpio_get_pin(GPIOE, 5) << 13;  // IN_14

	return res;
}




