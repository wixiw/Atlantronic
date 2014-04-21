#include "pince.h"
#include "ax12.h"
#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/semphr.h"
#include "kernel/module.h"
#include "kernel/driver/usb.h"
#include "kernel/log.h"

#define AX12_RIGHT_EMPTY_THRESHOLD   0x200
#define PINCE_STACK_SIZE               300

static void pince_cmd(void* arg);
static void pince_task(void* arg);

// mutex de protection
static xSemaphoreHandle pince_mutex;
static int32_t pince_order_right;
static int32_t pince_order_left;

static int pince_module_init()
{
	xTaskHandle xHandle;

	portBASE_TYPE err = xTaskCreate(pince_task, "pince", PINCE_STACK_SIZE, NULL, PRIORITY_TASK_PINCE, &xHandle);

	if(err != pdPASS)
	{
		return ERR_INIT_PINCE;
	}

	pince_mutex = xSemaphoreCreateMutex();
	
	if( pince_mutex == NULL )
	{
		return ERR_INIT_PINCE;
	}

	usb_add_cmd(USB_CMD_PINCE, &pince_cmd);

	ax12_set_goal_limit(AX12_PINCE_RIGHT, 0xed, 0x310);
	ax12_set_goal_limit(AX12_PINCE_LEFT, 0xed, 0x324);

	return 0;
}

module_init(pince_module_init, INIT_PINCE);

static void pince_task(void* arg)
{
	(void) arg;
	
	int obstruct_left;
	int obstruct_right;
	int32_t alpha_left = 1500000;
	int32_t alpha_right = -1500000;
	int32_t alpha_close_left = 15000000;
	int32_t actual_pos_left;
	int32_t actual_pos_right;
	
	struct ax12_error err_left;
	struct ax12_error err_right;

	// configuration des pinces
	ax12_set_moving_speed(AX12_PINCE_RIGHT, 0x3ff);
	ax12_set_moving_speed(AX12_PINCE_LEFT, 0x3ff);

	ax12_set_torque_limit(AX12_PINCE_RIGHT, 0x300);
	ax12_set_torque_limit(AX12_PINCE_LEFT, 0x300);

	ax12_set_torque_enable(AX12_PINCE_RIGHT, 1);
	ax12_set_torque_enable(AX12_PINCE_LEFT, 1);

//	ax12_write8(AX12_PINCE_RIGHT, AX12_ALARM_SHUTDOWN, 0x04);
//	ax12_write8(AX12_PINCE_LEFT, AX12_ALARM_SHUTDOWN, 0x04);
	
	while(1)
	{
		int32_t order_left;
		int32_t order_right;
		xSemaphoreTake(pince_mutex, portMAX_DELAY);
		order_left = pince_order_left;
		order_right = pince_order_right;
		xSemaphoreGive(pince_mutex);

		// recupération des positions courantes
		actual_pos_left = ax12_get_position(AX12_PINCE_LEFT, &err_left);
		actual_pos_right = ax12_get_position(AX12_PINCE_RIGHT, &err_right);

		// si la pince gauche est dans l'intervalle obstruant 
		if((actual_pos_right < alpha_right) && (order_right != -4821428))
		{
			obstruct_right = 1;
		}
		else
		{
			obstruct_right = 0;
		}

		// si la pince droite est dans l'intervalle obstruant 
		if((actual_pos_left > alpha_left) && (actual_pos_left < alpha_close_left) && (order_left != 4800000))
		{
			obstruct_left = 1;
		}
		else
		{
			obstruct_left = 0;
		}

		// si les deux pinces sont dans l'intervalle obstruant 
		if((obstruct_right)&&(obstruct_left))
		{
			ax12_set_goal_position(AX12_PINCE_RIGHT, alpha_right);
		}

		// si la pince gauche peut bouger
		if(!obstruct_left)
		{
			ax12_set_goal_position(AX12_PINCE_RIGHT, pince_order_right);
		}
		
		// si la pince droite peut bouger
		if((!obstruct_right)||(!obstruct_left))
		{
			ax12_set_goal_position(AX12_PINCE_LEFT, pince_order_left);
		}
		else
		{
			ax12_set_goal_position(AX12_PINCE_LEFT, actual_pos_left);
		}

		vTaskDelay(ms_to_tick(50));
	}
}

void pince_set_position(enum pince_cmd_type left, enum pince_cmd_type right)
{
	log(LOG_INFO, "pince_open");
	xSemaphoreTake(pince_mutex, portMAX_DELAY);

	switch(left)
	{
		case PINCE_OPEN:
			pince_order_left = -15000000;
			break;
		case PINCE_STRAT:
			pince_order_left = 4800000;
			break;
		case PINCE_MIDDLE:
			pince_order_left =  -800000;
			break;		
		case PINCE_CLOSE:
			pince_order_left = 16000000;
			break;
		default:
			break;
	}
	
	switch(right)
	{	
		case PINCE_OPEN:
			pince_order_right = 15000000;
			break;
		case PINCE_STRAT:
			pince_order_right = -4821428;
			break;
		case PINCE_MIDDLE:
			pince_order_right =  800000;
			break;		
		case PINCE_CLOSE:
			pince_order_right = -15000000;
			break;
		default:
			break;
	}
			
	xSemaphoreGive(pince_mutex);
}

static void pince_cmd(void* arg)
{
	struct pince_cmd_arg* cmd_arg = (struct pince_cmd_arg*) arg;

	pince_set_position(cmd_arg->type_left, cmd_arg->type_right);
}