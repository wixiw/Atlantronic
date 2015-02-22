#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>

#include "linux/tools/com.h"
#include "linux/tools/cmd.h"
#include "linux/tools/robot_interface.h"
#include "table.h"

static RobotInterface robotItf;
static Qemu qemu;
static pthread_mutex_t quitMutex;
static pthread_cond_t quitCond;
void cli_quit();

void ard_fail_scenario(RobotInterface* itf)
{
	int i = 0;
	while(i < 5000 )
	{
		printf("list %d\n", i);
		//itf->ptask();
		itf->dynamixel_set_goal_position(DYNAMIXEL_TYPE_AX12, AX12_FINGER_RIGHT, i/10000.);
		i++;
		usleep(5*1E3);
	}
}

int main()
{
	pthread_mutex_init(&quitMutex, NULL);
	pthread_cond_init(&quitCond, NULL);
	pthread_mutex_lock(&quitMutex);

	const char* file_stm_read = NULL;
	const char* file_stm_write = NULL;

	int res = qemu.init("qemu/arm-softmmu/qemu-system-arm", "./bin/disco/core", 1235);
	if( res )
	{
		fprintf(stderr, "qemu_init : error");
		return -1;
	}

	file_stm_read = qemu.file_board_read;
	file_stm_write = qemu.file_board_write;

	robotItf.init("discovery", file_stm_read, file_stm_write, NULL, NULL, NULL);
	cmd_init(&robotItf, &qemu, cli_quit);

	sleep(6);
	printf("Begin of scenario.\n");
	ard_fail_scenario(&robotItf);
	printf("End of scenario.\n");

	pthread_cond_wait(&quitCond, &quitMutex);
	pthread_mutex_unlock(&quitMutex);

	robotItf.destroy();
	qemu.destroy();
	return 0;
}

void cli_quit()
{
	pthread_cond_signal(&quitCond);
}
