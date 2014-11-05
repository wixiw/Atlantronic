#include "qemu.h"
#include <stdlib.h>
#include <signal.h>
#include <sys/stat.h>
#include "linux/tools/cli.h"
#include <math.h>

#define  MIN(a, b)      (((a) < (b)) ? (a) : (b))

enum
{
	EVENT_CLOCK_FACTOR = 1,
	EVENT_NEW_OBJECT,
	EVENT_MOVE_OBJECT,
	EVENT_MANAGE_CAN_MOTOR,
	EVENT_SET_IO,
};

enum
{
	EVENT_MANAGE_CAN_MOTOR_CONNECT,
	EVENT_MANAGE_CAN_MOTOR_DISCONNECT,
};

struct atlantronic_model_tx_event
{
	uint32_t type;        //!< type
	union
	{
		uint8_t data[256];    //!< données
		uint32_t data32[64];  //!< données
	};
};

int Qemu::init(const char* file_qemu_read, const char* file_qemu_write)
{
	com.init(file_qemu_read, file_qemu_write, NULL);
	com.open_block();

	return 0;
}

void Qemu::destroy()
{
	if(pid > 0)
	{
		kill(pid, SIGILL);
	}

    com.close();
    com.destroy();
}

int Qemu::set_clock_factor(unsigned int factor, unsigned int icount)
{
	struct atlantronic_model_tx_event event;

	event.type = EVENT_CLOCK_FACTOR;
	event.data32[0] = factor;
	event.data32[1] = icount;

	return com.write((void*) &event, sizeof(event));
}

int Qemu::add_object(const struct polyline polyline)
{
	struct atlantronic_model_tx_event event;
	unsigned int i = 0;

	event.type = EVENT_NEW_OBJECT;

	event.data[0] = MIN((unsigned int)polyline.size, (sizeof(event.data) - 1)/8);
	float* f = (float*)(event.data+1);
	for(i = 0; i < event.data[0]; i++ )
	{
		f[0] = polyline.pt[i].x;
		f[1] = polyline.pt[i].y;
		f += 2;
	}

	return com.write((void*) &event, sizeof(event));
}

int Qemu::move_object(int id, struct vect2 origin, VectPlan delta)
{
	struct atlantronic_model_tx_event event;

	event.type = EVENT_MOVE_OBJECT;

	event.data[0] = id;
	float* f = (float*)(event.data+1);
	f[0] = origin.x;
	f[1] = origin.y;
	f[2] = delta.x;
	f[3] = delta.y;
	f[4] = delta.theta;

	return com.write((void*) &event, sizeof(event));
}

//! @param nodeId : nodeId ou 0 pour tous
int Qemu::manage_canopen_connexion(int nodeId, bool connected)
{
	struct atlantronic_model_tx_event event;

	event.type = EVENT_MANAGE_CAN_MOTOR;
	event.data32[0] = nodeId;
	if(connected)
	{
		event.data32[1] = EVENT_MANAGE_CAN_MOTOR_CONNECT;
	}
	else
	{
		event.data32[1] = EVENT_MANAGE_CAN_MOTOR_DISCONNECT;
	}

	return com.write((void*) &event, sizeof(event));
}

int Qemu::set_io(uint32_t id, bool val)
{
	struct atlantronic_model_tx_event event;
	event.type = EVENT_SET_IO;
	event.data32[0] = id;
	event.data32[1] = val?1:0;

	return com.write((void*) &event, sizeof(event));
}
