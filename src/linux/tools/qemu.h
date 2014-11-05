#ifndef QEMU_H
#define QEMU_H

#include "linux/tools/com.h"
#include "kernel/math/polyline.h"
#include "kernel/math/vect_plan.h"

class Qemu
{
	public:
		Com com; //!< communication avec qemu
		pid_t pid; //!< pid de qemu

		int init(const char* file_qemu_read, const char* file_qemu_write);
		void destroy();
		int set_clock_factor(unsigned int factor, unsigned int icount);
		int add_object(const struct polyline polyline);
		int move_object(int id, struct vect2 origin, VectPlan delta);
		int manage_canopen_connexion(int nodeId, bool connected);
		int set_io(uint32_t id, bool val);
};

#endif
