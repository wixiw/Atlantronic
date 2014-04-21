#ifndef QEMU_H
#define QEMU_H

#include "linux/tools/com.h"
#include "kernel/math/polyline.h"
#include "kernel/math/vect_plan.h"

class Qemu
{
	public:
		char file_qemu_read[64];
		char file_qemu_write[64];
		char file_board_read[64];
		char file_board_write[64];
		Com com; //!< communication avec qemu
		pid_t pid; //!< pid de qemu

		int init(const char* qemu_path, const char* prog_name, int gdb_port);
		void destroy();
		int set_clock_factor(unsigned int factor, unsigned int icount);
		int add_object(const struct polyline polyline);
		int move_object(int id, struct vect2 origin, VectPlan delta);
		int manage_canopen_connexion(int nodeId, bool connected);
};

#endif
