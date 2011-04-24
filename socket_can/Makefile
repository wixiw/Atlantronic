all:
	make -C trunk/can-utils/ ${ROS_PARALLEL_JOBS}
	make -C trunk/can-utils/ install ${ROS_PARALLEL_JOBS}
	make -C trunk/can-utils/ clean
clean:
	rm bin/*
	make -C trunk/can-utils/ clean


