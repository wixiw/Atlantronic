all:
	bash configure.sh
	make -C src ${ROS_PARALLEL_JOBS}
	sudo make -C src install ${ROS_PARALLEL_JOBS}
	find objdictgen -name ".svn" | xargs rm -Rf

debug:
	bash configure.sh debug
	make -C src ${ROS_PARALLEL_JOBS}
	make -C src ${ROS_PARALLEL_JOBS}


clean:
	make -C src clean
