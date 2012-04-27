#Desactivated in waiting a new Advantech version
#rosbuild_add_executable(test_watchdog test_watchdog.c)
#add_subdirectory(orocos)

rosbuild_add_executable(test_rtcan_wla rt_dev_send.c)
target_link_libraries(test_rtcan_wla xenomai native pthread rt rtdm)