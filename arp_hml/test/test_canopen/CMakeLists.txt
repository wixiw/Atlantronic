#Desactivated in waiting a new Advantech version
#rosbuild_add_executable(test_watchdog test_watchdog.c)
#add_subdirectory(orocos)

rosbuild_add_executable(test_canopen main.cpp ../../src/orocos/can/dictionnary/CanARD.c)
target_link_libraries(test_canopen canfestival canfestival_unix)

rosbuild_add_executable(test_rttask rttask.cpp)
target_link_libraries(test_rttask rt)