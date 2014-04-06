#rosbuild_add_executable(testWla.ard ../src/orders/MotionOrder.cpp ../src/orders/ModeSelector.cpp testWla.cpp)
add_subdirectory(orders)
add_subdirectory(uTests)