### USER : add your Cmake stuff here

rosbuild_add_executable(PhysicsSimuNode PhysicsSimuNode.cpp PhysicsSimu.cpp PhysicsSimuRobot.cpp)

rosbuild_add_executable(HmlGraphicsNode HmlGraphicsNode.cpp HmlGraphicsFrame.cpp)
rosbuild_link_boost(HmlGraphicsNode thread)
target_link_libraries(HmlGraphicsNode ${wxWidgets_LIBRARIES})