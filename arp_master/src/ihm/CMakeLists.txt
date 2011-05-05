### USER : add your Cmake stuff here

rosbuild_add_executable(GraphicsSimuNode GraphicsSimuNode.cpp GraphicsSimuFrame.cpp GraphicsSimuRobot.cpp)
rosbuild_link_boost(GraphicsSimuNode thread)
target_link_libraries(GraphicsSimuNode ${wxWidgets_LIBRARIES})

