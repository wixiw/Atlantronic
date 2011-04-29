### USER : add your Cmake stuff here


rosbuild_add_executable(SimuNode SimuNode.cpp SimuFrame.cpp SimuRobot.cpp)
rosbuild_link_boost(SimuNode thread)
target_link_libraries(SimuNode ${wxWidgets_LIBRARIES})

rosbuild_add_executable(GraphicsSimuNode GraphicsSimuNode.cpp GraphicsSimuFrame.cpp GraphicsSimuRobot.cpp)
rosbuild_link_boost(GraphicsSimuNode thread)
target_link_libraries(GraphicsSimuNode ${wxWidgets_LIBRARIES})

rosbuild_add_executable(PhysicsSimuNode PhysicsSimuNode.cpp PhysicsSimu.cpp PhysicsSimuRobot.cpp)

rosbuild_add_executable(TeleopNode TeleopNode.cpp)
rosbuild_add_executable(CommandNode CommandNode.cpp Command.cpp)
rosbuild_add_executable(LocalizatorNode LocalizatorNode.cpp Localizator.cpp)
rosbuild_add_executable(MotionControlNode MotionControlNode.cpp MotionControl.cpp)
rosbuild_add_executable(TinyStratNode TinyStratNode.cpp TinyStrat.cpp)
rosbuild_add_executable(StratANode StratANode.cpp CircularTraj.cpp WayPoint.cpp StratA.cpp)

add_subdirectory(orocos)