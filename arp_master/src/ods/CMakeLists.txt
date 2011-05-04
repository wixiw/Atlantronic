### USER : add your Cmake stuff here

rosbuild_add_executable(CommandNode CommandNode.cpp Command.cpp)
rosbuild_add_executable(MotionControlNode MotionControlNode.cpp MotionControl.cpp)
rosbuild_add_executable(TeleopNode TeleopNode.cpp)
