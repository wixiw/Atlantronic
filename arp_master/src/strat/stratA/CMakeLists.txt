### USER : add your Cmake stuff here

rosbuild_add_executable(TinyStratNode TinyStratNode.cpp TinyStrat.cpp)
rosbuild_add_executable(StratANode StratANode.cpp CircularTraj.cpp WayPoint.cpp StratA.cpp)