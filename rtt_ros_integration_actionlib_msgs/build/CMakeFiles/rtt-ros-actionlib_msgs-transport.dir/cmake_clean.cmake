FILE(REMOVE_RECURSE
  "../src/orocos/types/ros_GoalID_typekit_plugin.cpp"
  "../src/orocos/types/ros_GoalStatusArray_typekit_plugin.cpp"
  "../src/orocos/types/ros_GoalStatus_typekit_plugin.cpp"
  "../src/orocos/types/ros_GoalID_transport_plugin.cpp"
  "../src/orocos/types/ros_GoalStatusArray_transport_plugin.cpp"
  "../src/orocos/types/ros_GoalStatus_transport_plugin.cpp"
  "../src/orocos/types/ros_actionlib_msgs_typekit.cpp"
  "../src/orocos/types/ros_actionlib_msgs_transport.cpp"
  "CMakeFiles/rtt-ros-actionlib_msgs-transport.dir/src/orocos/types/ros_actionlib_msgs_transport.o"
  "../lib/orocos/gnulinux/types/librtt-ros-actionlib_msgs-transport-gnulinux.pdb"
  "../lib/orocos/gnulinux/types/librtt-ros-actionlib_msgs-transport-gnulinux.so"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang CXX)
  INCLUDE(CMakeFiles/rtt-ros-actionlib_msgs-transport.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
