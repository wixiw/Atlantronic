FILE(REMOVE_RECURSE
  "../src/orocos/types/ros_GoalID_typekit_plugin.cpp"
  "../src/orocos/types/ros_GoalStatusArray_typekit_plugin.cpp"
  "../src/orocos/types/ros_GoalStatus_typekit_plugin.cpp"
  "../src/orocos/types/ros_GoalID_transport_plugin.cpp"
  "../src/orocos/types/ros_GoalStatusArray_transport_plugin.cpp"
  "../src/orocos/types/ros_GoalStatus_transport_plugin.cpp"
  "../src/orocos/types/ros_actionlib_msgs_typekit.cpp"
  "../src/orocos/types/ros_actionlib_msgs_transport.cpp"
  "CMakeFiles/clean-test-results"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/clean-test-results.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
