FILE(REMOVE_RECURSE
  "../src/orocos/types/ros_Velocity_typekit_plugin.cpp"
  "../src/orocos/types/ros_Obstacle_typekit_plugin.cpp"
  "../src/orocos/types/ros_DifferentialCommand_typekit_plugin.cpp"
  "../src/orocos/types/ros_Pose_typekit_plugin.cpp"
  "../src/orocos/types/ros_StartColor_typekit_plugin.cpp"
  "../src/orocos/types/ros_Start_typekit_plugin.cpp"
  "../src/orocos/types/ros_Odo_typekit_plugin.cpp"
  "../src/orocos/types/ros_Velocity_transport_plugin.cpp"
  "../src/orocos/types/ros_Obstacle_transport_plugin.cpp"
  "../src/orocos/types/ros_DifferentialCommand_transport_plugin.cpp"
  "../src/orocos/types/ros_Pose_transport_plugin.cpp"
  "../src/orocos/types/ros_StartColor_transport_plugin.cpp"
  "../src/orocos/types/ros_Start_transport_plugin.cpp"
  "../src/orocos/types/ros_Odo_transport_plugin.cpp"
  "../src/orocos/types/ros_arp_core_typekit.cpp"
  "../src/orocos/types/ros_arp_core_transport.cpp"
  "CMakeFiles/rospack_genmsg"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/rospack_genmsg.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
