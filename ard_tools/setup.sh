#!/bin/sh
# THIS IS A FILE AUTO-GENERATED BY rosinstall
# IT IS UNLIKELY YOU WANT TO EDIT THIS FILE BY HAND
# IF YOU WANT TO CHANGE THE ROS ENVIRONMENT VARIABLES
# USE THE rosinstall TOOL INSTEAD.
# see: http://www.ros.org/wiki/rosinstall 
export ROS_ROOT=/opt/ros/ros
export PATH=$ROS_ROOT/bin:$PATH
export PYTHONPATH=$ROS_ROOT/core/roslib/src:$PYTHONPATH
if [ ! "$ROS_MASTER_URI" ] ; then export ROS_MASTER_URI=http://localhost:11311 ; fi
export ROS_WORKSPACE=/opt/ros
function add_to_ros_path
{
        if [ -d $1 ] ; then
		if [[ $ROS_PACKAGE_PATH != "" ]] ; then 
	                export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$1
		else
			export ROS_PACKAGE_PATH=$1
		fi
        fi
} 
add_to_ros_path /opt/ros/ros/tools/rospack
add_to_ros_path /opt/ros/robot_model_tutorials
add_to_ros_path /opt/ros/visualization_tutorials
add_to_ros_path /opt/ros/geometry_tutorials
add_to_ros_path /opt/ros/common_tutorials
add_to_ros_path /opt/ros/ros_tutorials
add_to_ros_path /opt/ros/documentation
add_to_ros_path /opt/ros/geometry_experimental
add_to_ros_path /opt/ros/robot_model_visualization
add_to_ros_path /opt/ros/geometry_visualization
add_to_ros_path /opt/ros/diagnostics_monitors
add_to_ros_path /opt/ros/executive_smach_visualization
add_to_ros_path /opt/ros/visualization
add_to_ros_path /opt/ros/slam_gmapping
add_to_ros_path /opt/ros/navigation
add_to_ros_path /opt/ros/vision_opencv
add_to_ros_path /opt/ros/laser_pipeline
add_to_ros_path /opt/ros/image_pipeline
add_to_ros_path /opt/ros/image_transport_plugins
add_to_ros_path /opt/ros/perception_pcl
add_to_ros_path /opt/ros/image_common
add_to_ros_path /opt/ros/rx
add_to_ros_path /opt/ros/visualization_common
add_to_ros_path /opt/ros/physics_ode
add_to_ros_path /opt/ros/stage
add_to_ros_path /opt/ros/simulator_gazebo
add_to_ros_path /opt/ros/simulator_stage
add_to_ros_path /opt/ros/xacro
add_to_ros_path /opt/ros/executive_smach
add_to_ros_path /opt/ros/robot_model
add_to_ros_path /opt/ros/assimp
add_to_ros_path /opt/ros/pluginlib
add_to_ros_path /opt/ros/orocos_kinematics_dynamics
add_to_ros_path /opt/ros/nodelet_core
add_to_ros_path /opt/ros/geometry
add_to_ros_path /opt/ros/bullet
add_to_ros_path /opt/ros/filters
add_to_ros_path /opt/ros/eigen
add_to_ros_path /opt/ros/driver_common
add_to_ros_path /opt/ros/diagnostics
add_to_ros_path /opt/ros/common
add_to_ros_path /opt/ros/common_msgs
add_to_ros_path /opt/ros/bond_core
add_to_ros_path /opt/ros/common_rosdeps
add_to_ros_path /opt/ros/ros_comm 