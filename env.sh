#!/bin/bash
#ce script permet de mettre à jour les variables d'environnement qui vont bien, il doit être chargé dans un .bashrc ou /etc/bash.bashrc
#ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$ROS_ROOT/../ard/arp_core
ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:`rosstack find ard`/arp_core
ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:`rosstack find ard`/arp_hml
ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:`rosstack find ard`/advantech_susi
ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:`rosstack find ard`/arp_master
ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:`rosstack find ard`/rtt_ros_integration_arp_master_msgs
ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:`rosstack find ard`/rtt_ros_integration_actionlib_msgs
