#!/bin/bash
#ce script permet de mettre à jour les variables d'environnement qui vont bien, il doit être chargé dans un .bashrc ou /etc/bash.bashrc
ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/opt/ard

export CAN_FLAVOR="xenomai"
