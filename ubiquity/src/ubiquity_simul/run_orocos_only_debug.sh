#!/bin/bash
set -e

gdb /opt/ros_addons/orocos_toolchain/install/bin/deployer-gnulinux --eval-command="run -s /opt/ard/ubiquity/src/ubiquity_simul/orocos/ubiquity_simul.ops"
