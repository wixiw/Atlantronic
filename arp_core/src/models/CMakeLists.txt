###################################################################################################################
#    EDIT BELOW THIS LINE AND UNTIL NEXT LINE FOR A NEW PROJECT

rosbuild_add_library( arp_core-models UbiquityKinematics.cpp UbiquityParams.cpp )
target_link_libraries( arp_core-models arp_core-math )
###################################################################################################################