###################################################################################################################
#    EDIT BELOW THIS LINE AND UNTIL NEXT LINE FOR A NEW PROJECT


# add the orocos dependency
#
add_orocos()

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})


####################################################################################################################
