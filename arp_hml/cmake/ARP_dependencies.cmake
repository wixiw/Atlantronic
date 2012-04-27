###################################################################################################################
#    EDIT BELOW THIS LINE AND UNTIL NEXT LINE FOR A NEW PROJECT


# add the orocos dependency
#
add_orocos()

find_package(wxWidgets REQUIRED)
include(${wxWidgets_USE_FILE})
include_directories( ${wxWidgets_INCLUDE_DIRS} )

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})


include_directories("/opt/ard/can_festival/include")
include_directories("/opt/ard/can_festival/include/canfestival")

#xenomai config
include_directories("/usr/include/xenomai")
add_definitions("-D_GNU_SOURCE -D_REENTRANT -Wall -pipe -D__XENO__")
link_directories("/opt/ard/can_festival/lib")


rosbuild_add_boost_directories()

####################################################################################################################
