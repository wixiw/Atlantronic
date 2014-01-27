###################################################################################################################
#    EDIT BELOW THIS LINE AND UNTIL NEXT LINE FOR A NEW PROJECT


# add the orocos dependency
#
add_orocos()

find_package(Git)
 if(NOT GIT_FOUND)
     message("git NOT found!!!")
endif()

####################################################################################################################
