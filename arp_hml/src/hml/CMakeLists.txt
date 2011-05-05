###################################################################################################################
#    EDIT BELOW THIS LINE AND UNTIL NEXT LINE FOR A NEW PROJECT

orocos_component(arp_hml ${source_files})
target_link_libraries(arp_hml SUSI-3.02)
target_link_libraries(arp_hml canfestival_unix)
target_link_libraries(arp_hml canfestival)

###################################################################################################################

