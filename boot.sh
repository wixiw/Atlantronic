#!/bin/bash
rosclean purge <<EOF
y
EOF

source /opt/ard/ard_tools/env.sh
echo "Launch Tanguy strat."
roslaunch arp_master tanguy.launch
echo "Done."
