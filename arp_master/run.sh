cp ../arp_hml/script/orocos/ops/* script/orocos/ops -R
cp ../arp_hml/script/orocos/conf/* script/orocos/conf/

rosrun ocl deployer-gnulinux -s script/orocos/deployment/deploy_arp_master.ops
