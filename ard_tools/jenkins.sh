# this file is executed by jenkins when the ARD job is fired. 
# This prevent from saving datas in the configuration interface of Jenkins which is neither versionned or backuped

#This scrip has to be executed in the parent dir of ard

. ard/ard_tools/env.sh

echo ""
echo "***********************************************"
echo "   INFO DEBUG "
echo "   ----------
echo ""
echo "CanFestival compiled for : " $CAN_FLAVOR
echo "***********************************************"
echo ""

#configuration of ccache
export PATH=/var/lib/jenkins/ccache:$PATH
export CCACHE_DIR=/var/lib/jenkins/ccache/jenkins.cache
ccache -M 3G

rosmake arp_master

ard/test_all.sh