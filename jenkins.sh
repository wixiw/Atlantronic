# this file is executed by jenkins when the ARD job is fired. 
# This prevent from saving datas in the configuration interface of Jenkins which is neither versionned or backuped

#This scrip is executed in the parent dir of ard

. /opt/env.sh

cd ard/arp_core
make -j2