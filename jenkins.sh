# this file is executed by jenkins when the ARD job is fired. 
# This prevent from saving datas in the configuration interface of Jenkins which is neither versionned or backuped

. /opt/env.sh

rosmake arp_master