# this file is executed by jenkins when the ARD job is fired. 
# This prevent from saving datas in the configuration interface of Jenkins which is neither versionned or backuped

#This scrip is executed in the parent dir of ard

. ard/ard_tools/env.sh

export PATH=/var/lib/jenkins/ccache:$PATH
export CCACHE_DIR=/var/lib/jenkins/ccache/jenkins.cache
ccache -M 3G

rosmake arp_master

ard/test_all.sh