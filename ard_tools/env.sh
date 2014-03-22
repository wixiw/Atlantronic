#config linux
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH;/usr/local/lib"

#configuration de ROS
HOSTNAME=`cat /etc/hostname`
if [ $HOSTNAME == "alpha" ]; then
	IS_HOST="false"
else 
	if [ $HOSTNAME == "beta" ]; then
		IS_HOST="false"
	else
		IS_HOST="true"
	fi
fi

if [ $IS_HOST == "false" ]; then
	export EDITOR=nano
else
	export EDITOR=gedit
fi
export ROS_HOME=/opt/ros
export ROS_PARALLEL_JOBS=-j2
export ROS_OS_OVERRIDE=debian:squeeze

export CAN_FLAVOR="gnulinux"
export ORO_LOGFILE="/tmp/orocos.log"

#configuration des couleurs de log
. /opt/ard/ard_tools/vm/color.sh

#chargement des scripts d'environnement des sous modules.
. /opt/ros/setup.bash
. /opt/ros_addons/env.sh

ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/opt/ard



#ajout du path pour bricoler en LUA
RTTLUA_MODULES=`rospack find ocl`/lua/modules/?.lua
if [ "x$LUA_PATH" == "x" ]; then
    LUA_PATH=";;"
fi
export LUA_PATH="$LUA_PATH;$RTTLUA_MODULES"

#ajout d'opencv aux paths
export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:/opt/opencv/lib/pkgconfig

#alias ARD
alias alpha='ssh root@alpha'
alias beta='ssh root@beta'
alias ubiquity='ssh root@ubiquity'
alias vm='ssh root@vm'
alias wixibox='ssh ard@wixibox'
alias ard-install-ubiquity='/opt/ard/ard_tools/default/script/install_target.sh ard ubiquity'
alias rosbeta='ROS_MASTER_URI=http://beta:11311'
alias rosalpha='ROS_MASTER_URI=http://alpha:11311'
alias ard-update='bash /opt/kernel/check_update.sh'
alias ard-clean-build='find /opt/ard -name build | xargs rm -rf; find /opt/ros_addons  -name build | xargs rm -rf;strip /opt/ros/*;strip /opt/ros_addons/*'
alias myip="sudo ifconfig | grep 'inet addr:'| grep -v '127.0.0.1' | cut -d: -f2 | awk '{ print $1}'"
alias ard-stress='stress --cpu 2 --io 1 --vm 1 --vm-bytes 128M'
alias ard-mount-robot='sudo mkdir /media/ubiquity -p;sudo sshfs root@ubiquity:/ /media/ubiquity'
alias ard-kst='python /opt/ard/arp_core/src/tools/ksthelper.py'

#alias Linux utiles
export LS_OPTIONS='--color=auto -h'
eval "`dircolors`"
alias ls='ls $LS_OPTIONS'
alias ll='ls $LS_OPTIONS -l'
alias l='ls $LS_OPTIONS -lA'
alias du='du --max-depth=1 | sort -nr | cut -f 2-|xargs -i du -sh {};du -sch .'
alias df='df -h'

#configuration du prompt
force_color_prompt=yes
PS1='${debian_chroot:+($debian_chroot)}\[\033[01;35m\]\u@(`cat /etc/hostname`)\[\033[00m\](`cat /opt/kernel/ard-vm-version`):\[\033[01;34m\]\w\[\033[00m\]\$ '

#configuration de ccache
export CCACHE_DIR=/home/ard/ccache/compilation.cache
ccache -M 2G 2&>/dev/null
export CCACHE_HARDLINK=1
export PATH=/home/ard/ccache:$PATH
export DISTCC_HOSTS="--randomize 192.168.1.59 192.168.1.72"
#export CCACHE_PREFIX=distcc

#configuration des priorites de thread RT
ulimit -r 90

#message de fin
cecho "yellow" "Chargement des variables d'environement ARD fait. Ubiquity is on the road !"
