require("rttlib")
rttlib.color=true
Deployer = rtt.getTC()
print("====================")
print("début déploiment arp_ods manual")

--chargement des librairies contenants les composants d'arp_ods
Deployer:import("arp_ods");

dofile("/opt/ard/arp_ods/script/orocos/deployment/components/kinematics_base_deployer.lua");
dofile("/opt/ard/arp_ods/script/orocos/deployment/components/twist_teleop_deployer.lua");
dofile("/opt/ard/arp_ods/script/orocos/deployment/components/ods_monitor_deployer.lua");
dofile("/opt/ard/arp_ods/script/orocos/deployment/components/ros_ods_itf_deployer.lua");

TwistTeleopDeployer:load();
KinematicBaseDeployer:load();
RosOdsItfDeployer:load();
OdsMonitorDeployer:load();

TwistTeleopDeployer:connect();
KinematicBaseDeployer:connect();
--RosOdsItfDeployer:connect();
OdsMonitorDeployer:connect();

OdsMonitorDeployer:addToMonitor("KinematicBase")
OdsMonitorDeployer:addToMonitor("MotionControl")
--OdsMonitorDeployer:addToMonitor("RosOdsItf")
OdsMonitorDeployer:start();


print("fin déploiment arp_ods")
print("====================")
