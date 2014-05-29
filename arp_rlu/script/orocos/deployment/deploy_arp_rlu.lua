require("rttlib")
rttlib.color=true
Deployer = rtt.getTC()
print("====================")
print("début déploiment arp_rlu")

-- chargement des librairies contenants les composants d'arp_ods
Deployer:import("arp_rlu");
Deployer:import("rtt_sensor_msgs");

dofile("/opt/ard/arp_rlu/script/orocos/deployment/components/rlu_monitor_deployer.lua");
dofile("/opt/ard/arp_rlu/script/orocos/deployment/components/odometry_deployer.lua");
dofile("/opt/ard/arp_rlu/script/orocos/deployment/components/localizator_deployer.lua");
dofile("/opt/ard/arp_rlu/script/orocos/deployment/components/obstacle_detector_deployer.lua");
dofile("/opt/ard/arp_rlu/script/orocos/deployment/components/obstacle_manager_deployer.lua");
dofile("/opt/ard/arp_rlu/script/orocos/deployment/components/ros_rlu_itf_deployer.lua");

OdometryDeployer:load();
LocalizatorDeployer:load();
ObstacleDetector:load();
ObstacleManager:load();
RosRluItfDeployer:load();
RluMonitorDeployer:load();


OdometryDeployer:connect();
LocalizatorDeployer:connect();
ObstacleDetector:connect();
ObstacleManager:connect();
RosRluItfDeployer:connect();
RluMonitorDeployer:connect();
RluMonitorDeployer:start();

print("fin déploiment arp_rlu")
print("====================")
