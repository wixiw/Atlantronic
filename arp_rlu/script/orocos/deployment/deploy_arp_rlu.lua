require("rttlib")
rttlib.color=true
Deployer = rtt.getTC()
print("====================")
print("début déploiment arp_rlu")

-- chargement des librairies contenants les composants d'arp_ods
Deployer:import("arp_rlu");

dofile("/opt/ard/arp_rlu/script/orocos/deployment/rlu_monitor_deployer.lua");
dofile("/opt/ard/arp_rlu/script/orocos/deployment/odometry_deployer.lua");
dofile("/opt/ard/arp_rlu/script/orocos/deployment/localizator_deployer.lua");

OdometryDeployer:load();
LocalizatorDeployer:load();
RluMonitorDeployer:load();

OdometryDeployer:connect();
LocalizatorDeployer:connect();
RluMonitorDeployer:connect();

RluMonitorDeployer:start();

print("fin déploiment arp_rlu")
print("====================")
