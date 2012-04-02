require("rttlib")
rttlib.color=true
Deployer = rtt.getTC()
print("====================")
print("début déploiment arp_ods")

-- chargement des librairies contenants les composants d'arp_ods
Deployer:import("arp_ods");

dofile("/opt/ard/arp_ods/script/orocos/deployment/ods_monitor_deployer.lua");

Deployer:loadComponent("KinematicBase","arp_ods::KinematicBase");
Deployer:setActivity("KinematicBase",0,26,1);
KinematicBase = Deployer:getPeer("KinematicBase");

Deployer:loadComponent("MotionControl","arp_ods::LittleSexControl");
Deployer:setActivity("MotionControl",0,25,1);
MotionControl = Deployer:getPeer("MotionControl");

OdsMonitorDeployer:load();
OdsMonitorDeployer:connect();
OdsMonitorDeployer:start();

print("fin déploiment arp_ods")
print("====================")
