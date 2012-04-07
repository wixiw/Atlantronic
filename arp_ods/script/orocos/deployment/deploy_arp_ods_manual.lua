require("rttlib")
rttlib.color=true
Deployer = rtt.getTC()
print("====================")
print("début déploiment arp_ods")

--chargement des librairies contenants les composants d'arp_ods
Deployer:import("arp_ods");

dofile("/opt/ard/arp_ods/script/orocos/deployment/kinematics_base_deployer.lua");
dofile("/opt/ard/arp_ods/script/orocos/deployment/twist_teleop_deployer.lua");
dofile("/opt/ard/arp_ods/script/orocos/deployment/ods_monitor_deployer.lua");

TwistTeleopDeployer:load();
KinematicBaseDeployer:load();

TwistTeleopDeployer:connect();
KinematicBaseDeployer:connect();

OdsMonitorDeployer:load();
OdsMonitorDeployer:connect();
OdsMonitorDeployer:start();

--mise automatique de la puissance
HmlMonitor = Deployer:getPeer("HmlMonitor")
HmlMonitor:coSetMotorPower(true);


print("fin déploiment arp_ods")
print("====================")
