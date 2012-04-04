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
HmlMonitor = Deployer:getPeer("HmlMonitor");
--on s'ajoute en peer a HmlMonitor pour pouvoir faire les connections
Deployer:addPeer("HmlMonitor", "KinematicBase")
HmlMonitor:connect("LeftDriving","inSpeedCmd","KinematicBase","outLeftDrivingSpeedCmd");
HmlMonitor:connect("RightDriving","inSpeedCmd","KinematicBase","outRightDrivingSpeedCmd");
HmlMonitor:connect("RearDriving","inSpeedCmd","KinematicBase","outRearDrivingSpeedCmd");
HmlMonitor:connect("LeftSteering","inPositionCmd","KinematicBase","outLeftSteeringPositionCmd");
HmlMonitor:connect("RightSteering","inPositionCmd","KinematicBase","outRightSteeringPositionCmd");
HmlMonitor:connect("RearSteering","inPositionCmd","KinematicBase","outRearSteeringPositionCmd");
HmlMonitor:connect("KinematicBase","inLeftSteeringSpeedMeasure","LeftSteering","outComputedSpeed");
HmlMonitor:connect("KinematicBase","inRightSteeringSpeedMeasure","RightSteering","outComputedSpeed");
HmlMonitor:connect("KinematicBase","inRearSteeringSpeedMeasure","RearSteering","outComputedSpeed");

Deployer:loadComponent("MotionControl","arp_ods::LittleSexControl");
Deployer:setActivity("MotionControl",0,25,1);
MotionControl = Deployer:getPeer("MotionControl");
Deployer:connect("MotionControl.outTwistCmd","KinematicBase.inTwistCmd",cp);

OdsMonitorDeployer:load();
OdsMonitorDeployer:connect();
OdsMonitorDeployer:start();

print("fin déploiment arp_ods")
print("====================")
