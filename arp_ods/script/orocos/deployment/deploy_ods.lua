require("rttlib")
rttlib.color=true
Deployer = rtt.getTC()
print("====================")
print("début déploiment arp_ods")

-- chargement des librairies contenants les composants d'arp_ods
Deployer:import("arp_ods");

dofile("/opt/ard/arp_ods/script/orocos/deployment/ods_monitor_deployer.lua");
--[[
Deployer:loadComponent("MotionControl","arp_ods::TwistTeleop");
Deployer:setActivity("MotionControl",0.050,25,1);
MotionControl = Deployer:getPeer("MotionControl");
Deployer:addPeer("HmlMonitor", "MotionControl")

HmlMonitor:connect("MotionControl","inXSpeed","Joystick","outX1");
HmlMonitor:connect("MotionControl","inYSpeed","Joystick","outY1");
HmlMonitor:connect("MotionControl","inThetaSpeed","Joystick","outX2");
]]

Deployer:loadComponent("MotionControl","arp_ods::LittleSexControl");
Deployer:setActivity("MotionControl",0.0,25,1);
RluMonitor = Deployer:getPeer("RluMonitor");
Deployer:addPeer("RluMonitor", "MotionControl");
RluMonitor:connect("MotionControl","inThetaSpeed","Localizator","outPose");
--[[
Deployer:loadComponent("KinematicBase","arp_ods::KinematicBase");
Deployer:setActivity("KinematicBase",0.0,26,1);
KinematicBase = Deployer:getPeer("KinematicBase");
HmlMonitor = Deployer:getPeer("HmlMonitor");
--on s'ajoute en peer a HmlMonitor pour pouvoir faire les connections
Deployer:addPeer("HmlMonitor", "KinematicBase")
HmlMonitor:connect("KinematicBase","inMotorState","Syncronizator","outMotorMeasures");
Deployer:connect("KinematicBase.inParams","UbiquityParams.outParams",cp);
Deployer:connect("KinematicBase.inTwistCmd","MotionControl.outTwistCmd",cp);
HmlMonitor:connect("LeftDriving","inSpeedCmd","KinematicBase","outLeftDrivingVelocityCmd");
HmlMonitor:connect("RightDriving","inSpeedCmd","KinematicBase","outRightDrivingVelocityCmd");
HmlMonitor:connect("RearDriving","inSpeedCmd","KinematicBase","outRearDrivingVelocityCmd");
HmlMonitor:connect("LeftSteering","inPositionCmd","KinematicBase","outLeftSteeringPositionCmd");
HmlMonitor:connect("RightSteering","inPositionCmd","KinematicBase","outRightSteeringPositionCmd");
HmlMonitor:connect("RearSteering","inPositionCmd","KinematicBase","outRearSteeringPositionCmd");

OdsMonitorDeployer:load();
OdsMonitorDeployer:connect();
OdsMonitorDeployer:start();
]]

print("fin déploiment arp_ods")
print("====================")
