dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")

KinematicBaseDeployer = ComposantDeployer:new()
local me = "KinematicBase"

function KinematicBaseDeployer:load()
	Deployer:loadComponent(me,"arp_ods::KinematicBase");
	Deployer:setMasterSlaveActivity("MotionScheduler", me)
end


function KinematicBaseDeployer:connect()
	Deployer:addPeer("Reporting", me)

	
	--on s'ajoute en peer a HmlMonitor pour pouvoir faire les connections
	HmlMonitor = Deployer:getPeer("HmlMonitor");
	Deployer:addPeer("HmlMonitor", me)
	HmlMonitor:connect(me,"inMotorState","Syncronizator","outMotorMeasures");
	Deployer:connect(me..".inParams","UbiquityParams.outParams",cp);
	Deployer:connect(me..".inTwistCmd","MotionControl.outTwistCmd",cp);
	HmlMonitor:connect("LeftDriving","inSpeedCmd",me,"outLeftDrivingVelocityCmd");
	HmlMonitor:connect("RightDriving","inSpeedCmd",me,"outRightDrivingVelocityCmd");
	HmlMonitor:connect("RearDriving","inSpeedCmd",me,"outRearDrivingVelocityCmd");
	HmlMonitor:connect("LeftSteering","inPositionCmd",me,"outLeftSteeringPositionCmd");
	HmlMonitor:connect("RightSteering","inPositionCmd",me,"outRightSteeringPositionCmd");
	HmlMonitor:connect("RearSteering","inPositionCmd",me,"outRearSteeringPositionCmd");
	
	--on s'ajoute en peer a HmlMonitor pour pouvoir faire les connections
	RluMonitor = Deployer:getPeer("RluMonitor");
	Deployer:addPeer("RluMonitor", me)
	RluMonitor:connect(me,"inCurrentTwist","Localizator","outTwist");
end



