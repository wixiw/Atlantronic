dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")

KinematicBaseDeployer = ComposantDeployer:new()
local me = "KinematicBase"

function KinematicBaseDeployer:load()
	assert( Deployer:loadComponent(me,"arp_ods::KinematicBase"));
	assert( Deployer:addPeer("DotGraph",me))
	assert( Deployer:setMasterSlaveActivity("MotionScheduler", me))
	
	return true
end


function KinematicBaseDeployer:connect()
	Deployer:addPeer("Reporting", me)

	
	--on s'ajoute en peer a HmlMonitor pour pouvoir faire les connections
	HmlMonitor = Deployer:getPeer("HmlMonitor");
	assert(Deployer:addPeer("HmlMonitor", me))
	assert(HmlMonitor:connect(me,"inMotorState","Syncronizator","outMotorMeasures"));
	assert(Deployer:connect(me..".inHwBlocked","HmlMonitor.outAllDrivingAreBlocked",cp));
	assert(Deployer:connect(me..".inParams","UbiquityParams.outParams",cp));
	assert(Deployer:connect(me..".inICRSpeedCmd","MotionControl.outICRSpeedCmd",cp));
	assert(HmlMonitor:connect("LeftDriving","inSpeedCmd",me,"outLeftDrivingVelocityCmd"));
	assert(HmlMonitor:connect("RightDriving","inSpeedCmd",me,"outRightDrivingVelocityCmd"));
	assert(HmlMonitor:connect("RearDriving","inSpeedCmd",me,"outRearDrivingVelocityCmd"));
	assert(HmlMonitor:connect("LeftSteering","inPositionCmd",me,"outLeftSteeringPositionCmd"));
	assert(HmlMonitor:connect("RightSteering","inPositionCmd",me,"outRightSteeringPositionCmd"));
	assert(HmlMonitor:connect("RearSteering","inPositionCmd",me,"outRearSteeringPositionCmd"));
	
	--on s'ajoute en peer a HmlMonitor pour pouvoir faire les connections
	RluMonitor = Deployer:getPeer("RluMonitor");
	assert(Deployer:addPeer("RluMonitor", me))
	assert(RluMonitor:connect(me,"inCurrentICRSpeed","Localizator","outICRSpeed"));
	
	return true
end



