dofile("/opt/ard/ubiquity/src/subsystems/orocos/component_deployer_object.lua")

KinematicBaseDeployer = ComposantDeployer:new()
local me = "KinematicBase"

function KinematicBaseDeployer:load()
	assert( Deployer:loadComponent(me,"arp_ods::KinematicBase"))
	assert( Deployer:addPeer("DotGraph",me))
	assert( Deployer:addPeer("Reporting", me))
	assert( Deployer:setMasterSlaveActivity("MotionScheduler", me))
	
	return true
end


function KinematicBaseDeployer:connect(synchronizatorName, hmlMonitorName)

	
	--on s'ajoute en peer a HmlMonitor pour pouvoir faire les connections
	assert(Deployer:connect(me..".inMotorState", 		synchronizatorName..".outMotorMeasures",cp))
	assert(Deployer:connect(me..".inHwBlocked",			hmlMonitorName..".outAllDrivingAreBlocked",cp))
	assert(Deployer:connect(me..".inParams",			"UbiquityParams.outParams",cp))
	assert(Deployer:connect(me..".inICRSpeedCmd",		"MotionControl.outICRSpeedCmd",cp))
	assert(Deployer:connect(me..".inCurrentICRSpeed",	"Localizator.outICRSpeed",cp))
	
	return true
end



