dofile("/opt/ard/ubiquity/src/subsystems/orocos/component_deployer_object.lua")


OdometryDeployer = ComposantDeployer:new()
local me = "Odometry"

function OdometryDeployer:load()
	assert( Deployer:loadComponent(me, "arp_rlu::Odometry4UbiquityICR") )
	return true
end

function OdometryDeployer:connect(synchronizatorName)
	assert( Deployer:addPeer("DotGraph",me))
	assert( Deployer:addPeer("Reporting", me) )
	assert( Deployer:setMasterSlaveActivity("MotionScheduler", me) )

	--on s'ajoute en peer a HmlMonitor pour pouvoir faire les connections
	assert( Deployer:connect(me..".inTime", 		"RealTimeClock.outClock",cp) );
	assert( Deployer:connect(me..".inMotorState", 	synchronizatorName..".outMotorMeasures",cp) );
	assert( Deployer:connect(me..".inParams", 		"UbiquityParams.outParams",cp) );
	
	assert( OdometryDeployer:check(me) )
	return true
end


