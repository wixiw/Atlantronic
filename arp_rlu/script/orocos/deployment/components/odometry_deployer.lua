dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")


OdometryDeployer = ComposantDeployer:new()
local me = "Odometry"

function OdometryDeployer:load()
	assert( Deployer:loadComponent(me, "arp_rlu::Odometry4Ubiquity") )
	assert( Deployer:setMasterSlaveActivity("MotionScheduler", me) )
	return true
end


function OdometryDeployer:registerToSql()
	OrocosSqlMonitor = assert( Deployer:getPeer("OrocosSqlBridge") )
	assert( Deployer:addPeer("OrocosSqlBridge",me) )
	return true
end


function OdometryDeployer:connect()
	--on s'ajoute en peer a HmlMonitor pour pouvoir faire les connections
	assert( Deployer:addPeer("HmlMonitor", me) )
	assert( Deployer:addPeer("Reporting", me) )
	assert( HmlMonitor:connect(me, "inTime", 		"Syncronizator","outClock") );
	assert( HmlMonitor:connect(me, "inMotorState", 		"Syncronizator","outMotorMeasures") );
	assert( Deployer:connect(me..".inParams", "UbiquityParams.outParams",cp) );
	
	assert( OdometryDeployer:check(me) )
	return true
end


