dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")


ObstacleManager = ComposantDeployer:new()
local me = "ObstacleManager"

function ObstacleManager:load()
	assert( Deployer:loadComponent(me, "arp_rlu::ObstacleManager"))
	assert( Deployer:setMasterSlaveActivity("MotionScheduler", me))
	return true
end


function ObstacleManager:registerToSql()
	OrocosSqlMonitor = assert(Deployer:getPeer("OrocosSqlBridge"))
	assert( Deployer:addPeer("OrocosSqlBridge",me))
	return true
end


function ObstacleManager:connect()
	assert( Deployer:addPeer("Reporting", me))
	assert( Deployer:connect(me..".inObstacles", "Localizator.outObstacles",cp))
	assert( ObstacleManager:check(me))
	return true
end


