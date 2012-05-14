dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")


ObstacleManager = ComposantDeployer:new()
local me = "ObstacleManager"

function ObstacleManager:load()
	assert( Deployer:loadComponent(me, "arp_rlu::ObstacleManager"))
	Deployer:setActivity(me, 0.100, 0, rtt.globals.ORO_SCHED_OTHER)
	return true
end


function ObstacleManager:registerToSql()
	OrocosSqlMonitor = assert(Deployer:getPeer("OrocosSqlBridge"))
	assert( Deployer:addPeer("OrocosSqlBridge",me))
	return true
end


function ObstacleManager:connect()
	assert( Deployer:addPeer("Reporting", me))
    assert( Deployer:connect(me..".inRearObstacles", "Localizator.outObstacles",cp))
	assert( Deployer:connect(me..".inFrontObstacles", "FrontObstacleDetector.outObstacles",cp))
	assert( ObstacleManager:check(me))
	return true
end


