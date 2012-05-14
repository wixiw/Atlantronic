dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")


ObstacleManager = ComposantDeployer:new()
local me = "ObstacleManager"

function ObstacleManager:load()
	Deployer:loadComponent(me, "arp_rlu::ObstacleManager")
	Deployer:setMasterSlaveActivity("MotionScheduler", me)
end


function ObstacleManager:registerToSql()
	OrocosSqlMonitor = Deployer:getPeer("OrocosSqlBridge")
	Deployer:addPeer("OrocosSqlBridge",me)
end


function ObstacleManager:connect()
	Deployer:addPeer("Reporting", me)
	Deployer:connect(me..".inObstacles", "Localizator.outObstacles",cp);
	
	ObstacleManager:check(me)
end


