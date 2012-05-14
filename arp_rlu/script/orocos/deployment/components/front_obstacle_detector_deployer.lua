dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")


FrontObstacleDetector = ComposantDeployer:new()
local me = "FrontObstacleDetector"

function FrontObstacleDetector:load()
	assert( Deployer:loadComponent(me, "arp_rlu::FrontObstacleDetector"))
	assert( Deployer:setMasterSlaveActivity("MotionScheduler", me))
	return true
end


function FrontObstacleDetector:registerToSql()
	OrocosSqlMonitor = assert(Deployer:getPeer("OrocosSqlBridge"))
	assert( Deployer:addPeer("OrocosSqlBridge",me))
	return true
end


function FrontObstacleDetector:connect()
	assert( Deployer:addPeer("Reporting", me))
	assert( Deployer:connect(me..".inPose", "Localizator.outPose",cp))
    Deployer:stream(me..".inScan",ros:topic("/front_scan"))
	assert( FrontObstacleDetector:check(me))
	return true
end


