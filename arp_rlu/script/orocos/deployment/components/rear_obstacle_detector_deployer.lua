dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")


RearObstacleDetector = ComposantDeployer:new()
local me = "RearObstacleDetector"

function RearObstacleDetector:load()
	assert( Deployer:loadComponent(me, "arp_rlu::RearObstacleDetector"))
	assert( Deployer:addPeer("DotGraph",me))
	assert( Deployer:setActivity(me, 0.100, 0, rtt.globals.ORO_SCHED_OTHER) )
	return true
end

function RearObstacleDetector:connect()
	assert( Deployer:addPeer("Reporting", me))
	assert( Deployer:connect(me..".inPose", "Localizator.outPose",cp))
    assert( Deployer:stream(me..".inScan",ros:topic("/Ubiquity/rear_scan")) )
	assert( RearObstacleDetector:check(me))
	return true
end


