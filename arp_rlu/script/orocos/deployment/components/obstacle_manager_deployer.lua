dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")


ObstacleManager = ComposantDeployer:new()
local me = "ObstacleManager"

function ObstacleManager:load()
	assert( Deployer:loadComponent(me, "arp_rlu::ObstacleManager"))
	assert( Deployer:addPeer("DotGraph",me))
	assert( Deployer:setActivity(me, 0.100, 0, rtt.globals.ORO_SCHED_OTHER) )
	return true
end


function ObstacleManager:connect()
	assert( Deployer:addPeer("Reporting", me))
    
    DiscoveryMonitor = Deployer:getPeer("DiscoveryMonitor")
	assert( Deployer:addPeer("DiscoveryMonitor", me))
    assert( DiscoveryMonitor:connect(me,"inFrontObstacles","FrontHokuyo","outObstacles"))
	assert( DiscoveryMonitor:connect(me,"inRearObstacles","RearHokuyo","outObstacles"))
	assert( ObstacleManager:check(me))
	return true
end


