dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")


ObstacleDetector = ComposantDeployer:new()
local front = "FrontObstacleDetector"
local rear = "RearObstacleDetector"

function ObstacleDetector:load()
	assert( Deployer:loadComponent(front, "arp_rlu::FrontObstacleDetector"))
	assert( Deployer:addPeer("DotGraph",front))
	assert( Deployer:setActivity(front, 0.100, 0, rtt.globals.ORO_SCHED_OTHER) )
	
	assert( Deployer:loadComponent(rear, "arp_rlu::RearObstacleDetector"))
	assert( Deployer:addPeer("DotGraph",rear))
	assert( Deployer:setActivity(rear, 0.100, 0, rtt.globals.ORO_SCHED_OTHER) )
	return true
end

function ObstacleDetector:connect()
	DiscoveryMonitor = Deployer:getPeer("DiscoveryMonitor")
	assert( Deployer:addPeer("DiscoveryMonitor", front))
	assert( Deployer:addPeer("DiscoveryMonitor", rear))
	
	assert( Deployer:addPeer("Reporting", front))
	assert( Deployer:connect(front..".inPose", "Localizator.outPose",cp))
    assert( DiscoveryMonitor:connect(front,"inScan","FrontHokuyo","outScan"))
    assert( ObstacleDetector:check(front))
    
    assert( Deployer:addPeer("Reporting", rear))
	assert( Deployer:connect(rear..".inPose", "Localizator.outPose",cp))
    assert( DiscoveryMonitor:connect(rear,"inScan","FrontHokuyo","outScan"))
	assert( ObstacleDetector:check(rear))
	return true
end


