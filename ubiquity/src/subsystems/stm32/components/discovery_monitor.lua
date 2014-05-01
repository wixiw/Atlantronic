dofile("/opt/ard/ubiquity/src/subsystems/orocos/component_deployer_object.lua")


DiscoveryMonitorDeployer = ComposantDeployer:new()
local me = "DiscoveryMonitor"

function DiscoveryMonitorDeployer:load()
	assert( Deployer:loadComponent(me, "arp_core::Monitor"))
	assert( Deployer:addPeer("DotGraph",me))
	assert( Deployer:setActivity(me, 0.100, 0, rtt.globals.ORO_SCHED_OTHER))
	return true
end


function DiscoveryMonitorDeployer:addToMonitor(name)
	OdsMonitor = Deployer:getPeer(me)
	assert( Deployer:addPeer(me, name))
	assert( OdsMonitor:ooAddMonitoredPeer (name))
	Deployer:removePeer (name)
	return true
end


function DiscoveryMonitorDeployer:connect()
	DiscoveryMonitorDeployer:addToMonitor("LeftCannonFinger")
	DiscoveryMonitorDeployer:addToMonitor("RightCannonFinger")
	DiscoveryMonitorDeployer:addToMonitor("LeftCannonStocker")
	DiscoveryMonitorDeployer:addToMonitor("RightCannonStocker")
	DiscoveryMonitorDeployer:addToMonitor("LeftFinger")
	DiscoveryMonitorDeployer:addToMonitor("RightFinger")
	DiscoveryMonitorDeployer:addToMonitor("ArmSlider")
	DiscoveryMonitorDeployer:addToMonitor("ArmShoulder")
	DiscoveryMonitorDeployer:addToMonitor("ArmShoulderElbow")
	DiscoveryMonitorDeployer:addToMonitor("ArmWristElbow")
	DiscoveryMonitorDeployer:addToMonitor("ArmWrist")
	
	DiscoveryMonitorDeployer:addToMonitor("LeftFingerFireOmron")
	DiscoveryMonitorDeployer:addToMonitor("RightFingerFireOmron")
	DiscoveryMonitorDeployer:addToMonitor("ArmFireOmron")
	DiscoveryMonitorDeployer:addToMonitor("LeftFingerLateralOmron")
	DiscoveryMonitorDeployer:addToMonitor("RightFingerLateralOmron")
	DiscoveryMonitorDeployer:addToMonitor("LeftRecalOmron")
	DiscoveryMonitorDeployer:addToMonitor("RightRecalOmron")
	
	DiscoveryMonitorDeployer:addToMonitor("LeftFingerPump")
	DiscoveryMonitorDeployer:addToMonitor("RightFingerPump")
	DiscoveryMonitorDeployer:addToMonitor("ArmPump")
	
	DiscoveryMonitorDeployer:addToMonitor("Gyrometer")
	DiscoveryMonitorDeployer:addToMonitor("MatchData")
	
--	DiscoveryMonitorDeployer:addToMonitor("DynamixelBus")
	
--TODO a remettre quand les problemes opengl sont regles
--  DiscoveryMonitorDeployer:addToMonitor("Hmi")
	
	DiscoveryMonitorDeployer:check(me)
	return true
end

function DiscoveryMonitorDeployer:start()
	OdsMonitor = Deployer:getPeer(me)
	OdsMonitor:configure()
	OdsMonitor:start()
	return true
end

