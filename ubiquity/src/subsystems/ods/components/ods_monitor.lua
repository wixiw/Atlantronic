dofile("/opt/ard/ubiquity/src/subsystems/orocos/component_deployer_object.lua")


OdsMonitorDeployer = ComposantDeployer:new()
local me = "OdsMonitor"

function OdsMonitorDeployer:load()
	assert( Deployer:loadComponent(me, "arp_core::Monitor"))
	assert( Deployer:addPeer("DotGraph",me))
	assert( Deployer:setActivity(me, 0.100, 0, rtt.globals.ORO_SCHED_OTHER) )
	return true
end


function OdsMonitorDeployer:addToMonitor(name)
	OdsMonitor = Deployer:getPeer(me)
	assert( Deployer:addPeer(me, name))
	assert( OdsMonitor:ooAddMonitoredPeer(name))
	Deployer:removePeer (name)
	return true
end


function OdsMonitorDeployer:connect()
	assert( OdsMonitorDeployer:check(me))
	return true
end

function OdsMonitorDeployer:start()
	assert( OdsMonitorDeployer:addToMonitor("KinematicBase"))
	assert( OdsMonitorDeployer:addToMonitor("MotionControl"))
	assert( OdsMonitorDeployer:addToMonitor("RosOdsItf"))
	
	OdsMonitor = Deployer:getPeer(me)
	assert( OdsMonitor:configure())
	assert( OdsMonitor:start())
	return true
end

