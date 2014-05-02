dofile("/opt/ard/ubiquity/src/subsystems/orocos/component_deployer_object.lua")


SimuMonitorDeployer = ComposantDeployer:new()
local me = "SimuMonitor"

function SimuMonitorDeployer:load()
	assert( Deployer:loadComponent(me, "arp_core::Monitor"))
	assert( Deployer:addPeer("DotGraph",me))
	assert( Deployer:setActivity(me, 0.100, 0, rtt.globals.ORO_SCHED_OTHER) )
	return true
end


function SimuMonitorDeployer:addToMonitor(name)
	OdsMonitor = Deployer:getPeer(me)
	assert( Deployer:addPeer(me, name))
	assert( OdsMonitor:ooAddMonitoredPeer(name))
	Deployer:removePeer (name)
	return true
end


function SimuMonitorDeployer:connect()
	assert( SimuMonitorDeployer:check(me))
	return true
end

function SimuMonitorDeployer:start()
	assert( SimuMonitorDeployer:addToMonitor("RosHmlItf"))
	assert( SimuMonitorDeployer:addToMonitor("UbiquitySimul"))
	
	SimuMonitor = Deployer:getPeer(me)
	assert( SimuMonitor:configure())
	assert( SimuMonitor:start())
	return true
end

