dofile("/opt/ard/ubiquity/src/subsystems/orocos/component_deployer_object.lua")


LastComponentDeployer = ComposantDeployer:new()
local me = "LastComponent"

function LastComponentDeployer:load()
	assert( Deployer:loadComponent(me,"arp_master::LastComponent"))
	assert( Deployer:addPeer("DotGraph",me))
	assert( Deployer:setActivity(me,1,0,rtt.globals.ORO_SCHED_OTHER))
	return true

end

function LastComponentDeployer:connect()
	assert( Deployer:stream(me..".outDeployed",ros:topic("/Master/deployed")))
	return true
end

function LastComponentDeployer:start()

	LastComponent = assert( Deployer:getPeer(me))
	assert( LastComponent:configure())
	assert( LastComponent:start())
	
	DotGraph = Deployer:getPeer("DotGraph")
	assert( DotGraph:start() )
	return true
end