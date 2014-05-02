dofile("/opt/ard/ubiquity/src/subsystems/orocos/component_deployer_object.lua")
dofile("/opt/ard/ubiquity/src/subsystems/core/components/scheduler.lua")
dofile("/opt/ard/ubiquity/src/subsystems/core/components/time.lua")

CoreDeployer = ComposantDeployer:new()
local paramServer = "UbiquityParams"

function CoreDeployer:load(simulation)
	print("... LOAD core")
	assert( Deployer:import("arp_core"))
	
	assert(TimeDeployer:load(simulation))
	assert(SchedulerDeployer:load())
	
	assert( Deployer:loadComponent(paramServer,"arp_core::ParamsComponent"))
	return true
end

function CoreDeployer:connect()
	print("... CONNECT core")
	
	assert(TimeDeployer:connect())
	assert(SchedulerDeployer:connect())
	
	assert( Deployer:addPeer("DotGraph",paramServer))
	assert( Deployer:setActivity(paramServer,0.100,0,rtt.globals.ORO_SCHED_OTHER))
	
	return true
end


function CoreDeployer:start()
	print("... START core")
	
	assert(TimeDeployer:start())
	assert(SchedulerDeployer:start())
	
	UbiquityParams = assert(Deployer:getPeer(paramServer))
	assert( UbiquityParams:configure())
	assert( UbiquityParams:start())
	
	return true
end
