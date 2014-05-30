dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")


HmiDeployer = ComposantDeployer:new()
local me = "Hmi"

function HmiDeployer:load()
	assert( Deployer:loadComponent(me,"arp_stm32::Stm32Hmi"))
	assert( Deployer:addPeer("DotGraph",me))
	assert( Deployer:setActivity(me,0.050,20,rtt.globals.ORO_SCHED_RT));
	return true
end

function HmiDeployer:connect()
	assert( Deployer:addPeer("Reporting", me) )
	return true
end

function HmiDeployer:start()
	--to be added to a monitor ...
	return true
end


