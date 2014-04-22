dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")


HmiDeployer = ComposantDeployer:new()
local me = "Hmi"

function HmiDeployer:load()
	assert( Deployer:loadComponent(me,"arp_stm32::Hmi"))
	assert( Deployer:addPeer("DotGraph",me))
	assert( Deployer:setMasterSlaveActivity("Discovery", me))
	return true
end

function HmiDeployer:connect()
	assert( Deployer:addPeer("Reporting", me) )
	Deployer:addPeer("RluMonitor", me);
	RluMonitor:connect(me,"inPose","Localizator","outPose");
	return true
end

function HmiDeployer:start()
	Hmi = assert(Deployer:getPeer(me))
	assert(Hmi:configure())
--	assert(Hmi:start())
	return true
end


