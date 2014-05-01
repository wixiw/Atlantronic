dofile("/opt/ard/ubiquity/src/subsystems/orocos/component_deployer_object.lua")


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
	RluMonitor:connect(me,"inEstimatedPose","Localizator","outPose");
	return true
end

function HmiDeployer:start()
	--to be added to a monitor ...
	return true
end


