dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")


HokuyoStm32Deployer = ComposantDeployer:new()
local frontHokuyo = "FrontHokuyo"
local rearHokuyo = "RearHokuyo"

function HokuyoStm32Deployer:load()
	assert( Deployer:loadComponent(frontHokuyo,"arp_stm32::HokuyoItf"))
	assert( Deployer:addPeer("DotGraph",frontHokuyo))
	assert( Deployer:addPeer("Reporting", frontHokuyo) )
	assert( Deployer:setMasterSlaveActivity("Discovery", frontHokuyo))
	
	assert( Deployer:loadComponent(rearHokuyo,"arp_stm32::HokuyoItf"))
	assert( Deployer:addPeer("DotGraph",rearHokuyo))
	assert( Deployer:addPeer("Reporting", rearHokuyo) )
	assert( Deployer:setMasterSlaveActivity("Discovery", rearHokuyo))
	
	return true
end

function HokuyoStm32Deployer:connect()
	return true
end

function HokuyoStm32Deployer:start()
	--nothing to do as we are added in a monitor
	return true
end


