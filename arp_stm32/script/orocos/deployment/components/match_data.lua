dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")


MatchDataDeployer = ComposantDeployer:new()
local me = "MatchData"

function MatchDataDeployer:load()
	assert( Deployer:loadComponent(me,"arp_stm32::MatchData"))
	assert( Deployer:addPeer("DotGraph",me))
	assert( Deployer:setMasterSlaveActivity("Discovery", me))
	return true
end

function MatchDataDeployer:connect()
	assert( Deployer:addPeer("Reporting", me) )
	return true
end

function MatchDataDeployer:start()
	Discovery = assert(Deployer:getPeer(me))
	assert(Discovery:configure())
	assert(Discovery:start())
	return true
end


