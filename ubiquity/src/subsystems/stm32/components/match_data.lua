dofile("/opt/ard/ubiquity/src/subsystems/orocos/component_deployer_object.lua")


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
	assert( Deployer:stream(me..".outMatchData",ros:topic("/Ubiquity/match_data")))
	return true
end

function MatchDataDeployer:start()
	--nothing to do as we are added in a monitor
	return true
end


