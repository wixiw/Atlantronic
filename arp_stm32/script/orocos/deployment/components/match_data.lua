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
	assert( Deployer:stream(me..".outIoStart",ros:topic("/Ubiquity/start")))
	assert( Deployer:stream(me..".outIoStartColor",ros:topic("/Ubiquity/color")))
	assert( Deployer:stream(me..".inReadyForMatch",ros:topic("/Master/ready_for_match")))
	assert( Deployer:stream(me..".inInformInitialized",ros:topic("/Master/initialized")))
	return true
end

function MatchDataDeployer:start()
	--nothing to do as we are added in a monitor
	return true
end


