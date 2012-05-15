dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")


RosRluItfDeployer = ComposantDeployer:new()
local me = "RosRluItf"

function RosRluItfDeployer:load()
	assert( Deployer:loadComponent(me, "arp_rlu::RosRluItf") )
	assert( Deployer:setActivity(me, 0.050, 10, 1) )
	return true
end

function RosRluItfDeployer:connect()
	assert( Deployer:addPeer(me,"Localizator") )
	assert( Deployer:addPeer(me,"LaserOnlyLocalizator") )
	assert( Deployer:connect(me..".inPose","Localizator.outPose",cp) )
	--assert( Deployer:connect(me..".inLocalizatorState","Localizator.outLocalizatorState",cp) )
	--assert( Deployer:connect(me..".inLocalizatorMode","Localizator.outLocalizatorMode",cp) )
	--assert( Deployer:connect(me..".inLocalizatorQuality","Localizator.outLocalizatorQuality",cp) )
	--assert( Deployer:connect(me..".inLocalizatorVisibility","Localizator.outLocalizatorVisibility",cp) )
	assert( Deployer:connect(me..".inTwist","Localizator.outTwist",cp) )
	assert( Deployer:connect(me..".inOpponents","ObstacleManager.outOpponents",cp) )
	assert( Deployer:stream(me..".outPose",ros:topic("/Localizator/pose")) )
	--assert( Deployer:stream(me..".outLocalizatorState",ros:topic("/Localizator/state")) )
	assert( Deployer:stream(me..".outOpponents",ros:topic("/Localizator/opponents_detected")) )
	assert( RosRluItfDeployer:check(me) )
	return true
end


