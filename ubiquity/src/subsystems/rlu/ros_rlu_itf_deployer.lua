dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")


RosRluItfDeployer = ComposantDeployer:new()
local me = "RosRluItf"

function RosRluItfDeployer:load()
	assert( Deployer:loadComponent(me, "arp_rlu::RosRluItf") )
	assert( Deployer:addPeer("DotGraph",me))
	assert( Deployer:setActivity(me, 0.050, 10, 1) )
	return true
end

function RosRluItfDeployer:connect()
	assert( Deployer:addPeer(me,"Localizator") )
	assert( Deployer:connect(me..".inPose","Localizator.outPose",cp) )
	assert( Deployer:connect(me..".inLocalizationState","Localizator.outLocalizationState",cp) )
	assert( Deployer:connect(me..".inLocalizationMode","Localizator.outLocalizationMode",cp) )
	assert( Deployer:connect(me..".inLocalizationQuality","Localizator.outLocalizationQuality",cp) )
	assert( Deployer:connect(me..".inLocalizationVisibility","Localizator.outLocalizationVisibility",cp) )
	assert( Deployer:connect(me..".inICRSpeed","Odometry.outICRSpeed",cp) )
	assert( Deployer:connect(me..".inOpponents","ObstacleManager.outOpponents",cp) )
	assert( Deployer:stream(me..".outLocalizationState",ros:topic("/Localizator/state")) )
	assert( Deployer:stream(me..".outOpponents",ros:topic("/Localizator/opponents_detected")) )
	assert( RosRluItfDeployer:check(me) )
	return true
end


