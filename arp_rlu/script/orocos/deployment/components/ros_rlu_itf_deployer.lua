dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")


RosRluItfDeployer = ComposantDeployer:new()
local me = "RosRluItf"

function RosRluItfDeployer:load()
	Deployer:loadComponent(me, "arp_rlu::RosRluItf")
	Deployer:setActivity(me, 0.050, 10, 1)
end


function RosRluItfDeployer:connect()
	Deployer:addPeer(me,"Localizator")
	Deployer:addPeer(me,"LaserOnlyLocalizator")
	Deployer:stream(me..".outPose",ros:topic("/Localizator/pose"))
	Deployer:connect(me..".inPose","Localizator.outPose",cp)
	Deployer:connect(me..".inTwist","Localizator.outTwist",cp)
	Deployer:connect(me..".inOpponents","ObstacleManager.outOpponents",cp)
	RosRluItfDeployer:check(me)
	
	assert( Deployer:stream("RosRluItf.outOpponents",ros:topic("/Localizator/opponents_detected")))
end


