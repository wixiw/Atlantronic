dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")

RosOdsItfDeployer = ComposantDeployer:new()
local me = "RosOdsItf"

function RosOdsItfDeployer:load()
	assert( Deployer:loadComponent(me,"arp_ods::RosOdsItf"))
	assert( Deployer:addPeer("DotGraph",me))
	assert( Deployer:setActivity(me, 0.050, 10, 1))
	return true
end


function RosOdsItfDeployer:connect()
	assert( Deployer:connect(me..".inCurrentOrderIsFinished","MotionControl.outOrderFinished",cp))
	assert( Deployer:connect(me..".inCurrentOrderIsInError","MotionControl.outOrderInError",cp))
	assert( Deployer:connect(me..".inRobotBlocked","KinematicBase.outRobotBlocked",cp))
	assert( Deployer:connect(me..".inParams", "UbiquityParams.outParams",cp))
	assert( Deployer:addPeer(me,"MotionControl"))
	
	RluMonitor = Deployer:getPeer("RluMonitor")
	assert( Deployer:addPeer("RluMonitor", me))
	assert( RluMonitor:connect(me,"inPose","Localizator","outPose"))
	assert( RluMonitor:connect(me,"inSpeed","Localizator","outICRSpeed"))
	return true
end



