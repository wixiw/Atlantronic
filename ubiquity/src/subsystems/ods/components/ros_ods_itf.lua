dofile("/opt/ard/ubiquity/src/subsystems/orocos/component_deployer_object.lua")

RosOdsItfDeployer = ComposantDeployer:new()
local me = "RosOdsItf"

function RosOdsItfDeployer:load()
	assert( Deployer:loadComponent(me,"arp_ods::RosOdsItf"))
	assert( Deployer:addPeer("DotGraph",me))
	assert( Deployer:addPeer(me,"MotionControl"))
	assert( Deployer:setActivity(me, 0.050, 10, 1))
	return true
end


function RosOdsItfDeployer:connect()
	assert( Deployer:connect(me..".inCurrentOrderIsFinished",	"MotionControl.outOrderFinished",cp))
	assert( Deployer:connect(me..".inCurrentOrderIsInError",	"MotionControl.outOrderInError",cp))
	assert( Deployer:connect(me..".inRobotBlocked",				"KinematicBase.outRobotBlocked",cp))
	assert( Deployer:connect(me..".inParams", 					"UbiquityParams.outParams",cp))
	assert( Deployer:connect(me..".inPose",						"Localizator.outPose",cp))
	assert( Deployer:connect(me..".inSpeed",					"Localizator.outICRSpeed",cp))
	return true
end



