dofile("/opt/ard/ubiquity/src/subsystems/orocos/component_deployer_object.lua")

MotionControlDeployer = ComposantDeployer:new()
local me = "MotionControl"

function MotionControlDeployer:load()
	assert( Deployer:loadComponent(me,"arp_ods::MotionControl"))
	assert( Deployer:addPeer("DotGraph",me))
	assert( Deployer:addPeer("Reporting", me))
	assert( Deployer:setMasterSlaveActivity("MotionScheduler", me))
	return true
end


function MotionControlDeployer:connect()
    assert( Deployer:connect(me..".inPosition", 		"Localizator.outPose",cp))
    assert( Deployer:connect(me..".inCurrentICRSpeed", 	"Odometry.outICRSpeed",cp))
    assert( Deployer:connect(me..".inParams", 			"UbiquityParams.outParams",cp))
    assert( Deployer:connect(me..".inCanPeriod",			"RealTimeClock.outPeriod",cp))
    return true
end



