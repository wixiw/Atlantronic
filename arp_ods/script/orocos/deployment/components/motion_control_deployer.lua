dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")

MotionControlDeployer = ComposantDeployer:new()
local me = "MotionControl"

function MotionControlDeployer:load()
	assert( Deployer:loadComponent(me,"arp_ods::MotionControl"))
	assert( Deployer:addPeer("DotGraph",me))
	assert( Deployer:setMasterSlaveActivity("MotionScheduler", me))
	return true
end


function MotionControlDeployer:connect()
	assert( Deployer:addPeer("Reporting", me))
	RluMonitor = Deployer:getPeer("RluMonitor")
	assert( Deployer:addPeer("RluMonitor", me))
	assert( RluMonitor:connect(me,"inPosition","Localizator","outPose"))
	assert( RluMonitor:connect(me,"inCurrentICRSpeed","Odometry","outICRSpeed"))
    assert( RluMonitor:connect(me,"outSmoothLocNeeded","Localizator","inSmoothMode"))
    
    assert( Deployer:addPeer("HmlMonitor", me))
    assert( Deployer:connect(me..".inParams", "UbiquityParams.outParams",cp))
    
    HmlMonitor = Deployer:getPeer("HmlMonitor");
    assert( HmlMonitor:connect(me,"inCanPeriod","Can1","outPeriod"))
    return true
end



