dofile("/opt/ard/ubiquity/src/subsystems/orocos/component_deployer_object.lua")


LocalizatorDeployer = ComposantDeployer:new()
local me = "Localizator"

function LocalizatorDeployer:load()
	Deployer:loadComponent(me, "arp_rlu::Localizator")
	assert( Deployer:addPeer("DotGraph",me))
	Deployer:addPeer("Reporting", me)
	Deployer:setMasterSlaveActivity("MotionScheduler", me)
	return true
end


function LocalizatorDeployer:connect()
	assert(Deployer:connect(me..".inOdo",			"Odometry.outICRSpeed",cp))
	assert(Deployer:connect(me..".inSmoothMode",	"MotionControl.outSmoothLocNeeded",cp))
	assert( Deployer:stream(me..".inScan",			ros:topic("/top_scan")))
	LocalizatorDeployer:check(me)
	return true
end


