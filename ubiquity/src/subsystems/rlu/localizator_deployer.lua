dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")


LocalizatorDeployer = ComposantDeployer:new()
local me = "Localizator"

function LocalizatorDeployer:load()
	Deployer:loadComponent(me, "arp_rlu::Localizator")
	assert( Deployer:addPeer("DotGraph",me))
	Deployer:setMasterSlaveActivity("MotionScheduler", me)
	return true
end


function LocalizatorDeployer:connect()
	Deployer:connect(me..".inOdo","Odometry.outICRSpeed",cp)
	assert( Deployer:stream(me..".inScan",ros:topic("/top_scan")))
	Deployer:addPeer("Reporting", me)
	LocalizatorDeployer:check(me)
	return true
end


