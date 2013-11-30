dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")


LocalizatorFilterDeployer = ComposantDeployer:new()
local me = "LocalizatorFilter"

function LocalizatorFilterDeployer:load()
	assert( Deployer:loadComponent(me, "arp_rlu::LocFilterCpn"))
	assert( Deployer:setMasterSlaveActivity("MotionScheduler", me))
	return true
end


function LocalizatorFilterDeployer:connect()
	assert(Deployer:connect(me..".inPose","Localizator.outPose",cp))
	assert(Deployer:addPeer("Reporting", me))
	assert(LocalizatorFilterDeployer:check(me))
	return true
end

