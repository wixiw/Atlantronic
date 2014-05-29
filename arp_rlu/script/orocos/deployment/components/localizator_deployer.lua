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
	
	--TODO coupe 2014 c'est mal range, probleme de dependance cyclique => est corrige par le nouveau deploiement big_refactoring
	DiscoveryMonitor = Deployer:getPeer("DiscoveryMonitor")
	assert( Deployer:addPeer("DiscoveryMonitor", me))
	assert( Deployer:connect("Discovery.inPose", me..".outPose", cp))
	assert( DiscoveryMonitor:connect(me,"inScan","FrontHokuyo","outScan"))
	
	Deployer:addPeer("Reporting", me)
	LocalizatorDeployer:check(me)
	return true
end


