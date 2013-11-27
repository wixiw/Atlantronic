dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")


RluMonitorDeployer = ComposantDeployer:new()
local me = "RluMonitor"

function RluMonitorDeployer:load()
	assert( Deployer:loadComponent(me, "arp_core::Monitor") )
	assert( Deployer:setActivity(me, 0.100, 10, 1) )
	return true
end


function RluMonitorDeployer:addToMonitor(name)
	OdsMonitor = assert( Deployer:getPeer(me) )
	assert( Deployer:addPeer(me, name) )
	assert( OdsMonitor:ooAddMonitoredPeer (name) )
	Deployer:removePeer (name)
	return true
end


function RluMonitorDeployer:connect()
--ajout au monitor
	assert( RluMonitorDeployer:addToMonitor("Odometry"), "Failed to add Odometry in Monitor" )
	assert( RluMonitorDeployer:addToMonitor("Localizator") )
	assert( RluMonitorDeployer:addToMonitor("LaserOnlyLocalizator") )
	assert( RluMonitorDeployer:addToMonitor("LocalizatorFilter") )
	assert( RluMonitorDeployer:addToMonitor("FrontObstacleDetector") )
	assert( RluMonitorDeployer:addToMonitor("ObstacleManager") )
	assert( RluMonitorDeployer:addToMonitor("RosRluItf") )
	assert( RluMonitorDeployer:check(me) )
	return true
end

function RluMonitorDeployer:start()
	OdsMonitor = assert( Deployer:getPeer(me) )
	
	assert( OdsMonitor:configure() )
	assert( OdsMonitor:start() )
	return true
end

