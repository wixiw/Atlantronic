dofile("/opt/ard/ubiquity/src/subsystems/orocos/component_deployer_object.lua")


LaserOnlyLocalizatorDeployer = ComposantDeployer:new()
local me = "LaserOnlyLocalizator"

function LaserOnlyLocalizatorDeployer:load()
	assert( Deployer:loadComponent(me, "arp_rlu::LaserOnlyLocalizatorCpn") )
	assert( Deployer:addPeer("DotGraph",me))
	assert( Deployer:setActivity(me,0.0,0,rtt.globals.ORO_SCHED_OTHER) )
	return true
end


function LaserOnlyLocalizatorDeployer:connect()
	assert( Deployer:stream(me..".inScan",ros:topic("/top_scan")) )
	assert( Deployer:addPeer("Reporting", me) )
	assert( LaserOnlyLocalizatorDeployer:check(me) )
	return true
end


