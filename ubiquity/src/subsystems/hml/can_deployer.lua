dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")


CanDeployer = ComposantDeployer:new();

function CanDeployer:load()
	assert( Deployer:loadComponent("Can1","arp_hml::CanOpenController"));
	assert(Deployer:addPeer("DotGraph","Can1"))
	assert( Deployer:setActivity("Can1",0,40,rtt.globals.ORO_SCHED_RT));
	return true
end

function CanDeployer:connect()
	assert( Deployer:addPeer("Reporting", "Can1"));
	assert( CanDeployer:check("Can1"));
	return true
end



