dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")


CanDeployer = ComposantDeployer:new()

function CanDeployer:load()
	Deployer:loadComponent("Can1","arp_hml::CanOpenController");
	Deployer:setActivity("Can1",0,45,rtt.globals.ORO_SCHED_RT);

end

function CanDeployer:connect()
	Deployer:addPeer("Reporting", "Can1");
	CanDeployer:check("Can1")
end



