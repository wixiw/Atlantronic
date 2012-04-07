dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")


CanDeployer = ComposantDeployer:new()

function CanDeployer:load()
	Deployer:loadComponent("Can1","arp_hml::CanOpenController");
	Deployer:setActivity("Can1",0,50,1);

end

function CanDeployer:connect()
	Deployer:addPeer("Reporting", "Can1");
	CanDeployer:check("Can1")
end



