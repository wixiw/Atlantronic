dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")


IoDeployer = ComposantDeployer:new()

function IoDeployer:load()
	Deployer:loadComponent("WoodheadIn","arp_hml::WoodheadIn");
	Deployer:setActivity("WoodheadIn",0,30,1);

	Deployer:loadComponent("WoodheadOut","arp_hml::WoodheadOut");
	Deployer:setActivity("WoodheadOut",0,30,1);
end

function IoDeployer:connect()
	Deployer:addPeer("WoodheadIn", "Can1");
	Deployer:addPeer("Reporting", "WoodheadIn");

	Deployer:addPeer("WoodheadOut", "Can1");
	Deployer:addPeer("Reporting", "WoodheadOut");
end


