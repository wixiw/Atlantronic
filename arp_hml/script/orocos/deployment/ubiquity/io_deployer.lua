dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")


IoDeployer = ComposantDeployer:new()

function IoDeployer:load()
	Deployer:loadComponent("WoodheadIn","arp_hml::WoodheadIn")
	Deployer:setMasterSlaveActivity("Can1", "WoodheadIn")

	Deployer:loadComponent("WoodheadOut","arp_hml::WoodheadOut")
	Deployer:setMasterSlaveActivity("Can1", "WoodheadOut")
end

function IoDeployer:connect()
	Deployer:addPeer("WoodheadIn", "Can1")
	Deployer:addPeer("Reporting", "WoodheadIn")

	Deployer:addPeer("WoodheadOut", "Can1")
	Deployer:addPeer("Reporting", "WoodheadOut")
	
	IoDeployer:check("WoodheadIn")
	IoDeployer:check("WoodheadOut")
end



