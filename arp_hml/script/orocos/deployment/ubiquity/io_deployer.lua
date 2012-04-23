dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")


IoDeployer = ComposantDeployer:new()

function IoDeployer:load()
	Deployer:loadComponent("WoodheadIn","arp_hml::WoodheadIn")
	Deployer:setActivity("WoodheadIn",0,30,rtt.globals.ORO_SCHED_RT)

	Deployer:loadComponent("WoodheadOut","arp_hml::WoodheadOut")
	Deployer:setActivity("WoodheadOut",0,30,rtt.globals.ORO_SCHED_RT)
end

function IoDeployer:connect()
	Deployer:addPeer("WoodheadIn", "Can1")
	Deployer:addPeer("Reporting", "WoodheadIn")

	Deployer:addPeer("WoodheadOut", "Can1")
	Deployer:addPeer("Reporting", "WoodheadOut")
	
	IoDeployer:check("WoodheadIn")
	IoDeployer:check("WoodheadOut")
end



