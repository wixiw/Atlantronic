dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")


LastComponentDeployer = ComposantDeployer:new()

function LastComponentDeployer:load()
	Deployer:loadComponent("LastComponent","arp_master::LastComponent");
	Deployer:setActivity("LastComponent",1,0,0);

end

function LastComponentDeployer:connect()
	Deployer:stream("LastComponent.outDeployed",ros:topic("/Master/deployed"))

	LastComponent = Deployer:getPeer("LastComponent");
	LastComponent:configure();
	LastComponent:start();
end
