dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")


CanDeployer = ComposantDeployer:new()

function CanDeployer:load()
	Deployer:loadComponent("Can1","arp_hml::CanOpenController");
	Deployer:setActivity("Can1",0,50,1);

end

function CanDeployer:connect()
	Deployer:addPeer("Reporting", "Can1");
end

function CanDeployer:start()
	Can1 = Deployer:getPeer("Can1")
print("... ... config CAN1")
	Can1:configure();
print("... ... sleeping")
	os.execute("sleep 1")
print("... ... start CAN1")
	Can1:start();
end



