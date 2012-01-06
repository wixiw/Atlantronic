dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")


JoystickDeployer = ComposantDeployer:new()

function JoystickDeployer:load()
	Deployer:loadComponent("Joystick","arp_hml::GamepadPS1");
	Deployer:setActivity("Joystick",0.050,0,1);

end

function JoystickDeployer:connect()
	Deployer:addPeer("Reporting", "Joystick");
end

function JoystickDeployer:start()
	Joystick = Deployer:getPeer("Joystick")
print("... ... config Joystick")
	Joystick:configure();
print("... ... start Joystick")
	Joystick:start();
end

