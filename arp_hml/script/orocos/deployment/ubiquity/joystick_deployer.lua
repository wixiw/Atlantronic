dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")


JoystickDeployer = ComposantDeployer:new()

function JoystickDeployer:load()
	Deployer:loadComponent("Joystick","arp_hml::GamepadPS1");
	Deployer:setActivity("Joystick",0.050,5,rtt.globals.ORO_SCHED_RT);

end

function JoystickDeployer:connect()
	Deployer:addPeer("Reporting", "Joystick");
	
	JoystickDeployer:check("Joystick")
end

