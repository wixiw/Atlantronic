dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")


JoystickDeployer = ComposantDeployer:new()

function JoystickDeployer:load()
	assert( Deployer:loadComponent("Joystick","arp_hml::GamepadPS1"))
	assert( Deployer:setActivity("Joystick",0.050,0,rtt.globals.ORO_SCHED_OTHER))

	return true
end

function JoystickDeployer:connect()
	assert( Deployer:addPeer("Reporting", "Joystick"))
	
	assert( JoystickDeployer:check("Joystick"))
	
	return true
end

