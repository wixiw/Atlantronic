dofile("/opt/ard/ubiquity/src/subsystems/orocos/component_deployer_object.lua")

XXXDeployer = ComposantDeployer:new()

function XXXDeployer:load()
	print("... LOAD xxx")
	
	return true
end

function XXXDeployer:connect()
	print("... CONNECT xxx")
	
	return true
end


function XXXDeployer:start()
	print("... START xxx")
	
	return true
end