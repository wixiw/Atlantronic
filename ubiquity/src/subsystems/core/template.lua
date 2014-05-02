dofile("/opt/ard/ubiquity/src/subsystems/orocos/component_deployer_object.lua")

XXXDeployer = ComposantDeployer:new()

function XXXDeployer:load()
	print("... LOAD xxx")
	--import libraries
	--load components
	--attach Masters Activity (and only those one)
	return true
end

function XXXDeployer:connect()
	print("... CONNECT xxx")
	--register peers
	--connect ports
	--attach Slaves activity
	return true
end


function XXXDeployer:start()
	print("... START xxx")
	--configure components
	--update properties
	--start components
	return true
end