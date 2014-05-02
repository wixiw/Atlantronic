dofile("/opt/ard/ubiquity/src/subsystems/orocos/component_deployer_object.lua")
dofile("/opt/ard/ubiquity/src/subsystems/simu/components/ros_hml_simu_itf.lua");
dofile("/opt/ard/ubiquity/src/subsystems/simu/components/simu_monitor.lua");
dofile("/opt/ard/ubiquity/src/subsystems/simu/components/ubiquity_simulation.lua");

SimuDeployer = ComposantDeployer:new()

function SimuDeployer:load()
	print("... LOAD simu")
	assert(Deployer:import("arp_simu"))
	assert( UbiquitySimulDeployer:load())
	assert( RosHmlSimuItfDeployer:load())
	assert( SimuMonitorDeployer:load())
	return true
end

function SimuDeployer:connect()
	print("... CONNECT simu")
	assert( UbiquitySimulDeployer:connect())
	assert( RosHmlSimuItfDeployer:connect())
	assert( SimuMonitorDeployer:connect())
	return true
end


function SimuDeployer:start()
	print("... START simu")
	assert( SimuMonitorDeployer:start());
		--TODO bug opengl
	--assert( HmiDeployer:start())
	return true
end

