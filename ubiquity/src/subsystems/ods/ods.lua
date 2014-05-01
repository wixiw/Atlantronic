dofile("/opt/ard/ubiquity/src/subsystems/orocos/component_deployer_object.lua")
dofile("/opt/ard/ubiquity/src/subsystems/ods/components/kinematics_base.lua");
dofile("/opt/ard/ubiquity/src/subsystems/ods/components/motion_control.lua");
dofile("/opt/ard/ubiquity/src/subsystems/ods/components/ods_monitor.lua");
dofile("/opt/ard/ubiquity/src/subsystems/ods/components/ros_ods_itf.lua");

OdsDeployer = ComposantDeployer:new()

function OdsDeployer:load()
	print("... LOAD ods")
	assert(Deployer:import("arp_ods"))
	assert( MotionControlDeployer:load())
	assert( KinematicBaseDeployer:load())
	assert( RosOdsItfDeployer:load())
	assert( OdsMonitorDeployer:load())
	return true
end

function OdsDeployer:connect(synchronizatorName,hmlMonitorName)
	print("... CONNECT ods")
	assert( MotionControlDeployer:connect())
	assert( KinematicBaseDeployer:connect(synchronizatorName,hmlMonitorName))
	assert( RosOdsItfDeployer:connect())
	assert( OdsMonitorDeployer:connect())
	return true
end

function OdsDeployer:start()
	print("... START ods")
	assert( OdsMonitorDeployer:start())
	return true
end