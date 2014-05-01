dofile("/opt/ard/ubiquity/src/subsystems/orocos/component_deployer_object.lua")
dofile("/opt/ard/ubiquity/src/subsystems/rlu/components/rlu_monitor.lua");
dofile("/opt/ard/ubiquity/src/subsystems/rlu/components/odometry.lua");
dofile("/opt/ard/ubiquity/src/subsystems/rlu/components/localizator.lua");
dofile("/opt/ard/ubiquity/src/subsystems/rlu/components/front_obstacle_detector.lua");
dofile("/opt/ard/ubiquity/src/subsystems/rlu/components/obstacle_manager.lua");
dofile("/opt/ard/ubiquity/src/subsystems/rlu/components/ros_rlu_itf.lua");

RluDeployer = ComposantDeployer:new()

function RluDeployer:load()
	print("... LOAD rlu")
	assert(Deployer:import("arp_rlu"))
	assert(Deployer:import("rtt_sensor_msgs"))
	OdometryDeployer:load();
	LocalizatorDeployer:load();
	FrontObstacleDetector:load();
	ObstacleManager:load();
	RosRluItfDeployer:load();
	RluMonitorDeployer:load();
	return true
end

function RluDeployer:connect(synchronizatorName)
	print("... CONNECT rlu")
	OdometryDeployer:connect(synchronizatorName);
	LocalizatorDeployer:connect();
	FrontObstacleDetector:connect();
	ObstacleManager:connect();
	RosRluItfDeployer:connect();
	RluMonitorDeployer:connect();
	return true
end


function RluDeployer:start()
	print("... START rlu")
	RluMonitorDeployer:start();
	return true
end

