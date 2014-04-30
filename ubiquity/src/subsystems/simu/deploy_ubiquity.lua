require("rttlib")
--require("rfsm")
--require("rfsm_rtt")

rttlib.color=true
Deployer = rtt.getTC()
print("====================")
print("début déploiment arp_simu")

-- chargement des librairies contenants les composants d'arp_hml
Deployer:import("arp_simu");

dofile("/opt/ard/arp_simu/script/orocos/deployment/ubiquity/real_time_clock.lua");
dofile("/opt/ard/arp_hml/script/orocos/deployment/ubiquity/joystick_deployer.lua");
dofile("/opt/ard/arp_simu/script/orocos/deployment/ubiquity/ros_hml_simu_itf_deployer.lua");
dofile("/opt/ard/arp_simu/script/orocos/deployment/ubiquity/hml_monitor_deployer.lua");
--dofile("/opt/ard/arp_simu/script/orocos/deployment/ubiquity/motor_simul_deployer.lua");
--dofile("/opt/ard/arp_simu/script/orocos/deployment/ubiquity/syncronizator_deployer.lua");
dofile("/opt/ard/arp_simu/script/orocos/deployment/ubiquity/ubiquity_simulation_deployer.lua");
dofile("/opt/ard/arp_simu/script/orocos/deployment/ubiquity/scheduler_deployer.lua");

-- chargement de l'interface HML
print("... load components")
assert( RealTimeClockDeployer:load() , 		"Failed to load RealTimeClock")
assert( UbiquitySimulDeployer:load() , 		"Failed to load UbiquitySimul")
assert( SchedulerDeployer:load() , 			"Failed to load Scheduler")
--assert( MotorSimulDeployer:load() , 		"Failed to load MotorSimul")
--assert( Syncronizator:load() , 				"Failed to load Syncronizator")
--assert( JoystickDeployer:load() , 			"Failed to load Joystick")
assert( RosHmlSimuItfDeployer:load() , 		"Failed to load RosHmlSimuItf")
assert( HmlMonitorSimulDeployer:load() , 	"Failed to load HmlMonitorSimul")

print("... connect components")
assert( RealTimeClockDeployer:connect() , 	"Failed to connect RealTimeClock")
assert( UbiquitySimulDeployer:connect() , 	"Failed to connect UbiquitySimul" )
assert( SchedulerDeployer:connect() , 		"Failed to connect Scheduler" )
--assert( MotorSimulDeployer:connect() , 		"Failed to connect MotorSimul" )
--assert( Syncronizator:connect() , 			"Failed to connect Syncronizator" )
--assert( JoystickDeployer:connect() , 		"Failed to connect Joystick" )
assert( RosHmlSimuItfDeployer:connect() , 	"Failed to connect RosHmlSimuItf" )
assert( HmlMonitorSimulDeployer:connect() , "Failed to connect HmlMonitorSimul" )


print("... start components")
assert( HmlMonitorSimulDeployer:start() , "Failed to start HmlMonitor");

print("fin déploiment arp_simu")
print("====================")
