require("rttlib")
--require("rfsm")
--require("rfsm_rtt")

rttlib.color=true
Deployer = rtt.getTC()
print("====================")
print("début déploiment arp_hml")

-- chargement des librairies contenants les composants d'arp_hml
Deployer:import("arp_hml");

dofile("/opt/ard/arp_hml/script/orocos/deployment/ubiquity/joystick_deployer.lua");
dofile("/opt/ard/arp_hml/script/orocos/deployment/ubiquity/ros_hml_itf_deployer.lua");
dofile("/opt/ard/arp_hml/script/orocos/deployment/ubiquity_simul/hml_monitor_simul_deployer.lua");
dofile("/opt/ard/arp_hml/script/orocos/deployment/ubiquity/syncronizator_deployer.lua");
dofile("/opt/ard/arp_hml/script/orocos/deployment/ubiquity_simul/motor_simul_deployer.lua");
dofile("/opt/ard/arp_hml/script/orocos/deployment/ubiquity/scheduler_deployer.lua");
dofile("/opt/ard/arp_hml/script/orocos/deployment/ubiquity_simul/ubiquity_simul_deployer.lua");

-- chargement de l'interface HML
print("... load components")
assert( MotorSimulDeployer:load() , 		"Failed to load MotorSimul")
assert( SchedulerDeployer:load() , 			"Failed to load Scheduler")
assert( Syncronizator:load() , 				"Failed to load Syncronizator")
assert( JoystickDeployer:load() , 			"Failed to load Joystick")
assert( RosHmlItfDeployer:load() , 			"Failed to load RosHmlItf")
assert( UbiquitySimulDeployer:load() , 		"Failed to load UbiquitySimul")
assert( HmlMonitorSimulDeployer:load() , 	"Failed to load HmlMonitorSimul")

print("... connect components")
assert( MotorSimulDeployer:connect() , 		"Failed to connect MotorSimul" )
assert( SchedulerDeployer:connect() , 		"Failed to connect Scheduler" )
assert( Syncronizator:connect() , 			"Failed to connect Syncronizator" )
assert( JoystickDeployer:connect() , 		"Failed to connect Joystick" )
assert( RosHmlItfDeployer:connect() , 		"Failed to connect RosHmlItf" )
assert( UbiquitySimulDeployer:connect() , 	"Failed to connect UbiquitySimul" )
assert( HmlMonitorSimulDeployer:connect() , "Failed to connect HmlMonitorSimul" )


print("... start components")
assert( HmlMonitorSimulDeployer:start() , "Failed to start HmlMonitor");

print("fin déploiment arp_hml")
print("====================")
