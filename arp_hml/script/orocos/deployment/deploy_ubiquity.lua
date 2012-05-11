require("rttlib")
--require("rfsm")
--require("rfsm_rtt")

rttlib.color=true
Deployer = rtt.getTC()
print("====================")
print("début déploiment arp_hml")

-- chargement des librairies contenants les composants d'arp_hml
Deployer:import("arp_hml");

dofile("/opt/ard/arp_hml/script/orocos/deployment/ubiquity/can_deployer.lua");
dofile("/opt/ard/arp_hml/script/orocos/deployment/ubiquity/io_deployer.lua");
dofile("/opt/ard/arp_hml/script/orocos/deployment/ubiquity/joystick_deployer.lua");
dofile("/opt/ard/arp_hml/script/orocos/deployment/ubiquity/ros_hml_itf_deployer.lua");
dofile("/opt/ard/arp_hml/script/orocos/deployment/ubiquity/hml_monitor_deployer.lua");
dofile("/opt/ard/arp_hml/script/orocos/deployment/ubiquity/syncronizator_deployer.lua");
dofile("/opt/ard/arp_hml/script/orocos/deployment/ubiquity/motor_deployer.lua");
dofile("/opt/ard/arp_hml/script/orocos/deployment/ubiquity/scheduler_deployer.lua");

print("... load components")
assert( CanDeployer:load(), 		"Failed to load Can")
assert( SchedulerDeployer:load(), 	"Failed to load Scheduler")
assert( IoDeployer:load(), 			"Failed to load IoDeployer")
assert( MotorDeployer:load(), 		"Failed to load Motors")
assert( Syncronizator:load(), 		"Failed to load Syncronizator")
assert( JoystickDeployer:load(), 	"Failed to load Joystick")
assert( RosHmlItfDeployer:load(), 	"Failed to load RosHmlItf")
assert( HmlMonitorDeployer:load(), 	"Failed to load HmlMonitor")


print("... connect components")
assert( CanDeployer:connect(), 			"Failed to connect Can")
assert( SchedulerDeployer:connect() , 	"Failed to connect Scheduler")
assert( IoDeployer:connect() , 		"Failed to connect IoDeployer")
assert( MotorDeployer:connect() , 		"Failed to connect Motors")
assert( Syncronizator:connect() , 		"Failed to connect Syncronizator")
assert( JoystickDeployer:connect() , 	"Failed to connect Joystick")
assert( RosHmlItfDeployer:connect() , 	"Failed to connect RosHmlItf")
assert( HmlMonitorDeployer:connect() , "Failed to connect HmlMonitor")

print("... start components")
assert( HmlMonitorDeployer:start(), "Failed to start HmlMonitor")

print("fin déploiment arp_hml")
print("====================")
