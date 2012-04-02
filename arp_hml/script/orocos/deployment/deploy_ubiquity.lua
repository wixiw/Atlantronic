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
dofile("/opt/ard/arp_hml/script/orocos/deployment/ubiquity/motor_deployer.lua");
dofile("/opt/ard/arp_hml/script/orocos/deployment/ubiquity/io_deployer.lua");
dofile("/opt/ard/arp_hml/script/orocos/deployment/ubiquity/joystick_deployer.lua");
dofile("/opt/ard/arp_hml/script/orocos/deployment/ubiquity/ros_hml_itf_deployer.lua");
dofile("/opt/ard/arp_hml/script/orocos/deployment/ubiquity/hml_monitor_deployer.lua");

print("... load components")
CanDeployer:load()
IoDeployer:load()
MotorDeployer:load()
JoystickDeployer:load()
RosHmlItfDeployer:load()
HmlMonitorDeployer:load()


print("... connect components")
CanDeployer:connect()
IoDeployer:connect()
MotorDeployer:connect()
JoystickDeployer:connect()
RosHmlItfDeployer:connect()
HmlMonitorDeployer:connect()

print("... start components")
HmlMonitorDeployer:start()

print("fin déploiment arp_hml")
print("====================")
