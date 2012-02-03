require("rttlib")
rttlib.color=true
Deployer = rtt.getTC()
print("====================")
print("début déploiment arp_hml")

-- chargement des librairies contenants les composants d'arp_hml
Deployer:import("arp_hml");



dofile("script/orocos/deployment/ubiquity/can_deployer.lua");
dofile("script/orocos/deployment/ubiquity/motor_deployer.lua");
dofile("script/orocos/deployment/ubiquity/io_deployer.lua");
dofile("script/orocos/deployment/ubiquity/joystick_deployer.lua");
dofile("script/orocos/deployment/ubiquity/ros_hml_itf_deployer.lua");
dofile("script/orocos/deployment/ubiquity/hml_monitor_deployer.lua");

print("... load components")
CanDeployer:load()
MotorDeployer:load()
IoDeployer:load()
JoystickDeployer:load()
RosHmlItfDeployer:load()
HmlMonitorDeployer:load()


print("... connect components")
CanDeployer:connect()
MotorDeployer:connect()
IoDeployer:connect()
JoystickDeployer:connect()
RosHmlItfDeployer:connect()
HmlMonitorDeployer:connect()

print("... start components")
JoystickDeployer:start()
HmlMonitorDeployer:start()

print("fin déploiment arp_hml")
print("====================")
