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
dofile("script/orocos/deployment/ubiquity/hml_itf_deployer.lua");

print("... load components")
CanDeployer:load()
--print("... ...motor")
MotorDeployer:load()
--print("... ...io")
IoDeployer:load()
--print("... ...joystick")
JoystickDeployer:load()
--print("... ...hmi")
HmlItfDeployer:load()


print("... connect components")
CanDeployer:connect()
MotorDeployer:connect()
IoDeployer:connect()
JoystickDeployer:connect()
HmlItfDeployer:connect()

print("... start components")
JoystickDeployer:start()
HmlItfDeployer:start()

print("fin déploiment arp_hml")
print("====================")
