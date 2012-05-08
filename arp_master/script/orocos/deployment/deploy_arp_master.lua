
require("rttlib")
--require("rfsm")
--require("rfsm_rtt")

rttlib.color=true
Deployer = rtt.getTC()
print("====================")
print("début déploiment arp_master")

-- chargement des librairies contenants les composants d'arp_hml
Deployer:import("arp_master");

dofile("/opt/ard/arp_master/script/orocos/deployment/lua/telemetry.lua");
dofile("/opt/ard/arp_master/script/orocos/deployment/lua/last_component.lua");

--a activer pour avoir des traces dans reports.dat (soit dans le répertoire courant soit dans /opt/ros)
Telemetry:report()

LastComponentDeployer:load();
LastComponentDeployer:connect();

Scheduler = Deployer:getPeer("MotionScheduler");
Scheduler:start();

LastComponentDeployer:start();

print("fin déploiment arp_master")
print("====================")
