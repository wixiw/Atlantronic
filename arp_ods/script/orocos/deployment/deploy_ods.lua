require("rttlib")
rttlib.color=true
Deployer = rtt.getTC()
print("====================")
print("début déploiment arp_ods")

-- chargement des librairies contenants les composants d'arp_hml
Deployer:import("arp_ods");

Deployer:loadComponent("LeftTurret","arp_ods::Turret");
LeftTurret = Deployer:getPeer("LeftTurret")
LeftTurret:setPeriod (1);
LeftTurret:configure();
LeftTurret:start();

print("fin déploiment arp_ods")
print("====================")
