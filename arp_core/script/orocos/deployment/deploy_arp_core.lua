require "rttlib"
rttlib.color=true
Deployer = rtt.getTC()
print("====================")
print("début déploiment arp_core")

-- chargement des librairies contenants les composants d'arp_core
Deployer:import("arp_core");

-- chargement du composant de reporting
Deployer:loadComponent("Reporting","OCL::FileReporting")
Reporting = Deployer:getPeer("Reporting")
Reporting:setPeriod (0.050)

print("fin déploiment arp_core")
print("====================")
