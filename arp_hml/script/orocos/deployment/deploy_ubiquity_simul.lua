require("rttlib")
rttlib.color=true
Deployer = rtt.getTC()
print("====================")
print("début déploiment arp_hml")

-- chargement des librairies contenants les composants d'arp_hml
Deployer:import("arp_hml");

-- chargement de l'interface HML
Deployer:loadComponent("Hml","arp_hml::UbiquitySimul");
Hml = Deployer:getPeer("Hml")
Deployer:setActivity("Hml",0.050,0,1);
Hml:configure();
Hml:start();


print("fin déploiment arp_hml")
print("====================")
