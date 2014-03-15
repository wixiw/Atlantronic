print("====================")
print("début déploiment arp_ihm")

Deployer:import("arp_ihm");
Deployer:loadComponent("OrocosSqlBridge","arp_ihm::SqlBridge");
assert( Deployer:addPeer("DotGraph","OrocosSqlBridge"))
OrocosSqlBridge = Deployer:getPeer("OrocosSqlBridge")
OrocosSqlBridge:setPeriod (1);
OrocosSqlBridge:configure();
OrocosSqlBridge:start();

print("====================")
