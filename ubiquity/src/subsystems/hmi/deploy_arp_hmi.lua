print("====================")
print("début déploiment arp_hmi")

Deployer:import("arp_hmi");
Deployer:loadComponent("OrocosSqlBridge","arp_hmi::SqlBridge");
assert( Deployer:addPeer("DotGraph","OrocosSqlBridge"))
OrocosSqlBridge = Deployer:getPeer("OrocosSqlBridge")
OrocosSqlBridge:setPeriod (1);
OrocosSqlBridge:configure();
OrocosSqlBridge:start();

print("====================")
