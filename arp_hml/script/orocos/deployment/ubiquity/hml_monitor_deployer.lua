dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")


HmlMonitorDeployer = ComposantDeployer:new()

function HmlMonitorDeployer:load()
	Deployer:loadComponent("HmlMonitor","arp_hml::HmlMonitor")
	Deployer:setActivity("HmlMonitor",0.100,10,1)
end


function HmlMonitorDeployer:connectOneMotor(name)
	Deployer:connect("HmlMonitor.in"..name.."Enable", name..".outDriveEnable",cp)
	Deployer:connect("HmlMonitor.in"..name.."Connected", name..".outConnected",cp)
end


function HmlMonitorDeployer:addToMonitor(name)
	HmlMonitor = Deployer:getPeer("HmlMonitor")
	Deployer:addPeer("HmlMonitor", name)
	HmlMonitor:ooAddMonitoredPeer (name)
	Deployer:removePeer (name)
end

function HmlMonitorDeployer:addToBusMonitor(name)
	HmlMonitor = Deployer:getPeer("HmlMonitor")
	Deployer:addPeer("HmlMonitor", name)
	HmlMonitor:ooAddHmlBusMonitoredPeer(name)
	Deployer:removePeer (name)
end


function HmlMonitorDeployer:connect()

--connection des ports
	--HmlMonitorDeployer:connectOneMotor("LeftDriving")
	--HmlMonitorDeployer:connectOneMotor("RightDriving")
	HmlMonitorDeployer:connectOneMotor("RearDriving")
	--HmlMonitorDeployer:connectOneMotor("LeftSteering")
	--HmlMonitorDeployer:connectOneMotor("RightSteering")
	HmlMonitorDeployer:connectOneMotor("RearSteering")

	Deployer:connect("HmlMonitor.inWoodheadInConnected", "WoodheadIn.outConnected",cp)
	Deployer:connect("HmlMonitor.inWoodheadOutConnected", "WoodheadOut.outConnected",cp)

--ajout au monitor

	HmlMonitorDeployer:addToBusMonitor("Can1")

	--HmlMonitorDeployer:addToMonitor("LeftDriving")
	--HmlMonitorDeployer:addToMonitor("RightDriving")
	HmlMonitorDeployer:addToMonitor("RearDriving")
	--HmlMonitorDeployer:addToMonitor("LeftSteering")
	--HmlMonitorDeployer:addToMonitor("RightSteering")
	HmlMonitorDeployer:addToMonitor("RearSteering")

	--HmlMonitorDeployer:addToMonitor("WoodheadIn")
	--HmlMonitorDeployer:addToMonitor("WoodheadOut")

	HmlMonitorDeployer:addToMonitor("RosHmlItf")
end

function HmlMonitorDeployer:start()
	HmlMonitor = Deployer:getPeer("HmlMonitor")
	HmlMonitor:configure()

	HmlMonitor:start()

	--LeftDriving = HmlMonitor:getPeer("LeftDriving")
	--RightDriving = HmlMonitor:getPeer("RightDriving")
	RearDriving = HmlMonitor:getPeer("RearDriving")
	--LeftSteering = HmlMonitor:getPeer("LeftSteering")
	--RightSteering = HmlMonitor:getPeer("RightSteering")
	RearSteering = HmlMonitor:getPeer("RearSteering")

	RearDriving:ooSetOperationMode("other");
	RearSteering:ooSetOperationMode("other");

	RearDriving:ooFaulhaberCmd(0x81,RearDriving:getProperty("propMaximalTorque"):get()*1000); 
	RearSteering:ooFaulhaberCmd(0x81,RearSteering:getProperty("propMaximalTorque"):get()*1000);

	RearDriving:ooSleep(1);

	RearDriving:ooSetOperationMode("speed");
	RearSteering:ooSetOperationMode("speed");

end

