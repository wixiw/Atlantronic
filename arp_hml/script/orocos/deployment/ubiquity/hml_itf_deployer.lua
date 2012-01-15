dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")


HmlItfDeployer = ComposantDeployer:new()

function HmlItfDeployer:load()
	Deployer:loadComponent("RosHmlItf","arp_hml::UbiquityItf")
	Deployer:setActivity("RosHmlItf",0.050,10,1)

	Deployer:loadComponent("HmlMonitor","arp_hml::HmlMonitor")
	Deployer:setActivity("HmlMonitor",0.100,10,1)
end


function HmlItfDeployer:connectOneMotor(name)
	--Deployer:addPeer("RosHmlItf", name)
	--Deployer:connect("RosHmlItf.in"..name.."Position", name..".outMeasuredPosition",cp)
	--Deployer:connect("RosHmlItf.in"..name.."PositionTime", name..".outMeasuredPositionTime",cp)
	--Deployer:connect("RosHmlItf.in"..name.."Enable", name..".outDriveEnable",cp)
	--Deployer:connect("RosHmlItf.in"..name.."SpeedMeasure", name..".outComputedSpeed",cp)
	--Deployer:connect("RosHmlItf.in"..name.."Blocked", name..".outMaxTorqueTimeout",cp)
	--Deployer:connect("RosHmlItf.in"..name.."Connected", name..".outConnected",cp)

	Deployer:connect("HmlMonitor.in"..name.."Enable", name..".outDriveEnable",cp)
end


function HmlItfDeployer:addToMonitor(name)
	HmlMonitor = Deployer:getPeer("HmlMonitor")
	Deployer:addPeer("HmlMonitor", name)
	HmlMonitor:ooAddMonitoredPeer (name)
	Deployer:removePeer (name)
end

function HmlItfDeployer:addToBusMonitor(name)
	HmlMonitor = Deployer:getPeer("HmlMonitor")
	Deployer:addPeer("HmlMonitor", name)
	HmlMonitor:ooAddHmlBusMonitoredPeer(name)
	Deployer:removePeer (name)
end


function HmlItfDeployer:connect()
--connection des ports
	--HmlItfDeployer:connectOneMotor("LeftDriving")
	--HmlItfDeployer:connectOneMotor("RightDriving")
	HmlItfDeployer:connectOneMotor("RearDriving")
	--HmlItfDeployer:connectOneMotor("LeftSteering")
	--HmlItfDeployer:connectOneMotor("RightSteering")
	HmlItfDeployer:connectOneMotor("RearSteering")

	Deployer:connect("RosHmlItf.inIoStart", "WoodheadIn.outBit1",cp)
	Deployer:connect("RosHmlItf.inWoodheadInConnected", "WoodheadIn.outConnected",cp)
	Deployer:connect("RosHmlItf.inWoodheadOutConnected", "WoodheadOut.outConnected",cp)

--ajout au monitor

	HmlItfDeployer:addToBusMonitor("Can1")

	--HmlItfDeployer:addToMonitor("LeftDriving")
	--HmlItfDeployer:addToMonitor("RightDriving")
	HmlItfDeployer:addToMonitor("RearDriving")
	--HmlItfDeployer:addToMonitor("LeftSteering")
	--HmlItfDeployer:addToMonitor("RightSteering")
	HmlItfDeployer:addToMonitor("RearSteering")

	--HmlItfDeployer:addToMonitor("WoodheadIn")
	--HmlItfDeployer:addToMonitor("WoodheadOut")

	--HmlItfDeployer:addToMonitor("RosHmlItf")
end

function HmlItfDeployer:start()
	HmlMonitor = Deployer:getPeer("HmlMonitor")
	HmlMonitor:configure()

	HmlMonitor:start()

	--Definition des couples
	--TODO WLA post coupe faire mieux

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

