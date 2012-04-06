dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")
dofile("/opt/ard/arp_hml/script/orocos/deployment/ubiquity/hml_monitor_deployer.lua")

HmlMonitorSimulDeployer = HmlMonitorDeployer:new()
local me = "HmlMonitor"

function HmlMonitorSimulDeployer:connect()

--connection des ports
	HmlMonitorSimulDeployer:connectOneMotor("LeftDriving")
	HmlMonitorSimulDeployer:connectOneMotor("RightDriving")
	HmlMonitorSimulDeployer:connectOneMotor("RearDriving")
	HmlMonitorSimulDeployer:connectOneMotor("LeftSteering")
	HmlMonitorSimulDeployer:connectOneMotor("RightSteering")
	HmlMonitorSimulDeployer:connectOneMotor("RearSteering")
 --[[
	Deployer:connect("HmlMonitor.inWoodheadInConnected", "WoodheadIn.outConnected",cp)
	Deployer:connect("HmlMonitor.inWoodheadOutConnected", "WoodheadOut.outConnected",cp)
	]]

--ajout au monitor
	HmlMonitorSimulDeployer:addToMonitor("Joystick")
	HmlMonitorSimulDeployer:addToMonitor("Can1")
	HmlMonitorDeployer:addToMonitor("LeftDriving")
	HmlMonitorDeployer:addToMonitor("RightDriving")
	HmlMonitorDeployer:addToMonitor("RearDriving")
	HmlMonitorDeployer:addToMonitor("LeftSteering")
	HmlMonitorDeployer:addToMonitor("RightSteering")
	HmlMonitorDeployer:addToMonitor("RearSteering")
	HmlMonitorSimulDeployer:addToMonitor("RosHmlItf")
	HmlMonitorSimulDeployer:addToMonitor("UbiquitySimul")
	HmlMonitorSimulDeployer:addToMonitor("Syncronizator")


	--HmlMonitorDeployer:registerToSql();
	HmlMonitorSimulDeployer:check("HmlMonitor")
end


function HmlMonitorSimulDeployer:start()
	HmlMonitor = Deployer:getPeer(me)
	HmlMonitor:configure()
	HmlMonitor:start()
end