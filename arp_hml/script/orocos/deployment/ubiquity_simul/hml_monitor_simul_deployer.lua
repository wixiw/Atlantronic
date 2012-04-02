dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")
dofile("/opt/ard/arp_hml/script/orocos/deployment/ubiquity/hml_monitor_deployer.lua")

HmlMonitorSimulDeployer = HmlMonitorDeployer:new()
local me = "HmlMonitor"

function HmlMonitorSimulDeployer:connect()

--connection des ports
	--[[
	HmlMonitorDeployer:connectOneMotor("LeftDriving")
	HmlMonitorDeployer:connectOneMotor("RightDriving")
	HmlMonitorDeployer:connectOneMotor("RearDriving")
	HmlMonitorDeployer:connectOneMotor("LeftSteering")
	HmlMonitorDeployer:connectOneMotor("RightSteering")
	HmlMonitorDeployer:connectOneMotor("RearSteering")

	Deployer:connect("HmlMonitor.inWoodheadInConnected", "WoodheadIn.outConnected",cp)
	Deployer:connect("HmlMonitor.inWoodheadOutConnected", "WoodheadOut.outConnected",cp)
	]]

--ajout au monitor

--[[
	HmlMonitorDeployer:addToBusMonitor("Can1")

	HmlMonitorDeployer:addToMonitor("LeftDriving")
	HmlMonitorDeployer:addToMonitor("RightDriving")
	HmlMonitorDeployer:addToMonitor("RearDriving")
	HmlMonitorDeployer:addToMonitor("LeftSteering")
	HmlMonitorDeployer:addToMonitor("RightSteering")
	HmlMonitorDeployer:addToMonitor("RearSteering")

	HmlMonitorDeployer:addToMonitor("WoodheadIn")
	HmlMonitorDeployer:addToMonitor("WoodheadOut")
]]
	HmlMonitorDeployer:addToMonitor("Joystick")
	HmlMonitorDeployer:addToMonitor("RosHmlItf")

	--HmlMonitorDeployer:registerToSql();
	
	HmlMonitorDeployer:check("HmlMonitor")
end


function HmlMonitorDeployer:start()
	HmlMonitor = Deployer:getPeer(me)
	HmlMonitor:configure()
	HmlMonitor:start()
end