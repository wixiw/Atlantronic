dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")


HmlItfDeployer = ComposantDeployer:new()

function HmlItfDeployer:load()
	Deployer:loadComponent("Hml","arp_hml::UbiquityItf")
	Deployer:setActivity("Hml",0.050,0,1)
end

function HmlItfDeployer:connectOneMotor(name)
	Deployer:connect("Hml.in"..name.."Position", name..".outMeasuredPosition",cp)
	Deployer:connect("Hml.in"..name.."PositionTime", name..".outMeasuredPositionTime",cp)
	Deployer:connect("Hml.in"..name.."Enable", name..".outDriveEnable",cp)
	Deployer:connect("Hml.in"..name.."SpeedMeasure", name..".outComputedSpeed",cp)
	Deployer:connect("Hml.in"..name.."Blocked", name..".outMaxTorqueTimeout",cp)
	Deployer:connect("Hml.in"..name.."Connected", name..".outConnected",cp)
end

function HmlItfDeployer:connect()
	Deployer:addPeer("Reporting", "Can1")

	HmlItfDeployer:connectOneMotor("LeftDriving")
	HmlItfDeployer:connectOneMotor("RightDriving")
	HmlItfDeployer:connectOneMotor("RearDriving")
	HmlItfDeployer:connectOneMotor("LeftSteering")
	HmlItfDeployer:connectOneMotor("RightSteering")
	HmlItfDeployer:connectOneMotor("RearSteering")

	Deployer:connect("Hml.inIoStart", "WoodheadIn.outBit1",cp)

	Deployer:connect("Hml.inWoodheadIConnected", "WoodheadIn.outConnected",cp)
	Deployer:connect("Hml.inWoodheadOConnected", "WoodheadOut.outConnected",cp)
end

function HmlItfDeployer:start()
	Hml = Deployer:getPeer("Hml")
	Hml:configure()
	Hml:start()
end

