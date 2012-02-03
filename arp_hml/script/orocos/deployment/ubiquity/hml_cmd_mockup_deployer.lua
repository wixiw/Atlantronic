dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")


HmlCmdMockupDeployer = ComposantDeployer:new()

function HmlCmdMockupDeployer:load()
	Deployer:loadComponent("HmlCmdMockup","arp_hml::HmlCmdMockup")
	Deployer:setActivity("HmlCmdMockup",0.100,10,1)
end


function HmlCmdMockupDeployer:connectOneMotor(name)
	Deployer:connect(name..".inSpeedCmd",		"HmlCmdMockup.out"..name.."SpeedCmd",cp)
	Deployer:connect(name..".inPositionCmd",	"HmlCmdMockup.out"..name.."PositionCmd",cp)
	Deployer:connect(name..".inTorqueCmd",		"HmlCmdMockup.out"..name.."TorqueCmd",cp)
end

function HmlCmdMockupDeployer:connect()
	
	Deployer:addPeer("RosHmlItf", "HmlMonitor")

--connection des ports
	--HmlCmdMockupDeployer:connectOneMotor("LeftDriving")
	--HmlCmdMockupDeployer:connectOneMotor("RightDriving")
	HmlCmdMockupDeployer:connectOneMotor("RearDriving")
	--HmlCmdMockupDeployer:connectOneMotor("LeftSteering")
	--HmlCmdMockupDeployer:connectOneMotor("RightSteering")
	HmlCmdMockupDeployer:connectOneMotor("RearSteering")

	Deployer:connect("WoodheadOut.inBit1","HmlCmdMockup.outBit01",cp)
	Deployer:connect("WoodheadOut.inBit2","HmlCmdMockup.outBit02",cp)
	Deployer:connect("WoodheadOut.inBit3","HmlCmdMockup.outBit03",cp)
	Deployer:connect("WoodheadOut.inBit4","HmlCmdMockup.outBit04",cp)
	Deployer:connect("WoodheadOut.inBit5","HmlCmdMockup.outBit05",cp)
	Deployer:connect("WoodheadOut.inBit6","HmlCmdMockup.outBit06",cp)
	Deployer:connect("WoodheadOut.inBit7","HmlCmdMockup.outBit07",cp)
	Deployer:connect("WoodheadOut.inBit8","HmlCmdMockup.outBit08",cp)
end
