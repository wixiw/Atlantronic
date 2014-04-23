dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")


DynamixelsDeployer = ComposantDeployer:new()

local dynamixelBus 			= "DynamixelBus"

local leftCannonFinger		= "LeftCannonFinger"
local rightCannonFinger 	= "RightCannonFinger"
local leftCannonStocker 	= "LeftCannonStocker"
local rightCannonStocker 	= "RightCannonStocker"

local leftFinger 			= "LeftFinger"
local rightFinger 			= "RightFinger"

local armSlider 			= "ArmSlider"
local armShoulder 			= "ArmShoulder"
local armShoulderElbow 		= "ArmShoulderElbow"
local armWristElbow 		= "ArmWristElbow"
local armWrist 				= "ArmWrist"

function DynamixelsDeployer:loadCpnt(name, type)
	assert( Deployer:loadComponent(name,type))
	assert( Deployer:addPeer("DotGraph",name))
	assert( Deployer:setMasterSlaveActivity("Discovery", name))
end

function DynamixelsDeployer:load()
	DynamixelsDeployer:loadCpnt(	dynamixelBus, 			"arp_stm32::DynamixelBus")
	
	DynamixelsDeployer:loadCpnt(	leftCannonFinger, 		"arp_stm32::Dynamixel")
	DynamixelsDeployer:loadCpnt(	rightCannonFinger, 		"arp_stm32::Dynamixel")
	DynamixelsDeployer:loadCpnt(	leftCannonStocker, 		"arp_stm32::Dynamixel")
	DynamixelsDeployer:loadCpnt(	rightCannonStocker, 	"arp_stm32::Dynamixel")
	
	DynamixelsDeployer:loadCpnt(	leftFinger, 			"arp_stm32::Dynamixel")
	DynamixelsDeployer:loadCpnt(	rightFinger, 			"arp_stm32::Dynamixel")
	
	DynamixelsDeployer:loadCpnt(	armSlider, 				"arp_stm32::Dynamixel")
	DynamixelsDeployer:loadCpnt(	armShoulder, 			"arp_stm32::Dynamixel")
	DynamixelsDeployer:loadCpnt(	armShoulderElbow, 		"arp_stm32::Dynamixel")
	DynamixelsDeployer:loadCpnt(	armWristElbow, 			"arp_stm32::Dynamixel")
	DynamixelsDeployer:loadCpnt(	armWrist,				"arp_stm32::Dynamixel")
	return true
end

function DynamixelsDeployer:connectDynamixel(name)
	assert( Deployer:stream(name..".outTargetReached",	ros:topic("/Ubiquity/"..name.."/target_reached")))
	assert( Deployer:stream(name..".outPosition",		ros:topic("/Ubiquity/"..name.."/position")))
	assert( Deployer:stream(name..".outStucked",		ros:topic("/Ubiquity/"..name.."/stucked")))
end

function DynamixelsDeployer:connect()
	assert( Deployer:addPeer("Reporting", dynamixelBus) )
	
	DynamixelsDeployer:connectDynamixel(leftCannonFinger)
	DynamixelsDeployer:connectDynamixel(rightCannonFinger)
	DynamixelsDeployer:connectDynamixel(leftCannonStocker)
	DynamixelsDeployer:connectDynamixel(rightCannonStocker)
	
	DynamixelsDeployer:connectDynamixel(leftFinger)
	DynamixelsDeployer:connectDynamixel(rightFinger)
	
	DynamixelsDeployer:connectDynamixel(armSlider)
	DynamixelsDeployer:connectDynamixel(armShoulder)
	DynamixelsDeployer:connectDynamixel(armShoulderElbow)
	DynamixelsDeployer:connectDynamixel(armWristElbow)
	DynamixelsDeployer:connectDynamixel(armWrist)

	return true
end

function DynamixelsDeployer:start()
	DynamixelBus 		= assert(Deployer:getPeer(dynamixelBus))
	
	LeftCannonFinger 	= assert(Deployer:getPeer(leftCannonFinger))
	RightCannonFinger 	= assert(Deployer:getPeer(rightCannonFinger))
	LeftCannonStocker 	= assert(Deployer:getPeer(leftCannonStocker))
	RightCannonStocker 	= assert(Deployer:getPeer(rightCannonStocker))
	
	LeftFinger 			= assert(Deployer:getPeer(leftFinger))
	RightFinger 		= assert(Deployer:getPeer(rightFinger))
	
	ArmSlider	 		= assert(Deployer:getPeer(armSlider))
	ArmShoulder 		= assert(Deployer:getPeer(armShoulder))
	ArmShoulderElbow 	= assert(Deployer:getPeer(armShoulderElbow))
	ArmWristElbow 		= assert(Deployer:getPeer(armWristElbow))
	ArmWrist 			= assert(Deployer:getPeer(armWrist))
	
	assert(DynamixelBus:configure())
	
	assert(LeftCannonFinger:configure())
	assert(RightCannonFinger:configure())
	assert(LeftCannonStocker:configure())
	assert(RightCannonStocker:configure())
	
	assert(LeftFinger:configure())
	assert(RightFinger:configure())
	
	assert(ArmSlider:configure())
	assert(ArmShoulder:configure())
	assert(ArmShoulderElbow:configure())
	assert(ArmWristElbow:configure())
	assert(ArmWrist:configure())
	
	
	
	
	assert(DynamixelBus:start())
	
	assert(LeftCannonFinger:start())
	assert(RightCannonFinger:start())
	assert(LeftCannonStocker:start())
	assert(RightCannonStocker:start())
	
	assert(LeftFinger:start())
	assert(RightFinger:start())
	
	assert(ArmSlider:start())
	assert(ArmShoulder:start())
	assert(ArmShoulderElbow:start())
	assert(ArmWristElbow:start())
	assert(ArmWrist:start())
	
	return true
end


