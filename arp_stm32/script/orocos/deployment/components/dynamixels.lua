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
	return true
end

function DynamixelsDeployer:connect()
	assert( Deployer:addPeer("Reporting", dynamixelBus) )
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
	ArmSlider 			= assert(Deployer:getPeer(armSlider))
	
	assert(DynamixelBus:configure())
	assert(LeftCannonFinger:configure())
	assert(RightCannonFinger:configure())
	assert(LeftCannonStocker:configure())
	assert(RightCannonStocker:configure())
	assert(LeftFinger:configure())
	assert(RightFinger:configure())
	assert(ArmSlider:configure())
	
	assert(DynamixelBus:start())
	assert(LeftCannonFinger:start())
	assert(RightCannonFinger:start())
	assert(LeftCannonStocker:start())
	assert(RightCannonStocker:start())
	assert(LeftFinger:start())
	assert(RightFinger:start())
	assert(ArmSlider:start())
	
	return true
end


