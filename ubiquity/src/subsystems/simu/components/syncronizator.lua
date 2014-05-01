dofile("/opt/ard/ubiquity/src/subsystems/orocos/component_deployer_object.lua")


Syncronizator = ComposantDeployer:new()
local me = "Syncronizator"

function Syncronizator:load()
	assert( Deployer:loadComponent(me,"arp_hml::Syncronizator"))
	assert( Deployer:addPeer("DotGraph",me))
	assert( Deployer:setMasterSlaveActivity("MotionScheduler", me))
	
	return true
end


function Syncronizator:connectMotorMeasure(measure)
	assert( Deployer:connect(me..".inLeftDriving"..measure, 	"LeftDriving.out"..measure, cp))
	assert( Deployer:connect(me..".inRightDriving"..measure, 	"RightDriving.out"..measure, cp))
	assert( Deployer:connect(me..".inRearDriving"..measure, 	"RearDriving.out"..measure, cp))
	assert( Deployer:connect(me..".inLeftSteering"..measure, 	"LeftSteering.out"..measure, cp))
	assert( Deployer:connect(me..".inRightSteering"..measure, 	"RightSteering.out"..measure, cp))
	assert( Deployer:connect(me..".inRearSteering"..measure, 	"RearSteering.out"..measure, cp))
	
	return true
end


function Syncronizator:connect()
	assert( Syncronizator:connectMotorMeasure("Position"))
	assert( Syncronizator:connectMotorMeasure("Velocity"))
	assert( Syncronizator:connectMotorMeasure("Torque"))
	assert( Deployer:addPeer("Reporting", me))
	assert( Deployer:connect(me..".inCanSync", 	"RealTimeClock.outClock", cp))

	assert( Syncronizator:check(me))
	
	return true
end



