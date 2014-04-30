dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")


UbiquitySimulDeployer = ComposantDeployer:new()
local me = "UbiquitySimul"

function UbiquitySimulDeployer:load()
	assert( Deployer:loadComponent(me,"arp_simu::UbiquitySimul"))
	assert( Deployer:addPeer("DotGraph",me))
	assert( Deployer:addPeer("Reporting", me))
	--aperiodic as we are supposed to be weken up by a periodic timer
	assert( Deployer:setActivity(me,0.0,60,rtt.globals.ORO_SCHED_RT))
	return true
end

function UbiquitySimulDeployer:connect()
	assert( Deployer:connect(me..".inLeftDrivingSpeedCmd", 		"KinematicBase.outLeftDrivingVelocityCmd",		cp) )
	assert( Deployer:connect(me..".inRightDrivingSpeedCmd", 	"KinematicBase.outRightDrivingVelocityCmd",		cp) )
	assert( Deployer:connect(me..".inRearDrivingSpeedCmd", 		"KinematicBase.outRearDrivingVelocityCmd",		cp) )
	assert( Deployer:connect(me..".inLeftSteeringPositionCmd", 	"KinematicBase.outLeftSteeringPositionCmd",		cp) )
	assert( Deployer:connect(me..".inRightSteeringPositionCmd", "KinematicBase.outRightSteeringPositionCmd",	cp) )
	assert( Deployer:connect(me..".inRearSteeringPositionCmd", 	"KinematicBase.outRearSteeringPositionCmd",		cp) )
	
	assert( Deployer:connect(me..".inBlockMotorCmd", "RosHmlItf.outBlockRobot",cp))
	assert( Deployer:connect(me..".inParams", "UbiquityParams.outParams",cp) );
    assert( Deployer:connect(me..".inPeriod", "RealTimeClock.outPeriod",cp) );
    
	assert( UbiquitySimulDeployer:check(me) )
	return true
end



