	assert(Deployer:connect("LeftDriving.inSpeedCmd",			me..".outLeftDrivingVelocityCmd",cp))
	assert(Deployer:connect("RightDriving.inSpeedCmd",			me..".outRightDrivingVelocityCmd",cp))
	assert(Deployer:connect("RearDriving.inSpeedCmd",			me..".outRearDrivingVelocityCmd",cp))
	assert(Deployer:connect("LeftSteering.inPositionCmd",		me..".outLeftSteeringPositionCmd",cp))
	assert(Deployer:connect("RightSteering.inPositionCmd",		me..".outRightSteeringPositionCmd",cp))
	assert(Deployer:connect("RearSteering.inPositionCmd",		me..".outRearSteeringPositionCmd",cp))