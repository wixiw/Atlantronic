dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")


Syncronizator = ComposantDeployer:new()
me = "Syncronizator"

function Syncronizator:load()
	Deployer:loadComponent(me,"arp_hml::Syncronizator")
	Deployer:setActivity(me,0,30,1)
end


function Syncronizator:connectMotorMeasure(measure)
	Deployer:connect(me..".inLeftDriving"..measure, 	"LeftDriving.out"..measure, cp);
	Deployer:connect(me..".inRightDriving"..measure, 	"RightDriving.out"..measure, cp);
	Deployer:connect(me..".inRearDriving"..measure, 	"RearDriving.out"..measure, cp);
	Deployer:connect(me..".inLeftSteering"..measure, 	"LeftSteering.out"..measure, cp);
	Deployer:connect(me..".inRightSteering"..measure, 	"RightSteering.out"..measure, cp);
	Deployer:connect(me..".inRearSteering"..measure, 	"RearSteering.out"..measure, cp);
end


function Syncronizator:connect()
	Syncronizator:connectMotorMeasure("Clock");
	Syncronizator:connectMotorMeasure("Position");
	Syncronizator:connectMotorMeasure("Velocity");
	Syncronizator:connectMotorMeasure("Torque");

	Deployer:connect(me..".inCanSync", 	"Can1.outClock", cp);

	Syncronizator:check(me)
end



