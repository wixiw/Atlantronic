-- Ce fichier est chargé par le deployer de arp_master.
-- La raison de ce choix est qu'il contient en général des fils vers un peu partout donc il vaut mieux être sur le 
-- système complet pour eviter aux sous partie de faire des erreurs de déploiement
-- il serait évidemment possible de scinder ce fichier pour disséminer dans chaque package la partie qui lui revient
-- et de faire ici appel à ces suos fichiers, mais pour des raisons pratiques
-- on préférera faire tout ici pour ne pas passer sa vie à se balader partout.

Telemetry = 
{
	name="noname"
}

-- Trace le position et la vitesse du robot en sortie du composant Localizator
-- [0] : x			[5] : vx
-- [1] : y			[6] : vy
-- [2] : theta		[7] : vtheta
-- [3] : date		[8] : date
-- [4] : cov		
function Telemetry:reportRobotState()
	print("reporting pose ")
	Reporting=Deployer:getPeer("Reporting")
	Reporting:reportPort("Localizator","outPose")
	Reporting:reportPort("Localizator","outTwist")
end

-- Trace l'état des tourelles en sortie du composant Syncronizator
-- [0] : steering.left.pose			[3] : steering.right.pose		[6] : steering.rear.pose
-- [1] : steering.left.velocity		[4] : steering.right.velocity	[7] : steering.rear.velocity
-- [2] : steering.left.torque		[5] : steering.right.torque		[8] : steering.rear.torque
-- [9]  : driving.left.pose			[12] : driving.right.pose		[15] : driving.rear.pose
-- [10] : driving.left.velocity		[13] : driving.right.velocity	[16] : driving.rear.velocity
-- [11] : driving.left.torque		[14] : driving.right.torque		[17] : driving.rear.torque
function Telemetry:reportMotorState()
	print("reporting configuration")
	Reporting=Deployer:getPeer("Reporting")
	Reporting:reportPort("Syncronizator","outMotorMeasures")
end

-- Trace la commande envoyee au robot
-- [0] : vx		
-- [1] : vy			
-- [2] : vtheta		
function Telemetry:reportRobotCmd()
	print("reporting pose ")
	Reporting=Deployer:getPeer("Reporting")
	Reporting:reportPort("MotionControl","outTwistCmd")
	Reporting:reportPort("KinematicBase","outLeftSteeringPositionCmd")
	Reporting:reportPort("KinematicBase","outRightSteeringPositionCmd")
	Reporting:reportPort("KinematicBase","outRearSteeringPositionCmd")
	Reporting:reportPort("KinematicBase","outLeftDrivingVelocityCmd")
	Reporting:reportPort("KinematicBase","outRightDrivingVelocityCmd")
	Reporting:reportPort("KinematicBase","outRearDrivingVelocityCmd")
end


-- Enregistre les ports à plotter et lances l'enregistrement
-- pour voir les résultats :
-- _ si vous avez lancé avec roslaunch, il y a un fichier /opt/ros/reports.dat
-- _ si vous avez lancé avec rosrin un deployer, le fichier reports.dat est dans le path du sheel qui a lancé le programme
-- Le fichier peut être lu à la main ou avec kst. La première colonne correspond toujours à un timestamp. 
-- On spécifie avec "-x?" un axe d'abscisse, "-y?" un axe d'ordonnée, et on peut les enchainer
-- Ex : trace en fonction du temps : "kst -x1 -y2 -y3 -y4 /opt/ros/reports.dat"
-- Ex : trace y = f(x) : "kst -x2 -y3 /opt/ros/reports.dat"
function Telemetry:report()
	print("====================")
	print("début déploiment telemetry")
	Telemetry:reportRobotState()
	Telemetry:reportMotorState()
	Telemetry:reportRobotCmd()
	Reporting=Deployer:getPeer("Reporting")
	Reporting:start()
	print("====================")
end
