<?php 

require("motor.php");

class Hml 
{
	var $LeftSteering;
	var $RearSteering;
	var $RightSteering;
	var $LeftDriving;
	var $RearDriving;
	var $RightDriving;
	
	function Hml()
	{
		$this->LeftSteering = new Motor("LeftSteering");
		$this->RearSteering = new Motor("RearSteering");
		$this->RightSteering = new Motor("RightSteering");
		$this->LeftDriving = new Motor("LeftDriving");
		$this->RearDriving = new Motor("RearDriving");
		$this->RightDriving = new Motor("RightDriving");
	}
	
	
	function connectBdd()
	{
		try
		{
			$pdo_options[PDO::ATTR_ERRMODE] = PDO::ERRMODE_EXCEPTION;
			$bdd = new PDO('mysql:host=localhost;dbname=ubiquity', 'ard_user', 'robotik');

			$reponse = $bdd->query('SELECT * FROM orocos_message_to_hmi');

		   	while ($donnees = $reponse->fetch())
		    	{
				$this->update($donnees);
		    	}
		    	$reponse->closeCursor(); // Termine le traitement de la requête
		}
		catch (Exception $e)
		{
			die('Erreur : ' . $e->getMessage());
		}
	}


	function update($data)
	{
		//look for the point beetween component and port name
		$point=strrpos($data['port_name'],".");
		if($point)
		{
  			$component=substr($data['port_name'],0,$point); 
			$port=substr($data['port_name'],$point-strlen($data['port_name'])+1);
			$this->updateMotor($component,$port,$data['value']);
		}
	}

	function updateMotor($name,$port,$value)
	{
		switch ($name) 
		{
		    case "LeftSteering":
			$this->LeftSteering->updatePort($port,$value);
			break;
		    case "RearSteering":
			$this->RearSteering->updatePort($port,$value);
			break;
		    case "RightSteering":
			$this->RightSteering->updatePort($port,$value);
			break;
		    case "LeftDriving":
			$this->LeftDriving->updatePort($port,$value);  	    
			break;
		    case "RearDriving":
			$this->RearDriving->updatePort($port,$value);
			break;
	    	    case "RightDriving":
			$this->RightDriving->updatePort($port,$value);
			break;
		}
	}

}

?>
