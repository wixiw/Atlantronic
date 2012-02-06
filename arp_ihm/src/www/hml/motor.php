<?php 

class Motor 
{
	var $name;
	var $connected = false;
	var $powered = false;
	var $mode;
	var $speed;
	var $position;
	var $torque;	

	function Motor($name)
	{
		$this->name = $name;
	}

	function show()
	{
		echo "<p><b>".$this->name."</b></br>";
		if( $this->connected )
		{
			echo '<span style="background-color:green">connected</span></br>';
		}
		else
		{
			echo '<span style="background-color:red">disconnected</span></br>';
		}
		if( $this->powered )
		{
			echo '<span style="background-color:yellow">powered</span></br>';
		}
		else
		{
			echo '<span style="background-color:grey">freewheel</span></br>';
		}
		echo 'mode = '.$this->mode.'</br>';
		echo 'speed = '.$this->speed.'</br>';
		echo 'position = '.$this->position.'</br>';
		echo 'torque = '.$this->torque;
		echo "</p>";
	}

	function updatePort($portName, $value)
	{
		switch ($portName) 
		{
		    case "outConnected":
			$this->connected = $value;
			break;
		    case "outDriveEnable":
			$this->powered = $value;
			break;
		    case "outCurrentOperationMode":
			$this->mode = $value;
			break;
		    case "outComputedSpeed":
			$this->speed = $value;	    
			break;
		    case "outMeasuredPosition":
			$this->position = $value;
			break;
	    	    case "outMeasuredTorque":
			$this->torque = $value;
			break;
		}
	}
}
?>
