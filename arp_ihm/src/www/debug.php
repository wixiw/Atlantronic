<html>

<head>
	<meta http-equiv="refresh" content="0.5;URL='debug.php'>
	<META HTTP-EQUIV="pragma" CONTENT="no-cache">
	<title>DEBUG Ubiquity 2012</title>
</head>

<body><h1>You are on the ARD robot DEBUG page !</h1>
<p>This is the debug web page for this server.</br>
It is more or less an SQL dump</p>

<h1> ROS messages FROM robot </h1>
</p>
<?php
try
{
	$pdo_options[PDO::ATTR_ERRMODE] = PDO::ERRMODE_EXCEPTION;
	$bdd = new PDO('mysql:host=localhost;dbname=ubiquity', 'ard_user', 'robotik');

	$reponse = $bdd->query('SELECT * FROM ros_message_to_hmi');

   	while ($donnees = $reponse->fetch())
    	{
		echo "<b>";
		echo $donnees['topic_name'];
		echo "</b>=";
		echo $donnees['value'];
		echo "</br>";
    	}
    	$reponse->closeCursor(); // Termine le traitement de la requête
}
catch (Exception $e)
{
        die('Erreur : ' . $e->getMessage());
}
?>
</p>


<h1> OROCOS messages FROM robot </h1>
</p>
<?php
try
{
	$pdo_options[PDO::ATTR_ERRMODE] = PDO::ERRMODE_EXCEPTION;
	$bdd = new PDO('mysql:host=localhost;dbname=ubiquity', 'ard_user', 'robotik');

	$reponse = $bdd->query('SELECT * FROM orocos_message_to_hmi');

   	while ($donnees = $reponse->fetch())
    	{
		echo "<b>";
		echo $donnees['port_name'];
		echo "</b>=";
		echo $donnees['value'];
		echo "</br>";
    	}
    	$reponse->closeCursor(); // Termine le traitement de la requête
}
catch (Exception $e)
{
        die('Erreur : ' . $e->getMessage());
}
?>
</p>

</body></html>
