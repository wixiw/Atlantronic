<?php
	require("hml.php");
	$Hml = new Hml();
	$Hml->connectBdd();
?>

<html>

<head>
	<meta http-equiv="refresh" content="2;URL='index.php'>
	<META HTTP-EQUIV="pragma" CONTENT="no-cache">
	<title>HML Ubiquity 2012</title>
</head>

<body><h1>You are on the HML view of Ubiquity !</h1>
<p>This is the HML working view</br>
It contains all hardware related information<br/>
<br/>
<img src="../ressource/hml/HML_view.jpg"/>
</p>

<table style="border:1px solid black">
	<tr style="border:1px solid black">
		<td style="border:1px solid black">
		<?php 
			$Hml->LeftSteering->show();
		?>
		</td>
		<td style="border:1px solid black">
		<?php 
			$Hml->RearSteering->show();
		?>		
		</td>
		<td style="border:1px solid black">
		<?php 
			$Hml->RightSteering->show();
		?>	
		</td>
	</tr>
	<tr style="border:1px solid black">
		<td style="border:1px solid black">
		<?php 
			$Hml->LeftDriving->show();
		?>	
		</td>
		<td style="border:1px solid black">
		<?php 
			$Hml->RearDriving->show();
		?>		
		</td>
		<td style="border:1px solid black">
		<?php 
			$Hml->RightDriving->show(); 
		?>	
		</td>
	</tr>
</table>
</body></html>
