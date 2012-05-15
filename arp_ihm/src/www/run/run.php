<?php 
	switch ($_GET["action"]) {
		case "homologation":
			echo exec('sudo /etc/init.d/ard_ihm_launcher.sh start arp_master homologation_galaxy.launch');
			break;
		case "qualif":
			echo exec('sudo /etc/init.d/ard_ihm_launcher.sh start arp_master homologation_galaxy.launch');
			break;
		case "matchA":
			echo exec('sudo /etc/init.d/ard_ihm_launcher.sh start arp_master homologation_galaxy.launch');
			break;
		case "matchB":
			echo exec('sudo /etc/init.d/ard_ihm_launcher.sh start arp_master homologation_galaxy.launch');
			break;
		case "manual":
				echo exec('sudo /etc/init.d/ard_ihm_launcher.sh start arp_master manual_galaxy.launch');
				break;
		case "kill":
			echo exec('sudo /etc/init.d/ard_ihm_launcher.sh stop');
			break;
			
		default:
			;
		break;
	}
?>

<!DOCTYPE Html>
<html>

<head>
<meta charset="ISO-8859-1">
<meta http-equiv="Content-type" content="text/html;charset=ISO-8859-1" />
<meta name="author" content="Advanced Robotics Design" />

<link rel="stylesheet" href="run.css" />
<?php if ( file_exists("/var/run/ard_ihm_launcher.pid") ) { ?>
		<meta http-equiv="refresh" content="12;URL='run.php'" />
		<meta HTTP-EQUIV="pragma" CONTENT="no-cache" />
<?php }	else { ?>
		<meta http-equiv="refresh" content="0;URL='../index.php'" />
		<meta HTTP-EQUIV="pragma" CONTENT="no-cache" />
<?php } ?>

<title>ARP IHM -running</title>
</head>


<body>
	<header>
		<p>Running <?php echo $_GET["action"] ?> !</p>
	</header>

	<footer>
		<div class=button>
			<p><a href="run.php?action=kill">Cancel</a></p>
		</div>
	</footer>
</body>
</html>
