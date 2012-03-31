<?php 
	switch ($_GET["action"]) {
		case "dev":
			break;
		case "selftest":
			echo exec('sudo /etc/init.d/ard_ihm_launcher.sh start arp_hml ubiquity.launch');
			break;
		case "manual":
			break;
		case "matchA":
			break;
		case "matchB":
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
		<meta http-equiv="refresh" content="1;URL='run.php'" />
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

	<section>
		<p>ARP is running</p>
		<?php echo "<p>CPU temp :".exec('sensors | grep Core | cut -c14-21')."</p>"; ?>

	</section>



	<footer>
		<div class=button>
			<p><a href="run.php?action=kill">Cancel</a></p>
		</div>
	</footer>
</body>
</html>
