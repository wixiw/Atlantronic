<?php 
if ($_GET["action"] === "do")
{
	switch ($_GET["type"]) {
// 		case "Homologation":
// 			exec('sudo /etc/init.d/ard_ihm_launcher.sh start arp_master homologation_galaxy.launch');
// 			break;
// 		case "Qualif":
// 			exec('sudo /etc/init.d/ard_ihm_launcher.sh start arp_master homologation_galaxy.launch');
// 			break;
// 		case "MatchA":
// 			exec('sudo /etc/init.d/ard_ihm_launcher.sh start arp_master homologation_galaxy.launch');
// 			break;
// 		case "MatchB":
// 			exec('sudo /etc/init.d/ard_ihm_launcher.sh start arp_master homologation_galaxy.launch');
// 			break;
		case "Manual":
			exec('sudo /etc/init.d/ard_ihm_launcher.sh start arp_master manual_galaxy.launch');
			break;
		case "Kill":
			echo exec('sudo /etc/init.d/ard_ihm_launcher.sh stop');
			break;

		default:
			;
			break;
	}
}
?>

<?php
function read_file($file, $lines) {
	//global $fsize;
	$handle = fopen($file, "r");
	$linecounter = $lines;
	$pos = -2;
	$beginning = false;
	$text = array();
	while ($linecounter > 0) {
		$t = " ";
		while ($t != "\n") {
			if(fseek($handle, $pos, SEEK_END) == -1) {
				$beginning = true;
				break;
			}
			$t = fgetc($handle);
			$pos --;
		}
		$linecounter --;
		if ($beginning) {
			rewind($handle);
		}
		$text[$lines-$linecounter-1] = fgets($handle);
		if ($beginning) break;
	}
	fclose ($handle);
	return array_reverse($text);
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
<meta http-equiv="refresh"
	content="1;URL='run.php?type=<?php echo $_GET["type"] ?>'" />
<meta HTTP-EQUIV="pragma" CONTENT="no-cache" />
<?php }	else { ?>
<meta http-equiv="refresh" content="0;URL='../index.php'" />
<meta HTTP-EQUIV="pragma" CONTENT="no-cache" />
<?php } ?>

<title>ARP IHM -running</title>
</head>


<body>
	<header>
	<logOrocosError>
		<?php
		if( file_exists("/tmp/ARP_FAILED") )
		{
			echo "<strong>"."/tmp/orocos.log :"."</strong><br/>";
		
			$lines = read_file("/tmp/orocos.log", 25);
			foreach ($lines as $line)
			{
				if( strpos($line, "ERROR") )
				{
					echo "<strong>$line </strong> <br/>";
				}
				else
				{
				echo "$line <br/>";
				}
			}
		}
		?>
	</logOrocosError>
	<logOrocos>
		<?php
		if( file_exists("/tmp/ARP_LOADING") )
		{
			echo "<strong>"."/tmp/orocos.log :"."</strong><br/>";
		
			$lines = read_file("/tmp/orocos.log", 25);
			foreach ($lines as $line)
			{
				echo "$line <br/>";
			}
		}
		?>
	</logOrocos>
	</header>

	<footer>
		<span class=state> <?php  echo $_GET["type"] ?> : <?php system('sh /opt/ard/arp_core/script/linux/get_arp_state.sh') ?>
		</span> 
		<span class=floatright>
			<span class=button> 
				<a href="run.php?type=Kill&action=do">Cancel</a>
			</span>
		</span>
	</footer>
</body>
</html>
