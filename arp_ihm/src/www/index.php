<!DOCTYPE Html>
<html>
<head>
<meta charset="ISO-8859-1">
<meta http-equiv="Content-type" content="text/html;charset=ISO-8859-1" />
<meta name="author" content="Advanced Robotics Design" />
<meta http-equiv="refresh" content="12;URL='index.php'" />
<meta HTTP-EQUIV="pragma" CONTENT="no-cache" />
<link rel="stylesheet" href="ard.css" />
<title>ARP IHM</title>
</head>
<body>
	<nav>
		<a href="run/run.php?type=Homologation&action=do"><div class="bouton">Homolog</div></a>
		<a href="run/run.php?type=Qualif&action=do"><div class="bouton">Qualif</div></a>
		<a href="run/run.php?type=MatchA&action=do"><div class="bouton">Match A</div></a>
		<a href="run/run.php?type=MatchB&action=do"><div class="bouton">Match B</div></a>
		<a href="run/run.php?type=Manual&action=do"><div class="bouton">Manual</div></a>
	</nav>
	
	<header>
		<?php echo "<p>CPU temp :".exec('sensors | grep Core | cut -c14-21')."</p>"; ?>
		<p></p>
		<?php echo "<p>".exec('uname -r')."</p>"; ?>
		<?php echo "<p>".exec('cat /opt/ros/ard-version')."</p>"; ?>
		<?php echo "<p>".exec('/opt/ros_addons/orocos_toolchain/ocl/bin/deployer-gnulinux --version')."</p>"; ?>
		
	</header>
	



	<footer>
	<a href="hml/index.php">HML debug screen</a>-------------------<a href="halt.php">Halt PC</a>
	</footer>
</body>
</html>
