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
		<a href="run/run.php?action=dev"><div class="bouton">Dev</div></a>
		<a href="run/run.php?action=selftest"><div class="bouton">Self Test</div></a>
		<a href="run/run.php?action=manual"><div class="bouton">Manual</div></a>
		<a href="run/run.php?action=matchA"><div class="bouton">Match A</div></a>
		<a href="run/run.php?action=matchB"><div class="bouton">Match B</div></a>
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
