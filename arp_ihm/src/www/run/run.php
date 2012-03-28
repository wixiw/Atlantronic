<!DOCTYPE Html>
<html>
<head>
<meta charset="ISO-8859-1">
<meta http-equiv="Content-type" content="text/html;charset=ISO-8859-1" />
<meta name="author" content="Advanced Robotics Design" />
<link rel="stylesheet" href="run.css" />
<title>ARP IHM -running</title>
</head>
<body>
	<header>
		<p>Running !</p>
	</header>

	<section>
		<p>ARP is running</p>
		<?php echo "<p>CPU temp :".exec('sensors | grep Core | cut -c14-21')."</p>"; ?>
		<?php echo "<p>".exec('/opt/arp_ihm-selftest.sh')."</p>"; ?>
	</section>



	<footer> </footer>
</body>
</html>
