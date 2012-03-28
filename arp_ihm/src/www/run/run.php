<!DOCTYPE Html>
<html>
<head>
<meta charset="ISO-8859-1">
<meta http-equiv="Content-type" content="text/html;charset=ISO-8859-1" />
<meta name="author" content="Advanced Robotics Design" />
<meta http-equiv="refresh" content="1;URL='run.php'" />
<meta HTTP-EQUIV="pragma" CONTENT="no-cache" />
<link rel="stylesheet" href="run.css" />
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



	<footer> </footer>
</body>
</html>
