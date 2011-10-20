<!DOCTYPE Html>
<Html>
	<Head>
		<Meta charset="ISO-8859-1">
		<Meta http-equiv="Content-type" content="text/html;charset=ISO-8859-1" />
		<Meta name="author" content="Julien Beauduffe" />
		<Meta name="geo.placename" content="Rouen, Haute-Normandie, France" />
		<Title>R2D2</Title>
		<Style type="text/css">
		html {
			height : 100%;
			font-family : tahoma;
		}
		
		body {
			height : 100%;
			margin : auto;
			background-color : #2a2625;
		}
		
		.main {
			margin-left : 25%;
			margin-right : 25%;
			background-color : lightgrey;
			height : 100%;
		}
		
		#center_button {
			margin-left : 25%;
			margin-top : 5%;
			margin-bottom : 5%;
		}
		
		#etat {
			background-image : url("images/smiley.png");
			background-repeat : no-repeat;
			width : 350px;
			height : 350px;
			padding : 15px 15px 15px 15px;
			float : left;
		}
		
		#selection {
			margin-left : 50%;
		}
		
		#logo {
			background-image : url("images/logo.png");
			background-repeat : no-repeat;
			width : 216px;
			height : 90px;
		}
		</Style>
	</Head>
	<Body>
		<Div id ="main" class="main">
			<Section>
				<Div id="etat"></Div>
				<Div id="selection">
					
					<Span>Please, select the color I will be playing : </Span>
					<Div id="center_button">
						<Input type="radio" name="chooseColor" value="Red" />Red
						<Input type="radio" name="chooseColor" value="Blue" />Blue
					</Div>
					<Select>
						<Option> ----- 
						<Option>Mode match Strat A
						<Option>Mode match Strat B
						<Option>Mode dev Strat A
						<Option>Mode dev Strat B
					</Select>
				</Div>
			</Section>
		</Div>
	</Body>
</Html>