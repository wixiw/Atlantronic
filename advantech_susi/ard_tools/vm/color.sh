ARD_COLOR_GREEN="\\033[1;32m"
ARD_COLOR_DEFAULT="\\033[0;39m"
ARD_COLOR_RED="\\033[1;31m"
ARD_COLOR_PINK="\\033[1;35m"
ARD_COLOR_BLUE="\\033[1;34m"
ARD_COLOR_WHITE="\\033[0;02m"
ARD_COLOR_LIGHT_WHITE="\\033[1;08m"
ARD_COLOR_YELLOW="\\033[1;33m"
ARD_COLOR_CYAN="\\033[1;36mc"

cecho()
{
	case $1 in
		"green")
			echo -e $ARD_COLOR_GREEN $2 $ARD_COLOR_DEFAULT;;
		"pink")
			echo -e $ARD_COLOR_PINK $2 $ARD_COLOR_DEFAULT;;
		"blue")
			echo -e $ARD_COLOR_BLUE $2 $ARD_COLOR_DEFAULT;;
		"cyan")
			echo -e $ARD_COLOR_CYAN $2 $ARD_COLOR_DEFAULT;;
		"white")
			echo -e $ARD_COLOR_WHITE $2 $ARD_COLOR_DEFAULT;;
		"light_white")
			echo -e $ARD_COLOR_LIGHT_WHITE $2 $ARD_COLOR_DEFAULT;;
		"red")
			echo -e $ARD_COLOR_RED $2 $ARD_COLOR_DEFAULT;;
		"yellow")
			echo -e $ARD_COLOR_YELLOW $2 $ARD_COLOR_DEFAULT;;
		*)
			echo $2;;
	esac


}
