#!/bin/bash
#
# author : WLA
# date : 05/03/2011
#
# ce script permet de creer un nouveau projet ARD dans ROS au format Orocos.
# il faut lui passer en tant qu'argument le nom du projet respectant la grammaire [a-zA-Z0-9_-]*

VERT="\\033[1;32m" 
NORMAL="\\033[0;39m" 
ROUGE="\\033[1;31m" 
ROSE="\\033[1;35m" 
BLEU="\\033[1;34m" 
BLANC="\\033[0;02m" 
BLANCLAIR="\\033[1;08m" 
JAUNE="\\033[1;33m" 
CYAN="\\033[1;36m" 


#vaut 0 quand on n'a pas commence
creation_state=0
package_name=$1

###
# "clean" fonction appelee lors de l'interruption du script avec Ctrl+C
###
clean()
{
	if [ "$creation_state" = "0" ]
	then
		echo "Creation canceled"		
	else
		echo "Creation canceled, cleaning $package_name ..."
		rm $package_name -r
	fi
}
#enregistrement de la fonction sur le signal exit
trap "clean" 2

###
# "create_orocos_project" : generate the default orocos project 
###
create_orocos_project()
{
	echo "[*] Creating $package_name Orocos-ROS package..."
	`rospack find ocl`/scripts/pkg/orocreate-pkg $package_name 1> /dev/null
	#correction d'un bug orocos
	sed -i "s/provides()/this/" $package_name/$package_name-service.cpp
	sed -i "s/1.0.0/@project_version_major@.@project_version_minor@.@project_version_patch@/" $package_name/orocos-$package_name.pc.in
}

###
# "create_folder_structure" create folder structure of an ard project and arrange orocos generated files to fit ard folder structure
###
create_folder_structure()
{
	echo "[*] Creating ARD directories structure..."
	mkdir $package_name/cmake $package_name/doc $package_name/test $package_name/ressource $package_name/src $package_name/script $package_name/bin
	mkdir $package_name/script/deployment $package_name/script/linux $package_name/script/conf $package_name/script/ops $package_name/script/osd

	echo "[*] Moving default sources into src folder ..."
	mv $package_name/*.?pp $package_name/src
}

###
# "edit_cmakelists" : edit the CMakeLists.txt generated in the default orocos project to fit ard desires
###
edit_cmakelists()
{
	echo "[*] Editing CMakelists.txt ..."
	cp default/CMakeLists.txt $package_name/CMakeLists.txt
	sed -i "s/@PROJECT_NAME@/$package_name/" $package_name/CMakeLists.txt
	cp default/cmake/* $package_name/cmake
	sed -i "s/XXX/$package_name/" $package_name/cmake/ARP_project_names.cmake
	
	#creation des CMakeFiles de src
	touch $package_name/src/CMakeLists.txt
	echo "orocos_component($package_name $package_name-component.cpp)" >> $package_name/src/CMakeLists.txt
	echo "#not working presently due to Orocos issues : orocos_typegen_headers($package_name-types.hpp)" >> $package_name/src/CMakeLists.txt
	echo "orocos_library(support support.cpp)" >> $package_name/src/CMakeLists.txt
	echo "orocos_service($package_name-service $package_name-service.cpp) # ...only one service per library !" >> $package_name/src/CMakeLists.txt
	echo "orocos_plugin($package_name-plugin $package_name-plugin.cpp) # ...only one plugin function per library !" >> $package_name/src/CMakeLists.txt
	echo "orocos_install_headers( $package_name-component.hpp )" >> $package_name/src/CMakeLists.txt
	
	#creation des CMakeFiles de test
	touch $package_name/test/CMakeLists.txt

	#creation des CMakeFiles de script
	cp script/* $package_name/script
	touch $package_name/script/conf/CMakeLists.txt
	touch $package_name/script/deployment/CMakeLists.txt
	touch $package_name/script/linux/CMakeLists.txt
	touch $package_name/script/osd/CMakeLists.txt
	touch $package_name/script/ops/CMakeLists.txt
}

###
# "configure_ros" : edit ROS files to finish the ROS configuration in the ARD system
###
configure_ros()
{
	echo "[*] Editing env.sh ..."
	echo "ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:\`rosstack find ard\`/$package_name" >> env.sh

	echo "[*] Editing manifest.xml ..."
	sed -i "s/<\/description>/<\/description>\n    <author>ARD<\/author>\n    <license>BSD<\/license>/\n    <url>http://team-ard.com</url>"  $package_name/manifest.xml
}

###
# "generate_documentation_system" : generate the doxyfile configuration
###
generate_documentation_system()
{
	echo "[*] Configuration Doxygen ..."
	doxygen -g $package_name/doc/Doxyfile.sh 1>/dev/null

	sed -i "/^PROJECT_NAME /s/=/= $package_name/" $package_name/doc/Doxyfile.sh
	sed -i "/^PROJECT_NUMBER /s/=/= 10.0.0/" $package_name/doc/Doxyfile.sh
	sed -i "/^OUTPUT_DIRECTORY /s/=/= doc/" $package_name/doc/Doxyfile.sh
	sed -i "/^OUTPUT_LANGUAGE /s/= English /= French/" $package_name/doc/Doxyfile.sh
	sed -i "/^EXTRACT_STATIC /s/= NO/= YES/" $package_name/doc/Doxyfile.sh
	sed -i "/^SHOW_DIRECTORIES /s/= NO/= YES/" $package_name/doc/Doxyfile.sh
	sed -i "/^INPUT /s/=/= src test/" $package_name/doc/Doxyfile.sh
	sed -i "/^FILE_PATTERNS /s/=/= *.cpp *.hpp *.c *.h/" $package_name/doc/Doxyfile.sh
	sed -i "/^RECURSIVE /s/= NO/= YES/" $package_name/doc/Doxyfile.sh
	sed -i "/^SOURCE_BROWSER /s/= NO/= YES/" $package_name/doc/Doxyfile.sh
	sed -i "/^GENERATE_ECLIPSEHELP /s/= NO/= YES/" $package_name/doc/Doxyfile.sh
	sed -i "/^ECLIPSE_DOC_ID /s/=/= org.ard.Core/" $package_name/doc/Doxyfile.sh

	echo "[*] Generating default documentation ..."
	cd $package_name
	doxygen doc/Doxyfile.sh 2>/dev/null 1>/dev/null
	cd ..

}

###
# "create_default_deployment" : create a deployment script in order to launch a deployer
###
create_default_deployment()
{
	package_upper_case=`echo "$package_name" | python -c "print raw_input().capitalize()"`
	
	echo "[*] Generating default deployment ..."
	echo "/* ARD $package_name main deployment file */" > $package_name/script/deployment/deploy_$package_name.ops
	echo "path(\"`rospack find $package_name`/lib/orocos\");" >> $package_name/script/deployment/deploy_$package_name.ops
	echo 'require("print");' >> $package_name/script/deployment/deploy_$package_name.ops  
	echo "loadComponent(\"Component1\",\"$package_upper_case\");" >> $package_name/script/deployment/deploy_$package_name.ops 
	echo "loadService (\"Component1\",\"myservice\");" >> $package_name/script/deployment/deploy_$package_name.ops 
	echo "Component1.start()" >> $package_name/script/deployment/deploy_$package_name.ops  
}

###
# "create_scripts" : populate the script directory
###
create_scripts()
{
	echo "[*] Generating launching script ..."
	echo "# ARD $package_name launch script file (auto generated file, you shouldn't edit)" > $package_name/run.sh
	echo "rosrun ocl deployer-gnulinux -s script/deployment/deploy_$package_name.ops" > $package_name/run.sh
	chmod +x $package_name/run.sh

	echo "[*] Generating exploring documentation script ..."
	echo "# ARD $package_name exploring documentation file (auto generated file, you shouldn't edit)" > $package_name/doc.sh
	echo "rosrun ocl deployer-gnulinux -s script/deployment/deploy_$package_name.ops" > $package_name/doc.sh
	chmod +x $package_name/doc.sh

	echo "[*] Generating install_taget script ..."
	echo "# ARD $package_name install target script file (auto generated file, you shouldn't edit)" > $package_name/script/linux/install_target_$package_name.sh
	echo "\`rosstack find ard\`/default/script/install_target.sh $package_name \$1" > $package_name/script/linux/install_target_$package_name.sh
	chmod +x $package_name/script/linux/install_target_$package_name.sh

	echo "[*] Generating gen_doc script ..."
	echo "# ARD $package_name documentation generation script file (auto generated file, you shouldn't edit)" > $package_name/script/linux/gen_doc_$package_name.sh
	echo "\`rosstack find ard\`/default/script/gen_doc.sh $package_name" > $package_name/script/linux/gen_doc_$package_name.sh
	chmod +x $package_name/script/linux/gen_doc_$package_name.sh
}

###
# "main" : creation d'un nouveau projet
###
if [ $1 ]
then
	if [ -d $1 ]
	then
		echo -e $ROUGE "[ERROR] : package $1 already exists" $NORMAL
	else
		creation_state=1
		create_orocos_project
		create_folder_structure
		edit_cmakelists
		configure_ros
		generate_documentation_system
		create_default_deployment
		create_scripts

		echo -e $VERT "[Done] check success and relaunch your terminal !!" $NORMAL
		echo -e $VERT "[.] you must edit your cmake/ARP_project_names.cmake to finalize the configuration" $NORMAL
		echo -e $VERT "[.] you may use 'rosmake $1', 'firefox doc/html/index.html' and then 'bin/run.sh'" $NORMAL	
	fi
else
	echo -e $ROUGE "[ERROR] : Please enter a package name" $NORMAL
fi

#for debug
#sleep 3
