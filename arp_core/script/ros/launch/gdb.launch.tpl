<!-- This is a template to add a debugger to the orocos deployment -->
<launch>
	<node name="ArpOrocos" pkg="ocl" type="deployer-corba" output="screen"
		respawn="false"
		required="true"
		launch-prefix="gdb --eval-command=run -s $(find arp_master)/script/orocos/deployment/deploy_arp_master_manual_simul.ops --"
		/>
</launch>