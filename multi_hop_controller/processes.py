#!/usr/bin/env python3

# HANDLES OUR BACKEND STARTUP (MICROXRCE && GZ/PX4)

# Import the subprocess and time modules
import subprocess
import time
import create_points as creep 
import drone_controller as drc
import rclpy
import drone_server as srv
import lemme_drive as lemme


def main():
	# List of commands to run
	commands = [
	    # Run the Micro XRCE-DDS Agent
	    "MicroXRCEAgent udp4 -p 8888",

	    # Run the PX4 SITL simulation
	    " source ~/PX4-Autopilot/build/px4_sitl_default/rootfs/gz_env.sh && gz sim -r empty.sdf"
	    
	]

	# get destinations and spots to go to
	MAX_TENSION = 6
	MAX_RADIUS = 6
	MAX_ITER = 8
	dest = creep.get_destinations()
	spots = creep.create_force_graph(dest, MAX_TENSION, MAX_ITER, MAX_RADIUS)
	count = 1
	for spot in spots:
		commands.append(f"source ~/PX4-Autopilot/build/px4_sitl_default/rootfs/gz_env.sh && PX4_SYS_AUTOSTART=4001 PX4_GZ_STANDALONE=1 PX4_GZ_WORLD=empty PX4_GZ_MODEL_POSE='{count},{count}' PX4_SIM_MODEL=gz_x500 ~/PX4-Autopilot/build/px4_sitl_default/bin/px4 -i {count}")
		count += 1


	# Loop through each command in the list
	for command in commands:
	    # Each command is run in a new tab of the gnome-terminal
	    subprocess.run(["gnome-terminal", "--tab", "--", "bash", "-c", command + "; exec bash"])
	    
	    # Pause between each command
	    time.sleep(1)
	    
	# now that we have all our drones spun up, we need to create a driver per
	# thank god for ros concurrency
	rclpy.init(args=None)
	controllers = []
	drivers = []
	executor = rclpy.executors.MultiThreadedExecutor()
	count = 1
	for spot in spots:
	    control = drc.OffboardControl(spot, count)
	    controllers.append(control)
	    executor.add_node(control)
	    driver = lemme.Lemme_Drive(spot, count)
	    drivers.append(driver)
	    executor.add_node(driver)
	    count += 1
	server = srv.NetworkSimActionServer(count)
	executor.add_node(driver)
	executor.add_node(server)
	try:
	    executor.spin()
	except KeyboardInterrupt:
	    pass
	finally:
	    executor.shutdown()
	    for controller in controllers:
	    	controller.destroy_node()
	    rclpy.shutdown()
	
		
	
