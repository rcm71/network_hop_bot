#!/usr/bin/env python3

# HANDLES OUR BACKEND STARTUP (MICROXRCE && GZ/PX4)

# Import the subprocess and time modules
import subprocess
import time
import create_points as creep 

def main():
	# List of commands to run
	commands = [
	    # Run the Micro XRCE-DDS Agent
	    "MicroXRCEAgent udp4 -p 8888",

	    # Run the PX4 SITL simulation
	    "cd ~/PX4-Autopilot && make px4_sitl gz_x500"
	    
	]

	# get destinations and spots to go to
	MAX_TENSION = 6
	MAX_RADIUS = 6
	MAX_ITER = 8
	dest = creep.get_destinations()
	spots = creep.create_force_graph(dest, MAX_TENSION, MAX_ITER, MAX_RADIUS)
	count = 0
	for spot in spots:
		commands.append(f"PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE='{count},{count}' PX4_SIM_MODEL=gz_x500 ~/PX4-Autopilot/build/px4_sitl_default/bin/px4 -i {count}")
		count += 1


	# Loop through each command in the list
	for command in commands:
	    # Each command is run in a new tab of the gnome-terminal
	    subprocess.run(["gnome-terminal", "--tab", "--", "bash", "-c", command + "; exec bash"])
	    
	    # Pause between each command
	    time.sleep(1)
