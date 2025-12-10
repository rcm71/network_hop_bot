# network_hop_bot

VM Setup Procedure:
1. Install Ubuntu 22.04 on your preferred VM (Cannot be any other version)
2. Run the bash script shown below




#!/bin/bash sudo apt install -y build-essential software-properties-common git curl locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-humble-desktop
sudo apt install -y ros-dev-tools
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh




3. Reboot the VM before proceeding
4. Run the following bash script





#!/bin/bash
cd ~
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent/
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
mkdir -p ~/px4_ros_ws/src
cd ~/px4_ros_ws/src
git clone https://github.com/PX4/px4_msgs.git 
colcon build
source install/local_setup.bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500




5. At this point you should be fully setup to run ROS and our scripts
6. If you are struggling to get this working, Rory can provide the image file for a working VM Setup (For ARM CPUs not x86)
