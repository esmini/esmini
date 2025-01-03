# esminiROS

This project extends the esmini simulator to support integration with the Robot Operating System (ROS). 

It allows users to leverage the power of esmini's OpenSCENARIO player while utilizing ROS for communication and control.

## Features
- ROS Integration: Seamlessly connect esmini with ROS nodes.
- Bidirectional Communication: Translate data between esmini and ROS formats.
- Vehicle Control: Send control commands from ROS to esmini-simulated vehicles.

## Installation
Tested on Linux (Ubuntu 20.04) with ROS 1 noetic

1. [Install ROS 1](https://wiki.ros.org/noetic/Installation/Ubuntu)

You can install it by simply running the commands below.
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-get install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt-get update

# ros-base is enough
sudo apt-get install ros-noetic-ros-base

echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo apt-get install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo rosdep init
rosdep update

sudo apt-get install ros-noetic-tf2-geometry-msgs
```


2. Build esminiROS workspace
```
sudo apt install python3-catkin-tools
sudo apt-get install catkin-tools
cd ${esmini workspace}/EnvironmentSimulator/Libraries/esminiROS
catkin_init_workspace

# Default option is false
catkin build -DUSE_OSG=True
```

3. Install python packages (for keyboard controller)
```
sudo apt install python3-pip
cd ${esmini workspace}/EnvironmentSimulator/Libraries/esminiROS
pip install -r requirements.txt
```

## Usage
1. Source package:
```
cd ${esmini workspace}/EnvironmentSimulator/Libraries/esminiROS
source devel/setup.bash
```

2. Run launch file:
```
roslaunch esmini esmini_node.launch
```

You can change the scenario by modifying the `osc` argument in the `esmini_node.launch` file.

3. Fetch object state:
```
rostopic echo /esmini/object_states
```

4. Control with keyboard:

Run the controller script in another terminal:
```
cd ${esmini workspace}/EnvironmentSimulator/Libraries/esminiROS
python3 scripts/keyboard_controller.py
```

Keyboard input provides acceleration and steering control via ROS topics.

You can control the vehicle named **Ego** if its controller isn't set as an external controller.