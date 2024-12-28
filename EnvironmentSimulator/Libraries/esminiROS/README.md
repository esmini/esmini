# esminiROS

This project extends the esmini simulator to support integration with the Robot Operating System (ROS). 

It allows users to leverage the power of esmini's OpenSCENARIO player while utilizing ROS for communication and control.

## Features
- ROS Integration: Seamlessly connect esmini with ROS nodes.
- Bidirectional Communication: Translate data between esmini and ROS formats.
- Vehicle Control: Send control commands from ROS to esmini-simulated vehicles.

## Installation
Tested on Linux (Ubuntu 20.04) with ROS 1 noetic

1. Install ROS

2. Build esminiROS workspace
```
sudo apt-get install catkin-tools
cd ${esmini workspace}/EnvironmentSimulator/Libraries/esminiROS
catkin_init_workspace

# Default option is false
catkin build -DUSE_OSG=True
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
python scripts/keyboard_controller.py
```

Keyboard input provides acceleration and steering control via ROS topics.

You can control the vehicle named **Ego** if its controller isn't set as an external controller.