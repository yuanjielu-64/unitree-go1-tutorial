# Unitree Go1 tutorial
This repository is based on Unitree's repositories and aims to add the compatibility of the ROS packages with ROS Noetic.
We will need four repositories:
* [unitree_ros](https://github.com/unitreerobotics/unitree_ros)
* [unitree_legged_sdk](https://github.com/unitreerobotics/unitree_legged_sdk)
* [unitree_ros_to_real](https://github.com/unitreerobotics/unitree_ros_to_real)
* [unitree_guide](https://github.com/unitreerobotics/unitree_guide)

If you want to do the simulation, you might only need to use the package unitree_legged_msgs rather than unitree_ros_to_real. Once all the repositories are ready, we can perform low-level control of the robot's movement. In the Gazebo simulation, we cannot do high-level control, namely walking. For real robots, we can do both-level control using ROS packages.

# Dependencies
* [ROS Noetic](https://www.ros.org/)
* [Gazebo](http://gazebosim.org/)

## Build/Installation
For ROS Noetic:
1. You may need to download these packages from unitree go1, but you may not need to use all of them.
```
sudo apt-get update
sudo apt-get install liblcm-dev
sudo apt-get install ros-noetic-controller-interface ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control ros-noetic-joint-state-controller ros-noetic-effort-controllers ros-noetic-joint-trajectory-controller ros-noetic-amcl ros-noetic-move-base ros-noetic-slam-gmapping ros-noetic-hector-slam ros-noetic-map-server ros-noetic-global-planner ros-noetic-dwa-local-planner
```

2. Install Python dependencies
```
pip3 install defusedxml rospkg netifaces numpy
```

3. Create ROS workspace
```
mkdir -p /noetic/unitree_ws/src
cd /noetic/unitree_ws/src
```

4. Clone this repo and required ros packages:
```
git clone https://github.com/unitreerobotics/unitree_ros.git
git clone https://github.com/unitreerobotics/unitree_legged_sdk.git
git clone https://github.com/unitreerobotics/unitree_ros_to_real.git
git clone https://github.com/unitreerobotics/unitree_guide.git
git clone https://github.com/yuanjielu-64/barn_challenge_go1.git
```
Once you have download barn_challenge_go1, please change name to barn_challenge_lu

5. Use catkin_make to build:
```
cd /noetic/unitree_ws/
catkin_make
```

## Run Simulations for BARN challenge
Open a terminal and start Gazebo
```
roslaunch barn_challenge_lu gazebo_launch_test.launch 
```
You can change the world name, robot name and the position (x,y,z, yaw) of the robot in the gazebo_launch_test.launch. In Gazebo, the robot should be lying on the ground with joints not activated. You may find an issue where in many worlds, the initial robot dog cannot properly lie down from a standing position. This is because the joints are not enabled by default. When you add
```
<node name="junior_controller" pkg="unitree_guide" type="junior_ctrl" output="screen" />
```
in the launch file, or run the following command
```
rosrun unitree_guide junior_ctrl
```
the problem can be solved.
![Go1 in Rviz](./figure/a.png)

To launch the 










## Installation

Follow the instructions below to run simulations on your local machines.

1. Download the requirment from jackal robot, if you run successfully, then go step 2 (https://www.clearpathrobotics.com/assets/guides/noetic/jackal/simulation.html)
```
sudo apt-get install ros-noetic-jackal-simulator ros-noetic-jackal-desktop ros-noetic-jackal-navigation
```

2. Install Python dependencies
```
pip3 install defusedxml rospkg netifaces numpy
```
3. Create ROS workspace
```
mkdir -p /<YOUR_HOME_DIR>/jackal_ws/src
cd /<YOUR_HOME_DIR>/jackal_ws/src
```
4. Clone this repo and required ros packages: (replace <YOUR_ROS_VERSION> with your own, e.g. noetic)
```
git clone https://github.com/jackal/jackal.git --branch <YOUR_ROS_VERSION>-devel
git clone https://github.com/jackal/jackal_simulator.git --branch <YOUR_ROS_VERSION>-devel
git clone https://github.com/jackal/jackal_desktop.git --branch <YOUR_ROS_VERSION>-devel
git clone https://github.com/jackal/jackal_robot.git -- branch <YOUR_ROS_VERSION>-devel
```

5. Please replace this with jackal/jackal_control/config/robot_localization.yaml, as these are some changes from the previous version. Otherwise, you will see the robot's localization deviate.

Replace line 13 to line 19:
```
imu0: /imu/data
imu0_config: [false, false, false,
              true, true, false,
              false, false, false,
              true, true, true,
              false, false, false]
```
to 
```
imu0: /imu/data
imu0_config: [false, false, false,
              true, true, true,
              false, false, false,
              true, true, true,
              false, false, false]
```



6. Clone this code from Yuanjie Lu:
```
git clone https://github.com/yuanjielu-64/BARN2025.git
```
7. Install ROS package dependencies: (replace <YOUR_ROS_VERSION> with your own, e.g. noetic)
```
cd ..
source /opt/ros/<YOUR_ROS_VERSION>/setup.bash

rosdep init; rosdep update
rosdep install -y --from-paths . --ignore-src --rosdistro=<YOUR_ROS_VERSION>
```

8. Cd the jackal_ws/ and build the workspace
```
catkin_make
source devel/setup.bash
```

## Run Simulations

### Note: 
Many parts of the code are designed for future projects, meaning some files are not actively used. Please follow the instructions below for testing and execution.

### File Structure Overview
* Data storage: barn_challenge_lu/launch/data/
* Launch files: barn_challenge_lu/launch/
* Move base configuration files: barn_challenge_lu/params/
* Core implementation: barn_challenge_lu/src/
* Test results: barn_challenge_lu/scripts/

1. Running a Single Test
```
cd scripts
python run_ddp.py --world_idx &i --out "out.txt"
```
Where &i is the world index you want to test.

2. Running All Tests
```
./test.sh
```

3. If you want to run with DWA, DDPDWAPlanner, LuPlanner and DDPLuPlanner, please change /launch/gazebo_launch_ddp.launch at line 44

Please note that MPPIPlanner and DDPMPPIPlanner represent LuPlanner and DDPLuPlanner respectively, not the MPPI algorithm

```
args="RunMP $(find barn_challenge_lu)/data/ParamsForJackalGlobal.txt UseMP DWAPlanner"/>
args="RunMP $(find barn_challenge_lu)/data/ParamsForJackalGlobal.txt UseMP DDPDWAPlanner"/>
args="RunMP $(find barn_challenge_lu)/data/ParamsForJackalGlobal.txt UseMP MPPIPlanner"/>
args="RunMP $(find barn_challenge_lu)/data/ParamsForJackalGlobal.txt UseMP DDPMPPIPlanner"/>
``

### Contribution
If you would like to contribute to this project, feel free to submit a pull request or open an issue on GitHub.

### License
This project is licensed under the MIT License. See the LICENSE file for details

### Contact
For any questions or support, please contact:
📧 Yuanjie Lu - ylu22@gmu.edu