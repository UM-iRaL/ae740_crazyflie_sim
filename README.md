# CrazySim: SITL for the Crazyflies (AEROSP 740 Version)

The simulated Crazyflie firmware communicates with the Crazyflie Python library ([CFLib](https://github.com/bitcraze/crazyflie-lib-python)), which is also provided in this codebase. This allows you to simulate the behavior of CFLib scripts that would normally control single or multiple Crazyflies in real hardware demonstrations. Crazyflie is a lightweight, open-source nano quadcopter platform widely used for research and education in robotics and control.

With CFLib communication capabilities, you can use [CrazySwarm2](https://github.com/IMRCLab/crazyswarm2) with CFLib as the backend for a ROS 2 interface with the simulator. This creates a Crazyflie server that communicates with Gazebo using CFLib backend based on attitude commands published as ROS2 messages. Additionally, it publishes the state data of each Crazyflie that you can use in your controller implementations.

This codebase also includes examples using model predictive control (MPC) with [Acados](https://github.com/acados/acados) for decentralized control of Crazyflie drone fleets, which you can study and modify for your assignments.
 
# CrazySim Setup and Installation

## System Requirements
The simulator has been built, tested, and verified on Ubuntu 22.04 with Gazebo Harmonic. Make sure your system meets these requirements before proceeding. There are three primary parts in the simulator -- **(a) Crazyflie Firmware**: a modified version of the Crazyflie firmware for SITL implementation, **(b) CFLib Python Library**: a library to communicate with the Crazyflie, **(c) ROS2 Workspace**: a workspace that contains the Crazyswarm2 package to create a ROS2 server and the main controller package. You would be adding/editing the ROS2 nodes in the `controller_pkg` to send the commands over the ROS2 network. This setup can then be used for both SITL and hardware experiments.  

To install this repository, use the recursive command shown below for HTTPS:
```bash
git clone https://github.com/UM-iRaL/ae740_crazyflie_sim.git
```

## Python Virtual Environment Setup

Before installing any packages, create and activate a Python virtual environment named `ae740_venv`. This ensures all dependencies for this course are isolated from your system Python and other projects. Run the following commands in your terminal:

```bash
python3 -m venv ae740_venv
source ae740_venv/bin/activate
```

All subsequent Python package installations should be performed inside this activated environment.


## System Dependencies
Install all required system packages with a single command:
```bash
sudo apt update && sudo apt install cmake build-essential libboost-program-options-dev libusb-1.0-0-dev ros-humble-tf-transformations libjsoncpp-dev
```

Make sure you have ROS 2 [Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html). Then, install Gazebo Harmonic from [here](https://gazebosim.org/docs/harmonic/install_ubuntu/).

We use Acados for MPC, follow the [Acados installation guide](https://docs.acados.org/installation/index.html). You can clone and build the acados inside the simulation root directory. Build the acados using the CMake, as recommended in the [Acados installation guide](https://docs.acados.org/installation/index.html). 

Next, we need to install the Python interface for the acados. Follow the instructions (atleast until step 5) in their [Python interface documentation](https://docs.acados.org/python_interface/index.html). Make sure to add the path to the `.bashrc` as highlighted in step 4. 

The main intention of using this (slightly complex) simulator to introduce a software-in-the-loop version of the Crazyflie, to develop the contollers that can be directly used on the hardware experiments. 

## Python Dependencies
Make sure your Python virtual environment (`ae740_venv`) is activated before installing the following dependencies. Run:

```bash
source ae740_venv/bin/activate
```

Then install all required Python packages:

```bash
pip install Jinja2 rowan transforms3d yq jq empy==3.3.4 catkin_pkg lark
```


## crazyflie-firmware
Navigate to the `ae740_crazyflie_sim` directory in the terminal, and run the following commands to build the firmware and Gazebo plugins:
```bash
cd crazyflie-firmware
mkdir -p sitl_make/build && cd $_
cmake ..
make all
```

You can ignore the warnings for the CMake versions for now.

## crazyflie-lib-python
Navigate back to the `ae740_crazyflie_sim` directory and run the following commands to install the Crazyflie Python library (be sure to be inside the `ae740_venv`): 
```bash
cd crazyflie-lib-python
pip install -e .
```

## Crazyswarm 2 and ROS2 Interface
This section covers the setup of CrazySwarm2 with CrazySim and demonstrates a case study that uses a model predictive controller (MPC) with Acados to track a set of predefined temporally parametrized trajectories.

Then build the ROS 2 workspace. Make sure to return to the main directory `ae740_crazyflie_sim` first.
```bash
cd ros2_ws
colcon build --symlink-install
```


# How to use
**Note:** The workflow requires four separate terminals, each dedicated to a specific component:

1. **Terminal 1:** Launches the SITL Gazebo simulation (custom Crazyflie firmware).
2. **Terminal 2:** Starts the Crazyswarm2 `crazyflie_server` (ROS 2 interface).
3. **Terminal 3:** Runs the MPC controller for our drone.
4. **Terminal 4:** Give high-level commands to our drone. 

Always start the SITL simulation in Terminal 1 before launching the server or controller nodes in the other terminals. Gazebo SITL simulation (terminal 1) and server (terminal 2) must be restarted every time you run a new controller, as the current setup does not support reusing the server or simulation instance. This ensures a clean environment for each experiment.

### Terminal 1: Start SITL Gazebo (custom Crazyflie Firmware) 
Open a terminal and run (inside the `ae740_crazyflie_sim`)
```bash
cd crazyflie-firmware
bash tools/crazyflie-simulation/simulator_files/gazebo/launch/sitl_singleagent.sh 
```

This script `sitl_singleagent.sh` launches the SITL for a single crazyflie. The crazyflies have UDP addresses starting from `udp://0.0.0.0:19850`. Also, make sure that *ONLY* `cf_1` is enabled in the configuration file `ae740_crazyflie_sim/ros2_ws/src/crazyswarm2/crazyflie/config/crazyflies.yaml`. This is important to make sure that the `crazyflie_server` in terminal 2 runs correctly. 

### Terminal 1: Start SITL Gazebo (custom Crazyflie Firmware) [ALTERNATIVE SCRIPT/RECOMMENDED]
Alternatively, you can launch multiple/single crazyflies (used for lab 4 target tracking) using the following command, which takes care of the configuration file internally. Make sure you are in the `crazyflie-firmware` directory.
```bash
cd crazyflie-firmware
bash tools/crazyflie-simulation/simulator_files/gazebo/launch/sitl_target_tracking.sh -t 0 -p 1 -m crazyflie
```
Here, `-t` is the number of target drones, `-p` is the number of pursuer drones, and `-m` is the model type. The above command launches just one drone, labeled as the pursuer. You can modify the `-t` and `-p` values to launch multiple drones. The above script launches one crazyflie at the origin. This script automatically modifies the `crazyflies.yaml` configuration file to enable/disable the crazyflies based on the number of target/pursuer drones.


If the Gazebo opens the window, but becomes unresponsive or shows `Gazebo GUI not responding`, check the following [page](https://gazebosim.org/docs/latest/troubleshooting/#network-configuration-issue). Your system might have the ROS Multicast disabled, which can be solved by following the steps [here](https://docs.ros.org/en/rolling/How-To-Guides/Installation-Troubleshooting.html#enable-multicast). 


### Terminal 2: Start the Crazyswarm2 `crazyflie_server`
The crazyswarm2  configuration files can be found in `ros2_ws/src/crazyswarm2/crazyflie/config/`. The `crazyflies.yaml` describes the robots currently being used. If a robot is not in the simulator or hardware, then it can be disabled by setting the enabled parameter to false. A more detailed description for crazyswarm2 configurations can be found [here](https://imrclab.github.io/crazyswarm2/usage.html).

Run the following to launch the `crazyflie_server`. Make sure to source the ros2 or is already added to the `~/.bashrc` file.
```bash
cd ros2_ws
source install/setup.bash
ros2 launch crazyflie launch.py backend:=cflib
```

Due to Wayland/X11 windowing system in the Ubuntu, you might get the following error while launching the RViz window:
```bash
[rviz2]: rviz::RenderSystem: error creating render window: RenderingAPIException: Invalid parentWindowHandle (wrong server or screen)
[rviz2]: Unable to create the rendering window after 100 tries
```

This can be solved by forcing the system to use XWayland, if using the Wayland windowing system. Use the following command instead:
```bash
QT_QPA_PLATFORM=xcb ros2 launch crazyflie launch.py backend:=cflib
```

In the end, you should see the RViz2 window and the following message in the terminal: 
```bash
[crazyflie_server]: All Crazyflies loggging are initialized.
```


### Terminal 3: MPC Node for a Single Crazyflie
The `controller_pkg` contains the ROS2 nodes for our MPC controller. This package is not present in this repository, but can be found in the `lab3` directory in the `ae740_labs` repository. This will not run correctly in it's given form, you need to complete all the `TODO` parts before you try running the controller. To run the MPC demonstration for one drone, use the following commands:
```bash
cd ros2_ws
source install/setup.bash
ros2 run controller_pkg crazyflie_mpc_controller
```
This will launch the MPC node for the single Crazyflie, but the drone will not start flying until you send a takeoff command. If you have completed the state callback function but still get the warning `Empty state message.`, restart the simulation. Rarely, some of the states are not properly initiated on the ROS2 server causing this error. To resolve, re-run the main commands on terminals 1 and 2 before attempting again. 

### Terminal 4: Commands for Targets
Using the command line publisher we can command all vehicles to take off using MPC.
```bash
ros2 topic pub -t 1 /all/mpc_takeoff std_msgs/msg/Empty
```
Using the command line publisher we can command all vehicles to start the trajectory.
```bash
ros2 topic pub -t 1 /all/mpc_trajectory std_msgs/msg/Empty
```
Using the command line publisher we can command all vehicles to stop the trajectory and hover.
```bash
ros2 topic pub -t 1 /all/mpc_hover std_msgs/msg/Empty
```
Using the command line publisher we can command all vehicles to stop the trajectory and land.
```bash
ros2 topic pub -t 1 /all/mpc_land std_msgs/msg/Empty
```


