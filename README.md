# CrazySim: SITL for the Crazyflies (AEROSP 740 Version)

A software-in-the-loop (SITL) simulator for the [Crazyflie](https://www.bitcraze.io/products/crazyflie-2-1/) nano quadcopter. The simulated Crazyflie firmware runs in Gazebo and communicates through the Crazyflie Python library ([CFLib](https://github.com/bitcraze/crazyflie-lib-python)), as a real Crazyflie would. [Crazyswarm2](https://github.com/IMRCLab/crazyswarm2) provides the ROS 2 interface: it publishes the state of each Crazyflie and forwards the control commands from the ROS 2 controller nodes. Controllers developed in simulation can therefore be used directly on the hardware.

The repository has three parts:
- **`crazyflie-firmware`**: Crazyflie firmware modified for SITL, with the Gazebo models and launch scripts.
- **`crazyflie-lib-python`**: CFLib, the library used to communicate with the Crazyflie.
- **`ros2_ws`**: ROS 2 workspace with Crazyswarm2 (the `crazyflie_server`) and the controller package `controller_pkg`.

# Installation

> [!NOTE]
> Tested on Ubuntu 22.04 with ROS 2 Humble and Gazebo Harmonic.

## 1. Clone the repository
```bash
git clone https://github.com/UM-iRaL/ae740_crazyflie_sim.git
cd ae740_crazyflie_sim
```

## 2. System dependencies
Install [ROS 2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) and [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/install_ubuntu/), then:
```bash
sudo apt update && sudo apt install cmake build-essential libboost-program-options-dev libusb-1.0-0-dev ros-humble-tf-transformations libjsoncpp-dev
```

## 3. Python virtual environment
```bash
python3 -m venv ae740_venv
source ae740_venv/bin/activate
pip install Jinja2 rowan transforms3d empy==3.3.4 catkin_pkg lark
```

> [!IMPORTANT]
> Activate `ae740_venv` before every Python package installation and before running the simulator.

## 4. Crazyflie firmware (SITL) and Gazebo plugins
From `ae740_crazyflie_sim`:
```bash
cd crazyflie-firmware
mkdir -p sitl_make/build && cd $_
cmake ..
make all
```

> [!NOTE]
> CMake version warnings can be ignored.

## 5. Crazyflie Python library
From `ae740_crazyflie_sim`, with `ae740_venv` active:
```bash
cd crazyflie-lib-python
pip install -e .
```

## 6. Acados (MPC controllers only)
The MPC controllers use [Acados](https://github.com/acados/acados). Other controllers, such as the geometric controller, do not need it.

1. Build Acados with CMake following the [installation guide](https://docs.acados.org/installation/index.html). It can be cloned inside `ae740_crazyflie_sim`.
2. Install the Python interface following the [Python interface guide](https://docs.acados.org/python_interface/index.html), at least up to step 5.

> [!IMPORTANT]
> Add the Acados paths to `~/.bashrc`, as described in step 4 of the Python interface guide.

## 7. ROS 2 workspace
From `ae740_crazyflie_sim`:
```bash
cd ros2_ws
colcon build --symlink-install
```

# Usage

The simulation runs in four terminals:

| Terminal | Purpose |
|---|---|
| 1 | Gazebo SITL (simulated Crazyflie firmware) |
| 2 | Crazyswarm2 `crazyflie_server` (ROS 2 interface) |
| 3 | Controller node |
| 4 | Flight commands (takeoff, trajectory, hover, land) |

> [!IMPORTANT]
> Start terminal 1 first, then terminals 2, 3 and 4 in order. Restart terminals 1 and 2 before every new run: the simulation and the server cannot be reused.

## Terminal 1: Gazebo SITL
From `ae740_crazyflie_sim`:
```bash
cd crazyflie-firmware
bash tools/crazyflie-simulation/simulator_files/gazebo/launch/sitl_target_tracking.sh -t 0 -p 1 -m crazyflie
```

| Option | Description |
|---|---|
| `-t` | Number of target drones (default 0) |
| `-p` | Number of pursuer drones (default 1) |
| `-m` | Vehicle model: `crazyflie` or `crazyflie_thrust_upgrade` |

The drones are numbered with the targets first, then the pursuers. For example, `-t 1 -p 1` spawns `cf_1` (target) and `cf_2` (pursuer). The command above spawns a single drone, `cf_1`, at the origin. For a single drone, `sitl_singleagent.sh` in the same folder can be used instead.

> [!WARNING]
> The drones used by the `crazyflie_server` are set in `ros2_ws/src/crazyswarm2/crazyflie/config/crazyflies.yaml`. For N spawned drones, set `enabled: true` for **only** `cf_1`, ..., `cf_N`, and `enabled: false` for all others. By default, only `cf_1` is enabled. A mismatch between the spawned and the enabled drones prevents the server from starting correctly.

> [!TIP]
> If the Gazebo window opens but stops responding, see the Gazebo [network troubleshooting](https://gazebosim.org/docs/latest/troubleshooting/#network-configuration-issue). Multicast may be disabled; enable it as described [here](https://docs.ros.org/en/rolling/How-To-Guides/Installation-Troubleshooting.html#enable-multicast).

## Terminal 2: Crazyswarm2 server
```bash
cd ros2_ws
source install/setup.bash
ros2 launch crazyflie launch.py backend:=cflib
```

The server is ready when RViz opens and the terminal shows:
```
[crazyflie_server]: All Crazyflies loggging are initialized.
```

The Crazyswarm2 configuration files are in `ros2_ws/src/crazyswarm2/crazyflie/config/` (see the [Crazyswarm2 documentation](https://imrclab.github.io/crazyswarm2/usage.html)).

> [!TIP]
> If the server fails to connect, or never reaches the message above, check the enabled drones in `crazyflies.yaml`: only `cf_1`, ..., `cf_N` must be `enabled: true` for N drones spawned in terminal 1.

> [!TIP]
> On Wayland, RViz may fail with `Invalid parentWindowHandle (wrong server or screen)`. Launch with XWayland instead:
> ```bash
> QT_QPA_PLATFORM=xcb ros2 launch crazyflie launch.py backend:=cflib
> ```

## Terminal 3: Controller
The controller nodes are in `ros2_ws/src/controller_pkg`. This package is not part of this repository; it is provided with the lab files. Run a controller with:
```bash
cd ros2_ws
source install/setup.bash
ros2 run controller_pkg <executable>
```

| Controller | `<executable>` | Command prefix | Requires Acados |
|---|---|---|---|
| MPC | `crazyflie_mpc_controller` | `mpc` | Yes |
| Geometric | `crazyflie_geometric_controller` | `geo` | No |

The drone stays on the ground until a takeoff command is sent (terminal 4).

> [!NOTE]
> Complete all `TODO` parts of a controller before running it.

> [!TIP]
> If the controller keeps printing `Empty state message.` although the state callbacks are complete, the server did not initialize all state topics. Check the `pose` and `twist` topics in separate terminals using `ros2 topic echo /cf_1/pose` and `ros2 topic echo /cf_1/twist`. Restart terminals 1 and 2.

## Terminal 4: Flight commands
Commands are sent to all drones as `Empty` messages on `/all/<prefix>_<command>`, where `<prefix>` is the command prefix of the running controller (see the table above):
```bash
ros2 topic pub -t 1 /all/<prefix>_takeoff std_msgs/msg/Empty
```

| Command | Action |
|---|---|
| `takeoff` | Take off and hover at the takeoff height |
| `trajectory` | Start the reference trajectory |
| `hover` | Stop and hover at the current position |
| `land` | Land at the current position |

For example, with the geometric controller:
```bash
ros2 topic pub -t 1 /all/geo_takeoff std_msgs/msg/Empty
ros2 topic pub -t 1 /all/geo_trajectory std_msgs/msg/Empty
ros2 topic pub -t 1 /all/geo_hover std_msgs/msg/Empty
ros2 topic pub -t 1 /all/geo_land std_msgs/msg/Empty
```

> [!TIP]
> RViz shows `/cf_N/mpc_solution_path` by default. To view the reference of another controller (e.g. `/cf_1/geo_reference_path`), add a `Path` display in RViz.
