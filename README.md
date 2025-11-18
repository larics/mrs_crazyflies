## MRS-Crazyflies
This ROS2 package contains adapted configuration files and launch files from [CrazySim](https://github.com/gtfactslab/CrazySim), to be used in the 2nd part of the MRS Project for the simulation part.

The simulation part is run in physics simulator Gazebo Ignition and uses ROS2 packages based on [CrazySwarm2](https://imrclab.github.io/crazyswarm2/)

## Reporting problems
If you encounter an error or a problem during the installation, setup or usage, please check the [Issue tab](https://github.com/larics/mrs_crazyflies/issues). If there is no solution to your problem there (in closed and open issues), feel free to open a new issue. When reporting a problem, specify your operating system and method of installation, describe your problem, and include the entire output of the command that resulted in the error. This will be the quickest way to get feedback and will help other students who may encounter the same error in the future. Likewise, if you think something is missing in this README, let us know by opening an issue.

## Installation

Again, there are two ways you can set up your computer to run the simulation:
1. **Using Docker** (recommended!!!)
2. If you **already have ROS2** installed and are having a hard time using Docker on your laptop.

### 1) Docker installation (recommended!!!)
If you haven't set up Docker in the first part of a project, please follow the instructions on [mrs_simulation](https://github.com/larics/mrs_simulation?tab=readme-ov-file#1-docker-installation-recommended) repo.

Next, clone the [this repository](https://github.com/larics/mrs_crazyflies):
```
git clone https://github.com/larics/mrs_crazyflies.git
```
Add the following line to  `~/.bashrc` and source it, or type this command in the current terminal:
```
export DOCKER_BUILDKIT=1
```
Run Dockerfile from the project root directory using the following commands:
```bash
cd mrs_crazyflies

# Build the Dockerfile.
# To install ros1_bridge and ROS Noetic, set the argument INSTALL_BRIDGE to true.
# Otherwise, set it to false, and it will only install ROS2.
docker build -t mrs_crazyflies_img .

# Run the crazysim_img2 container for the first time
./first_run.sh

# This will create docker container crazyswarm_container and position you into the container
```

For future runs, you can use the following commands:
```bash
# Start the container:
docker start -i mrs_crazyflies_cont

# Open the container in another terminal, while it is already started:
docker exec -it mrs_crazyflies_cont bash

# Stop the container
docker stop mrs_crazyflies_cont

# Delete the container (WARNING: this will delete all data inside the container)
docker rm mrs_crazyflies_cont

```
The Docker contains packages for crazyflies simulator [CrazySim](https://github.com/gtfactslab/CrazySim). General information about Crazyflies can be found [here](https://www.bitcraze.io/products/crazyflie-2-1/).

> [!NOTE]
> The ros2 workspace is located in /root/ros2_ws

### 2) Manual installation (if you already have ROS2 installed)
> We are assuming that you have ROS2 Humble installed.

Please follow the instructions given on the [CrazySim](https://github.com/gtfactslab/CrazySim) page to set up the simulation. Additionally, check out the aliases script: https://github.com/larics/docker_files/tree/ros-humble-cf/ros2/ros2-humble/crazyflies/to_copy and the README in this repository, which might come in handy.


## Topics and services

Velocity commands are published on `/cf_x/cmd_vel` to the crazyflie cf_x. Pose can also be obtained from the topic `/cf_x/pose` and velocity from `/cf_x/velocity`, just keep in mind that for this topic, the message type is not Twist, but a custom message: [LogDataGeneric](https://github.com/IMRCLab/crazyswarm2/blob/main/crazyflie_interfaces/msg/LogDataGeneric.msg), whose field elements are defined as: [v_x, v_y, v_z] in a world frame.
To take off/land, you can call services  `/cf_x/takeoff`, `/cf_x/land`. Current vel_mux.py does take off automatically, after the first cmd_vel command, but you can call it on your own.


## Test the simulation
> [!NOTE]
> Within MRS docker, the `mrs_crazyflies` package is located in `/root/ros2_ws/src/`. All folders and files mentioned later in these instructions are located inside the package. In Docker, there is an alias `cd_mrs_crazyflies` which changes the directory to this package.

This example showcases how to run the simulation using sessions, tmuxinator and environment variables. You do not need to use this format if you do not find it useful.

To run the example, navigate to `startup` folder in `mrs_crazyflies` package and run:
```
./start.sh
```
It will open one window with several panes.

![alt text](<images/terminal_screenshot.png>)

#### 1. The first pane starts the gazebo simulation:
```
 bash /path-to-workspace/ros2_ws/src/mrs_crazyflies/launch/sitl_multiagent_text.sh -m crazyflie -f $SPAWN_POSE_DOC -w $ENV_NAME
```
Please note that an example assumes that the installation is done in Docker. If you didn't use Docker, you may have a different path. A Bash script that starts Gazebo requires several arguments:
- -m is for the model. Please always use crazyflie.
- -f stands for the .txt file with the x and y initial positions for each crazyflie. The example for 4 crazyflies is given in the folder `launch/drone_spawn_list` (feel free to change or add yours here).
- -w specifies the world name, which can be found in the worlds folder.

The environment variables `$SPAWN_POSE_DOC` and `$ENV_NAME`, alongside the `$NUM_ROB`, which defines the number of robots, are located in `mrs_example_setup.sh`. This file should be sourced, alongside ros2 workspaces before (alias: ros2_ws and source_ros2) - check out the pre_window section in `example.yml`. :slightly_smiling_face:

#### 2. In the second pane (up right), ROS2 crazyflies server, RViz and crazyflie nodes that publish cmd_vel, are started.
```
 waitForCfsGazebo;sleep 2; ros2 launch mrs_crazyflies cf_velmux_launch.py
```
The shell function `waitForCfsGazebo` waits until all crazyflies are spawned in Gazebo, plus an additional 5 seconds of sleep, just in case, to have enough time to start. It can be found in `to_copy/aliases` (in Docker it is copied to `/root/.bash_aliases`).

Crazyflies server takes the data from `crazyflies_mrs.yaml`. For more info, please read about: [CrazySim](https://github.com/gtfactslab/CrazySim) and [CrazySwarm2](https://imrclab.github.io/crazyswarm2/).

**Please keep in mind that the variable `$NUM_ROB` should correspond to the number of enabled crazyflies in the `crazyflies_mrs.yaml` and the number of rows in the `$SPAWN_POSE_DOC`, otherwise the server won't be able to connect with Gazebo. Also, initial positions in `$SPAWN_POSE_DOC` should correspond to the ones in `crazyflies_mrs.yaml`.** Feel free to change them according to your task.

> [!TIP]
> This (second) pane should say: 'All Crazyflies parameters are initialized.' when everything is ok.

If you are waiting in this second pane, and it doesn't say that 'All Crazyflies parameters are initialized.', please check this [issue](https://github.com/gtfactslab/CrazySim/issues/1#issue-2123839637) and its [solution](https://github.com/gtfactslab/CrazySim/issues/1#issuecomment-1933212957). Just keep in mind that the world (.sdf) files that you need to adapt are in this package's world directory (`mrs_crazyflies/worlds`). If the simulation is still heavy for your laptop (real-time factor is below 70-80%), you can disable Gazebo GUI and watch the state in RViz only. You can do this by replacing [this line](https://github.com/larics/mrs_crazyflies/blob/89fb2e18bc0da1cfb863b75ce718fef6d0150de1/launch/sitl_multiagent_text.sh#L103) in mrs_crazyflies/launch/sitl_multiagent_text.sh with: `gz sim -s`. This will start only the Gazebo server in the background, and only RViz will open.

If the message 'All Crazyflies are fully connected.' doesn't appear, please check the number of crazyflies in use and simply try closing and starting everything again. The instructions on how to kill all terminals are given at the end of this section.


#### 3. The third pane (bottom) is given as an example to test if crazyflie cf_1 is moving.
The command to control a crazyflie is stored in history. Move to this pane (using `ctrl + down arrow`), press up arrow, and press enter, when everything else is already on.

```
history -s "ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/cf_1/cmd_vel"
```

<br>

To kill all terminals, press `ctrl+b`, then press `k` (watch out that you don't have Caps Lock on). After killing the terminals, there might still be some ROS 2 nodes running in the background. To kill them, use the command: `kill_ros2`. The command is defined in `.bash_aliases`. Keep this in mind when starting the next session. :slightly_smiling_face:

## Working on your project

For developing your solution, you can either create a new package or you can continue to work in this package. You can write your code in Python or C++.

The folder structure of this package is:
1. worlds - contains the .sdf file of an empty Gazebo world.
2. scripts - additional node for static transformation broadcaster from world to odom.
3. launch - contains file to launch Gazebo simulation with crazyflies (sitl_multiagent_text.sh) with the initial poses from file in folder drone spawn list and launch file which starts crazyflies server, rviz and nodes for publishing velocity to crazyflies.
4. config - the configuration files for RViz and the main .yaml file for crazyflies server
5. startup - convenience scripts for starting the simulation and ROS2 nodes.

Feel free to add more windows or to create your own setups and sessions. :slightly_smiling_face:

> [!TIP]
> **Summary of configurations that need to be changed**
> - `config/crazyflies_mrs.yaml`
>   - Set enable to true/false or comment/uncomment the number of crazyflie specification blocks you want to use.
> - `launch/drone_spawn_list/...`
>   - Specify initial positions for each crazyflie you want to use. Each row is one crazyflie.
>   - Modify the existing file or create your own.
> - `startup/mrs_example_setup.sh`
>   - Modify the environment variables according to your needs.
>   - `$NUM_ROB` - Number of crazyflies you want to use. Should correspond to the number of enabled crazyflies in the `crazyflies_mrs.yaml` and the number of rows in the initial positions file.
>   - `$SPAWN_POSE_DOC` - Path to the .txt file with initial positions for each crazyflie (e.g., `launch/drone_spawn_list/four_example.txt`).
>   - `$ENV_NAME` - World name from `worlds` folder. Currently, only `empty_borderless` is available.
> - `startup/example.yml`
>   - You can create your own tmuxinator session files, or modify this one to your needs.
>   - You can add more panes/windows to run your own nodes/scripts.
>   - Update the file in use in `startup/start.sh` if you want to use a different session file.

