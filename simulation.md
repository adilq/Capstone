> note: these instructions are for ROS2 foxy with Gazebo Classic
    
> note: instructions may be inaccurate 

# setup

1. switch to `simulation` branch

2. install gazebo packages: `sudo apt install ros-foxy-gazebo-ros-pkgs`

3. update submodules: `git submodule update --init --recursive`

4. unzip any large model files: e.g., `unzip zebra.zip` in `PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/zebra/meshes/`
> (large files are zipped to push them to github)

# open a gazebo world

1. source ROS installation: i.e., `source /opt/ros/foxy/setup.bash`

2. run a world: `gazebo --verbose path/to/gazebo_world_file.world`
    * the `--verbose` flag prints out more details that may be useful for debugging
    * e.g., `gazebo --verbose PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/grassland.world`
  
> if missing model files or Ogre resources etc. are a recurring problem, try copying the problematic models (folders under `PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models`) to `~/.gazebo/models`. Or just skip ahead to the [drone simulation](#run-the-drone-simulation) section

## open a gazebo world with ROS interaction

1. source ROS installation

2. add the models folder to `GAZEBO_MODEL_PATH`: `export GAZEBO_MODEL_PATH=PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/`

3. run the `box_robot.world` world using the above instructions

4. echo the position of the box: `ros2 topic echo /box/joint_states`

5. move the box one unit in the `y`-direction: `ros2 topic pub /box/set_joint_trajectory trajectory_msgs/msg/JointTrajectory  "{header: {frame_id: world}, joint_names: {y_joint}, points: [{positions: {1}}]}"`

# run the drone simulation

## first time PX4 simulation setup

* make sure mavros is installed (from the [ROB498 repo](https://github.com/utiasSTARS/ROB498-flight/blob/main/instructions/hardware/jetson_nano.md)):
  
        sudo apt remove modemmanager
        sudo adduser ${USER} dialout
        
        sudo apt install ros-foxy-mavros-extras
        
        cd /opt/ros/foxy/lib/mavros/
        sudo ./install_geographiclib_datasets.sh 

resources

* [building PX4 software](https://docs.px4.io/main/en/dev_setup/building_px4.html)
* [PX4 ROS2 user guide](https://docs.px4.io/main/en/ros2/user_guide.html)
  > skip the _Setup Micro XRCE-DDS Agent & Client_ section

run
* run PX4 setup script: `bash ./PX4-Autopilot/Tools/setup/ubuntu.sh`
  > If you want to install PX4 but keep your existing simulator installation, run ubuntu.sh above with the `--no-sim-tools` flag
* make for the first time: `make px4_sitl` in `PX4-Autopilot`
* install additional dependencies: `pip install --user -U empy==3.3.4 pyros-genmsg setuptools`

## drone simulation

1. source ROS installation and `drone_ws/install/setup.bash`

2. select the world to use with `export PX4_SITL_WORLD=world_name`
    * e.g., `export PX4_SITL_WORLD=box_robot` (note no `.world` extension)
    * skip this step to use the default world

3. run the drone simulation: `make px4_sitl gazebo-classic` in the `PX4-Autopilot` folder
    * soon after the `home set` message, a green message saying `Ready for takeoff!` should appear. if it doesn't, try running it again.

4. launch mavros: `ros2 launch px4_autonomy_modules mavros.launch.py fcu_url:=udp://:14540@127.0.0.1:14557`
    * check ros2 is connected - if the message `High resolution IMU detected!` appears, everything's good
    * try echoing some topics such as `/mavros/local_position/pose`

5. profit

## zebra box

1. source ROS installation and workspace

2. select `box_robot` world: `export PX4_SITL_WORLD=box_robot` (no `.world` extension!)

3. run drone simulation with downward-facing camera: `make px4_sitl gazebo-classic_iris_downward_depth_camera`

4. launch mavros

5. move box and publish its pose: `ros2 launch simulation loop_path_launch.yaml`

6. visualise in rviz: `rviz2`

relevant topics:

* camera images: `/camera/image_raw`
* box position: `/box/position/global`
* drone position: `mavros/local_position/pose`

# nodes

world: **box_robot**

* `ros2 run simulation line_path`: box moves from 0 to +10 in the x direction

* `ros2 run simulation loop_path`: box does a big loop in something of a cardioid shape. modify the parameters in `loop_path.py` to adjust size and speed.

* `ros2 run simulation joint_to_pose`: publish the box's position as a `PoseStamped` message in the `map` frame (global).
