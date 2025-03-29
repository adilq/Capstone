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

## open a gazebo world with ROS interaction

1. source ROS installation

2. add the models folder to `GAZEBO_MODEL_PATH`: `export GAZEBO_MODEL_PATH=PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/`

3. run the `box_robot.world` world using the above instructions

4. echo the position of the box: `ros2 topic echo /box/joint_states`

5. move the box one unit in the `y`-direction: `ros2 topic pub /box/set_joint_trajectory trajectory_msgs/msg/JointTrajectory  "{header: {frame_id: world}, joint_names: {y_joint}, points: [{positions: {1}}]}"`

# run the drone simulation

1. source ROS installation and `drone_ws/devel/setup.bash`

2. select the world to use with `export PX4_SITL_WORLD=world_name`
    * e.g., `export PX4_SITL_WORLD=box_robot` (note no `.world` extension)
    * skip this step to use the default world

3. run the drone simulation: `make px4_sitl gazebo-classic`

4. launch mavros: `ros2 launch px4_autonomy_modules mavros.launch.py fcu_url:=udp://:14540@127.0.0.1:14557`

5. profit

## zebra box

1. source ROS installation and workspace

2. select `box_robot` world: `export PX4_SITL_WORLD=box_robot`

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
