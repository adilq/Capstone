    note: these instructions are for ROS2 foxy with Gazebo Classic
    
    note: instructions may be inaccurate 

# setup

1. switch to `simulation` branch

2. update submodules: `git submodule update --init --recursive`

3. unzip any large model files: e.g., `unzip zebra.zip` in `PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/zebra/meshes/'
    (large files are zipped to push them to github)

# open a gazebo world

1. source ROS installation: i.e., `source /opt/ros/foxy/setup.bash`

2. run a world: `gazebo --verbose path/to/gazebo_world_file.world`
    * the `--verbose` flag prints out more details that may be useful for debugging
    * e.g., `gazebo --verbose PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/grassland.world`

## open a gazebo world with ROS interaction

1. source ROS installation

2. run the `box_robot.world` world using the above instructions

3. echo the position of the box: `ros2 topic echo /box/joint_states`

4. move the box one unit in the `y`-direction: `ros2 topic pub /box/set_joint_trajectory trajectory_msgs/msg/JointTrajectory  "{header: {frame_id: world}, joint_names: {y_joint}, points: [{positions: {1}}]}"`

# run the drone simulation

1. source ROS installation and `drone_ws/devel/setup.bash`

2. select the world to use with `export PX4_SITL_WORLD=world_name`
    * e.g., `export PX4_SITL_WORLD=box_robot` (note no `.world` extension)
    * skip this step to use the default world

3. run the drone simulation: `make px4_sitl gazebo-classic`

4. launch mavros: `ros2 launch px4_autonomy_modules mavros.launch.py fcu_url:=udp://:14540@127.0.0.1:14557`
