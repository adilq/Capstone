# Setup Instructions

1. extract models `.zip` files into `~/.gazebo/models/` (should end up with directory structure like `~/.gazebo/models/zebra/`)

2. move `.world` files into appropriate worlds directory of PX4 simulations: `PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/`

3. run world (without drone) with `gazebo drone_world.world --verbose` in the directory with the world file (replace `drone_world` with the appropriate world name; `--verbose` option for debugging)

4. to run the `drone_world`  world with the drone simulation, first specify which world to run by exporting `export PX4_SITL_WORLD=drone_world` and then run the simulation as normal with `make px4_sitl gazebo-classic` in the `PX4-Autopilot` directory. (the default world is `empty.world`)
