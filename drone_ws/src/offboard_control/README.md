# Usage
## node: flight2

run: `ros2 run offboard_control flight2`

modes:

* `CONNECT`: publish setpoints, arm the drone and set to offboard mode
* `TAKEOFF`: publish setpoints a distance of `TAKEOFF_INCREMENT` above the drone's position until the goal altitude is reached
* `HOVER`: publish setpoints at goal position
* `LAND`: publish setpoints a distance of `LANDING_INCREMENT` below the drone's position
* `ABORT`: put drone into `AUTO.LAND` mode. sets `mode = GROUND` once landing is detected and puts drone into `AUTO.LOITER`
* `GROUND`: enables launch
    
> tip: send `abort` command to re-enable `launch` after landing

set `GOAL_HEIGHT` in `flight_test_2.py` to desired goal height

### simulation

1. run PX4 simulation: `make px4_sitl gazebo-classic` in `PX4-Autopilot`

2. run `mavros`: `ros2 launch px4_autonomy_modules mavros.launch.py fcu_url:=udp://:14540@127.0.0.1:14557`

3. run node for flight exercise 2: `ros2 run offboard_control flight2`

4. call services

    * `launch`: `ros2 service call /rob498_drone_08/comm/launch std_srvs/srv/Trigger`

    * `test`: `ros2 service call /rob498_drone_08/comm/test std_srvs/srv/Trigger`

    * `land`: `ros2 service call /rob498_drone_08/comm/land std_srvs/srv/Trigger`

    * `abort`: `ros2 service call /rob498_drone_08/comm/abort std_srvs/srv/Trigger`

## node: flight3

run: `ros2 run offboard_control flight3`

modes:

* `CONNECT`: publish setpoints, arm the drone and set to offboard mode
* `TAKEOFF`: publish setpoints a distance of `TAKEOFF_INCREMENT` above the drone's position until the goal altitude is reached
* `HOVER`: publish setpoints at initial goal height
* `NAVIGATE`: publish an array of waypoints to visit
* `LAND`: publish setpoints a distance of `LANDING_INCREMENT` below the drone's position
* `ABORT`: put drone into `AUTO.LAND` mode. sets `mode = GROUND` once landing is detected and puts drone into `AUTO.LOITER`
* `GROUND`: enables launch
    
> tip: send `abort` command to re-enable `launch` after landing

set `GOAL_HEIGHT` in `flight_test_2.py` to desired goal height

### simulation

1. run PX4 simulation: `make px4_sitl gazebo-classic` in `PX4-Autopilot`

2. run `mavros`: `ros2 launch px4_autonomy_modules mavros.launch.py fcu_url:=udp://:14540@127.0.0.1:14557`

3. run node for flight exercise 2: `ros2 run offboard_control flight3`

4. run node for publishing waypoints: `ros2 run offboard_control waypoints`

5. call services

    * `launch`: `ros2 service call /rob498_drone_8/comm/launch std_srvs/srv/Trigger`

    * `test`: `ros2 service call /rob498_drone_8/comm/test std_srvs/srv/Trigger`

    * `land`: `ros2 service call /rob498_drone_8/comm/land std_srvs/srv/Trigger`

    * `abort`: `ros2 service call /rob498_drone_8/comm/abort std_srvs/srv/Trigger`

## node: project
Camera frame for reference
![alt text](\Capstone\imgs\drone_axis.jpg "camframe")
