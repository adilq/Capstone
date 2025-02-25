# node: flight2

run: `ros2 run offboard_control flight2`

modes:

* `CONNECT`: publish setpoints, arm the drone and set to offboard mode
* `TAKEOFF`: publish setpoints a distance of `TAKEOFF_INCREMENT` above the drone's position until the goal altitude is reached
* `HOVER`: publish setpoints at goal position
* `LAND`: publish setpoints a distance of `LANDING_INCREMENT` below the drone's position
* `ABORT`: put drone into `AUTO.LAND` mode. sets `mode = GROUND` once landing is detected and puts drone into `AUTO.LOITER`
* `GROUND`: enables launch
    
> tip: send `abort` command to re-enable `launch` after landing