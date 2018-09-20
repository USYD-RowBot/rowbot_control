# Rowbot Control

Controls the rowbot in Gazebo.

## Requirements

As well as the standard rowbot requirements, this package also needs:
- rowbot_msgs
- shapely (pip install shapely)

## Running rowbot_control

# Option 1 - basic operation

To launch the course controller for Gazebo:
```bash
roslaunch rowbot_control basic_control.launch
```
You can then publish a desired course using:
```bash
 rostopic pub -r 10 /cmd_course rowbot_msgs/Course  '{speed: 2.0, yaw: 0.0}'
```

## Option 2 - Waypoint Following

Ensure to not launch move_base or navigation when running this - #TODO make compatible.

To launch the waypoint following controller (this also launches the course controller):
```bash
roslaunch rowbot_control waypoint_control.launch
```

This controller listens for a nav_msgs/Path message on the topic /waypoints,
when it receives a path message it will follow the path until it is completed.

The following parameters can be added to the waypoint_control.launch folder as
private parameters for the navigate_wamv.py node:
- speed: the speed of the vehicle
- use_nlgl: Whether to use non-linear guidance law
- nlgl_radius: The radius for the NLGL algorithm - a larger number means more stability, a smaller number will turn towards the path more aggressively.
- tolerance: The allowable distance to the waypoint which will trigger the waypoint
- speed_control: Whether to use speed control - in dev, not working well.
- braking_distance: If speed control is on, the distance from the waypoint to start braking.
- minimum_forwards_speed: If speed control is on, the lower limit on the forwards speed
- request_mode: options:
  - 'hard': reaches the waypoints regardless
  - 'soft': if it veers to far from the path, or if it misses a waypoint. Requests more waypoints (and stops)
- allowable_distance_to_path: if request_mode is soft, the maxmimum distance from the path allowed until new waypoints are requested.


To send it some testing waypoints, use the simple_waypoints package:
```bash
roslaunch simple_waypoints simple_waypoints.launch
```

# Known bugs
- speed_control not working well - leave off
