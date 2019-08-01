# Caster APP
ROS package for caster app

## Nodes

### dock_server.py

This node provides auto dock function for caster robot

#### Action API

The `dock_server.py` node provides an implementation of the `ActionServer` (see [actionlib documentation](http://wiki.ros.org/actionlib)), that takes in goals containing `caster_app/Dock` (see [caster_app/Dock](action/ock.action)) messages. You can communicate with the `dock_server.py` node over ROS directly, but the recommended way to send goals to `dock_server.py` if you care about tracking their status is by using the `SimpleActionClient`. Please see [actionlib documentation](http://wiki.ros.org/actionlib) for more information

##### Action Subscribed Topics

`dock_action/goal` (caster_app/DockActionGoal)

- A goal for `dock_action`

`dock_action/cancel` ([actionlib_msgs/GoalID](http://docs.ros.org/api/actionlib_msgs/html/msg/GoalID.html))

- A request to cancel a specific goal

##### Action Published Topics

`dock_action/feedback` (caster_app/DockActionFeedback)

- Feedback contains the current status of auto dock action

`dock_action/status` ([actionlib_msgs/GoalStatusArray](http://docs.ros.org/api/actionlib_msgs/html/msg/GoalStatusArray.html))

- Provides status information on the goals that are sent to the `dock_action` action

`dock_action/result` (caster_app/DockActionResult)

- Result for the auto dock action

#### Subscribed Topics

`dock_pose` ([geometry_msgs/PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html))

- Provides the dock pose in laser_link frame

#### Published Topics

`cmd_vel` ([geometry_msgs/Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html))

- A stream of velocity commands meant for execution by a mobile base.

#### Parameters

`~debug` (string, default: false)

- Debug mode

`~robot_radius` (double, default: 0.27)

- Robot radius, this value determine the distance between dock front-side and docked-robot ahead(delta_d)
- delta_d = dock_distance - robot_radius

`~map_frame` (string, default: map)

- Map frame

`~odom_frame` (, default:  odom)

- Odom frame

`~base_frame` (string, default: base_footprint)

- Robot base frame

`~dock/speed`(double, default: 0.05)

- Move speed for docking

`~dock/dock_distance`(double, default: 1.0)

- Distance between dock front-side and DockReady2 point 

`~/dock/pose_x` (double, default: )

- DockReady point x

`~/dock/pose_y` (double, default: )

- DockReady point y

`~/dock/pose_z` (double, default: )

- DockReady point z

`~/dock/orientation_x `(double, default: )

- DockReady point orientation_x

`~/dock/orientation_y` (double, default: )

- DockReady point orientation_y

`~/dock/orientation_z` (double, default: )

- DockReady point orientation_z

`~/dock/orientation_w` (double, default: )

- DockReady point orientation_w