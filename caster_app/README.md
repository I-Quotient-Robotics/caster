# Caster APP
ROS package for caster app

## 1 Nodes

### 1.1 dock_detect.py

This node provides dock pose when dock is in lidar detect range

#### 1.1.1 Subscribed Topics

`scan` ([sensor_msgs/LaserScan](http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html))

Laser scans

#### 1.1.2 Published Topics

`dock_pose` ([geometry_msgs/PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html))

Dock pose in laser_link frame

`dock_pointcloud` ([sensor_msgs/PointCloud](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud.html))

Ideal dock pointcloud in detected pose

`cluster_pointcloud` ([sensor_msgs/PointCloud](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud.html))

Potential dock pointcloud

#### 1.1.3 Transforms

dock_detect.py provides TF bewteen `laser_link` and `dock`

### 1.2 dock_server.py

This node provides auto dock function for caster robot

#### 1.2.1 Action API

The `dock_server.py` node provides an implementation of the `ActionServer` (see [actionlib documentation](http://wiki.ros.org/actionlib)), that takes in goals containing `caster_app/Dock` (see [caster_app/Dock](action/Dock.action)) messages. You can communicate with the `dock_server.py` node over ROS directly, but the recommended way to send goals to `dock_server.py` if you care about tracking their status is by using the `SimpleActionClient`. Please see [actionlib documentation](http://wiki.ros.org/actionlib) for more information

##### 1.2.1.1 Action Subscribed Topics

`dock_action/goal` (caster_app/DockActionGoal)

- A goal for `dock_action`

`dock_action/cancel` ([actionlib_msgs/GoalID](http://docs.ros.org/api/actionlib_msgs/html/msg/GoalID.html))

- A request to cancel a specific goal

##### 1.2.1.2 Action Published Topics

`dock_action/feedback` (caster_app/DockActionFeedback)

- Feedback contains the current status of auto dock action

`dock_action/status` ([actionlib_msgs/GoalStatusArray](http://docs.ros.org/api/actionlib_msgs/html/msg/GoalStatusArray.html))

- Provides status information on the goals that are sent to the `dock_action` action

`dock_action/result` (caster_app/DockActionResult)

- Result for the auto dock action

#### 1.2.2 Subscribed Topics

`dock_pose` ([geometry_msgs/PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html))

Provides the dock pose in laser_link frame

#### 1.2.3 Published Topics

`cmd_vel` ([geometry_msgs/Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html))

A stream of velocity commands meant for execution by a mobile base.

#### 1.2.4 Parameters

| Name                 | Type   | Default        | Description                                                  |
| -------------------- | ------ | -------------- | ------------------------------------------------------------ |
| ~debug               | string | false          | debug mode                                                   |
| ~robot_radius        | double | 0.27           | robot radius, this value determine the distance between dock front-side and docked-robot ahead(delta_d), delta_d = dock_distance - robot_radius |
| ~base_frame          | String | base_footprint | base frame                                                   |
| ~map_frame           | string | map            | map frame                                                    |
| ~odom_fram           | string | odom           | odom frame                                                   |
| ~dock/dock_distance  | double | 1.0            | distance between dock front-side and DockReady2 point        |
| ~dock/dock_speed     | double | 0.05           | move speed for docking                                       |
| ~/dock/pose_x        | double |                | DockReady point x                                            |
| ~/dock/pose_y        | double |                | DockReady point y                                            |
| ~/dock/pose_z        | double |                | DockReady point z                                            |
| ~/dock/orientation_x | double |                | DockReady point orientation x                                |
| ~/dock/orientation_y | double |                | DockReady point orientation y                                |
| ~/dock/orientation_z | double |                | DockReady point orientation z                                |
| ~/dock/orientation_w | double |                | DockReady point orientation w                                |

## 2 Launch files

#### 2.1 caster_app.launch

This launch file runs `dock_detect.py` and `dock_server.py` with specific params in `caster_app.yaml`

