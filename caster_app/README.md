# Caster APP
ROS package for caster app

## Nodes

### dock_server.py

This node provides auto dock function for caster robot

#### Action API

The `dock_server.py` node provides an implementation of the `ActionServer` (see [actionlib documentation](http://wiki.ros.org/actionlib)), that takes in goals containing `caster_app/Dock` (see [caster_app/Dock](action/ock.action)) messages. You can communicate with the `dock_server.py` node over ROS directly, but the recommended way to send goals to `dock_server.py` if you care about tracking their status is by using the `SimpleActionClient`. Please see [actionlib documentation](http://wiki.ros.org/actionlib) for more information

##### Action Subscribed Topics

`dock_action/goal` (caster_app/DockActionGoal)

- A goal for `move_base` to pursue in the world.

`move_base/cancel` ([actionlib_msgs/GoalID](http://docs.ros.org/api/actionlib_msgs/html/msg/GoalID.html))

- A request to cancel a specific goal.

##### Action Published Topics

`move_base/feedback` (caster_app/DockActionFeedback)

- Feedback contains the current position of the base in the world.

`move_base/status` ([actionlib_msgs/GoalStatusArray](http://docs.ros.org/api/actionlib_msgs/html/msg/GoalStatusArray.html))

- Provides status information on the goals that are sent to the `move_base` action.

`move_base/result` (caster_app/DockActionResult)

- Result is empty for the `move_base` action.