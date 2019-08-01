#!/usr/bin/env python

import math
import rospy
import tf

from actionlib import SimpleActionClient
from actionlib.action_server import ActionServer

from geometry_msgs.msg import Pose, Twist, PoseStamped
from move_base_msgs.msg import MoveBaseGoal, MoveBaseFeedback, MoveBaseAction

from caster_base.srv import SetDigitalOutput
from caster_app.msg import DockAction, DockFeedback, DockResult

class DockActionServer(ActionServer):
    def __init__(self, name):
        ActionServer.__init__(self, name, DockAction, self.__goal_callback, self.__cancel_callback, False)

        self.__docked = False
        
        self.__dock_speed = rospy.get_param("~dock/dock_speed", 0.05)
        self.__dock_distance = rospy.get_param("~dock/dock_distance", 1.0)
        self.__map_frame = rospy.get_param("~map_frame", "map")
        self.__odom_frame = rospy.get_param("~odom_frame", "odom")
        self.__base_frame = rospy.get_param("~base_frame", "base_footprint")

        self.__dock_ready_pose = Pose()
        self.__dock_ready_pose.position.x = rospy.get_param("~dock/pose_x")
        self.__dock_ready_pose.position.y = rospy.get_param("~dock/pose_y")
        self.__dock_ready_pose.position.z = rospy.get_param("~dock/pose_z")
        self.__dock_ready_pose.orientation.x = rospy.get_param("~dock/orientation_x")
        self.__dock_ready_pose.orientation.y = rospy.get_param("~dock/orientation_y")
        self.__dock_ready_pose.orientation.z = rospy.get_param("~dock/orientation_z")
        self.__dock_ready_pose.orientation.w = rospy.get_param("~dock/orientation_w")

        self.__dock_ready_pose_2 = PoseStamped()

        rospy.loginfo("param: dock_spped, %s, dock_distance %s" % (self.__dock_speed, self.__dock_distance))
        rospy.loginfo("param: map_frame %s, odom_frame %s, base_frame %s" % (self.__map_frame, self.__odom_frame, self.__base_frame))
        rospy.loginfo("dock_pose:")
        rospy.loginfo(self.__dock_ready_pose)

        self.__movebase_client = SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("wait for movebase server...")
        self.__movebase_client.wait_for_server()
        rospy.loginfo("movebase server connected")

        self.__cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        rospy.Subscriber("dock_pose", PoseStamped, self.__dock_pose_callback)

        self.__tf_listener = tf.TransformListener()

        self.__docked = False
        self.__saved_goal = MoveBaseGoal()

        rospy.loginfo("Creating ActionServer [%s]\n", name)

    def __dock_pose_callback(self, data):
        ps = PoseStamped()
        # ps.header.stamp = rospy.Time.now()
        ps.header.frame_id = "dock"
        ps.pose.position.x = -self.__dock_distance

        try:
            self.__dock_ready_pose_2 = self.__tf_listener.transformPose("map", ps)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            self.__dock_ready_pose_2.pose.position.z = -1.0
            rospy.logwarn("tf 1error, %s" % e)

        self.__dock_ready_pose_2.pose.position.z = 0.0

        # rospy.loginfo("get dock pose")

    def __goal_callback(self, gh):
        self.__saved_gh = gh
        self.__saved_goal = gh.get_goal()

        if self.__saved_goal.dock == True:
            if self.__docked == True:
                rospy.logwarn("rejected, robot has already docked")
                gh.set_rejected(None, "already docked")
            else: 
                rospy.loginfo("Docking")
                gh.set_accepted("Docking")
                self.__moveto_dock_ready_allin()
        elif self.__saved_goal.dock == False:
            if self.__docked == False:
                rospy.logwarn("cancel_all_goals")
                self.__movebase_client.cancel_all_goals()
                cmd = Twist()
                cmd.linear.x = self.__dock_speed
                cmd.linear.x = 0
                self.__cmd_pub.publish(cmd)
                rospy.Rate(1).sleep()
                rospy.logwarn("rejected, robot is not on charging")
                gh.set_rejected(None, "robot is not on charging")
            else: 
                rospy.loginfo("Start undock")
                gh.set_accepted("Start undock")
                self.__undock()
        else:
            rospy.logwarn("unknown dock data type, should be true or false")

    def __set_charge_relay(self, state):
        rospy.loginfo("set relay %d" % state)
        rospy.loginfo("check caster base service...")
        rospy.wait_for_service('caster_base_node/set_digital_output')
        rospy.loginfo("service exist")

        try:
            set_digital_output = rospy.ServiceProxy('caster_base_node/set_digital_output', SetDigitalOutput)
            resp = set_digital_output(4, state)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def __cancel_callback(self, gh):
        self.__movebase_client.cancel_goal()
        rospy.logwarn("cancel callback")

    def __moveto_dock(self):
        cmd = Twist()
        cmd.linear.x = self.__dock_speed

        ca_feedback = DockFeedback()

        try :
            last_pose, last_quaternion = self.__tf_listener.lookupTransform(self.__odom_frame, self.__base_frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn(e)
            rospy.logwarn("tf error")

        delta_distance = 0
        while delta_distance < self.__dock_distance-0.30 and not rospy.is_shutdown():
            self.__cmd_pub.publish(cmd)

            try :
                current_pose, current_quaternion = self.__tf_listener.lookupTransform(self.__odom_frame, self.__base_frame, rospy.Time(0))
                delta_distance = math.sqrt(math.pow(current_pose[0]-last_pose[0],2)+math.pow(current_pose[1]-last_pose[1],2)+math.pow(current_pose[2]-last_pose[2],2))

                ca_feedback.dock_feedback = "Moving to Dock, %fm left" % (self.__dock_distance-delta_distance)
                self.__saved_gh.publish_feedback(ca_feedback)

                rospy.loginfo("Moving to Dock, %fm left" % (self.__dock_distance-delta_distance))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logwarn(e)
                rospy.logwarn("tf error aa")

            rospy.Rate(20).sleep()

        ca_feedback.dock_feedback = "Stop on Dock"
        self.__saved_gh.publish_feedback(ca_feedback)
        rospy.loginfo("stop robot")

        # stop robot
        cmd.linear.x = 0
        self.__cmd_pub.publish(cmd)

        # set charge relay on
        self.__set_charge_relay(True)

        self.__docked = True
        self.__saved_gh.set_succeeded(None, "Docked")
        rospy.loginfo("Docked")

    def __moveto_dock_ready_allin(self):
        # step 1
        mb_goal = MoveBaseGoal()
        mb_goal.target_pose.header.stamp = rospy.Time.now()
        mb_goal.target_pose.header.frame_id = self.__map_frame
        mb_goal.target_pose.pose = self.__dock_ready_pose

        self.__movebase_client.send_goal(mb_goal)

        self.__movebase_client.wait_for_result()
        rospy.loginfo("arrived __dock_ready_pose")

        rospy.Rate(1).sleep()

        mb_goal.target_pose.header.stamp = rospy.Time.now()
        mb_goal.target_pose.header.frame_id = self.__map_frame

        if self.__dock_ready_pose_2.pose.position.z == -0.3:
            rospy.logwarn("dock_ready_pose_2 failed")
            return
        else:
            t = self.__dock_ready_pose_2.pose

        t.position.z == 0.0
        mb_goal.target_pose.pose = t

        rospy.logwarn("move to dock_ready_pose_2")
        self.__movebase_client.send_goal(mb_goal)

        self.__movebase_client.wait_for_result()
        rospy.loginfo("arrived dock_ready_pose_2")

        self.__moveto_dock()

    def __undock(self):
        cmd = Twist()
        cmd.linear.x = -self.__dock_speed

        ca_feedback = DockFeedback()

        try :
            last_pose, last_quaternion = self.__tf_listener.lookupTransform(self.__odom_frame, self.__base_frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn(e)
            rospy.logwarn("tf error")

        delta_distance = 0
        while delta_distance < self.__dock_distance and not rospy.is_shutdown():
            self.__cmd_pub.publish(cmd)

            try :
                current_pose, current_quaternion = self.__tf_listener.lookupTransform(self.__odom_frame, self.__base_frame, rospy.Time(0))
                delta_distance = math.sqrt(math.pow(current_pose[0]-last_pose[0],2)+math.pow(current_pose[1]-last_pose[1],2)+math.pow(current_pose[2]-last_pose[2],2))

                ca_feedback.dock_feedback = "Undock, %fm left" % (self.__dock_distance-delta_distance)
                self.__saved_gh.publish_feedback(ca_feedback)

                rospy.loginfo("Undock, %fm left" % (self.__dock_distance-delta_distance))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logwarn(e)
                rospy.logwarn("tf error aa")

            rospy.Rate(20).sleep()

        ca_feedback.dock_feedback = "Stop on DockReady"
        self.__saved_gh.publish_feedback(ca_feedback)
        rospy.loginfo("stop robot")

        # stop robot
        cmd.linear.x = 0.0
        self.__cmd_pub.publish(cmd)

        # set charge relay off
        self.__set_charge_relay(False)

        self.__docked = False
        self.__saved_gh.set_succeeded(None, "Undocked")
        rospy.loginfo("UnDocked")

def main():
    rospy.init_node("dock_server")
    ref_server = DockActionServer("dock_action")
    ref_server.start()

    rospy.spin()

if __name__ == '__main__':
    main()