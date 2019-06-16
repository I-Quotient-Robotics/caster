
#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

global vel

def callback(data):
    vel.linear.x = data.axes[1] * 0.4
    vel.angular.z = data.axes[2] * 1

if __name__ == '__main__':
  pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
  rospy.Subscriber('joy', Joy, callback)

  rospy.init_node('joy_ctrl', anonymous=True)
  rate = rospy.Rate(10)

  vel = Twist()
  while not rospy.is_shutdown():
    pub.publish(vel)
    rate.sleep()

  #rospy.spin()