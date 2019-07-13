#!/usr/bin/env python

import random
import math
import struct

import rospy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import ChannelFloat32
from geometry_msgs.msg import Point32

def callback(data):
    global cluster_scan_pub, cluster_pointcloud_pub, cluster_pointcloud
    angle_min = data.angle_min
    angle_max = data.angle_max
    angle_increment = data.angle_increment

    # rospy.loginfo("scan type, min angle: %f, max_angle: %f, increment: %f" % (angle_min, angle_max, angle_increment))
    # rospy.loginfo("get scan data: %d points" % (len(data.ranges)))

    cluster_scan = LaserScan()
    cluster_scan.header = data.header
    cluster_scan.header.stamp = rospy.Time.now()

    cluster_scan.angle_min = data.angle_min
    cluster_scan.angle_max = data.angle_max
    cluster_scan.angle_increment = data.angle_increment

    cluster_scan.time_increment = data.time_increment
    cluster_scan.scan_time = data.scan_time

    cluster_scan.range_min = data.range_min
    cluster_scan.range_max = data.range_max

    cluster_pointcloud = PointCloud()
    cluster_pointcloud.header = cluster_scan.header

    last_range = 0;
    cluster_index = 0

    color_r = ChannelFloat32('rgb', [])
    color_map = [0x00ff00, 0xffff00, 0xff00ff, 0x00ffff]

    point_count = 0

    for i in xrange(len(data.ranges)):
        if(abs(data.ranges[i] - last_range) > 0.025):
            cluster_index += 1
            # color = ChannelFloat32('rgb', [random.random(), random.random(), random.random()])
            # color = ChannelFloat32('rgb', [0, 0, 255])
            # if(cluster_index == 3):
            #     break
        
        cluster_scan.ranges.append(data.ranges[i])
        cluster_scan.intensities.append(1000 * cluster_index)

        point = Point32()
        point.x = math.cos(angle_min + i * angle_increment) * data.ranges[i];
        point.y = math.sin(angle_min + i * angle_increment) * data.ranges[i];
        point.z = 0
        cluster_pointcloud.points.append(point)
        color_r.values.append(color_map[cluster_index%4])
        # color_g.values.append(255)
        # color_b.values.append(255)
        # color_f.values.append(255)

        last_range = data.ranges[i]

    cluster_scan_pub.publish(cluster_scan)

    cluster_pointcloud.channels.append(color_r)
    # cluster_pointcloud.channels.append(color_g)
    # cluster_pointcloud.channels.append(color_b)
    # cluster_pointcloud.channels.append(color_f)
    # cluster_pointcloud_pub.publish(cluster_pointcloud)
    
def main():
    global cluster_scan_pub, cluster_pointcloud_pub, cluster_pointcloud

    rospy.init_node('dock_detect', anonymous=True)

    rospy.Subscriber("scan", LaserScan, callback)
    cluster_scan_pub = rospy.Publisher('cluster_scan', LaserScan, queue_size=10)
    cluster_pointcloud_pub = rospy.Publisher('cluster_pointcloud', PointCloud, queue_size=10)

    cluster_pointcloud = PointCloud()

    rate = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():
        cluster_pointcloud_pub.publish(cluster_pointcloud)
        rate.sleep()

if __name__ == '__main__':
    main()