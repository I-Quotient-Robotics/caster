#!/usr/bin/env python

import random
import math
import struct

import numpy as np

import rospy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import ChannelFloat32
from geometry_msgs.msg import Point32

import icp

def GenerateIdealDock(points_count):
    global viz_pointcloud

    base_length = 0.40239               # 402.39mm
    hypotenuse_length = 0.17518         # 175.18mm
    base_angle = math.pi / 3.0 * 2.0    # 120 degrees

    base_points_count = points_count/4*2 + points_count%4
    hypotenuse_points_count = points_count / 4

    base_step_delta = base_length / base_points_count
    hypotenuse_delta_x = math.sin(base_angle) * hypotenuse_length / hypotenuse_points_count
    hypotenuse_delta_y = math.cos(base_angle) * hypotenuse_length / hypotenuse_points_count

    ideal_dock_cloud = []

    # right hypotenuse points
    for i in range(hypotenuse_points_count):
        point = Point32()
        point.x = hypotenuse_delta_x * (hypotenuse_points_count-i)
        point.y = -(base_length/2.0) + hypotenuse_delta_y*(hypotenuse_points_count-i)
        point.z = 0.0 # i * 0.01
        ideal_dock_cloud.append(point)

    # base points
    for i in range(base_points_count):
        point = Point32(0.0, base_length/2.0-i*base_step_delta, 0.0)
        ideal_dock_cloud.append(point)

    # left hypotenuse points
    for i in range(hypotenuse_points_count):
        point = Point32()
        point.x = hypotenuse_delta_x * (hypotenuse_points_count-i)
        point.y = (base_length/2.0) - hypotenuse_delta_y*(hypotenuse_points_count-i)
        point.z = 0.0 # i * 0.01
        ideal_dock_cloud.append(point)

    # viz ideal dock
    viz_pointcloud = PointCloud()
    viz_pointcloud.header.frame_id = "base_footprint"
    viz_pointcloud.header.stamp = rospy.Time.now()

    viz_pointcloud.points = ideal_dock_cloud

def callback(data):
    global cluster_scan_pub, cluster_pointcloud_pub, cluster_pointcloud
    angle_min = data.angle_min
    angle_max = data.angle_max
    angle_increment = data.angle_increment

    # rospy.loginfo("scan type, min angle: %f, max_angle: %f, increment: %f" % (angle_min, angle_max, angle_increment))
    # rospy.loginfo("get scan data: %d points" % (len(data.ranges)))

    last_range = 0;
    cluster_index = 0

    point_count = 0
    clouds = []
    potential_clouds = []

    for i in xrange(len(data.ranges)):
        if(abs(data.ranges[i] - last_range) > 0.03):
            # filter cloud with points count
            if(len(clouds)>80):
                potential_clouds.append(clouds)
            clouds = []
            # cluster_index += 1

        point = Point32()
        point.x = math.cos(angle_min + i * angle_increment) * data.ranges[i];
        point.y = math.sin(angle_min + i * angle_increment) * data.ranges[i];
        point.z = 0
        clouds.append(point)
        # cluster_pointcloud.points.append(point)
        # color_r.values.append(color_map[cluster_index%4])

        last_range = data.ranges[i]

    # add last cloud into potential_clouds
    # filter cloud with points count
    if(len(clouds)>80):
        potential_clouds.append(clouds)

    cluster_pointcloud = PointCloud()
    cluster_pointcloud.header = data.header
    cluster_pointcloud.header.stamp = rospy.Time.now()
    
    color_r = ChannelFloat32('rgb', [])
    color_map = [0x00ff00, 0xffff00, 0xff00ff, 0x00ffff]

    for i in xrange(len(potential_clouds)):
        for j in xrange(len(potential_clouds[i])):
            cluster_pointcloud.points.append(potential_clouds[i][j])
            color_r.values.append(color_map[i%4])
    
    cluster_pointcloud.channels.append(color_r)

def main():
    global cluster_scan_pub, cluster_pointcloud_pub, cluster_pointcloud, viz_pointcloud

    rospy.init_node('dock_detect', anonymous=True)

    rospy.Subscriber("scan", LaserScan, callback)
    cluster_scan_pub = rospy.Publisher('cluster_scan', LaserScan, queue_size=10)
    cluster_pointcloud_pub = rospy.Publisher('cluster_pointcloud', PointCloud, queue_size=10)

    cluster_pointcloud = PointCloud()

    rate = rospy.Rate(2) # default 1hz
    GenerateIdealDock(130)
    while not rospy.is_shutdown():
        # cluster_pointcloud_pub.publish(cluster_pointcloud)
        cluster_pointcloud_pub.publish(viz_pointcloud)
        rate.sleep()

if __name__ == '__main__':
    main()