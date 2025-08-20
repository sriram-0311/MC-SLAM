#!/usr/bin/env python

import rospy
import rosbag
import sys

# rosbag_file = "/home/marley/datasets/ISEC/5_1_zed.bag"
rosbag_file = "/home/aryaman/catkin_ws/bagfiles/combined_kri.bag"
output_file = "/home/aryaman/catkin_ws/src/TRI-SLAM/test_scripts/gps_imu_fusion/data/imu_test/imu_test_6.txt"


bag = rosbag.Bag(rosbag_file)

imu_topic = "/imu/imu_uncompensated" #/imu/imu_uncompensated
# imu_topic = "/imu"


with open(output_file, 'w+') as f:

    f.write("Time dt accelX accelY accelZ omegaX omegaY omegaZ\n")

    prev_time = 0

    # for topic, msg, t in bag.read_messages(topics=['/zed2i/zed_node/imu/data']):
    for topic, msg, t in bag.read_messages(topics=[imu_topic]):
        
        # if imu_topic=="/imu":
            
        if imu_topic == "/zed2i/zed_node/imu/data":
            time = str(msg.header.stamp)
            # convert to seconds

            time_sec = msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9



            dt = str(time_sec - prev_time)
            accelX = str(msg.linear_acceleration.x)
            accelY = str(msg.linear_acceleration.y)
            accelZ = str(msg.linear_acceleration.z)
            omegaX = str(msg.angular_velocity.x)
            omegaY = str(msg.angular_velocity.y)
            omegaZ = str(msg.angular_velocity.z)

            f.write(str(time_sec) + " " + str(dt) + " " + accelX + " " + accelY + " " + accelZ + " " + omegaX + " " + omegaY + " " + omegaZ + "\n")

            prev_time = time_sec

        elif imu_topic == "/imu/imu_uncompensated":

            time = str(msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9)
            dt = str(msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9 - prev_time)
		
            accelX = str(msg.linear_acceleration.x)
            accelY = str(msg.linear_acceleration.y)
            accelZ = str(msg.linear_acceleration.z)
            omegaX = str(msg.angular_velocity.x)
            omegaY = str(msg.angular_velocity.y)
            omegaZ = str(msg.angular_velocity.z)

            f.write(time + " " + dt + " " + accelX + " " + accelY + " " + accelZ + " " + omegaX + " " + omegaY + " " + omegaZ + "\n")

            prev_time = msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9
        

bag.close()
