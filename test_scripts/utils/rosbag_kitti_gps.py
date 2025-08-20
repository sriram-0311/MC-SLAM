"""
code to read gps data from a rosbag, convert the lat long to ENU frame and write to a text file

"""

import rosbag
from sensor_msgs.msg import NavSatFix
from pyproj import Proj, transform
import numpy as np
import pymap3d as pm

bag_file = "/home/neural/datasets/highway_july_19_2023/KRI_2loops_3/combined_kri_loop.bag"
output_file = "/home/neural/catkin_ws/src/TRI-SLAM/test_scripts/gps_imu_fusion/data/kri/combined_kri_loop_gps_data.txt"

gps_topic = "/fix"

# Define the reference and target coordinate systems for conversion
reference_frame = "WGS84"  
target_frame = "EPSG:5973" 

# Initialize empty lists to store the data
timestamps = []
x_values = []
y_values = []
z_values = []


# Open the ROS bag file
bag = rosbag.Bag(bag_file)
first_message = True
lat0 = 0
lon0 = 0
alt0 = 0

for topic, msg, t in bag.read_messages(topics=[gps_topic]):


    if msg.header.stamp.secs > 1689614041:  #the data starts from here for gps - since it stays its at the same place for the first 10 seconds

        try:

            if first_message:
                print("first message")
                first_message = False
                lat0 = msg.latitude
                lon0 = msg.longitude
                alt0 = msg.altitude
                print("lat0:", lat0)
                print("lon0:", lon0)
                print("alt0:", alt0)
                
                first_message = False

            print("converting message")

            # Perform coordinate transformation
            x, y, z = pm.geodetic2enu(msg.latitude, msg.longitude, msg.altitude, lat0, lon0, alt0)

            # Append the data to the lists
            timestamps.append(msg.header.stamp.to_sec())
            x_values.append(x)
            y_values.append(y)
            z_values.append(z)
        
        except Exception as e:
            print("Coordinate transformation failed for timestamp:", msg.header.stamp)
            print(e)

bag.close()

# Save the data to the output text file
data = np.column_stack((timestamps, x_values, y_values, z_values))
np.savetxt(output_file, data, fmt="%.6f")

print("Data saved to", output_file)
