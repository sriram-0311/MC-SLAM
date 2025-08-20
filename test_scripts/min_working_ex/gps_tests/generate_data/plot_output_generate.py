"""
:Plot the output and the generated data

"""

import matplotlib.pyplot as plt
import numpy as np

straight = True

if straight:
    gt_file = "/home/neural/catkin_ws/src/TRI-SLAM/test_scripts/min_working_ex/gps_tests/data/straight/sensor_data_straight_gt.txt"

else:
    gt_file = "/home/neural/catkin_ws/src/TRI-SLAM/test_scripts/min_working_ex/gps_tests/data/curve/sensor_data_curve_gt.txt"


# initial_gps_file = "/home/neural/catkin_ws/src/TRI-SLAM/test_scripts/min_working_ex/gps_tests/data/curve/sensor_data_curve_rot_15.txt"
# output_file = "/home/neural/catkin_ws/src/TRI-SLAM/test_scripts/min_working_ex/gps_tests/results/output_sensor_data_curve_rot_15.txt"

initial_gps_file = "/home/neural/catkin_ws/src/TRI-SLAM/test_scripts/min_working_ex/gps_tests/data/straight/sensor_data_straight_rot_10.txt"
output_file = "/home/neural/catkin_ws/src/TRI-SLAM/test_scripts/min_working_ex/gps_tests/results/output_sensor_data_straight_rot_10.txt"

# Plot the ground truth
gt_data_x = []
gt_data_y = []
with open(gt_file) as f:

    for line in f:
        parts = line.split() 
        
        if parts[0] == "x":
            print(parts)
            x = float(parts[3])
            y = float(parts[4])
        
            gt_data_x.append(x)
            gt_data_y.append(y)


# Plot the initial gps data
initial_gps_data_x = []
initial_gps_data_y = []
with open(initial_gps_file) as f:

    for line in f:
        parts = line.split() 
        
        if parts[0] == "g":
            print(parts)
            x = float(parts[3])
            y = float(parts[4])
        
            initial_gps_data_x.append(x)
            initial_gps_data_y.append(y)


# Plot the output


output_data_x = []
output_data_y = []

with open(output_file) as f:

    for line in f:
        parts = line.split() 
        x = float(parts[2])
        y = float(parts[3])
       
        output_data_x.append(x)
        output_data_y.append(y)



# Plot the ground truth

plt.plot(initial_gps_data_x, initial_gps_data_y, label="Initial GPS", color="red", linestyle="dashed")
plt.plot(output_data_x, output_data_y, label="Output", color="green", linestyle="dashed")
plt.plot(gt_data_x, gt_data_y, label="Ground Truth", color="black", linestyle="dotted")
plt.legend()
plt.show()