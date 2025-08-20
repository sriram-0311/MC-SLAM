"""
:Plot the output and the generated data

"""

import matplotlib.pyplot as plt
import numpy as np


# Plot the ground truth
gt_file = "/home/neural/catkin_ws/src/TRI-SLAM/test_scripts/min_working_ex/gps_imu_fusion/data/sensor_data_gt1.txt"
gt_data_x = []
gt_data_y = []
with open(gt_file) as f:

    for line in f:
        parts = line.split() 
        
        if parts[0] == "g":
            print(parts)
            x = float(parts[3])
            y = float(parts[4])
        
            gt_data_x.append(x)
            gt_data_y.append(y)


# Plot the output
output_file = "/home/neural/catkin_ws/src/TRI-SLAM/test_scripts/min_working_ex/gps_imu_fusion/data/output.txt"

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
plt.plot(gt_data_x, gt_data_y, label="Ground Truth")
plt.plot(output_data_x, output_data_y, label="Output")
plt.legend()
plt.show()