"""
Code to generate to types of data:
1. Ground truth data
2. GPS data rotated at different angles w.r.t the ground truth to 
depict ENU vs local frame

"""


import time
import numpy as np
import math
import matplotlib.pyplot as plt

rotation = 5
straight = True

if(straight):
  ground_truth_data_file = "/home/neural/catkin_ws/src/TRI-SLAM/test_scripts/min_working_ex/gps_tests/data/straight/sensor_data_straight_gt.txt"
  gps_data = "/home/neural/catkin_ws/src/TRI-SLAM/test_scripts/min_working_ex/gps_tests/data/straight/sensor_data_straight_rot_{}.txt".format(rotation)
else:
  ground_truth_data_file = "/home/neural/catkin_ws/src/TRI-SLAM/test_scripts/min_working_ex/gps_tests/data/curve/sensor_data_curve_gt.txt"
  gps_data = "/home/neural/catkin_ws/src/TRI-SLAM/test_scripts/min_working_ex/gps_tests/data/curve/sensor_data_curve_rot_{}.txt".format(rotation)


# Open files to write data
f1 = open(ground_truth_data_file, 'w+')
f2 = open(gps_data, 'w+')

# Set start x position  
x = 0
ang_vel = 0.05
radius = 10.0
heading = 0.0
x_init = radius * math.cos(heading - np.pi/2)
y_init = radius * math.sin(heading - np.pi/2)

# to plot and debug
x_gt = []
y_gt = []
x_rot_all = []
y_rot_all = []

# rotation matrix to rotate the ground truth 
c, s = np.cos(math.radians(rotation)), np.sin(math.radians(rotation))
R = np.array(((c, -s), (s, c)))
print("Rotation matrix: ", R)

# Set start time
start_time = time.time()

for i in range(100):

  if straight:
    x = i
    y = 0
  else:
    x = radius * math.cos(ang_vel * i - np.pi/2)  - x_init
    y = radius * math.sin(ang_vel * i - np.pi/2) - y_init


  # Write ground truth GPS
  x_gt.append(x)
  y_gt.append(y)

  # rotate the ground truth
  x_rot, y_rot = np.dot(R, np.array([x, y]))
  x_rot_all.append(x_rot)
  y_rot_all.append(y_rot)

  # write ground truth
  g1 = f"x {i} {start_time + i} {x} {y} {0}\n"
  f1.write(g1)  

  # write rotated ground truth
  g2 = f"x {i} {start_time + i} {x} {y} {0}\n"
  f2.write(g2)

  print(x_rot, y_rot)
  x_rot = x_rot + np.random.uniform(-0.05, 0.05)
  y_rot = y_rot + np.random.uniform(-0.05, 0.05)
  print(x_rot, y_rot)

  g3 = f"g {i} {start_time + i} {x_rot} {y_rot} {0}\n"
  f2.write(g3)

  # Add noise to GPS
  x_noisy = x_rot + np.random.uniform(-0.05, 0.05)
  y_noisy = y_rot + np.random.uniform(-0.05, 0.05) 
  z_noisy = np.random.uniform(-0.1, 0.1)

  # Write noisy GPS 
  # g2 = f"g {i} {start_time + i} {x_noisy} {y_noisy} {z_noisy}\n"
  # f2.write(g2)

  # Increment x position
  x += 1
  print(f"Writing data {i}")


# plot the ground truth and rotated ground truth
plt.plot(x_rot_all, y_rot_all, 'x')
plt.plot(x_gt, y_gt, 'o')
plt.show()




f1.close()
f2.close()

print("Done writing data")