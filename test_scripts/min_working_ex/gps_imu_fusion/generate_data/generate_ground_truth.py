import time
import numpy as np
import os
import math
import matplotlib.pyplot as plt

noisy_data_file = "/home/neural/catkin_ws/src/TRI-SLAM/test_scripts/min_working_ex/gps_imu_fusion/data/sensor_data_noisy1.txt"
ground_truth_data_file = "/home/neural/catkin_ws/src/TRI-SLAM/test_scripts/min_working_ex/gps_imu_fusion/data/sensor_data_gt1.txt"
rotated_data = "/home/neural/catkin_ws/src/TRI-SLAM/test_scripts/min_working_ex/gps_imu_fusion/data/sensor_data_rotated1.txt"


# if not os.path.exists(ground_truth_data_file):
#   open(ground_truth_data_file, 'w+').close()

# if not os.path.exists(noisy_data_file):
#   open(noisy_data_file, 'w+').close()


# Open files to write data
f1 = open(ground_truth_data_file, 'w+')
f2 = open(noisy_data_file, 'w+')
f3 = open(rotated_data, 'w+')

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
angle  = 15
c, s = np.cos(math.radians(angle)), np.sin(math.radians(angle))
R = np.array(((c, -s), (s, c)))
print("Rotation matrix: ", R)



# Set start time
start_time = time.time()

for i in range(100):

  # Write ground truth GPS
  heading = ang_vel * i
  x = radius * math.cos(heading - np.pi/2)  - x_init
  y = radius * math.sin(heading - np.pi/2) - y_init
  x_gt.append(x)
  y_gt.append(y)

  # rotate the ground truth
  x_rot, y_rot = np.dot(R, np.array([x, y]))
  x_rot_all.append(x_rot)
  y_rot_all.append(y_rot)

  # write ground truth
  g1 = f"g {i} {start_time + i} {x} {y} {0}\n"
  f1.write(g1)  

  # write rotated ground truth
  g3 = f"g {i} {start_time + i} {x_rot} {y_rot} {0}\n"
  f3.write(g3)

  # Add noise to GPS
  x_noisy = x_rot + np.random.uniform(-0.1, 0.1)
  y_noisy = y_rot + np.random.uniform(-0.1, 0.1) 
  z_noisy = np.random.uniform(-0.1, 0.1)

  # Write noisy GPS 
  g2 = f"g {i} {start_time + i} {x_noisy} {y_noisy} {z_noisy}\n"
  f2.write(g2)

  # Write IMU data to both
  for j in range(0, 10):
    ax, ay, az = 0, 0, 9.8
    wx, wy, wz = 0, 0, ang_vel
    imu = f"imu_raw {start_time + i + j/10} {ax} {ay} {az} {wx} {wy} {wz}\n"
    f1.write(imu)
    f2.write(imu)
    f3.write(imu)

  # Increment x position
  x += 1
  print(f"Writing data {i}")


# plot the ground truth and rotated ground truth
plt.plot(x_rot_all, y_rot_all)
plt.plot(x_gt, y_gt)
plt.show()




f1.close()
f2.close()

print("Done writing data")