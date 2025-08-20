######Process Kimera Log
import numpy as np
import pandas as pd
import os
from datetime import datetime
import matplotlib.pyplot as plt
from matplotlib import patches
from scipy.spatial.transform import Rotation as R

# The original trajectoru estimates in TUM format
data_file = "/home/auv/Trajectories/VINS-Fusion/complete_building/complete_building_vins_14_03_isec_loop_closure.tum"
df = pd.read_csv(data_file, header=None, sep= " ") 
#df = df[['#timestamp','x','y','z','qx','qy','qz','qw']]

# Relative transform between the imu and cam2 .
Tc2_i = np.array([[-0.014717448030483915, 0.9998772349793116, -0.005376959512298662, 0.24870122345739343],
                [-0.002915514158615351, 0.005334606124805935, 0.9999815207066001, 0.005432018735669777],
                [0.9998874419156695, 0.014732852664032015, 0.0028366444470543928, -0.05379197879298332],
                [0.0, 0.0, 0.0, 1.0]])
# Relative transform between the cam1 and cam2 .
Tc2_c1 = np.array([[0.9998363980211518, -0.0003473714958483863, -0.018084704175858397, -0.1650359869677547],
                   [0.00027089450969152524, 0.9999910121661716, -0.004231099506707433, -0.00040226567143760605],
                   [0.0180860113969072, 0.0042255082433848064, 0.9998275057587841, -0.0033312692471531807],
                   [0.0, 0.0, 0.0, 1.0]])

# Relative transform between the imu and cam1.
# T_c1_i = Tc1_c2 . Tc2_i
Tc1_i  =  np.linalg.inv(Tc2_c1) @ Tc2_i
print(Tc1_i[0:3, 0:3])

# Extract the rotation of first pose in VIO estimate, R_w_i and take its transpose to get R_i_w.
quat = np.array(df.iloc[0, 4:8])
R_i_w = R.from_quat(quat).as_matrix().T
print(R_i_w)

#Obtain the rotation of world W.R.T cam1. R_c1_w = Rc1_i . R_i_w
# this gives the relative orientation between world frame in which the VIO estimates are specified
# and cam1 at the initial estimate.
R_c1_w = Tc1_i[0:3,0:3] @ R_i_w

#for each VIO estimate
for row in range(df.shape[0]):
    quat = np.array(df.iloc[row, 4:8])
    #get the rotation of at time t
    R_w_it = R.from_quat(quat).as_matrix()
    # Convert the rotation of imu in world frame into rotation of cam1 w.r.t cam1's pose at 0th time step
    # R_c1_c1t = Rc1_i . R_i_w . R_w_it . R_i_c1
    rot_1 =  Tc1_i[0:3, 0:3] @ R_i_w @ R_w_it @ Tc1_i[0:3, 0:3].T
    df.iloc[row, -4:] = R.from_matrix(rot_1).as_quat()
    # Convert the position of the VIO estimates form world frame into cam1's frame at 0th times step.
    trans =  np.array(df.iloc[row, 1:4]).reshape(3,1)
    tmp = R_c1_w @ trans
    df.iloc[row, 1:4] = tmp.reshape(-1)
df.to_csv("/home/auv/Trajectories/VINS-Fusion/complete_building/complete_building_vins_14_03_isec_loop_closure_changed.tum", sep=" ", index=False, header=None)


