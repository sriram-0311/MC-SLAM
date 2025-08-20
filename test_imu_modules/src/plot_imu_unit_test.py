#  code to plot imu data from a txt file


import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def read_imu_data(filename):

    data = np.loadtxt(filename, skiprows=2, delimiter=",")

    time = data[:,0]

    # choose valeus such that they are below a partiucalr time
    data1 = data[time < 20]

    x = data[:,1]
    y = data[:,2]
    z = data[:,3]

    x1 = data1[:,1]
    y1 = data1[:,2]
    z1 = data1[:,3]

    return x, y, z, x1, y1, z1


def plot_imu_data(x, y, z):

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.plot(x, y, z)

    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')

    
   
    

    #  save the plot

    plt.savefig("/home/marley/tri_ws/src/TRI-SLAM/test_imu_modules/src/zed_gtsam_rest_nobias.png")
     # make the axes equal
    # plt.gca().set_aspect('equal', adjustable='box')
    plt.show()

def main():




    #  read the imu data
    x, y, z, x1 , x2, x3 = read_imu_data("/home/marley/tri_ws/src/TRI-SLAM/test_imu_modules/src/zed_move_gtsam_bias_5.txt")

    #  plot the imu data
    plot_imu_data(x, y, z)
    plot_imu_data(x1, x2, x3)


if __name__ == "__main__":
    main()




    