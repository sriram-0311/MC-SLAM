# read imu data from text file in kitti format and perform dead reckoning on it

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D



world_imu_frame = np.eye(3)

def read_imu_data(filename, rest_time):

    data = np.loadtxt(filename, skiprows=2)

    # extract all data between prev_time and curr_time
    data_rest = data[(data[:,0] <= rest_time)]
    data_move = data[(data[:,0] > rest_time)]

    # extract columns
    timestamp_rest = data_rest[:,0]
    dt_rest = data_rest[:,1]
    accel_rest = data_rest[:,2:5]
    gyro_rest = data_rest[:,5:8]

    timestamp_move = data_move[:,0]
    dt_move = data_move[:,1]
    accel_move = data_move[:,2:5]
    gyro_move = data_move[:,5:8]

    return timestamp_rest, dt_rest, accel_rest, gyro_rest, timestamp_move, dt_move, accel_move, gyro_move


def kabsch(A, B):

    assert A.shape == B.shape
    n, m = A.shape

    EA = np.mean(A, axis=0)
    EB = np.mean(B, axis=0)
    VarA = np.mean(np.linalg.norm(A - EA, axis=1) ** 2)

    H = (np.dot((A - EA).T , (B - EB))) / n
    U, D, VT = np.linalg.svd(H)
    d = np.sign(np.dot(np.linalg.det(U) , np.linalg.det(VT)))
    S = np.diag([1] * (m - 1) + [d])

    R = np.dot(U , np.dot(S , VT))
    
    
    
    
    c = VarA / np.trace(np.dot(np.diag(D) , S))
    t = EA - c * np.dot(R , EB)

    return R, c, t



def procrustes(accel, g):

    accel_x = accel[:,0]
    accel_y = accel[:,1]
    accel_z = accel[:,2]

    accel_data=np.stack((accel_x, accel_y, accel_z), axis=1)
    rows = len(accel_data)

    g_data = np.array([[0, 0, -g]]*rows)

    R, c, t = kabsch(accel_data, g_data)

    print("R: ", R)
    print("determinant: ", np.linalg.det(R))

    return R



def compute_bias(dt_rest, dt_move, accel_rest, gyro_rest, accel_move, gyro_move, g, world_imu_frame):


    # calcualte bias for gyro and accel
    accel_bias = np.mean(accel_rest, axis=0)
    gyro_bias = np.mean(gyro_rest, axis=0)
    print("accel bias: ", accel_bias)
    print("gyro bias: ", gyro_bias)

    accel_bias[2] = 0


    # remove bias from accel and gyro
    accel_rest = accel_rest - accel_bias
    gyro_rest = gyro_rest - gyro_bias
    accel_move = accel_move - accel_bias
    gyro_move = gyro_move - gyro_bias

    # compute rotation matrix using procrustes

    if world_imu_frame is not None:
        world_imu_frame = world_imu_frame
    else:
        world_imu_frame = procrustes(accel_rest, g)

    print("world_imu_frame: ", world_imu_frame)

    # compute acceleration in world frame
    accel_move = np.dot(world_imu_frame, accel_move.T).T


    return accel_bias, gyro_bias, world_imu_frame



def dead_reckoning(dt_rest, dt_move, accel_rest, gyro_rest, accel_move, gyro_move, g, accel_bias, gyro_bias, world_imu_frame):

    positions = []

    prev_vel = np.array([0, 0, 0])
    prev_pos = np.array([0, 0, 0])
    prev_accel = np.array([0, 0, 0])

    bias = True 
    rest = False 

    if(bias):
        #  subtract bias from accel and gyro
        accel_rest = accel_rest - accel_bias
        gyro_rest = gyro_rest - gyro_bias
        accel_move = accel_move - accel_bias
        gyro_move = gyro_move - gyro_bias



    if(rest):


        for i in range(len(dt_rest)):

             # compute velocity
            vel = prev_vel + (prev_accel - g*np.array([0, 0, 1]))*dt_rest[i]

            # compute position
            pos = prev_pos + 0.5*(vel + prev_vel)*dt_rest[i] + 0.5*(prev_accel - g*np.array([0, 0, 1]))*dt_rest[i]**2


            # update prev values
            prev_vel = vel
            prev_pos = pos

            prev_accel = accel_rest[i]

            positions.append(pos)


    else:
        # rotate accel to world frame
        accel_move = np.dot(world_imu_frame, accel_move.T).T

        for i in range(len(dt_move)):
            # compute velocity
            vel = prev_vel + (prev_accel - g*np.array([0, 0, 1]))*dt_move[i]

            # compute position
            pos = prev_pos + 0.5*(vel + prev_vel)*dt_move[i] + 0.5*(prev_accel - g*np.array([0, 0, 1]))*dt_move[i]**2


            # update prev values
            prev_vel = vel
            prev_pos = pos

            prev_accel = accel_move[i]

            positions.append(pos)

    return np.array(positions)


def main():



    # For ZED data

    imu_file = "/home/auv/tri_ws/src/TRI-SLAM/test_imu_modules/src/zed_combined.txt"


    rest_end_time = 1682709193.0760608
    g = 9.81
    # read imu data
    timestamp_rest, dt_rest, accel_rest, gyro_rest, timestamp_move, dt_move, accel_move, gyro_move = read_imu_data(imu_file,rest_end_time)


    # compute bias
    accel_bias, gyro_bias, world_imu_frame = compute_bias(dt_rest, dt_move, accel_rest, gyro_rest, accel_move, gyro_move, g, None)

    # perform dead reckoning
    positions_zed = dead_reckoning(dt_rest, dt_move, accel_rest, gyro_rest, accel_move, gyro_move, g, accel_bias, gyro_bias, world_imu_frame)

    print(world_imu_frame)

    # plot positions
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(positions_zed[:,0], positions_zed[:,1], positions_zed[:,2])
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    plt.show()

    # save positions in a txt file
    np.savetxt("/home/auv/tri_ws/src/TRI-SLAM/test_imu_modules/src/zed_positions_dead_move_bias.txt", positions_zed, delimiter=",")

    # for vector_nav data
    imu_file = "/home/auv/tri_ws/src/TRI-SLAM/test_imu_modules/src/vector_nav_combined.txt"

    rest_end_time = 1682709193.0760608
    g = -9.81

    # read imu data
    timestamp_rest, dt_rest, accel_rest, gyro_rest, timestamp_move, dt_move, accel_move, gyro_move = read_imu_data(imu_file,rest_end_time)


    # compute bias
    accel_bias, gyro_bias, world_imu_frame = compute_bias(dt_rest, dt_move, accel_rest, gyro_rest, accel_move, gyro_move, g, None)

    # perform dead reckoning
    positions_vector = dead_reckoning(dt_rest, dt_move, accel_rest, gyro_rest, accel_move, gyro_move, 9.81, accel_bias, gyro_bias, world_imu_frame)

    print(world_imu_frame)

    # plot positions
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(positions_vector[:,0], positions_vector[:,1], positions_vector[:,2])
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')


    plt.show()

    np.savetxt("/home/auv/tri_ws/src/TRI-SLAM/test_imu_modules/src/vector_positions_dead_move_bias.txt", positions_vector, delimiter=",")



    # plot both positions together in same plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(positions_zed[:,0], positions_zed[:,1], positions_zed[:,2], label="zed")
    ax.plot(positions_vector[:,0], positions_vector[:,1], positions_vector[:,2], label="vector")

    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')



    plt.legend()
    plt.show()


    # plot only x and y positions together
    fig = plt.figure()

    plt.plot(positions_zed[:,0], positions_zed[:,1], label="zed")
    plt.plot(positions_vector[:,0], positions_vector[:,1], label="vector")

    # name the plot 



    plt.xlabel('X Label')
    plt.ylabel('Y Label')



    plt.legend()

    plt.title("Dead Reckoning - zed and vector nav")

    # save the plot
    plt.savefig("/home/auv/tri_ws/src/TRI-SLAM/test_imu_modules/src/dead_reckoning_zed_vector.png")
    plt.show()







if __name__ == "__main__":
    main()