#  code to plot imu data from a txt file


import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def read_imu_data(filename, skiprow):

    if skiprow:
        data = np.loadtxt(filename, skiprows=2, delimiter=",")

    else:
        data = np.loadtxt(filename, delimiter=",")


    x = data[:,0]
    y = data[:,1]
    z = data[:,2]

    return x, y, z

def main():


    
    #files
    # vectornav
    # move
    v_move_dead_bias = "/home/marley/tri_ws/src/TRI-SLAM/test_imu_modules/src/data/vector/move/dead/vector_positions_dead_move_bias.txt"
    v_move_dead_no_bias = "/home/marley/tri_ws/src/TRI-SLAM/test_imu_modules/src/data/vector/move/dead/vector_positions_dead_move_no_bias.txt"

    v_move_gtsam_bias = "/home/marley/tri_ws/src/TRI-SLAM/test_imu_modules/src/data/vector/move/gtsam/vector_nav_move_bias_5.txt"
    v_move_gtsam_no_bias = "/home/marley/tri_ws/src/TRI-SLAM/test_imu_modules/src/data/vector/move/gtsam/vector_nav_move_no_bias_5.txt"

    v_rest_dead_bias = "/home/marley/tri_ws/src/TRI-SLAM/test_imu_modules/src/data/vector/rest/dead/vector_positions_dead_rest_bias.txt"
    v_rest_dead_no_bias = "/home/marley/tri_ws/src/TRI-SLAM/test_imu_modules/src/data/vector/rest/dead/vector_positions_dead_rest_no_bias.txt"


    v_rest_gtsam_bias = "/home/marley/tri_ws/src/TRI-SLAM/test_imu_modules/src/data/vector/rest/gtsam/vector_nav_rest_bias_5.txt"
    v_rest_gtsam_no_bias = "/home/marley/tri_ws/src/TRI-SLAM/test_imu_modules/src/data/vector/rest/gtsam/vector_nav_rest_no_bias_5.txt"

    # zed
    # move
    z_move_dead_bias = "/home/marley/tri_ws/src/TRI-SLAM/test_imu_modules/src/data/zed/move/dead_reckoning/zed_positions_dead_move_bias.txt"
    z_move_dead_no_bias = "/home/marley/tri_ws/src/TRI-SLAM/test_imu_modules/src/data/zed/move/dead_reckoning/zed_positions_dead_move_no_bias.txt"

    z_move_gtsam_bias = "/home/marley/tri_ws/src/TRI-SLAM/test_imu_modules/src/data/zed/move/gtsam/zed_move_gtsam_bias_5.txt"
    z_move_gtsam_no_bias = "/home/marley/tri_ws/src/TRI-SLAM/test_imu_modules/src/data/zed/move/gtsam/zed_move_gtsam_no_bias_5.txt"

    z_rest_dead_bias =  "/home/marley/tri_ws/src/TRI-SLAM/test_imu_modules/src/data/zed/rest/dead/zed_positions_dead_rest_bias.txt"
    z_rest_dead_no_bias = "/home/marley/tri_ws/src/TRI-SLAM/test_imu_modules/src/data/zed/rest/dead/zed_positions_dead_rest_no_bias.txt"

    z_rest_gtsam_bias = "/home/marley/tri_ws/src/TRI-SLAM/test_imu_modules/src/data/zed/rest/gtsam/zed_rest_gtsam_bias_5.txt"
    z_rest_gtsam_no_bias = "/home/marley/tri_ws/src/TRI-SLAM/test_imu_modules/src/data/zed/rest/gtsam/zed_rest_gtsam_no_bias_5.txt"


    # kitti
    # move
    k_move_gtsam_no_bias = "/home/marley/tri_ws/src/TRI-SLAM/test_imu_modules/src/data/kitti/state_kitti_t1_5.txt"



    # read data
    # vectornav
    # move
    v_move_dead_bias_x, v_move_dead_bias_y, v_move_dead_bias_z = read_imu_data(v_move_dead_bias, False)
    v_move_dead_no_bias_x, v_move_dead_no_bias_y, v_move_dead_no_bias_z = read_imu_data(v_move_dead_no_bias, False)

    v_move_gtsam_bias_x, v_move_gtsam_bias_y, v_move_gtsam_bias_z = read_imu_data(v_move_gtsam_bias, True)
    v_move_gtsam_no_bias_x, v_move_gtsam_no_bias_y, v_move_gtsam_no_bias_z = read_imu_data(v_move_gtsam_no_bias, True)

    v_rest_dead_bias_x, v_rest_dead_bias_y, v_rest_dead_bias_z = read_imu_data(v_rest_dead_bias, False)
    v_rest_dead_no_bias_x, v_rest_dead_no_bias_y, v_rest_dead_no_bias_z = read_imu_data(v_rest_dead_no_bias, False)

    v_rest_gtsam_bias_x, v_rest_gtsam_bias_y, v_rest_gtsam_bias_z = read_imu_data(v_rest_gtsam_bias, True)
    v_rest_gtsam_no_bias_x, v_rest_gtsam_no_bias_y, v_rest_gtsam_no_bias_z = read_imu_data(v_rest_gtsam_no_bias, True)

    # zed
    # move
    z_move_dead_bias_x, z_move_dead_bias_y, z_move_dead_bias_z = read_imu_data(z_move_dead_bias, False)
    z_move_dead_no_bias_x, z_move_dead_no_bias_y, z_move_dead_no_bias_z = read_imu_data(z_move_dead_no_bias, False)

    z_move_gtsam_bias_x, z_move_gtsam_bias_y, z_move_gtsam_bias_z = read_imu_data(z_move_gtsam_bias, True)
    z_move_gtsam_no_bias_x, z_move_gtsam_no_bias_y, z_move_gtsam_no_bias_z = read_imu_data(z_move_gtsam_no_bias, True)

    z_rest_dead_bias_x, z_rest_dead_bias_y, z_rest_dead_bias_z = read_imu_data(z_rest_dead_bias, False)
    z_rest_dead_no_bias_x, z_rest_dead_no_bias_y, z_rest_dead_no_bias_z = read_imu_data(z_rest_dead_no_bias, False)

    z_rest_gtsam_bias_x, z_rest_gtsam_bias_y, z_rest_gtsam_bias_z = read_imu_data(z_rest_gtsam_bias, True)
    z_rest_gtsam_no_bias_x, z_rest_gtsam_no_bias_y, z_rest_gtsam_no_bias_z = read_imu_data(z_rest_gtsam_no_bias, True)


    # kitti
    # move
    k_move_gtsam_no_bias_x, k_move_gtsam_no_bias_y, k_move_gtsam_no_bias_z = read_imu_data(k_move_gtsam_no_bias, False)


    # plot data

    # vectornav: move = dead - bias vs no bias
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(v_move_dead_bias_x, v_move_dead_bias_y, v_move_dead_bias_z, label='dead reckoning - bias')
    ax.plot(v_move_dead_no_bias_x, v_move_dead_no_bias_y, v_move_dead_no_bias_z, label='dead reckoning - no bias')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    
    plt.title('VectorNav: Move - Dead Reckoning - Bias vs No Bias')
    # plt.legend()
    plt.legend(bbox_to_anchor=(0.5, 0), loc='upper left')
    # save plot
    plt.savefig('/home/marley/tri_ws/src/TRI-SLAM/test_imu_modules/src/images/vectornav_move_dead_bias_vs_no_bias.png')

    plt.show()


    # vectornav: move = gtsam - bias vs no bias
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(v_move_gtsam_bias_x, v_move_gtsam_bias_y, v_move_gtsam_bias_z, label='gtsam - bias')
    ax.plot(v_move_gtsam_no_bias_x, v_move_gtsam_no_bias_y, v_move_gtsam_no_bias_z, label='gtsam - no bias')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')

    plt.title('VectorNav: Move - GTSAM - Bias vs No Bias')
    plt.legend(bbox_to_anchor=(0.5, 0), loc='upper left')
    # save plot
    plt.savefig('/home/marley/tri_ws/src/TRI-SLAM/test_imu_modules/src/images/vectornav_move_gtsam_bias_vs_no_bias.png')
    plt.show()

    

    # vectornav: rest = dead - bias vs no bias
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(v_rest_dead_bias_x, v_rest_dead_bias_y, v_rest_dead_bias_z, label='dead reckoning - bias')
    ax.plot(v_rest_dead_no_bias_x, v_rest_dead_no_bias_y, v_rest_dead_no_bias_z, label='dead reckoning - no bias')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')

    plt.title('VectorNav: Rest - Dead Reckoning - Bias vs No Bias')
    plt.legend(bbox_to_anchor=(0.5, 0), loc='upper left')
    # save plot
    plt.savefig('/home/marley/tri_ws/src/TRI-SLAM/test_imu_modules/src/images/vectornav_rest_dead_bias_vs_no_bias.png')
    plt.show()

    # vectornav: rest = gtsam - bias vs no bias
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(v_rest_gtsam_bias_x, v_rest_gtsam_bias_y, v_rest_gtsam_bias_z, label='gtsam - bias')
    ax.plot(v_rest_gtsam_no_bias_x, v_rest_gtsam_no_bias_y, v_rest_gtsam_no_bias_z, label='gtsam - no bias')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')

    plt.title('VectorNav: Rest - GTSAM - Bias vs No Bias')
    plt.legend(bbox_to_anchor=(0.5, 0), loc='upper left')
    # save plot
    plt.savefig('/home/marley/tri_ws/src/TRI-SLAM/test_imu_modules/src/images/vectornav_rest_gtsam_bias_vs_no_bias.png')
    plt.show()

    # zed: move = dead - bias vs no bias
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(z_move_dead_bias_x, z_move_dead_bias_y, z_move_dead_bias_z, label='dead reckoning - bias')
    ax.plot(z_move_dead_no_bias_x, z_move_dead_no_bias_y, z_move_dead_no_bias_z, label='dead reckoning - no bias')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')

    plt.title('ZED: Move - Dead Reckoning - Bias vs No Bias')
    plt.legend(bbox_to_anchor=(0.5, 0), loc='upper left')
    # save plot
    plt.savefig('/home/marley/tri_ws/src/TRI-SLAM/test_imu_modules/src/images/zed_move_dead_bias_vs_no_bias.png')
    plt.show()

    # zed: move = gtsam - bias vs no bias
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(z_move_gtsam_bias_x, z_move_gtsam_bias_y, z_move_gtsam_bias_z, label='gtsam - bias')
    ax.plot(z_move_gtsam_no_bias_x, z_move_gtsam_no_bias_y, z_move_gtsam_no_bias_z, label='gtsam - no bias')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')

    plt.title('ZED: Move - GTSAM - Bias vs No Bias')
    plt.legend(bbox_to_anchor=(0.5, 0), loc='upper left')
    # save plot
    plt.savefig('/home/marley/tri_ws/src/TRI-SLAM/test_imu_modules/src/images/zed_move_gtsam_bias_vs_no_bias.png')
    plt.show()

    # zed: rest = dead - bias vs no bias
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(z_rest_dead_bias_x, z_rest_dead_bias_y, z_rest_dead_bias_z, label='dead reckoning - bias')
    ax.plot(z_rest_dead_no_bias_x, z_rest_dead_no_bias_y, z_rest_dead_no_bias_z, label='dead reckoning - no bias')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')

    plt.title('ZED: Rest - Dead Reckoning - Bias vs No Bias')
    plt.legend(bbox_to_anchor=(0.5, 0), loc='upper left')
    # save plot
    plt.savefig('/home/marley/tri_ws/src/TRI-SLAM/test_imu_modules/src/images/zed_rest_dead_bias_vs_no_bias.png')
    plt.show()

    # zed: rest = gtsam - bias vs no bias
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(z_rest_gtsam_bias_x, z_rest_gtsam_bias_y, z_rest_gtsam_bias_z, label='gtsam - bias')
    ax.plot(z_rest_gtsam_no_bias_x, z_rest_gtsam_no_bias_y, z_rest_gtsam_no_bias_z, label='gtsam - no bias')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')

    plt.title('ZED: Rest - GTSAM - Bias vs No Bias')
    plt.legend(bbox_to_anchor=(0.5, 0), loc='upper left')
    # save plot
    plt.savefig('/home/marley/tri_ws/src/TRI-SLAM/test_imu_modules/src/images/zed_rest_gtsam_bias_vs_no_bias.png')
    plt.show()



    # vectornav: move dead vs gtsam - bias
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(v_move_dead_bias_x, v_move_dead_bias_y, v_move_dead_bias_z, label='dead reckoning - bias')
    ax.plot(v_move_gtsam_bias_x, v_move_gtsam_bias_y, v_move_gtsam_bias_z, label='gtsam - bias')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')

    plt.title('VectorNav: Move - Dead Reckoning vs GTSAM - Bias')
    plt.legend(bbox_to_anchor=(0.5, 0), loc='upper left')
    # save plot
    plt.savefig('/home/marley/tri_ws/src/TRI-SLAM/test_imu_modules/src/images/vectornav_move_dead_vs_gtsam_bias.png')

    # vectornav: move dead vs gtsam - no bias
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(v_move_dead_no_bias_x, v_move_dead_no_bias_y, v_move_dead_no_bias_z, label='dead reckoning - no bias')
    ax.plot(v_move_gtsam_no_bias_x, v_move_gtsam_no_bias_y, v_move_gtsam_no_bias_z, label='gtsam - no bias')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')

    plt.title('VectorNav: Move - Dead Reckoning vs GTSAM - No Bias')
    plt.legend(bbox_to_anchor=(0.5, 0), loc='upper left')
    # save plot
    plt.savefig('/home/marley/tri_ws/src/TRI-SLAM/test_imu_modules/src/images/vectornav_move_dead_vs_gtsam_no_bias.png')

    # zed: move dead vs gtsam - bias
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(z_move_dead_bias_x, z_move_dead_bias_y, z_move_dead_bias_z, label='dead reckoning - bias')
    ax.plot(z_move_gtsam_bias_x, z_move_gtsam_bias_y, z_move_gtsam_bias_z, label='gtsam - bias')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')

    plt.title('ZED: Move - Dead Reckoning vs GTSAM - Bias')
    plt.legend(bbox_to_anchor=(0.5, 0), loc='upper left')
    # save plot
    plt.savefig('/home/marley/tri_ws/src/TRI-SLAM/test_imu_modules/src/images/zed_move_dead_vs_gtsam_bias.png')

    # zed: move dead vs gtsam - no bias
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(z_move_dead_no_bias_x, z_move_dead_no_bias_y, z_move_dead_no_bias_z, label='dead reckoning - no bias')
    ax.plot(z_move_gtsam_no_bias_x, z_move_gtsam_no_bias_y, z_move_gtsam_no_bias_z, label='gtsam - no bias')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')

    plt.title('ZED: Move - Dead Reckoning vs GTSAM - No Bias')
    plt.legend(bbox_to_anchor=(0.5, 0), loc='upper left')
    # save plot
    plt.savefig('/home/marley/tri_ws/src/TRI-SLAM/test_imu_modules/src/images/zed_move_dead_vs_gtsam_no_bias.png')


    # vectornav: rest gtsam - bias 
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(v_rest_gtsam_bias_x, v_rest_gtsam_bias_y, v_rest_gtsam_bias_z, label='gtsam - bias')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')

    plt.title('VectorNav: Rest - GTSAM - Bias')
    plt.legend(bbox_to_anchor=(0.5, 0), loc='upper left')
    # save plot
    plt.savefig('/home/marley/tri_ws/src/TRI-SLAM/test_imu_modules/src/images/vectornav_rest_gtsam_bias.png')

    # zed: rest gtsam - bias
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(z_rest_gtsam_bias_x, z_rest_gtsam_bias_y, z_rest_gtsam_bias_z, label='gtsam - bias')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')

    plt.title('ZED: Rest - GTSAM - Bias')
    plt.legend(bbox_to_anchor=(0.5, 0), loc='upper left')
    # save plot
    plt.savefig('/home/marley/tri_ws/src/TRI-SLAM/test_imu_modules/src/images/zed_rest_gtsam_bias.png')

 
    # kitti : move gtsam - no bias
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(k_move_gtsam_no_bias_x, k_move_gtsam_no_bias_y, k_move_gtsam_no_bias_z, label='gtsam - no bias')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')

    plt.title('KITTI: Move - GTSAM - No Bias')
    plt.legend(bbox_to_anchor=(0.5, 0), loc='upper left')
    # save plot
    plt.savefig('/home/marley/tri_ws/src/TRI-SLAM/test_imu_modules/src/images/kitti_move_gtsam_no_bias.png')
    


if __name__ == '__main__':
    main()
