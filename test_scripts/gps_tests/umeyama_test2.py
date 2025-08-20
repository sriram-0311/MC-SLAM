"""
dummy code - as of now

Test to see if the umeyama function works as expected and understand the influence of wrong correspondences on the
resulting transformation.

"""



import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def generate_points(start, line_length1, semicircle_radius, line_length2, num_points_line1, num_points_semicircle, num_points_line2):
    line_points = np.array([(x, 0, 0) for x in np.linspace(start[0], line_length1, num_points_line1)])

    theta = np.linspace(0, np.pi, num_points_semicircle)
    semicircle_points = np.array([(line_length1 + semicircle_radius + semicircle_radius * np.cos(t), semicircle_radius * np.sin(t), 0) for t in theta])

    end_of_semicircle = semicircle_points[0]
    # print(semicircle_points)
    # print(end_of_semicircle)

    line_after_semicircle = np.array([(end_of_semicircle[0] + x, end_of_semicircle[1], 0) for x in np.linspace(0, line_length2, num_points_line2)])

    return np.concatenate((line_points, semicircle_points, line_after_semicircle))

def plot_3d_points(points1, points2,  colour1, colour2):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(points1[:, 0], points1[:, 1], points1[:, 2], c=colour1)
    ax.scatter(points2[:, 0], points2[:, 1], points2[:, 2], c=colour2)

    # ax.plot(points[:, 0], points[:, 1], points[:, 2], c='b')
    # set x, y, z limits
    ax.set_xlim3d(-5, 50)
    ax.set_ylim3d(-20, 20)
    ax.set_zlim3d(-20, 20)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.show()

def kabsch(A, B):

    print("A shape: ", A.shape)
    print("B shape: ", B.shape)
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


def rotate_points(points, yaw):

    R = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                  [np.sin(yaw), np.cos(yaw), 0],
                  [0, 0, 1]])

    rotated_points = np.array([R @ point for point in points])
    return rotated_points

def kabsch_rotate_points(points, R, t):

    rotated_points = np.array([R @ point + t for point in points])
    return rotated_points

def plot_3_sets(points1, points2, points3, colour1, colour2, colour3):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(points1[:, 0], points1[:, 1], points1[:, 2], c=colour1)
    ax.scatter(points2[:, 0], points2[:, 1], points2[:, 2], c=colour2)
    ax.scatter(points3[:, 0], points3[:, 1], points3[:, 2], c=colour3)

    # ax.plot(points[:, 0], points[:, 1], points[:, 2], c='b')
    # set x, y, z limits
    ax.set_xlim3d(-5, 50)
    ax.set_ylim3d(-20, 20)
    ax.set_zlim3d(-20, 20)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.show()
    

# Input parameters
line_length1 = 10
semicircle_radius = 10
line_length2 = 10
num_points_line1 = 5
num_points_semicircle = 10
num_points_line2 = 5
start = [0,0,0]
# Generate points and plot
pointset1 = generate_points(start, line_length1, semicircle_radius, line_length2, num_points_line1, num_points_semicircle, num_points_line2)
# plot_3d_points(pointset1, 'b')


# Input parameters
line_length1 = 10
semicircle_radius = 10
line_length2 = 12
num_points_line1 = 5
num_points_semicircle = 10
num_points_line2 = 6
start = [0,0,0]
pointset2 = generate_points(start, line_length1, semicircle_radius, line_length2, num_points_line1, num_points_semicircle, num_points_line2)
# plot_3d_points(pointset1, 'g')


# plot_3d_points(pointset1, pointset2, 'b', 'g')

pointset1_rot = rotate_points(pointset1, np.pi/4)
# plot_3d_points(pointset1_rot, pointset2, 'b', 'g')

pointset2_rot = rotate_points(pointset2[1:], np.pi/4)
# plot_3d_points(pointset2[1:], pointset2_rot, 'b', 'g')



#test1 : p1 vs 45 deg * p1
R, c, t = kabsch(pointset1, pointset1_rot)
print("R: ", R)
print("t : ", t)
pointset1_norm = kabsch_rotate_points(pointset1_rot, R, t)
# plot_3d_points(pointset1_norm, pointset1, 'b', 'g')
plot_3_sets(pointset1_norm, pointset1, pointset1_rot, 'g', 'b', 'r')



# test2: p1 vs p2
R, c, t = kabsch(pointset1, pointset2[1:])
print("R: ", R)
print("t : ", t)
pointset2_norm = kabsch_rotate_points(pointset2[1:], R, t)

plot_3_sets(pointset2_norm, pointset1, pointset2[1:], 'g', 'b', 'r')


# test3: p1 vs 45 deg * p2

pointset2_rot = rotate_points(pointset2[1:], np.pi/4)
R, c, t = kabsch(pointset1, pointset2_rot)
print("R: ", R)
print("t : ", t)
pointset2_norm = kabsch_rotate_points(pointset2_rot, R, t)

plot_3_sets(pointset2_norm, pointset1, pointset2_rot, 'g', 'b', 'r')