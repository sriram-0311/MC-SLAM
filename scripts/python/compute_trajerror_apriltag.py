###################################################################
#####  Process the drift error from April Grid              ##### 
###################################################################
import numpy as np
import pandas as pd
import os
from datetime import datetime
import matplotlib.pyplot as plt
from matplotlib import patches
from scipy.spatial.transform import Rotation as R
import cv2
import sys
import apriltag

np.set_printoptions(suppress=True, formatter={'float_kind':'{:f}'.format})


def draw(image, corners):
	(ptA, ptB, ptC, ptD) = corners
	ptB = (int(ptB[0]), int(ptB[1]))
	ptC = (int(ptC[0]), int(ptC[1]))
	ptD = (int(ptD[0]), int(ptD[1]))
	ptA = (int(ptA[0]), int(ptA[1]))

	# draw the bounding box of the AprilTag detection
	cv2.line(image, ptA, ptB, (0, 255, 0), 2)
	cv2.line(image, ptB, ptC, (0, 255, 0), 2)
	cv2.line(image, ptC, ptD, (0, 255, 0), 2)
	cv2.line(image, ptD, ptA, (0, 255, 0), 2)

	return image

def undistort_img(image, intrinsics_mtx, dist):
	h,  w = image.shape[:2]
	newcameramtx, roi = cv2.getOptimalNewCameraMatrix(intrinsics_mtx, dist, (w,h), 1, (w,h))
	dst = cv2.undistort(image, intrinsics_mtx, dist, None, newcameramtx)
	# x, y, w, h = roi
	# image = dst[y:y+h, x:x+w]
	return dst

def get_pose_aprilgrid(image_path, intrinsics, dist, intrinsics_mtx):
	# print(f"Reading image: {image_path}")
	image = cv2.imread(image_path, 0)
	image = undistort_img(image, intrinsics_mtx, dist)

	options = apriltag.DetectorOptions(families="tag36h11")
	detector = apriltag.Detector(options)
	results = detector.detect(image)

	pose = None
	# for r in results:
	# 	draw(image, r.corners)
	# 	pose, e0, e1 = detector.detection_pose(r, intrinsics)

	if len(results)>1:
		(cX, cY) = (int(results[0].center[0]), int(results[0].center[1]))
		(cX2, cY2) = (int(results[1].center[0]), int(results[1].center[1]))

		if cX<cX2:
			r = results[1]
		else:
			r = results[0]

		draw(image, r.corners)
		pose, e0, e1 = detector.detection_pose(r, intrinsics, tag_size=0.146)

	elif len(results)==1:
		r = results[0]
		draw(image, r.corners)
		pose, e0, e1 = detector.detection_pose(r, intrinsics, tag_size=0.146)
	else:
		pose = None


	# print(abs(pose[2,3]))
	# if abs(pose[2,3])>2:
	# 	print("April tag is too far, estimate wont be accurate!")
	# 	return None

	cv2.imshow("Image", image)
	cv2.waitKey(2)

	print(pose)
	print(np.linalg.inv(pose))

	if pose is not None:
		return np.linalg.inv(pose)
	else:
		return None


def computeDrift_with_time(df, T_w_r,T_0_I, image_list, data_path, intrinsics, distortion, intrinsics_mtx):
	trans_e = []
	rot_e = []
	n = len(image_list)

	df_numpy = df.to_numpy()
	# image_timestamps = []
	for image in image_list:
		image_timestamp = float(image[:-4])*1e-9
		# print(image_timestamp)

		# df_timestamps = np.asarray(df[df.columns[0]])
		df_timestamps = df_numpy[:,0]

		dt = np.abs(image_timestamp - df_timestamps)
		closest = df_numpy[np.argmin(dt)]
		
		print(image)
		print(list(closest))

		# Get the ground trut
		T_wc_n_gt = get_pose_aprilgrid(os.path.join(data_path, image), intrinsics, distortion, intrinsics_mtx) #@ T_0_I

		if T_wc_n_gt is not None:

			# Make the complete prediction matrix
			quat = closest[4:]
			rot = R.from_quat(quat).as_matrix()
			trans = closest[1:4].reshape((3,1))

			T_R_IN = np.vstack((np.hstack((rot, trans)), np.array([0,0,0,1])))

			T_wc_n_pred = T_w_r @ T_R_IN


			deltapose = np.linalg.inv(T_wc_n_pred) @ T_wc_n_gt
			rotvec_cv,_ = cv2.Rodrigues(deltapose[0:3,0:3])
			trans_error = np.linalg.norm(T_wc_n_gt[0:3, 3] - T_wc_n_pred[0:3, 3])
			rot_error = np.rad2deg(np.linalg.norm(rotvec_cv))

			trans_e.append(trans_error)
			rot_e.append(rot_error)


	return trans_e, rot_e


def read_traj(traj):
	data = []
	with open(traj, "r") as file:
		data = [line.split(" ") for line in file.readlines()]

	return data

def main():
	data_path = sys.argv[1]
	traj_path = sys.argv[2]

	fx = 893.6263545058326
	fy = 893.9655105687939
	cx = 376.95348001716707
	cy = 266.57152598273194
	intrinsics = (fx, fy, cx, cy)
	intrinsics_mtx = np.array([[fx, 0, cx],
								[0, fy, cy],
								[0, 0, 1]])
	distortion = np.array([-0.21272110177039052, 0.18283401892861978, -0.00018083866109219808, 0.0011164116025029272])

	W_T_I0 = np.array([])

	# Read the tum format trajectory file
	df = pd.read_csv(traj_path, header=None, skipinitialspace=True, sep=' ')
	# df = read_traj(traj_path)
	
	# Get the images list
	image_list = sorted(os.listdir(data_path))

	# Read the first image to compare drift against
	# img_first = cv2.imread(os.path.join(data_path, image_list[0]), 0)

	# Get the starting pose
	T_wc_0 = get_pose_aprilgrid(os.path.join(data_path, image_list[0]), intrinsics, distortion, intrinsics_mtx)
	print(T_wc_0)

	T_0_I = np.array([[ 0.00336815,  0.99998156, -0.00505389,  0.41275847],
       [ 0.00131466,  0.00504948,  0.99998639,  0.00547729],
       [ 0.99999346, -0.00337474, -0.00129763, -0.05795901],
       [ 0.        ,  0.        ,  0.        ,  1.        ]])

	T_w_I0 = T_wc_0 #@ T_0_I

	df_numpy = df.to_numpy()
	closest = df_numpy[0]
	quat = closest[4:]
	rot = R.from_quat(quat).as_matrix()
	trans = closest[1:4].reshape((3,1))

	T_0_r = np.linalg.inv(np.vstack((np.hstack((rot, trans)), np.array([0,0,0,1]))))
	T_w_r = T_w_I0 #@ T_0_r

	image_list.pop(0)
	
	# Compute average error for last 20 images
	trans_e, rot_e = computeDrift_with_time(df, T_w_r, T_0_I, image_list, data_path, intrinsics, distortion, intrinsics_mtx)

	# trans_error = []
	# rot_error = []
	# for i in range(1,21):
	# 	T_cw = get_pose_aprilgrid(os.path.join(data_path, image_list[-1*i]), intrinsics, distortion, intrinsics_mtx)
	# 	trans_e, rot_e = computeError(df, T_cw, image_list[-1*i])
	# 	trans_error.append(trans_e)
	# 	rot_error.append(rot_e)

	print(f"trans_e: {trans_e}")
	print(f"rot_e: {rot_e}")
	print(f"Average translation error: {np.asarray(trans_e).mean()}")
	print(f"Average rotation error: {np.asarray(rot_e).mean()}")

if __name__=="__main__":
	if len(sys.argv)<3:
		print(f"Syntax is: {sys.argv[0]} path_to_images_directory path_to_trajectory")
		exit()

	main()