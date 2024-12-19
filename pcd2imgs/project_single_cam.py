import os
import cv2
import math
import json
import shutil
import argparse
import numpy as np
import pyntcloud
import open3d as o3d


def parse_opt():
	parser = argparse.ArgumentParser(description='undisort images data and project point cloud to image')
	parser.add_argument('--data_dir', type=str, default='/media/xubenxiang/L4TeamShare/Datasets/p01/p01_datasets/calib_test_rosbag/anns_front/0903', \
		help='multi sensor data directory')
	parser.add_argument('--scenes_dir', nargs='+', type=str, default=['lidar_camera', 'lidar_camera2', 'lidar_camera3' ], help='sub scenes dir')
	parser.add_argument('--distort_dir', type=str, default='distort_camera', help='distortion camera data dir')
	parser.add_argument('--undistort_dir', type=str, default='undistort_camera', help='undistortion camera data dir')
	parser.add_argument('--sub_cam_sensors', nargs='+', type=str, default=['front'], help='sub image sensor dir')
	parser.add_argument('--lidar_dir', type=str, default='lidar', help='lidar data dir')
	parser.add_argument('--visual_dir', type=str, default='show_camera', help='pointclound project to camera image')

	parser.add_argument('--calib_dir', type=str, default='calib', help='the calib params directory')
	parser.add_argument('--main_calib_dir', type=str, default='calib', help='the calib params convert to main top lidar directory')
	

	opt = parser.parse_args()
	return opt

def read_pcd1(pcd_file):
	"""
	Read a PCD file and return the data as a numpy array
	"""
	# pcd_file = "./CD569_704055_2023_03_07_16_01_23_lidareth_3.pcap_data/1678176083_800005674.pcd"
	data = []
	with open(pcd_file, 'r') as f:
		lines = f.readlines()
		lines = lines[11:]
		for line in lines:
			line = list(line.strip('\n').split(' '))
			x = float(line[0])
			y = float(line[1])
			z = float(line[2])
			i = float(line[3])
			r = math.sqrt(x**2 + y**2 + z**2)*i
			data.append(np.array([x, y, z, i, r]))
	points = np.array(data)

	return points

def read_pcd(pcd_file):

    pcd = o3d.io.read_point_cloud(pcd_file)
    points = np.array(pcd.points)
    
    return points

def read_save_img(mkv_file, save_path):
	"""
	Read a MKV file and save the images to a directory
	"""
	cap = cv2.VideoCapture(mkv_file)
	while True:
		ret, frame = cap.read()
		if not ret:
			break
		cv2.imwrite(os.path.join(save_path, 'frame_%d.jpg' % cap.get(cv2.CAP_PROP_POS_FRAMES)), frame)

def undisort_img(img, K, D):
	"""
	Undisort an image using the camera intrinsic matrix and distortion coefficients
	"""
	h, w = img.shape[:2]
	#newcameramtx, _ = cv2.getOptimalNewCameraMatrix(K, D, (w, h), 0, (w, h))
	undis_img = cv2.undistort(img, K, D, None, K)

	return undis_img, None

def undisort_dir_imgs(dir_path, save_path, K, D, newK_file):
	"""
	Undisort all images in a directory and save them to a new directory
	"""
	newK = None
	for filename in os.listdir(dir_path):
		img = cv2.imread(os.path.join(dir_path, filename))
		undis_img, newK = undisort_img(img, K, D)
		cv2.imwrite(os.path.join(save_path, filename), undis_img)
	print("K: ", newK)
	if newK is not None:
		np.save(newK_file, newK)

def project_points(points, M1, M2):
	"""
	Project 3D points to 2D image coordinates using the camera matrix
	"""
	coords = points[:, :3]
	ones = np.ones((coords.shape[0], 1))
	coords_homogeneous = np.concatenate((coords, ones), axis=1)
	P = M1 @ M2
	coords = coords_homogeneous @ P.T
	coords = np.concatenate([coords, points[:, 3:6]], axis=1)
	coords = coords[np.where(coords[:, 2] > 0)]
	coords[:, 2] = np.clip(coords[:, 2], 1e-5, 1e5)
	coords[:, 0] = coords[:, 0] / coords[:, 2]
	coords[:, 1] = coords[:, 1] / coords[:, 2]
	
	return coords

def gradient_point_cloud_color_map(points):
	"""
	Gradient a point cloud color map
	"""
	colors = np.zeros((points.shape[0], 3))
	dist = np.sqrt(np.square(points[:, 0]) + np.square(points[:, 1]))
	dist_max = np.max(dist)

	dist = dist/51.2
	min = [127, 0, 255]
	max = [255, 255, 0]
	all_color_val = np.array(min).sum() + np.array(max).sum()
	dist_color = dist * all_color_val

	#R reduce (127->0)
	r = 127
	dy_r = 127 - dist_color 
	tmp = np.zeros([colors[dist_color<r].shape[0], 3])
	tmp[:, 0] = dy_r[dist_color<r]
	tmp[:, 1] = 0
	tmp[:, 2] = 255
	colors[dist_color<r] = tmp

	#G add (0->255)
	g = 127 + 255
	dy_g = dist_color - r
	tmp2 = np.zeros([colors[(dist_color>=r) & (dist_color<g)].shape[0], 3])
	tmp2[:, 0] = 0
	tmp2[:, 1] = dy_g[(dist_color>=r) & (dist_color<g)]
	tmp2[:, 2] = 255
	colors[(dist_color>=r) & (dist_color<g)] = tmp2

	#B reduce (255->0)
	b = g + 255
	dy_b = dist_color - g
	tmp3 = np.zeros([colors[(dist_color>=g) & (dist_color<b)].shape[0], 3])
	tmp3[:, 0] = 0
	tmp3[:, 1] = 255
	tmp3[:, 2] = dy_b[(dist_color>=g) & (dist_color<b)]
	colors[(dist_color>=g) & (dist_color<b)] = tmp3

	#R add (0->255)
	r = b + 255
	dy_r = dist_color - b
	tmp4 = np.zeros([colors[(dist_color>=b) & (dist_color<r)].shape[0], 3])
	tmp4[:, 0] = dy_r[(dist_color>=b) & (dist_color<r)]
	tmp4[:, 1] = 255
	tmp4[:, 2] = 0
	colors[(dist_color>=b) & (dist_color<r)] = tmp4

	tmp5 = np.zeros([colors[dist_color>=r].shape[0], 3])
	tmp5[:, 0] = 255
	tmp5[:, 1] = 255
	tmp5[:, 2] = 0
	colors[dist_color>=r] = tmp5

	points = np.concatenate([points[:, :3], colors], axis=1)

	return points

def show_project_result(image, coords):
	"""
	Show the projected result
	"""
	canvas = image.copy()
	for coord in coords:
		color = [int(coord[-1]), int(coord[-2]), int(coord[-3])]
		p = (int(coord[0]), int(coord[1]))
		cv2.circle(canvas, p, 1, color, thickness=2)
	canvas = cv2.resize(canvas, (1920, 1080))
	cv2.imshow("canvas", canvas)
	cv2.waitKey(0)

def project_points_img(points, image, M1, M2):
	"""
	Project 3D points to 2D image coordinates using the camera matrix and distortion coefficients
	"""
	points = gradient_point_cloud_color_map(points)
	coords = project_points(points, M1, M2)
	h, w = image.shape[:2]
	coords = coords[np.where(coords[:, 0] > 0)]
	coords = coords[np.where(coords[:, 0] < w)]
	coords = coords[np.where(coords[:, 1] > 0)]
	coords = coords[np.where(coords[:, 1] < h)]

	return coords

def project_pcd2img(pcd_file, img_file, M1, M2):
	"""
	Project point cloud to image
	"""
	points = read_pcd(pcd_file)
	points = points[~np.isnan(points).any(axis=1), :]
	image = cv2.imread(img_file)
	coords = project_points_img(points, image, M1, M2)
	show_project_result(image, coords)

def batch_undisort_dir_imgs(opt):
	data_dir = opt.data_dir
	scenes_dir = opt.scenes_dir
	sub_cam_sensors = opt.sub_cam_sensors
	for scene in scenes_dir:
		scene_dir = os.path.join(data_dir, scene)
		distort_dir = os.path.join(scene_dir, opt.distort_dir)
		undistort_dir = os.path.join(scene_dir, opt.undistort_dir)
		calib_dir = os.path.join(data_dir, scene, opt.calib_dir)
		for cam_sensor in sub_cam_sensors:
			distort_img_dir = os.path.join(distort_dir, "cam_" + cam_sensor)
			undistort_img_dir = os.path.join(undistort_dir, "cam_" + cam_sensor)
			with open(os.path.join(calib_dir, cam_sensor + ".json"), 'r') as f:
				calib_data = json.load(f)
			extrinsic = calib_data["extrinsic"]
			intrinsic = calib_data["intrinsic"]
			distCoeffs = calib_data["distCoeffs"]
			extrinsic = np.array(extrinsic).reshape(4, 4)
			intrinsic = np.array(intrinsic).reshape(3, 3)
			distCoeffs = np.array(distCoeffs).reshape(5, )
			if not os.path.exists(undistort_img_dir):
				os.makedirs(undistort_img_dir)
			undisort_dir_imgs(distort_img_dir, undistort_img_dir, intrinsic, distCoeffs, None)

def batch_project_pcd2img(opt):
	data_dir = opt.data_dir
	scenes_dir = opt.scenes_dir
	sub_cam_sensors = opt.sub_cam_sensors
	for scene in scenes_dir:
		scene_dir = os.path.join(data_dir, scene)
		undistort_dir = os.path.join(scene_dir, opt.distort_dir)
		calib_dir = os.path.join(data_dir, scene, opt.main_calib_dir)
		visual_dir = os.path.join(data_dir, scene, opt.visual_dir)
		lidar_dir = os.path.join(data_dir, scene, opt.lidar_dir)
		lidar_files = os.listdir(lidar_dir)
		lidar_files = sorted(lidar_files)
		for cam_sensor in sub_cam_sensors:
			undistort_img_dir = os.path.join(undistort_dir, "cam_" + cam_sensor)
			show_img_dir = os.path.join(visual_dir, "cam_" + cam_sensor)
			with open(os.path.join(calib_dir, cam_sensor + ".json"), 'r') as f:
				calib_data = json.load(f)
			extrinsic = calib_data["extrinsic"]
			intrinsic = calib_data["intrinsic"]
			extrinsic = np.array(extrinsic).reshape(4, 4)
			intrinsic = np.array(intrinsic).reshape(3, 3)
			M1 = np.identity(4)
			M1[:3, :3] = intrinsic
			if not os.path.exists(show_img_dir):
				os.makedirs(show_img_dir)
			undisort_imgs = os.listdir(undistort_img_dir)
			undisort_imgs = sorted(undisort_imgs)
			for i in range(len(lidar_files)):
				lidar_file_path = os.path.join(lidar_dir, lidar_files[i])
				img_file_path = os.path.join(undistort_img_dir, undisort_imgs[i])
				show_img_file_path = os.path.join(show_img_dir, undisort_imgs[i])
				project_pcd2img(lidar_file_path, img_file_path, M1, extrinsic)


def main():
	opt = parse_opt()
	# batch_undisort_dir_imgs(opt)
	batch_project_pcd2img(opt)


if __name__ == '__main__':
	main()
