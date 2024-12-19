import os
import cv2
import json
import argparse
import numpy as np
import open3d as o3d

def parse_opt():
	parser = argparse.ArgumentParser(description='merge different lidar point cloud ')
	parser.add_argument('--data_dir', type=str, default='/media/xubenxiang/L4TeamShare/Datasets/p01/p01_datasets/calib_test_rosbag/src', \
		help='multi sensor data directory')
	parser.add_argument('--main_lidar',  type=str, default='lidar_main_front', help='the combine lidar name')
	parser.add_argument('--sub_dirs', nargs='+', type=str, default=['0903' ], help='sub rosbag dir')
	parser.add_argument('--cam_sensors', nargs='+', type=str, default=['front', 'back_left', 'back_right'], help='the cam sensor name')
	parser.add_argument('--save_dir', type=str, default='/media/xubenxiang/L4TeamShare/Datasets/p01/p01_datasets/calib_test_rosbag/src/calib', help='the ext params of main lidar directory')
	parser.add_argument('--calib_dir', type=str, default='/media/xubenxiang/L4TeamShare/Datasets/p01/data/anns/0817/chanjituopan/calib', help='the calib params directory')

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

def lidar_extrinsic_main_lidar():
	front = np.array([1.0, 0.0, 0.0, 0.0, 
					0.0, 1.0, 0.0, 0.0, 
					0.0, 0.0, 1.0, 0.0, 
					0.0, 0.0, 0.0, 1.0])
	back_left = np.array([-7.9993098126934267e-02, -9.9678963342027960e-01,-3.3957264200169140e-03, 1.4918768208109727e+00,
					9.9554759573253171e-01, -8.0062900186094813e-02,4.9748534098678215e-02, 6.1554268691317446e+00,
					-4.9860694772842828e-02, 5.9893209661342748e-04,9.9875600243358398e-01, -2.7566651466353278e+00,
					0.0, 0.0, 0.0, 1.0])
	back_right = np.array([-7.9628337727310969e-02, 9.9636425079856761e-01,3.0292037920789189e-02, -1.4033205181720156e+00,
					-9.9626216323157613e-01, -8.0567356171226914e-02,3.1154505817280066e-02, 6.1571700291502687e+00,
					3.3481785255950400e-02, -2.7698029716712518e-02,9.9905544851418859e-01, -2.9039775823351230e+00,
					0.0, 0.0, 0.0, 1.0])
	front_left = np.array([0.690437, -0.723214, -0.0160522, 1.36948,
					0.723377, 0.690396, 0.00885946, -2.61797,
					0.00467508, -0.0177287, 0.999832, -2.663,
					0.0, 0.0, 0.0, 1.0])
	front_right = np.array([0.70815, 0.706061, -0.00126777, -1.24875,
					-0.706002, 0.708112, 0.0117572, -2.62518,
					0.00919903, -0.00743083, 0.99993, -2.663,
					0.0, 0.0, 0.0, 1.0])
	front = np.array(front).reshape(4, 4)
	back_left = np.array(back_left).reshape(4, 4)
	back_right = np.array(back_right).reshape(4, 4)
	front_left = np.array(front_left).reshape(4, 4)
	front_right = np.array(front_right).reshape(4, 4)

	calib_params = {}
	calib_params['front'] = front
	calib_params['back_left'] = back_left
	calib_params['back_right'] = back_right
	calib_params['front_left'] = front_left
	calib_params['front_right'] = front_right

	return calib_params

def calib_params(opt):
	calib_dir = opt.calib_dir
	cam_sensors = opt.cam_sensors
	calib_params = {}
	for cam_sensor in cam_sensors:
		calib_params[cam_sensor] = {}
		calib_file = os.path.join(calib_dir, cam_sensor + '.json')
		with open(calib_file, 'r') as f:
			calib_data = json.load(f)
		calib_params[cam_sensor]['extrinsic'] = np.array(calib_data['extrinsic']).reshape(4, 4)
		calib_params[cam_sensor]['intrinsic'] = np.array(calib_data['intrinsic']).reshape(3, 3)
		calib_params[cam_sensor]['distCoeffs'] = np.array(calib_data['distCoeffs'])
	return calib_params

def main_lidar2sensors_calib_params(opt):
	lidar_extrinsic_params = lidar_extrinsic_main_lidar()
	sensor2lidar_params = calib_params(opt)
	cam_sensors = opt.cam_sensors
	for cam_sensor in cam_sensors:
		import ipdb; ipdb.set_trace()
		lidar2main_cam = lidar_extrinsic_params[cam_sensor]
		lidar2sensor_param = sensor2lidar_params[cam_sensor]
		lidar2sensor_extrisic = lidar2sensor_param['extrinsic']
		main2lidar_extrisic = lidar2sensor_extrisic @ np.linalg.inv(lidar2main_cam)
		sensor2lidar_params[cam_sensor]['extrinsic'] = main2lidar_extrisic
	
	return sensor2lidar_params

def save_mainTop_calib_params(opt):
	save_calib_dir = "/media/xubenxiang/L4TeamShare/Datasets/p01/data/anns/0817/chanjituopan/mainTop_calib"
	sensor2lidar_params = main_lidar2sensors_calib_params(opt)
	cam_sensors = opt.cam_sensors
	for cam_sensor in cam_sensors:
		sensor2lidar_params[cam_sensor]['extrinsic'] = (sensor2lidar_params[cam_sensor]['extrinsic'].reshape(-1)).tolist()
		sensor2lidar_params[cam_sensor]['intrinsic'] = (sensor2lidar_params[cam_sensor]['intrinsic'].reshape(-1)).tolist()
		sensor2lidar_params[cam_sensor]['distCoeffs'] = (sensor2lidar_params[cam_sensor]['distCoeffs'].reshape(-1)).tolist()
		save_file_path = os.path.join(save_calib_dir, cam_sensor + ".json")
		with open(save_file_path, 'w') as f:
			json.dump(sensor2lidar_params[cam_sensor], f, indent=4, ensure_ascii=False)

def sensor_key_timestamp(timestamps, key_timestamps):
	total_key_frame = len(key_timestamps)
	dst_key_frame = len(timestamps)
	dst_key_timestamps = []
	for i, key_timestamp in enumerate(key_timestamps):
		min_j = max(0, i-2)
		max_j = min(dst_key_frame-1, i+2)
		min_diff = np.abs(key_timestamp - timestamps[min_j])
		k = min_j
		for j in range(min_j+1, max_j+1):
			diff = np.abs(key_timestamp - timestamps[j])
			if diff<min_diff:
				min_diff = diff
				k = j
		dst_key_timestamps.append(timestamps[k])
	return dst_key_timestamps

def combine_lidar_points(opt, lidar_front_file, lidar_back_left_file, lidar_back_right_file):
	lidars_extrinsic = lidar_extrinsic_main_lidar()
	cam_lidar_params = calib_params(opt)
	pcd_front = o3d.io.read_point_cloud(lidar_front_file)
	pcd_back_left = o3d.io.read_point_cloud(lidar_back_left_file)
	pcd_back_right = o3d.io.read_point_cloud(lidar_back_right_file)
	
	points_front = np.array(pcd_front.points)
	colors_front = np.array(pcd_front.colors)

	points_back_left = np.array(pcd_back_left.points)
	colors_back_left = np.array(pcd_back_left.colors)
	back_left_num = points_back_left.shape[0]
	tmp_left_one = np.ones((back_left_num, 1))
	concat_points_back_left = np.concatenate((points_back_left, tmp_left_one), axis=1)

	points_back_right = np.array(pcd_back_right.points)
	colors_back_right = np.array(pcd_back_right.colors)
	back_right_num = points_back_right.shape[0]
	tmp_right_one = np.ones((back_right_num, 1))
	concat_points_back_right = np.concatenate((points_back_right, tmp_right_one), axis=1)

	back_left2front = lidars_extrinsic['back_left']
	back_right2front = lidars_extrinsic['back_right']

	back_left2front_points = back_left2front @ concat_points_back_left.T
	back_right2front_points = back_right2front @ concat_points_back_right.T

	back_left2front_points = back_left2front_points[:3, ].T
	back_right2front_points = back_right2front_points[:3, ].T

	main_points = np.concatenate((points_front, back_left2front_points, back_right2front_points), axis=0)
	main_colors = np.concatenate((colors_front, colors_back_left, colors_back_right), axis=0)

	pcd = o3d.geometry.PointCloud()
	pcd.points = o3d.utility.Vector3dVector(main_points)
	pcd.colors = o3d.utility.Vector3dVector(main_colors)
	# import ipdb; ipdb.set_trace()

	return pcd

def combine_points(opt):
	data_dir = opt.data_dir
	sub_dirs = opt.sub_dirs
	main_lidar_dir = opt.main_lidar
	for sub_dir in sub_dirs:
		sub_dir_path = os.path.join(data_dir, sub_dir)
		rosbags = os.listdir(sub_dir_path)
		for rosbag in rosbags:
			rosbag_lidar_path = os.path.join(sub_dir_path, rosbag, 'lidar')
			lidar_front_path = os.path.join(rosbag_lidar_path, 'lidar_front')
			lidar_back_left_path = os.path.join(rosbag_lidar_path, 'lidar_back_left')
			lidar_back_right_path = os.path.join(rosbag_lidar_path, 'lidar_back_right')
			lidar_front_files = os.listdir(lidar_front_path)
			lidar_back_left_files = os.listdir(lidar_back_left_path)
			lidar_back_right_files = os.listdir(lidar_back_right_path)
			lidar_front_timestamps = [int(file.split('.')[0]) for file in lidar_front_files]
			lidar_back_left_timestamps = [int(file.split('.')[0]) for file in lidar_back_left_files]
			lidar_back_right_timestamps = [int(file.split('.')[0]) for file in lidar_back_right_files]

			lidar_front_timestamps = sorted(lidar_front_timestamps)
			lidar_back_left_timestamps = sorted(lidar_back_left_timestamps)
			lidar_back_right_timestamps = sorted(lidar_back_right_timestamps)

			back_left_key_timestamps = sensor_key_timestamp(lidar_back_left_timestamps, lidar_front_timestamps)
			back_right_key_timestamps = sensor_key_timestamp(lidar_back_right_timestamps, lidar_front_timestamps)

			lidar_main_path = os.path.join(rosbag_lidar_path, main_lidar_dir)
			if not os.path.exists(lidar_main_path):
				os.makedirs(lidar_main_path)
			for i in range(len(lidar_front_timestamps)):
				front_file = str(lidar_front_timestamps[i]) + '.pcd'
				back_left_file = str(back_left_key_timestamps[i]) + '.pcd'
				back_right_file = str(back_right_key_timestamps[i]) + '.pcd'

				front_file_path = os.path.join(lidar_front_path, front_file)
				back_left_file_path = os.path.join(lidar_back_left_path, back_left_file)
				back_right_file_path = os.path.join(lidar_back_right_path, back_right_file)
				pcd_points = combine_lidar_points(opt, front_file_path, back_left_file_path, back_right_file_path)

				pcd_name = str(lidar_front_timestamps[i]) + '.pcd'
				pcd_path = os.path.join(lidar_main_path, pcd_name)
				o3d.io.write_point_cloud(pcd_path, pcd_points)


def main():
	opt = parse_opt()
	save_mainTop_calib_params(opt)
	#combine_points(opt)


if __name__ == '__main__':
	main()

      

      
