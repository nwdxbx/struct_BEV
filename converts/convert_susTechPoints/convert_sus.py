import os
import cv2
import json
import shutil
import argparse
import numpy as np


def parse_opt():
	parser = argparse.ArgumentParser(description='convert multi sensor data to SUSTechPoints Annotation Formats')
	parser.add_argument('--data_dir', type=str, default='/media/xubenxiang/L4TeamShare/Datasets/p01/p01_datasets/calib_test_rosbag/src/0903', \
		help='multi sensor data directory')
	parser.add_argument('--scenes_dir', nargs='+', type=str, default=['lidar_camera', 'lidar_camera2', 'lidar_camera3'], help='sub scenes dir')
	parser.add_argument('--out_dir', type=str, default='/media/xubenxiang/L4TeamShare/Datasets/p01/p01_datasets/calib_test_rosbag/anns_right_back/0903', help='output directory')
	parser.add_argument('--main_channel', type=str, default='lidar_back_right', help='main_channel')
	parser.add_argument('--sensor_lidar', type=str, default='lidar', help='sensors lidar dir')
	parser.add_argument('--sensor_camera', type=str, default='images', help='sensors camera dir')
	parser.add_argument('--lidar_freq', type=int, default=10, help='the frame rate of main lidar')
	parser.add_argument('--calib_dir', type=str, default='/media/xubenxiang/L4TeamShare/Datasets/p01/calib/0828_result', help='the calib params directory')
	parser.add_argument('--calib_sub_sensor', nargs='+', type=str, default=['back_left', 'back_right', 'front'], help='sub sensor calibration')
	
	opt = parser.parse_args()
	return opt


def sensor_key_timestamp_1(timestamps, key_timestamps):
	total_key_frame = len(key_timestamps)
	index_key_frame = 0
	dst_key_timestamps = []
	dst_total_frame = len(timestamps)
	for i, timestamp in enumerate(timestamps):
		if index_key_frame >=total_key_frame:
			break
		if i+1 < dst_total_frame:
			if key_timestamps[index_key_frame]>=timestamp and key_timestamps[index_key_frame]<timestamps[i+1]:
				dst_key_timestamps.append(timestamp)
				index_key_frame += 1
		else:
			dst_key_timestamps.append(timestamp)
			index_key_frame +=1
			break
	return dst_key_timestamps

def sensor_key_timestamp_2(timestamps, key_timestamps):
	total_key_frame = len(key_timestamps)
	index_key_frame = 0
	dst_key_timestamps = []
	dst_total_frame = len(timestamps)
	for i in range(dst_total_frame):
		if index_key_frame>=total_key_frame:
			break
		if i+1 <dst_total_frame:
			if key_timestamps[index_key_frame]>=timestamps[i] and key_timestamps[index_key_frame]<timestamps[i+1]:
				diff_left = key_timestamps[index_key_frame] - timestamps[i]
				diff_right = timestamps[i+1] - key_timestamps[index_key_frame]
				if diff_left<=diff_right:
					dst_key_timestamps.append(timestamps[i])
				else:
					dst_key_timestamps.append(timestamps[i+1])
				index_key_frame +=1
		else:
			dst_key_timestamps.append(timestamps[i])
			index_key_frame +=1
			break
	return dst_key_timestamps
				

def convert2sus(opt):
	out_dir = opt.out_dir
	data_dir = opt.data_dir
	scenes_dir = opt.scenes_dir
	main_channel = opt.main_channel
	sensor_lidar = opt.sensor_lidar
	sensor_camera = opt.sensor_camera
	inter_freq = opt.lidar_freq//2
	for scene_dir in scenes_dir:
		sub_path = os.path.join(data_dir, scene_dir)

		out_sub_path = os.path.join(out_dir, scene_dir)
		out_calib_path = os.path.join(out_sub_path, 'calib')
		out_distort_camera_path = os.path.join(out_sub_path, 'distort_camera')
		out_camera_path = os.path.join(out_sub_path, 'camera')
		out_label_path = os.path.join(out_sub_path, 'label')
		out_lidar_path = os.path.join(out_sub_path, 'lidar')
		
		if not os.path.exists(out_sub_path): os.makedirs(out_sub_path)
		if not os.path.exists(out_calib_path): os.makedirs(out_calib_path)
		if not os.path.exists(out_distort_camera_path): os.makedirs(out_distort_camera_path)
		if not os.path.exists(out_camera_path): os.makedirs(out_camera_path)
		if not os.path.exists(out_label_path): os.makedirs(out_label_path)
		if not os.path.exists(out_lidar_path): os.makedirs(out_lidar_path)

		main_channel_path = os.path.join(sub_path, sensor_lidar, main_channel)
		main_pcd_files = os.listdir(main_channel_path)
		main_timestamps = [int(file.split('.')[0]) for file in main_pcd_files]
		main_timestamps = sorted(main_timestamps)
		key_timestamps = main_timestamps[inter_freq-1::inter_freq]
		
		cam_data_path = os.path.join(sub_path, sensor_camera)
		cam_channels = os.listdir(cam_data_path)
		cam_channel_dict=dict()
		for cam_channel in cam_channels:
			imgpath = os.path.join(cam_data_path, cam_channel)
			dstpath = os.path.join(out_distort_camera_path, cam_channel)
			if not os.path.exists(dstpath): os.makedirs(dstpath)
			imgfiles = os.listdir(imgpath)
			img_timestamps = [int(file.split('.')[0]) for file in imgfiles]
			img_timestamps = sorted(img_timestamps)
			cam_key_timestamps = sensor_key_timestamp_1(img_timestamps, key_timestamps)
			cam_channel_dict[cam_channel] = cam_key_timestamps	


			assert len(cam_key_timestamps) == len(cam_channel_dict[cam_channel])
			cam_channel_file = os.path.join(out_distort_camera_path, cam_channel) + '.txt'
			fw = open(cam_channel_file, 'w')
			for i in range(len(cam_key_timestamps)):
				cam_file = str(cam_key_timestamps[i]) + '.jpg'
				srcfile = os.path.join(imgpath, cam_file)
				dstfile = os.path.join(dstpath, cam_file)
				shutil.copyfile(srcfile, dstfile)
				res_txt = str(cam_key_timestamps[i]) + " " + str(key_timestamps[i]) + '\n'
				fw.write(res_txt)
			fw.close()
		
		for i in range(len(key_timestamps)):
			lidar_file = str(key_timestamps[i]) + '.pcd'
			src_file = os.path.join(main_channel_path, lidar_file)
			dst_file = os.path.join(out_lidar_path, lidar_file)
			shutil.copyfile(src_file, dst_file)

def calib2sus(calib_intri_path, calib_extri_path):
	cam_intri_fs = cv2.FileStorage(calib_intri_path, cv2.FILE_STORAGE_READ)
	intrinsics = cam_intri_fs.getNode('K').mat()
	distCoeffs = cam_intri_fs.getNode('D').mat()
	cam_intri_fs.release()

	extri_fs = cv2.FileStorage(calib_extri_path , cv2.FILE_STORAGE_READ)
	extrinsic = extri_fs.getNode('TransMatFromLaserToCamera').mat()
	extri_fs.release()

	intrinsics = np.reshape(intrinsics, (-1)).tolist()
	distCoeffs = np.reshape(distCoeffs, (-1)).tolist()
	extrinsic = np.reshape(extrinsic, (-1)).tolist()
	calib_params = dict()
	calib_params['extrinsic'] = extrinsic
	calib_params['intrinsic'] = intrinsics
	calib_params['distCoeffs'] = distCoeffs

	return calib_params


def batch_calib2sus(opt):
	calib_dit = opt.calib_dir
	calib_sub_sensor = opt.calib_sub_sensor
	for sensor_dir in calib_sub_sensor:
		calib_path = os.path.join(calib_dit, sensor_dir)
		calib_intri_path = os.path.join(calib_path, 'CamIntrinsicPara.yaml')
		calib_extri_path = os.path.join(calib_path, 'LaserPointsAndExtrinsicPara.yaml')
		calib_params = calib2sus(calib_intri_path, calib_extri_path)
		result_path = os.path.join(calib_path, sensor_dir + '.json')
		with open(result_path, 'w') as f:
			json.dump(calib_params, f, indent=4, ensure_ascii=False)



def main():
	opt = parse_opt()
	convert2sus(opt)
	# batch_calib2sus(opt)

if __name__ == '__main__':
	main()