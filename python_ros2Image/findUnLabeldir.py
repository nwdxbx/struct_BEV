import os
import cv2
import shutil
import numpy as np

def find_undir_label():
	cam_paths = set()
	file2camPath = dict()

	srcdir = "/media/xubenxiang/data/suzhou_C03/images/suzhou0523_image"
	subdirs = os.listdir(srcdir)
	for subdir in subdirs:
		subdir_path = os.path.join(srcdir, subdir)
		subdir_timestamps = os.listdir(subdir_path)
		for subdir_timestamp in subdir_timestamps:
			subdir_timestamp_path = os.path.join(subdir_path, subdir_timestamp)
			lms_cams = os.listdir(subdir_timestamp_path)
			for cam in lms_cams:
				cam_path = os.path.join(subdir_timestamp_path, cam)
				cam_paths.add(cam_path)
				src_files = os.listdir(cam_path)
				for src_file in src_files:
					print("run: ",subdir, " : ",  src_file)
					file2camPath[src_file] = cam_path

	label_cam_paths = set()
	labeldir = "/media/xubenxiang/data/suzhou_C03/annos/suzhou_anno/TrafficLight_gy0523_image"
	labelfiles = os.listdir(labeldir)
	for labelfile in labelfiles:
		campath = file2camPath[labelfile]
		label_cam_paths.add(campath)
	print("waitkey...")
	import pdb; pdb.set_trace()
	print("finish...")

def mvlable_file():
	file2camPath = dict()
	srcdir = "/media/xubenxiang/data/suzhou_C03/images/suzhou_re_image"
	subdirs = os.listdir(srcdir)
	for subdir in subdirs:
		subdir_path = os.path.join(srcdir, subdir)
		subdir_timestamps = os.listdir(subdir_path)
		for subdir_timestamp in subdir_timestamps:
			subdir_timestamp_path = os.path.join(subdir_path, subdir_timestamp)
			lms_cams = os.listdir(subdir_timestamp_path)
			for cam in lms_cams:
				cam_path = os.path.join(subdir_timestamp_path, cam)
				src_files = os.listdir(cam_path)
				for src_file in src_files:
					print("run: ",subdir, " : ",  src_file)
					file2camPath[src_file] = cam_path

	labeldir = "/media/xubenxiang/data/suzhou_C03/annos/suzhou_anno/TrafficLight_gy0523_image"
	dstdir = "/media/xubenxiang/data/suzhou_C03/images/label_images"
	labelfiles = os.listdir(labeldir)
	for labelfile in labelfiles:
		print(labelfile)
		campath = file2camPath[labelfile]
		srcpath = os.path.join(campath, labelfile)
		dstpath = os.path.join(dstdir, labelfile)
		shutil.move(srcpath, dstpath)
	print("finish...")


if __name__ == '__main__':
	mvlable_file()