import os
import shutil
import numpy as np

def get_map_path(srcdir, subdirs):
	file2camPath = dict()
	# for subdate in subdates:
	# 	subdate_path = os.path.join(srcdir, subdate)
	# 	subdirs = os.listdir(subdate_path)
	for subdir in subdirs:
		subdir_path = os.path.join(srcdir, subdir)
		subdir_timestamps = os.listdir(subdir_path)
		for subdir_timestamp in subdir_timestamps:
			subdir_timestamp_path = os.path.join(subdir_path, subdir_timestamp)
			lms_cams = os.listdir(subdir_timestamp_path)
			for cam in lms_cams:
				cam_path = os.path.join(subdir_timestamp_path, cam)
				#cam_paths.add(cam_path)
				src_files = os.listdir(cam_path)
				for src_file in src_files:
					print("run: ",subdir, " : ",  src_file)
					file2camPath[src_file] = cam_path

	return file2camPath

def fromAnnMVImgs(jsondir, file2camPath, dstdir):
	labelfiles = os.listdir(jsondir)
	for labelfile in labelfiles:
		imgfile = os.path.splitext(labelfile)[0] + '.jpg'

		if imgfile not in file2camPath.keys(): continue
		campath = file2camPath[imgfile]
		dstpath = os.path.join(dstdir, imgfile)
		srcpath = os.path.join(campath, imgfile)
		shutil.move(srcpath, dstpath)

def fromAnnCPImgs(jsondir, file2camPath, dstdir):
	labelfiles = os.listdir(jsondir)
	for labelfile in labelfiles:
		imgfile = os.path.splitext(labelfile)[0] + '.jpg'

		if imgfile not in file2camPath.keys(): continue
		campath = file2camPath[imgfile]
		dstpath = os.path.join(dstdir, imgfile)
		srcpath = os.path.join(campath, imgfile)
		shutil.copyfile(srcpath, dstpath)

def cpImgs(jsondir, imgdir, dstdir):
	labelfiles = os.listdir(jsondir)
	for labelfile in labelfiles:
		print(labelfile)
		imgfile = os.path.splitext(labelfile)[0] + '.jpg'
		imgpath = os.path.join(imgdir, imgfile)
		if not os.path.exists(imgpath): continue
		dstpath = os.path.join(dstdir, imgfile)
		shutil.copyfile(imgpath, dstpath)

if __name__ == '__main__':
	# cpImgs("/media/xubenxiang/data/suzhou_C03/images/TrafficLight_gy0724ys_label",
	# 		"/media/xubenxiang/data/suzhou_C03/images/TrafficLight_gy0724ys_image",
	# 		"/media/xubenxiang/data/suzhou_C03/images/images")
	file2camPath = get_map_path("/media/xubenxiang/data/suzhou_C03/images/",
								["suzhou0705_image"])
	fromAnnCPImgs("/media/xubenxiang/data/suzhou_C03/annos/clean_dir/labels_all_anno", 
				  file2camPath, "/media/xubenxiang/data/suzhou_C03/annos/clean_dir/labels_all_image")

