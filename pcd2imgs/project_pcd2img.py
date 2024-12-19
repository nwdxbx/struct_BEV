import os
import cv2
import math
import numpy as np
# from pypcd import pypcd
import pyntcloud
import open3d as o3d

def read_pcd(pcd_file):
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
	newcameramtx, _ = cv2.getOptimalNewCameraMatrix(K, D, (w, h), 0, (w, h))
	undis_img = cv2.undistort(img, K, D, None, newcameramtx)

	return undis_img, newcameramtx

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

def tmp_test_pcd2img():
	pcd_file = "/media/xubenxiang/L4TeamShare/works/p01_datasets/tmp_code/C03_3#_camera_m_flidar_mini/lidar/lidar_front/1713244739258926526.pcd"
	imgfile = "/media/xubenxiang/L4TeamShare/works/p01_datasets/tmp_code/outs/undistort/1713244739295348791.jpg"
	pcd = o3d.io.read_point_cloud(pcd_file)
	points = np.array(pcd.points)
	image = cv2.imread(imgfile)
	M1 = [1.59447900e+03, 0.0, 1.07217206e+03, 0,
          0.0, 1.81145984e+03, 5.63607628e+02, 0,
          0.0, 0.0, 1.0, 0.0, 
          0.0, 0.0, 0.0, 1.0]
	M2 = [6.7643418028865421e-03, -9.9989148658903859e-01,-1.3086585755488656e-02, -1.0280098627826749e-02,
          -4.1761072921821962e-02, 1.2792999537671879e-02,-9.9904572065108566e-01, -7.8334396383429789e-02,
          9.9910472747775114e-01, 7.3043965932278465e-03,-4.1670005063797033e-02, -2.1364888956257047e-01, 
          0., 0., 0., 1.]
	M1 = np.array(M1).reshape(4,4)
	M2 = np.array(M2).reshape(4,4)
	coords = project_points_img(points, image, M1, M2)
	show_project_result(image, coords)

def valid_pcd2img():
	pass


if __name__ == '__main__':

	# dir_path = "./FOV30"
	# save_path = "./FOV30_undisort"
	# K = np.array([[7350.78527499, 0, 1870.94554418], [0, 7391.62947635, 1058.87754414], [0, 0, 1]])
	# D = np.array([-0.24895644, 1.84610906, 0.00805164, -0.00362409, -16.68933325])
	# newK_file = "./newK_30.npy"
	# undisort_dir_imgs(dir_path, save_path, K, D, newK_file)


	# newK_file = "./newK_30.npy"
	# pcd_file = "./CD569_704055_2023_03_07_16_01_23_lidareth_3.pcap_data/1678176083_800005674.pcd"
	# img_file = "./FOV30_undisort/frame_1.jpg"
	# # M1 = np.ones((4,4), dtype=np.float32)
	# M1 = np.identity(4)
	# M2 = [[1.3017263360489351e-01, -9.9149007323348126e-01, -1.5874948289856805e-03, 1.7460211875861645e-01], 
	#    	  [2.4878176249154005e-03, 1.9277376449206018e-03, -9.9999504728325395e-01, -5.7965511381836188e-01], 
	# 	  [9.9148822293753502e-01, 1.3016803949909622e-01, 2.7175851652575522e-03, -9.9702227171141733e-01], 
	# 	  [0, 0, 0, 1]]
	# M2 = np.array(M2)
	# K = np.load(newK_file)
	# # K = np.array([[7350.78527499, 0, 1870.94554418], [0, 7391.62947635, 1058.87754414], [0, 0, 1]])
	# M1[:3, :3] = K

	# # import ipdb; ipdb.set_trace()
	# project_pcd2img(pcd_file, img_file, M1, M2)




	# dir_path = "./FOV120"
	# save_path = "./FOV120_undisort"
	# K = np.array([[1931.21706051, 0, 1910.29161736], [0, 1936.40460938, 1065.68399194], [0, 0, 1]])
	# D = np.array([-0.32681871, 0.12189407, 0.0024711, 0.00011403, -0.0231879])
	# newK_file = "./newK_120.npy"
	# undisort_dir_imgs(dir_path, save_path, K, D, newK_file)

	newK_file = "./newK_120.npy"
	pcd_file = "./CD569_704055_2023_03_07_16_01_23_lidareth_3.pcap_data/1678176083_800005674.pcd"
	img_file = "./FOV120_undisort/frame_10.jpg"
	# M1 = np.ones((4,4), dtype=np.float32)
	M1 = np.identity(4)
	M2 = [[1.3456599756930976e-02, -9.9990845246580728e-01, -1.4165487695290691e-03, 8.8270155688775880e-03], 
	   	  [1.8261121660467700e-01, 3.8503976568478593e-03, -9.8317766350138558e-01, -8.6389661916459737e-01], 
		  [9.8309311028668156e-01, 1.2971550613509131e-02, 1.8264631225825412e-01, -8.9115996364610739e-01], 
		  [0, 0, 0, 1]]

	M2 = np.array(M2)
	K = np.load(newK_file)
	# K = [[1931.21706051, 0, 1910.29161736], [0, 1936.40460938, 1065.68399194], [0, 0, 1]]
	M1[:3, :3] = K

	# import ipdb; ipdb.set_trace()
	project_pcd2img(pcd_file, img_file, M1, M2)


	# tmp_test_pcd2img()