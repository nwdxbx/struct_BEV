# -*- coding: utf-8 -*-

from distutils.log import info
from itertools import starmap
import os
import cv2
import rospy
import roslib
import rosbag

import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

"""
import rosbag
import numpy as np
import sensor_msgs.point_cloud2 as pc2
for topic, msg, t in bag_data:
	lidar = pc2.read_points(msg)
	points = np.array(list(lidar))
"""

"""
subdir = ["day_morning", "vertical_road", "qlg-ntc_InvClockwise",
		  "qlg-ntc_clockwise", "qlg-hxj_InvClockwise", "qlg-hxj_clockwise"]
extra_subdir = ["day_extra", "night", "night_extra"]
"""

dstdir = "/media/xubenxiang/L4TeamShare/Datasets/C03/rosbags/mutil_sensor/0711_images"
bagdir = "/media/xubenxiang/L4TeamShare/Datasets/C03/rosbags/mutil_sensor/0711"

topics = ["/driver/camera/front_m/image/compressed", 
		 "/driver/camera/front_l/image/compressed"]
#cameras = ["front_s", "front_m", "front_l"]
cam2K = {"front_s": [9.1951032939496133e+02, 0., 1.0065431709325795e+03, 
					 0., 9.0751951533796046e+02, 5.5002874908966010e+02, 
					 0., 0., 1.],
		 "front_m": [1.9765137914531631e+03, 0., 8.6089337912962515e+02, 
		 			 0., 1.9814992842786060e+03, 5.4462687094339401e+02, 
		 			 0., 0., 1.],
		 "front_l": [3.7317528362499952e+03, 0., 1.1925982562515139e+03, 
		 			 0., 3.7649276045714264e+03, 2.6542406492521411e+02, 
					 0., 0., 1.]}
cam2disCoef = {"front_s": [-4.6713028166145149e-01, 3.6611304276153639e-01,
             			   6.5267768072069399e-03, -9.4930536903043358e-03,
             			   -1.8661364216780138e-01],
			   "front_m": [-5.9843593339068069e-01, 7.3775595248826753e-01,
             			   1.2549531373987034e-03, 1.0618005365566015e-02,
             			   -8.6289607349141817e-01],
			   "front_l": [-1.4736352409549625e-01, 6.0245574753830322e-01,
             			   -1.7797119509150819e-02, 1.3305972968808482e-02,
             			   -1.7234041477198472e+00]}

def parseros2img(bridge, bagpath, dstdir, subdir, bagfile):
	bag = rosbag.Bag(bagpath, "r")
	for i, topic_name in enumerate(topics):
		bag_data = bag.read_messages(topic_name)
		frameIndex = -1
		for topic, msg, t in bag_data:
			frameIndex = frameIndex + 1
			if frameIndex%5!=0: continue
			_, _, _, camera, _, _ = topic.split('/')
			starmap_str = str(t)
			print("imgfile name: ", starmap_str)
			imgfile = camera + "_" + starmap_str + ".jpg"
			cv_image = bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
			
			# K = np.array(cam2K[camera]).reshape(3, 3)
			# distCoeff = np.array(cam2disCoef[camera])
			# undist_image = cv2.undistort(cv_image, K, distCoeff)
			# cv2.imshow("undist_image", undist_image)

			file_dir = bagfile.split('.')[0] 
			# imgdir = os.path.join(dstdir, subdir, file_dir, camera)
			# if not os.path.exists(imgdir): os.makedirs(imgdir)
			# imgpath = os.path.join(imgdir, imgfile)

			# if not os.path.exists(dstdir): os.makedirs(dstdir)
			# imgpath = os.path.join(dstdir, imgfile)

			imgdir = os.path.join(dstdir, subdir, file_dir)
			if not os.path.exists(imgdir): os.makedirs(imgdir)
			imgpath = os.path.join(imgdir, imgfile)
			cv2.imwrite(imgpath, cv_image, [int(cv2.IMWRITE_JPEG_QUALITY), 100])

			# cv2.imshow("cv_image", cv_image)
			# ch = cv2.waitKey(1)
			# if(ch==ord('q')): exit(0)
			# elif(ch==ord('b')): import pdb; pdb.set_trace() 

bridge = CvBridge()

sub_bags = os.listdir(bagdir)
for bagfile in sub_bags:
	bagpath = os.path.join(bagdir, bagfile)
	parseros2img(bridge, bagpath, dstdir, "", bagfile)
