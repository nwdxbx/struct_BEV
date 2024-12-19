# -*- coding: utf-8 -*-
import argparse
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

import open3d as o3d
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2


def parseros2img(bridge, bagpath, out_dir, subdir, bagfile, topics, save_names, skip_num):
    print("bagpath: ", bagpath)
    bag = rosbag.Bag(bagpath, "r")

    file_dir = bagfile.split('.')[0] 

    for i, topic_name in enumerate(topics):
        bag_data = bag.read_messages(topic_name)
        frameIndex = -1
        for topic, msg, t in bag_data:
            frameIndex = frameIndex + 1
            # import pdb; pdb.set_trace()
            if frameIndex%skip_num!=0: 
            	continue

            # secs = t.secs
            # nanoseconds = t.nsecs
            # starmap = secs + nanoseconds*1e-9
            # import datetime
            # dt = datetime.datetime.fromtimestamp(starmap)

            if "image" in topic:
                topic_splits = topic.split('/')
                camera = topic_splits[3]
                if topic_splits[-1]=="compressed":
                    img_mode = "compressed"
                else: 
                    img_mode = "original"
                # starmap_str = str(t)
                starmap_str = str(msg.header.stamp)

                # imgfile = f'{camera}_{img_mode}_{starmap_str}.jpg'
                imgfile = f'{starmap_str}.jpg'

                if "compressed" in topic:
                    cv_image = bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
                else:
                    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")


                # K = np.array(cam2K[camera]).reshape(3, 3)
                # distCoeff = np.array(cam2disCoef[camera])
                # undist_image = cv2.undistort(cv_image, K, distCoeff)
                # cv2.imshow("undist_image", undist_image)

                # file_dir = bagfile.split('.')[0] 
                imgdir = os.path.join(out_dir, subdir, file_dir, 'images', f'{save_names[i]}')

                if not os.path.exists(imgdir): 
                    os.makedirs(imgdir)
                    
                imgpath = os.path.join(imgdir, imgfile)
                cv2.imwrite(imgpath, cv_image, [int(cv2.IMWRITE_JPEG_QUALITY), 100])

            else:
                assert "lidar" in topic
                # 解析点云数据并转换成numpy数组
                gen = pc2.read_points(msg)
                points = np.array(list(gen))
                # 创建Open3D点云对象并设置属性
                pcd = o3d.geometry.PointCloud()
                pcd.points = o3d.utility.Vector3dVector(points[:, :3])
                pcd.colors = o3d.utility.Vector3dVector(points[:, 3:6] / 255.0)
                # 根据时间戳生成保存文件名，并将点云保存成.pcd格式文件
                pcd_dir = os.path.join(out_dir, subdir, file_dir, 'lidar', f'{save_names[i]}')

                if not os.path.exists(pcd_dir): 
                    os.makedirs(pcd_dir)

                # pcd_name = str(t.to_nsec()) + '.pcd'
                pcd_name = str(msg.header.stamp) + '.pcd'
                pcd_path = os.path.join(pcd_dir, pcd_name)
                o3d.io.write_point_cloud(pcd_path, pcd)

                # points = points.astype(np.float32)
                # pcd_name = str(msg.header.stamp) + '.bin'
                # pcd_path = os.path.join(pcd_dir, pcd_name)
                # points.tofile(pcd_path)
                # pp = np.fromfile(pcd_path, dtype=np.float32)#解析bin文件


def run(opt):
    bridge = CvBridge()
    for subdir in opt.bag_subdirs:
        bag_subpath = os.path.join(opt.bagdir, subdir)
        sub_bags = os.listdir(bag_subpath)
        
        for bagfile in sub_bags:
            bagpath = os.path.join(bag_subpath, bagfile)
            parseros2img(bridge, bagpath, opt.out_dir, subdir, bagfile, opt.topics, opt.save_names, opt.skip_num)
        

def parse_opt():
    parser = argparse.ArgumentParser(description='extract image and pointcloud messages from rosbag')
    parser.add_argument('--out_dir', type=str, default='/media/xubenxiang/L4TeamShare/Datasets/p01/p01_datasets/calib_test_rosbag/src/', \
        help='output directory')
    parser.add_argument('--bagdir', type=str, default='/media/xubenxiang/L4TeamShare/Datasets/p01/p01_datasets/calib_test_rosbag/', \
        help='total bag dir, including sub_bag_dir1, sub_bag_dir2...')
    parser.add_argument('--bag_subdirs', nargs='+', type=str, default=['0903'], \
        help='bag sub directory list, eg. sub_bag_dir1')
    parser.add_argument('--skip_num', type=int, default=1, help='filter data, frames[::skip_num], not filter when skip_num=1')
    parser.add_argument(
        '--topics', nargs='+', type=str, default=[
        '/driver/camera/back_left/image/compressed', 
        '/driver/camera/back_right/image/compressed', 
        '/driver/camera/front/image/compressed', 
        '/driver/lidar/hesai/pandar_back_left', 
        '/driver/lidar/hesai/pandar_back_right', 
        '/driver/lidar/hesai/pandar_front', 
        '/driver/lidar/hesai/pandar_front_left', 
        '/driver/lidar/hesai/pandar_front_right', 
        ], 
        help='rosbag topic(s)')
    parser.add_argument(
        '--save_names', nargs='+', type=str, default=[
        'cam_back_left',
        'cam_back_right',
        'cam_front',
        'lidar_back_left',
        'lidar_back_right',
        'lidar_front',
        'lidar_front_left',
        'lidar_front_right',
        ],
        help='save topic name(s)')
    opt = parser.parse_args()
    #opt.imgsz *= 2 if len(opt.imgsz) == 1 else 1  # expand
    return opt

    
def main():
    opt = parse_opt()
    print('opt', opt)
    # import pdb; pdb.set_trace()
    run(opt)

    
if __name__=="__main__":
    main()
