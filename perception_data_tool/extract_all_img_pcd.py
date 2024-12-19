# -*- coding: utf-8 -*-
import argparse
from distutils.log import info
from itertools import starmap
from pyntcloud import PyntCloud
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
import json

# def pcd_to_bin(pcd_file_path, bin_file_path):
#     # 读取 .pcd 文件
#     cloud = PyntCloud.from_file(pcd_file_path)
    
#     # 获取点云数据
#     points = cloud.points[['x', 'y', 'z', 'intensity']].values.astype('f4')  # 确保数据类型为 float32
    
#     # 确保输出目录存在
#     output_dir = os.path.dirname(bin_file_path)
#     if not os.path.exists(output_dir):
#         os.makedirs(output_dir)
    
#     # 将点云数据写入二进制文件
#     points.tofile(bin_file_path)

def parseros2img(bridge, bagpath, out_dir, subdir, bagfile, topics, save_names, skip_num, json_file_dir):
    cam_intrinsic_param = read_intrinsics_from_json(json_file_dir)
    
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
                camera = topic_splits[0]
                if topic_splits[-1]=="compressed":
                    img_mode = "compressed"
                else: 
                    img_mode = "original"
                
                # starmap_str = str("%.9f" %  msg.header.stamp.toSec())
                starmap_str = str(msg.header.stamp)
                # imgfile = f'{camera}_{img_mode}_{starmap_str}.jpg'
                imgfile = f'{starmap_str}.jpg'

                if "compressed" in topic:
                    cv_image = bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
                else:
                    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")

                #图像去畸变
                # save_names[i]=np.array(cam_intrinsic_param['back_left']['camera_matrix'])
                h, w = cv_image.shape[:2]
                camera_intrinsic = np.array(cam_intrinsic_param[save_names[i]]['camera_matrix']).reshape(3,3)
                # import pdb; pdb.set_trace()
                # camera_distortion_coeffs = np.array(cam_intrinsic_param[save_names[i]]['distortion_coeffs']).reshape(5,)
                # 根据畸变系数的个数给畸变系数的np数组个数
                distortion_coeff_len = len(cam_intrinsic_param[save_names[i]]['distortion_coeffs'])
                camera_distortion_coeffs = np.array(cam_intrinsic_param[save_names[i]]['distortion_coeffs']).reshape(distortion_coeff_len,)

                # import pdb; pdb.set_trace()
                #方法一
                # new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(camera_intrinsic, 
                #                         camera_distortion_coeffs, (w, h), 1, (w, h))
                # undistorted_image = cv2.undistort(cv_image, camera_intrinsic, camera_distortion_coeffs, None, new_camera_matrix)
                #方法二
                undistorted_image = cv2.undistort(cv_image, camera_intrinsic, camera_distortion_coeffs, None, camera_intrinsic)

                # K = np.array(cam2K[camera]).reshape(3, 3)
                # distCoeff = np.array(cam2disCoef[camera])
                # undist_image = cv2.undistort(cv_image, K, distCoeff)
                # cv2.imshow("undist_image", undist_image)

                # file_dir = bagfile.split('.')[0] 
                # print('save_names[i]:',f'{save_names[i]}')
    
                imgdir = os.path.join(out_dir, subdir, file_dir, 'images', f'{save_names[i]}')
                undis_imagedir = os.path.join(out_dir, subdir, file_dir, 'images', f'{save_names[i]}_undistort')
		
                if not os.path.exists(imgdir): 
                    os.makedirs(imgdir)
                if not os.path.exists(undis_imagedir): 
                    os.makedirs(undis_imagedir)
                
                #保存路径
                imgpath = os.path.join(imgdir, imgfile)
                undistorted_imgpath = os.path.join(undis_imagedir, imgfile)
                
                # 保存原图和去畸变的图片
                cv2.imwrite(imgpath, cv_image, [int(cv2.IMWRITE_JPEG_QUALITY), 100])
                cv2.imwrite(undistorted_imgpath, undistorted_image, [int(cv2.IMWRITE_JPEG_QUALITY), 100])

            else:
                
                assert "pointcloud" in topic
                # # 解析点云数据并转换成numpy数组
                gen = pc2.read_points(msg)
                
                points = np.array(list(gen))
                points=points.astype(np.float32)
                
                

                # points to pcd 
                # # 创建Open3D点云对象并设置属性

                # 解析点云数据
                #cloud_points = list(pc2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True))
                # 提取点云数据
                #points = np.array(cloud_points)
                # 创建Open3D点云对象
                # pcd = o3d.geometry.PointCloud()
                # pcd.points = o3d.utility.Vector3dVector(points[:, :3])
                # #pcd.colors = o3d.utility.Vector3dVector(points[:, 3:6] / 255.0)
                # pcd.colors = o3d.utility.Vector3dVector(np.zeros_like(points[:, :3]))  # 初始化颜色为黑    
                # # 将intensity作为颜色信息保存
                # intensity_colors = (points[:, 3] - np.min(points[:, 3])) / (np.max(points[:, 3]) - np.min(points[:, 3]))
                # pcd.colors = o3d.utility.Vector3dVector(np.repeat(intensity_colors.reshape(-1, 1), 3, axis=1))
                # # 根据时间戳生成保存文件名，并将点云保存成.pcd格式文件
                # pcd_dir = os.path.join(out_dir, subdir, file_dir, 'lidar', f'{save_names[i]}')



                # if not os.path.exists(pcd_dir): 
                #     os.makedirs(pcd_dir)

                # pcd_name = str(t.to_nsec()) + '.pcd'
                # bin_name = str(t.to_nsec()) + '.bin'
                # pcd_path = os.path.join(pcd_dir, pcd_name)

                # bin_path = os.path.join(pcd_dir, bin_name)

                # points to bin
                # output_dir = os.path.dirname(bin_path)
                # if not os.path.exists(output_dir):
                #         os.makedirs(output_dir)
                # points.tofile(bin_path)
                # import pdb; pdb.set_trace()
                #o3d.io.write_point_cloud(pcd_path, pcd)
                
                #pcd_to_bin(pcd_path, bin_path)
def read_intrinsics_from_json(json_file_dir):
    with open(json_file_dir, 'r') as f:
        cam_intrinsic_data = json.load(f)
    return cam_intrinsic_data

def run(opt):
    bridge = CvBridge()
    for subdir in opt.bag_subdirs:
        bag_subpath = os.path.join(opt.bagdir, subdir)
        sub_bags = os.listdir(bag_subpath)
        
        for bagfile in sub_bags:
            bagpath = os.path.join(bag_subpath, bagfile)
            parseros2img(bridge, bagpath, opt.out_dir, subdir, bagfile, opt.topics, opt.save_names, opt.skip_num, opt.json_file_dir)
        

def parse_opt():
    parser = argparse.ArgumentParser(description='extract image and pointcloud messages from rosbag')
    parser.add_argument('--out_dir', type=str, default='/media/minzhengyi/data_make_sense/perception_data_tool/Result/Data_lidar_cam_test', \
        help='output directory')
    parser.add_argument('--bagdir', type=str, default='/media/minzhengyi/data_make_sense/perception_data_tool/data', \
        help='total bag dir, including sub_bag_dir1, sub_bag_dir2...')
    parser.add_argument('--bag_subdirs', nargs='+', type=str, default=[''], \
        help='bag sub directory list, eg. sub_bag_dir1')
    parser.add_argument('--skip_num', type=int, default=1, help='filter data, frames[::skip_num], not filter when skip_num=1')
    parser.add_argument(
        '--topics', nargs='+', type=str, default=[
        '/driver/camera/front_l/image/compressed', 
        '/driver/camera/front_s/image/compressed', 
        '/driver/camera/left_back/image/compressed', 
        '/driver/camera/left_front/image/compressed',
        '/driver/camera/right_back/image/compressed',
        '/driver/camera/right_front/image/compressed',
        '/driver/camera/back/image/compressed',
        ], 
        help='rosbag topic(s)')
    parser.add_argument(
        '--save_names', nargs='+', type=str, default=[
        'front_l',
        'front_s',
        'left_back',
        'left_front',
        'right_back',
        'right_front',
        'back',
        ],
        help='save topic name(s)')
    parser.add_argument(
        '--json_file_dir', type=str, default='/home/wangxin/Disk2/KOTEI标注/camera_params.json', \
        help='cam intrinsic json file directory')
    opt = parser.parse_args()
    #opt.imgsz *= 2 if len(opt.imgsz) == 1 else 1  # expand
    return opt

    
def main():
    opt = parse_opt()
    print('opt', opt)

    run(opt)

    
if __name__=="__main__":
    main()
