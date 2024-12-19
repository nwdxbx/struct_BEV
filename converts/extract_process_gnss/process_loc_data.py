import os
import json
import math
import argparse
import numpy as np

from pyquaternion import Quaternion


def parse_opt():
    parser = argparse.ArgumentParser(description='interpolation location data')
    # parser.add_argument('--out_dir', type=str, default='/media/xubenxiang/L4TeamShare/Datasets/p01/p01_datasets/calib_test_rosbag/src/', \
    #     help='output directory')
    parser.add_argument('--srcDatadir', type=str, default='/media/xubenxiang/L4TeamShare/Datasets/p01/data/label2nuscenes_data/pre_rosbag/Data_lidar_cam/', \
        help='total bag frames dirs, including sub_bag_frame_dir1, sub_bag_frame_dir2...')
    parser.add_argument('--srcData_subdirs', nargs='+', type=str, default=["2024-09-12-10-17-32_pre"], \
        help='source parse bag data sub directory list, eg. sub_bag_dir1')
    parser.add_argument('--cam_subdirs', nargs='+', type=str, default=['back_left_undistort', 'back_right_undistort', 'back_undistort'], \
        help='cam sub directory list, eg. sub_bag_dir1')
    parser.add_argument('--lidar_subdirs', nargs='+', type=str, default=['lidar_front'], \
        help='lidar sub directory list, eg. sub_bag_dir1')
    parser.add_argument('--location_dir', type=str, default='/media/xubenxiang/L4TeamShare/Datasets/p01/data/label2nuscenes_data/pre_rosbag/0912_txt/0912/', \
        help='location parse gnss data dir')
    parser.add_argument(
        '--location_file', nargs='+', type=str, default=['2024-09-12-10-17-32_pre_gnss.txt', '2024-09-12-10-20-13_pre_gnss.txt'], 
        help='rosbag topic(s) files txt')
    opt = parser.parse_args()

    return opt

def euler_to_quaternion(yaw, pitch, roll):
	q = Quaternion(axis=[1, 0, 0], angle=roll) * \
		Quaternion(axis=[0, 1, 0], angle=pitch) * \
		Quaternion(axis=[0, 0, 1], angle=yaw)
	return q

def get_stamptimes(opt, bagdir):
    imgDir = os.path.join(bagdir, 'images')
    lidarDir = os.path.join(bagdir, 'lidar')
    stamptimes = []
    for sub_imgdir in opt.cam_subdirs:
        sub_img_path = os.path.join(imgDir, sub_imgdir)
        imgfiles = os.listdir(sub_img_path)
        img_stamps = [int(imgfile.split('.')[0]) for imgfile in imgfiles]
        stamptimes.extend(img_stamps)
    for sub_lidar_dir in opt.lidar_subdirs:
        sub_lidar_path = os.path.join(lidarDir, sub_lidar_dir)
        lidarfiles = os.listdir(sub_lidar_path)
        lidar_stamps = [int(lidarfile.split('.')[0]) for lidarfile in lidarfiles]
        stamptimes.extend(lidar_stamps)
        
    return stamptimes

def relative_pos(start_translation, start_rotation, cur_translation, cur_rotation):
	start_translation = np.array(start_translation).reshape(3,1)
	cur_translation = np.array(cur_translation).reshape(3,1)
	inv_start_rotation = np.linalg.inv(start_rotation)
	# rel_rot = start_rotation.T*cur_rotation
	# rel_trans = start_rotation.T @ (cur_translation - start_translation)
     
	rel_rot = inv_start_rotation @ cur_rotation
	rel_trans = inv_start_rotation @ (cur_translation - start_translation)

	rel_trans = rel_trans.reshape(3).tolist()
	return rel_rot, rel_trans

def interpolate_pos(t0, t, t1, trans_c0, trans_c1, rot0, rot1):
	# trans_c = np.interp(t, [t0, t1], [trans_c0, trans_c1]) # 插值位置
	trans_c = [np.interp(t, [t0, t1], [trans_c0[i], trans_c1[i]]) for i in range(3)]
	q = Quaternion.slerp(q0=Quaternion(rot0), q1=Quaternion(rot1), amount=(t-t0)/(t1-t0)) # 插值姿态

	return trans_c, q

def interpolate_loc_data(cur_loc, next_loc, stamp):
	cur_stamp, cur_trans_late, cur_yaw_pitch_roll, cur_q, cur_rotmatrix = cur_loc
	n_stamp, n_trans_late, n_yaw_pitch_roll, n_q, n_rotmatrix = next_loc

	inter_trans, inter_q = interpolate_pos(cur_stamp, stamp*1e-9, n_stamp, cur_trans_late, n_trans_late, cur_q, n_q)
	inter_yaw_pitch_roll = inter_q.yaw_pitch_roll
	inter_rotmatrix = inter_q.rotation_matrix
     
	return [stamp, inter_trans, inter_yaw_pitch_roll, inter_q, inter_rotmatrix]

def transform_to_first_loc(first_loc, inter_elements):
    first_stamp, first_trans_late, first_yaw_pitch_roll, first_q, first_rotmatrix = first_loc
    transform_inter_elements = []
    # import ipdb; ipdb.set_trace()
    for i, inter_element in enumerate(inter_elements):
        stamp, trans_late, yaw_pitch_roll, q, rotmatrix = inter_element
        transform_rot, transform_trans = relative_pos(first_trans_late, first_rotmatrix, trans_late, rotmatrix)
        transform_q = Quaternion(matrix=transform_rot, rtol=1e-06, atol=1e-06)
        transform_yaw_pitch_roll = transform_q.yaw_pitch_roll
        transform_inter_elements.append([stamp, transform_trans, transform_yaw_pitch_roll, transform_q, transform_rot])
        
    return transform_inter_elements

def interpolate_loc_stamps_force(loc_elements, stamptimes):
     inter_elements = []
     for i, stamp in enumerate(stamptimes):
          s_stamp = stamp*1e-9
          for j, loc in enumerate(loc_elements[:-1]):
               cur_stamp, cur_trans_late, cur_yaw_pitch_roll, cur_q, cur_rotmatrix = loc
               n_stamp, n_trans_late, n_yaw_pitch_roll, n_q, n_rotmatrix = loc_elements[j+1]
               if s_stamp >= cur_stamp and s_stamp <= n_stamp:
                    inter_loc = interpolate_loc_data(loc, loc_elements[j+1], stamp)
                    inter_elements.append(inter_loc)
     
     return inter_elements

def interpolate_loc_stamps(loc_elements, stamptimes):
    inter_elements = []
    k = 0
    len_total = len(stamptimes)
    stamp = stamptimes[k]
    s_stamp = stamp*1e-9
    for i, loc in enumerate(loc_elements[:-1]):
        cur_stamp, cur_trans_late, cur_yaw_pitch_roll, cur_q, cur_rotmatrix = loc
        n_stamp, n_trans_late, n_yaw_pitch_roll, n_q, n_rotmatrix = loc_elements[i+1]
        if k>=len_total:
             break
        
        while s_stamp<cur_stamp:
               k +=1
               if k>=len_total:
                     break
               stamp = stamptimes[k]
               s_stamp = stamp*1e-9
        if s_stamp > n_stamp:
             continue
        while s_stamp >= cur_stamp and s_stamp <= n_stamp:
             inter_loc = interpolate_loc_data(loc, loc_elements[i+1], stamp)
             inter_elements.append(inter_loc)
             k +=1
             if k>=len_total:
                  break
             stamp = stamptimes[k]
             s_stamp = stamp*1e-9
    return inter_elements
                
def save2json(inter_elements, jsonpath):
    json_list = []
    for ele in inter_elements:
        quaternion_list = [ele[3].w, ele[3].x, ele[3].y, ele[3].z]
        json_list.append({
            'timestamp': ele[0],
            'rotation': quaternion_list,
            'translation': ele[1],
        })
    with open(jsonpath, 'w') as f:
        json.dump(json_list, f, indent=4)

def run(opt):
    for i, subdir in enumerate(opt.srcData_subdirs):
        bagdir = os.path.join(opt.srcDatadir, subdir)
        loc_file_path = os.path.join(opt.location_dir, opt.location_file[i])
        with open(loc_file_path, 'r') as f:
            loc_data = f.readlines()
        loc_data = [[float(x) for x in line.strip().split()] for line in loc_data]
        loc_elements = []
        for x in loc_data:
            stamp = x[0]
            trans_late = x[1:4]
            yaw, pitch, roll = x[4:7]
            q = euler_to_quaternion(yaw, pitch, roll)
            rotmatrix = q.rotation_matrix
            ele = [stamp, trans_late, x[4:7], q, rotmatrix]
            loc_elements.append(ele)
        loc_elements.sort(key=lambda x: x[0])
        stamptimes = get_stamptimes(opt, bagdir)
        stamptimes = sorted(list(set(stamptimes)))
        # inter_elements = interpolate_loc_stamps_force(loc_elements, stamptimes)
        inter_elements = interpolate_loc_stamps(loc_elements, stamptimes)
        transform_inter_elements = transform_to_first_loc(loc_elements[0], inter_elements)
        jsonPath = os.path.join(opt.location_dir, opt.location_file[i].replace('.txt', '.json'))
        save2json(transform_inter_elements, jsonPath)
        

def main():
    opt = parse_opt()
    run(opt)

if __name__ == '__main__':
    main()